#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "cpu6.h"
#include "dma.h"
#include "dsk.h"
#include "hawk.h"
#include "scheduler.h"

#ifndef O_BINARY
#define O_BINARY 0
#endif

/* DSK: A controller for the CDC 9427H "Hawk" drive
 *
 * Split across two cards: DSK/AUT and DSKII
 *
 *	F140	unit select
 *	F141	}
 *	F142	}	C/H/S as xxCC_CCCC_CCCH_SSSS
 *  F143    Write enable bitmask
 *  F144    Controller status
 *  F145    Unit status
 *	F148 W	command
 *	F148 R  status	bit 0 is some kind of busy/accept
 *
 *	Command codes
 *	0:		Read
 *	1:		Write
 *	2:		Seek
 *	3:		Restore
 *
 *	The Hawk interface seems to be an oddity as it doesn't appear to use
 *	the Fin Fout Busy style interface and sequencer but something smarter
 *	of its own.
 *
 *
 */

/* I've taken the number from the ceiling */
#define NUM_HAWK_UNITS 8

// This seems to be hardcoded (or configurable by jumpers?)
static uint8_t dsk_irq = 2;

// F140 selected unit
static uint8_t dsk_selected_unit;

// F141/F142 selected sector
// bits are xxCC_CCCC_CCCH_SSSS
static uint16_t dsk_selected_sector;

static unsigned dsk_interrupt_enabled;
static uint16_t dsk_status;

static unsigned dsk_tracing;
static unsigned dsk_sync_count;


// Busy
// Controller State machine is processing a command
static uint8_t dsk_busy;

// Format Error
// Controller couldn't find the sync pattern before address or data.
// Sync pattern is ~87 zeros then a one
static uint8_t dsk_fmt_err;

// Addr Error
// Controller read 16bit address at start of sector, and it was for the
// it didn't match the controllers sector register (F141/F142)
static uint8_t dsk_addr_err;

// Timeout Error
// Controller state machine timed out.
// For reads/writes, It didn't see correct sector index from hawk unit.
// For seeks/rtz, It probally didn't see on_cyl from unit
static uint8_t dsk_timeout;

// CRC Error
// Controller encountered a CRC error after address read or data read.
static uint8_t dsk_crc_error;

static uint8_t dsk_seek_active;
static uint8_t dsk_seek_complete;

static struct hawk_unit hawk[NUM_HAWK_UNITS];

static void dsk_seek(unsigned trace);
static void dsk_update_status();

enum dsk_state_t {
	STATE_SEEK,
	STATE_WAIT_SEEK,
	STATE_RTZ,
	STATE_START, // read or write
	STATE_WAIT_SECTOR,
	STATE_ADDR_SYNC,
	STATE_READ_ADDR,
	STATE_CHECK_ADDR,
	STATE_DATA_SYNC,
	STATE_READ_DATA,
	STATE_IDLE,
	STATE_DONE,
};

static const char *dsk_state_names[] = {
	"SEEK",
	"WAIT_SEEK",
	"RTZ",
	"START",
	"WAIT_SECTOR",
	"ADDR_SYNC",
	"READ_ADDR",
	"CHECK_ADDR",
	"DATA_SYNC",
	"READ_DATA",
	"IDLE",
	"DONE",
};

static enum dsk_state_t dsk_state = STATE_IDLE;
static enum dsk_state_t dsk_old_state = STATE_IDLE;

static unsigned dsk_check_sync() {
	struct hawk_unit* unit = &hawk[dsk_selected_unit];
	uint64_t time = get_current_time();

	while (hawk_remaining_bits(unit, time) > 8) {
		uint8_t data;
		hawk_read_bits(unit, 8, &data);
		if (data == 0) {
			dsk_sync_count += 8;
		} else if (dsk_sync_count > 60) {
			// At some threshold, the state-machine has seen enough bits
			hawk_rewind(unit, __builtin_clz(data));
			return 1;
		}
	}
	return 0;
}

static void dsk_run_state_machine(unsigned trace)
{
	unsigned unit = dsk_selected_unit;
	uint64_t reschedule_ns = 0;

	if (dsk_old_state != dsk_state && trace) {
		fprintf(stderr, "DSK: state machine moved to %s\n", dsk_state_names[dsk_state]);
		dsk_old_state = dsk_state;
	}

	switch (dsk_state) {
	case STATE_SEEK:
		// Start a sync
		dsk_seek(trace);
		if (hawk[unit].addr_ack) {
			dsk_state = STATE_WAIT_SEEK;
			dsk_seek_active |= 1 << (unit / 2);
		}
		break;
	case STATE_RTZ:
		// start a rtz
		hawk_rtz(&hawk[unit]);
		if (hawk[unit].addr_ack) {
			dsk_state = STATE_WAIT_SEEK;
		}
		break;
	case STATE_WAIT_SEEK:
		// Wait until all active seeks/RTZs are complete
		if (dsk_seek_active == 0) {
			dsk_busy = 0;
			dsk_state = STATE_IDLE;
		}
		break;
	case STATE_START:
		// Start of a read or write
		hawk_wait_sector(&hawk[unit], dsk_selected_sector & 0xf);
		dsk_state = STATE_WAIT_SECTOR;
		break;
	case STATE_WAIT_SECTOR:
		// wait for the sector
		if (hawk[unit].sector_addr == (dsk_selected_sector & 0xf)) {
			dsk_state = STATE_ADDR_SYNC;
			dsk_sync_count = 0;
		} else {
			fprintf("DSK: sector mismatch %hx %x\n", hawk[unit].sector_addr, dsk_selected_sector & 0xf);
		}
		break;
	case STATE_ADDR_SYNC:
		// wait for a sync
		if(dsk_check_sync()) {
			dsk_state = STATE_READ_ADDR;
		} else {
			reschedule_ns = 4000;
		}
		break;
	case STATE_DONE:
		dsk_busy = 0;
		dsk_state = STATE_IDLE;
		break;
	case STATE_IDLE:
		if (dsk_interrupt_enabled) {
			cpu_assert_irq(dsk_irq);
		}
		reschedule_ns = 0;
		break;
	}

	dsk_tracing = trace;

	// Testing on real hardware suggests status register is latched to only
	// update on state machine change
	dsk_update_status();

	if (dsk_old_state != dsk_state && trace) {
		fprintf(stderr, "DSK: state machine moved to %s\n", dsk_state_names[dsk_state]);
		dsk_old_state = dsk_state;
	}
}

void dsk_hawk_changed(unsigned unit)
{
	printf("DSK: unit %u changed\n", unit);
	if (hawk[unit].on_cyl) {
		unsigned drive_bit = 1 << (unit >> 1);

		if (dsk_seek_active & drive_bit) {
			dsk_seek_active &= ~drive_bit;
			dsk_seek_complete |= drive_bit;
		}
	}

	dsk_run_state_machine(dsk_tracing);
}

void dsk_init(void)
{
	int i;
	char name[32];

	for (i = 0; i < NUM_HAWK_UNITS; i++) {
		memset(&hawk[i], 0, sizeof(hawk[i]));

		sprintf(name, "hawk%u.disk", i);
		hawk[i].unit_num = i;
		hawk[i].fd = open(name, O_RDWR|O_BINARY);
		hawk[i].wprotect = 1;

		if (hawk[i].fd > 0) {
			// On a real drive it's more complex. But for us, if
			// we have an image file for the unit, it's ready.

			hawk[i].ready = 1;

			// Hawk units automatically RTZ when first coming online
			hawk_rtz(&hawk[i]);
		}
	}
}

static int hawk_get_fd(void)
{
	return dsk_selected_unit < NUM_HAWK_UNITS ? hawk[dsk_selected_unit].fd : -1;
}

static void dsk_update_status() {
	struct hawk_unit *u = &hawk[dsk_selected_unit];

	dsk_status = (dsk_seek_complete & 0x0f) // bits 0-3. One per hawk unit
	     | (u->ready      << 4)   // Probally the ready signal from drive
	     | (u->on_cyl     << 5)   // Head is on the correct cylinder
	     | (0             << 6)   // write enable
	     | (u->wprotect   << 7)   // Write Protect bit
		 | (dsk_busy      << 8)   // command in progress
	     | (u->fault      << 9)   // drive fault
	     | (u->seek_error << 10)  // Guess. Causes OPSYS to retry
	     | (0             << 11)  // not seen
	     | (dsk_fmt_err   << 12)  // Format Error (couldn't find preamble before address or data)
	     | (dsk_addr_err  << 13)  // Address Error (address didn't match)
	     | (dsk_timeout   << 14)  // Seek error line from drive
	     | (0             << 15); // not seen
}

static void hawk_clear_controller_error() {
	dsk_crc_error = 0;
	dsk_addr_err = 0;
	dsk_fmt_err = 0;
	dsk_timeout = 0;
}

/*	"Cylinder  0 - 405
 *	 MSB:

 *		Cyl 256
 *		Cyl 128
 *		Cyl 64
 *		Cyl 32
 *		Cyl 16
 *		Cyl 8
 *		Cyl 4
 *		Cyl 2
 *		Cyl 1
 *		Head 0/1,
 *		Sector 8,
 *		Sector 4,
 *		Sector 2,
 *		Sector 1.
 *	 LSB
 *
 *       Max Cyl - 405,  Max Heads 0 /1 ,  Max Sectors 0-F .   So Max
 *	 Tracks are 810  =   405 Cyl x two Heads with 16 sectors of 400
 *	 bytes per track"
 *		-- Ken Romain
 */
static void dsk_seek(unsigned trace)
{
	unsigned unit = dsk_selected_unit;
	unsigned sec = dsk_selected_sector & 0x0F;
	unsigned head = !!(dsk_selected_sector & 0x10);
	unsigned cyl = dsk_selected_sector >> 5;

	if (trace)
		fprintf(stderr, "%04x: %i Seek to %u/%u/%u\n", cpu6_pc(), unit,
			cyl, head, sec);

	if (hawk[unit].ready) {
		hawk_seek(&hawk[unit], cyl, head);
	}
}

uint8_t hawk_read_next(void)
{
	int fd = hawk_get_fd();
	uint8_t c;

	if (fd == -1) {
		dsk_timeout = 1;
	}
	else if (read(fd, (void *)&c, 1) != 1) {
		fprintf(stderr, "hawk I/O error\n");
		// Treat this as if we read invalid data
		dsk_crc_error = 1;
	}
	return c;
}

void hawk_write_next(uint8_t c)
{
	int fd = hawk_get_fd();

	if (fd == -1) {
		dsk_timeout = 1;
	} else if (write(fd, (void *) &c, 1) != 1) {
		fprintf(stderr, "hawk I/O error\n");
		// Not sure if we have a write error, lets just set a few
		dsk_crc_error = 1;
		dsk_addr_err = 1;
		dsk_fmt_err = 1;
	}
}

void hawk_dma_done(void)
{
    hawk_set_dma(0);
	dsk_busy = 0;

	unsigned unit = dsk_selected_unit;

	// The Hawk disk unit will fault if a read/write is done off cylinder
	if (hawk[unit].on_cyl == 0) {
		hawk[unit].fault = 1;
	}

	// If hawk is on the wrong cylinder
	if (hawk[unit].current_track != (dsk_selected_sector >> 4)) {
		dsk_addr_err = 1;
	}
}

/*
 *	Commands and registers as described by Ken Romain except status
 *	was in a slightly different spot.
 *
 *	"The Hawk disk controller design (DSK/Auto & DSKII) was 6 years
 *	 before the AMD2901 was available.  The (DSK/Auto & DSKII) is a
 *	 simple MMIO with a TTL state machine which runs in sync with the
 *	 Hawks Read / Write data stream to advance the state machine.
 *
 *	 A basic sector read from the Hawk (DSK/Auto & DSKII)  is....
 *	 Drive select Reg.  0xF140,    Sector address Reg. 0xF141-F142
 *	 Status Reg ( I think ) 0xF143   and Command Reg 0xF148 (00 = read,
 *	 01 = write,  02 = seek,   03 = RTZ Return Track Zero.
 *       The (DSK/Auto & DSKII)  Hawk sectors are 400 bytes ( 0x190) long
 *	 and R / W can be 1 to 16 sector operations based on the total bytes
 *	 the DMA was setup to move in /out of DRAM. "
 *			-- Ken Romain
 *
 *	"For the longest time the OPSYS minimum disk file size was 16 sectors
 *	 for 400 bytes ( being one logical disk track and also one physical
 *	 track on the CDC 9427H Hawk drive). When we had large soft sector
 *	 drives like the 9448 Phoenix we still kept the 400 byte logical
 *	 sector length and padded the sector to 512 bytes."
 *			-- Ken Romain
 */
static void dsk_cmd(uint8_t cmd, unsigned trace)
{
	// Controller errors appear to be cleared when starting a new command
	hawk_clear_controller_error();

	dsk_busy = 1;

	switch (cmd) {
	case 0:		/* Multi sector read  - 1 to 16 sectors */
		if (trace)
			fprintf(stderr, "%04X: hawk %i Read %f sectors\n", cpu6_pc(),
				dsk_selected_unit, cpu6_dma_count() / 400.0);
		dsk_state = STATE_START;
		break;
	case 1:		/* Multi sector write - ditto */
		if (trace)
			fprintf(stderr, "%04X: hawk %i Write %f sectors\n", cpu6_pc(),
				dsk_selected_unit, cpu6_dma_count() / 400.0);
        dsk_state = STATE_START;
		break;
	case 2:		/* Seek */
		dsk_state = STATE_SEEK;
		break;
	case 3:		/* Return to Track Zero Sector (Recalibrate) */
		if (trace)
			fprintf(stderr, "%04X: hawk %i Return to Zero\n", cpu6_pc(),
				dsk_selected_unit);
		dsk_state = STATE_RTZ;
		break;
	case 4:		/* Format sector - Ken thinks but not sure */
	default:
		fprintf(stderr, "%04X: Unknown hawk command %02X\n",
			cpu6_pc(), cmd);
		dsk_busy = 0;
		break;
	}
}

void dsk_write(uint16_t addr, uint8_t val, unsigned trace)
{
	switch (addr) {
	case 0xF140:
		dsk_selected_unit = val;
		if (trace)
			fprintf(stderr, "Selected hawk unit %i\n", val);
		break;
	case 0xF141:
		dsk_selected_sector &= 0x00ff;
		dsk_selected_sector |= val << 8;
		break;
	case 0xF142:
		dsk_selected_sector &= 0xff00;
		dsk_selected_sector |= val;
		break;
	// case 0xF143:
		/* "It is a Write Enable Bit Mask to help protect against writing to a
		*  disk platter in error should someone just enter a 0x1 Write Command
		*  or 0x4 Format Command in error to 0xF148.
		*  Layout is something like this.
		*  0xF143  MSB     128    64   32   16    8   4   2    1    LSB
		*  1 = Write Enable Platter = 0 drive 1
		*  2 = Write Enable Platter = 1 drive 1
		*  4 = Write Enable Platter = 0 drive 2
		*  8 = Write Enable Platter = 1 drive 2
		*  ...............
		*  128 = Write Enable Platter = 1 drive 4
		*  Regards, Ken R."
		*/

	case 0xF144:
	case 0xF145:
		/* Guess.. it's done early in boot */
		hawk_clear_controller_error();
		break;
	case 0xF148:
		dsk_cmd(val, trace);
		break;
	case 0xF14C:
		// Strobe. This seems to force an interrupt.
		// Guess, it's forcing the statemachine to DONE.
		dsk_state = STATE_DONE;
		break;
	case 0xF14E:
		// Strobe. Enable interrupt on DONE
		dsk_interrupt_enabled = 1;
		break;
	case 0xF14F:
		// Strobe. Acknowledge interrupt
		dsk_interrupt_enabled = 0;
		break;
	default:
		fprintf(stderr,
			"%04X: Unknown hawk I/O write %04X with %02X\n",
			cpu6_pc(), addr, val);
		return;
	}

	dsk_run_state_machine(trace);
}

uint8_t dsk_read(uint16_t addr, unsigned trace)
{
	uint8_t status;
	switch (addr) {
	case 0xF141:
		return dsk_selected_sector >> 8;
	case 0xF142:
		return dsk_selected_sector & 0xff;
	case 0xF144:
		status = dsk_status >> 8;
		// if (trace)
		// 	fprintf(stderr, "%04X: hawk status read high | %02x__\n", cpu6_pc(), status);
		return status;
	case 0xF145:
		status = dsk_status & 0xff;
		// if (trace)
		// 	fprintf(stderr, "%04X: hawk status read low  | __%02x\n", cpu6_pc(), status);
		return status ;
	case 0xF148:		/* Bit 0 seems to be set while it is processing */
		return dsk_busy;
	default:
		fprintf(stderr, "%04X: Unknown hawk I/O read %04X\n",
			cpu6_pc(), addr);
		return 0xFF;
	}
}

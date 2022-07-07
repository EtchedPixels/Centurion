#include <assert.h>
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

static uint16_t dsk_cylinder;
static uint8_t  dsk_head;
static uint8_t  dsk_sector;

static unsigned dsk_interrupt_enabled;
static unsigned dsk_interrupt_ack;
static uint16_t dsk_status;

static unsigned dsk_tracing;
static unsigned dsk_sync_count; // Number of zero bits seen during sync
static unsigned dsk_transfer_mode; // 1=read, 0=write

static unsigned dsk_transfer_count; // number of bytes transferred during current sector

static void dsk_timeout_cb(struct event_t* event, int64_t late_ns);
static struct event_t dsk_timeout_evt = {
	.name = "dsk_timeout",
	.delta_ns = 100 * ONE_MILISECOND_NS,
	.callback = dsk_timeout_cb
};

static void dsk_runstate_cb(struct event_t* event, int64_t late_ns);
static struct event_t dsk_runstate_evt = {
	.name = "dsk_runstate",
	.delta_ns = 0,
	.callback = dsk_runstate_cb
};

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
	STATE_CHECK_ADDR,
	STATE_DATA_SYNC,
	STATE_READ_DATA,
	STATE_CRC,
	STATE_IDLE,
	STATE_FINISH,
};

static const char *dsk_state_names[] = {
	"SEEK",
	"WAIT_SEEK",
	"RTZ",
	"START",
	"WAIT_SECTOR",
	"ADDR_SYNC",
	"CHECK_ADDR",
	"DATA_SYNC",
	"READ_DATA",
	"CRC",
	"IDLE",
	"FINISH",
};

static enum dsk_state_t dsk_state = STATE_IDLE;
static enum dsk_state_t dsk_old_state = STATE_IDLE;


static void dsk_reschedule(int64_t delta_ns)
{
	dsk_runstate_evt.delta_ns = delta_ns;
	schedule_event(&dsk_runstate_evt);
}

static void dsk_goto_idle() {
	dsk_state = STATE_FINISH;
	cancel_event(&dsk_timeout_evt);
	hawk_set_dma(0);

	dsk_reschedule(0); // Immediately
}

static void dsk_check_sync(enum dsk_state_t success_state, int64_t time)
{
	struct hawk_unit* unit = &hawk[dsk_selected_unit];
	int remaining = hawk_remaining_bits(unit, time);

	// At some threshold, the state-machine has seen enough zero bits
	// Guess, threshold is ~60 bits
	const int sync_threshold = 60;

	// if (remaining < (sync_threshold - dsk_sync_count)) {
	// 	dsk_reschedule(HAWK_BIT_NS * (remaining - (sync_threshold - dsk_sync_count)) + HAWK_BIT_NS);
	// 	return;
	// }

	// Do this in chunks of 8 bits, for better performance
	while (remaining > 8) {
		uint8_t data = hawk_read_byte(unit);
		remaining -= 8;

		if (data == 0) {
			dsk_sync_count += 8;
			continue;
		}

		dsk_sync_count += 8;
		unsigned rewind_count = 0;

		while (data != 1) {
			data >>= 1;
			dsk_sync_count--;
			rewind_count++;

		}
		fprintf(stderr, "Sync after %i zeros. Rewind %d bits\n", dsk_sync_count, rewind_count);
		// if we read too much, rewind to simplify our code
		hawk_rewind(unit, rewind_count);

		if (dsk_sync_count > 60) {
			dsk_state = success_state;
			return;
		} else {
			// We saw a 1 bit too early. Format Error
			dsk_fmt_err = 1;
			dsk_goto_idle();
			return;
		}
	}

	dsk_reschedule(HAWK_BIT_NS * (16 - remaining));
	return;
}

static void dsk_verify_addr(int64_t time)
{
	struct hawk_unit* unit = &hawk[dsk_selected_unit];

	int remaining = hawk_remaining_bits(unit, time);

	if (remaining < 32) {
		dsk_reschedule(HAWK_BIT_NS * (32 - remaining));
		return;
	}

	uint16_t expected = (dsk_cylinder << 5) | (dsk_head << 4) | dsk_sector;
	uint16_t addr = hawk_read_word(unit);
	// Guess: checkword is just inverted addrs
	uint16_t checkword = ~hawk_read_word(unit);

	if (addr != expected || checkword != expected) {
		fprintf(stderr, "Addr error: %04hx != %04hx || %04hx != %04hx\n", addr, expected, checkword, expected);
		dsk_addr_err = 1;
		dsk_goto_idle();
		return;
	}

	dsk_state = STATE_DATA_SYNC;
}

static void dsk_read_data(int64_t time)
{
	struct hawk_unit* unit = &hawk[dsk_selected_unit];
	int remaining = hawk_remaining_bits(unit, time);

	while (remaining >= 8) {
		uint8_t data = hawk_read_byte(unit);
		cpu6_dma_write(data);
		fprintf(stderr, "%02x ", data);
		if (dsk_transfer_count % 16 == 1) {
			fprintf(stderr, "\n");
		}
		remaining -= 8;
		if (--dsk_transfer_count == 0) {
			dsk_state = STATE_CRC;
			return;
		}
	}
	if (remaining == 0)
		remaining = 8;
	dsk_reschedule(HAWK_BIT_NS * remaining);
}

static void dsk_do_crc(int64_t time)
{
	struct hawk_unit* unit = &hawk[dsk_selected_unit];
	int remaining = hawk_remaining_bits(unit, time);

	if (remaining < 16) {
		dsk_reschedule(HAWK_BIT_NS * (16 - remaining));
		return;
	}

	if (dsk_transfer_mode == 1) {
		uint16_t crc = hawk_read_word(unit);
		// TODO: Proper CRC function
		if (crc != 0xcccc) {
			fprintf(stderr, "DSK: CRC error. Got 0x%04x\n", crc);
			dsk_crc_error = 1;
			dsk_goto_idle();
		} else {
			dsk_sector = (dsk_sector + 1) & 0xf;
			hawk_wait_sector(unit, dsk_sector);
			dsk_state = STATE_WAIT_SECTOR;
		}
	} else {
		fprintf(stderr, "DISK unimplemented transfer mode %d\n", dsk_transfer_mode);
	}
}

static void dsk_run_state_machine(unsigned trace, int64_t time)
{
	unsigned unit = dsk_selected_unit;
	dsk_tracing = trace;

	do {
		if (dsk_old_state != dsk_state) {
			dsk_old_state = dsk_state;

			if (trace)
				fprintf(stderr, "DSK: state machine moved to %s\n", dsk_state_names[dsk_state]);
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
				dsk_seek_active |= 1 << (unit / 2);
				dsk_state = STATE_WAIT_SEEK;
			}
			break;
		case STATE_WAIT_SEEK:
			// Wait until all active seeks/RTZs are complete
			if (dsk_seek_active == 0) {
				dsk_goto_idle();
			}
			break;

		case STATE_START:
			// Start of a read or write
			hawk_wait_sector(&hawk[unit], dsk_sector);
			dsk_state = STATE_WAIT_SECTOR;
			break;
		case STATE_WAIT_SECTOR:
			// wait for the sector
			hawk_update(&hawk[unit], time);
			if (hawk[unit].sector_pulse && hawk[unit].sector_addr == dsk_sector) {
				dsk_state = STATE_ADDR_SYNC;
				dsk_sync_count = 0;
			}
			break;
		case STATE_ADDR_SYNC:
			// wait for a sync
			dsk_check_sync(STATE_CHECK_ADDR, time);
			break;
		case STATE_CHECK_ADDR:
			dsk_verify_addr(time);
			break;
		case STATE_DATA_SYNC:
			// wait for a sync
			// guess: In order to allow enough time for the current instruction to finish
			//        DSK requests a DMA lock as soon as it starts looking for sync
			hawk_set_dma(dsk_transfer_mode);
			dsk_check_sync(STATE_READ_DATA, time);
			dsk_transfer_count = HAWK_SECTOR_BYTES;
			break;
		case STATE_READ_DATA:
			// read data
			dsk_read_data(time);
			break;
		case STATE_CRC:
			//
			dsk_do_crc(time);
			break;
		case STATE_FINISH:
			// Guess: If interrupts are enabled, we stay here until the
			// interrupt is cleared.
			if (dsk_interrupt_enabled && !dsk_interrupt_ack) {
				cpu_assert_irq(dsk_irq);
				if (trace)
					fprintf(stderr, "DSK: interrupt asserted\n");
			} else if (dsk_interrupt_ack) {
				if (trace)
					fprintf(stderr, "DSK: interrupt acked\n");
				dsk_interrupt_ack = 0;
				cpu_deassert_irq(dsk_irq);
				dsk_state = STATE_IDLE;
			} else {
				dsk_state = STATE_IDLE;
			}
		case STATE_IDLE:
			break;
		}

		dsk_interrupt_ack = 0;

		// Testing on real hardware suggests status register is latched to only
		// update on state machine change
		dsk_update_status();

	} while (dsk_state != dsk_old_state);
}

static void dsk_runstate_cb(struct event_t* event, int64_t late_ns)
{
	int64_t time = get_current_time() - late_ns;
	dsk_run_state_machine(dsk_tracing, time);
}

static void dsk_timeout_cb(struct event_t* event, int64_t late_ns)
{
	if (dsk_tracing) {
		fprintf(stderr, "DSK: timeout in state %s\n",
			dsk_state_names[dsk_state]);
	}

	// Kill any outstanding DMA transfers
	hawk_set_dma(0);

	dsk_timeout = 1;
	dsk_goto_idle();
}

void dsk_hawk_changed(unsigned unit, int64_t time)
{
	if (hawk[unit].on_cyl) {
		unsigned drive_bit = 1 << (unit >> 1);

		if (dsk_seek_active & drive_bit) {
			dsk_seek_active &= ~drive_bit;
			dsk_seek_complete |= drive_bit;
		}
	}

	dsk_run_state_machine(dsk_tracing, time);
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

			// Hack: I don't think this happens on real hardware, but this
			//       allows us to skip WIPL and start with LOAD
			dsk_seek_active |= 1 << (i / 2);
		}
	}
}

static void dsk_update_status() {
	struct hawk_unit *u = &hawk[dsk_selected_unit];

	unsigned busy = dsk_state != STATE_IDLE;

	dsk_status = (dsk_seek_complete & 0x0f) // bits 0-3. One per hawk unit
	     | (u->ready      << 4)   // Probally the ready signal from drive
	     | (u->on_cyl     << 5)   // Head is on the correct cylinder
	     | (0             << 6)   // write enable
	     | (u->wprotect   << 7)   // Write Protect bit
		 | (busy          << 8)   // command in progress
	     | (u->fault      << 9)   // drive fault
	     | (u->seek_error << 10)  // Guess. Causes OPSYS to retry
	     | (0             << 11)  // not seen
	     | (dsk_fmt_err   << 12)  // Format Error (couldn't find preamble before address or data)
	     | (dsk_addr_err  << 13)  // Address Error (address didn't match)
	     | (dsk_timeout   << 14)  // Seek error line from drive
	     | (0             << 15); // not seen

	if (busy & u->fault) {
		// If we have a fault, abort any running command
		dsk_goto_idle();
	}
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

	if (trace)
		fprintf(stderr, "%04x: %i Seek to %u/%u/%u\n", cpu6_pc(), unit,
			dsk_cylinder, dsk_head, dsk_sector);

	if (hawk[unit].ready) {
		hawk_seek(&hawk[unit], dsk_cylinder, dsk_head);
	}
}

void hawk_dma_done(void)
{
    dsk_goto_idle();
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
	if (dsk_state != STATE_IDLE) {
		if (trace)
			fprintf(stderr, "%04X: statemachine busy. cmd=%i\n", cpu6_pc(), cmd);
	}

	schedule_event(&dsk_timeout_evt);

	// Controller errors appear to be cleared when starting a new command
	hawk_clear_controller_error();

	switch (cmd) {
	case 0:		/* Multi sector read  - 1 to 16 sectors */
		if (trace)
			fprintf(stderr, "%04X: hawk %i Read %i bytes\n", cpu6_pc(),
				dsk_selected_unit, cpu6_dma_count());
		dsk_transfer_mode = 1;
		dsk_state = STATE_START;
		break;
	case 1:		/* Multi sector write - ditto */
		if (trace)
			fprintf(stderr, "%04X: hawk %i Write %i bytes\n", cpu6_pc(),
				dsk_selected_unit, cpu6_dma_count());
		dsk_transfer_mode = 2;
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
		break;
	}
}

void dsk_write(uint16_t addr, uint8_t val, unsigned trace)
{
	switch (addr) {
	case 0xF140:
		dsk_selected_unit = val;
		// guess, selecting unit updates status
		dsk_update_status();
		if (trace)
			fprintf(stderr, "Selected hawk unit %i. %i%i\n", val, hawk[val].on_cyl, hawk[val].ready);
		break;
	case 0xF141:
		// bits are xxCC_CCCC_CCCH_SSSS
		dsk_cylinder &= 0x007;
		dsk_cylinder |= (val << 3);
		break;
	case 0xF142:
		// continued
		dsk_cylinder &= 0x7f8;
		dsk_cylinder |= val >> 5;
		dsk_head = !!(val & 0x10);
		dsk_sector = val & 0x0f;
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
		// Guess, it's forcing the state machine to FINISH which will trigger
		// an interrupt if interrupts are enabled.
		if (trace)
			fprintf(stderr, "%04X: hawk %i Force Interrupt\n", cpu6_pc(), dsk_selected_unit);
		dsk_state = STATE_FINISH;
		break;
	case 0xF14D:
		// Disable interrupts.
		if (trace)
			fprintf(stderr, "%04X: hawk %i Disable Interrupts\n", cpu6_pc(), dsk_selected_unit);
		dsk_interrupt_enabled = 0;
		break;
	case 0xF14E:
		// Strobe. Enable interrupt on FINISH
		if (trace)
			fprintf(stderr, "%04X: hawk %i Enable Interrupt\n", cpu6_pc(), dsk_selected_unit);
		dsk_interrupt_enabled = 1;
		break;
	case 0xF14F:
		// Strobe. Acknowledge interrupt
		if (trace)
			fprintf(stderr, "%04X: hawk %i Acknowledge Interrupt\n", cpu6_pc(), dsk_selected_unit);
		dsk_interrupt_ack = 1;
		break;
	default:
		fprintf(stderr,
			"%04X: Unknown hawk I/O write %04X with %02X\n",
			cpu6_pc(), addr, val);
		return;
	}

	dsk_run_state_machine(trace, get_current_time());
}

uint8_t dsk_read(uint16_t addr, unsigned trace)
{
	uint8_t status;
	switch (addr) {
	case 0xF141:
		// 16bit word
		// bits are xxCC_CCCC_CCCH_SSSS
		// These two registers seem to be the current sector under head.
		// But the unit doesn't report the actual cylinder/head position just
		// the sector.
		// So DSK probally combines the sector address with the last cyclinder
		// and head written to 0xF141
		return dsk_cylinder >> 3;
	case 0xF142:
		// Continued

		// Make sure hawk unit has latest state
		hawk_update(&hawk[dsk_selected_unit], get_current_time());

		return (dsk_cylinder << 5) | (dsk_head << 4) | hawk[dsk_selected_unit].sector_addr;
	case 0xF144:
		status = dsk_status >> 8;
		if (trace)
		 	fprintf(stderr, "%04X: hawk status read high | %02x__\n", cpu6_pc(), status);
		return status;
	case 0xF145:
		status = dsk_status & 0xff;
		if (trace)
		 	fprintf(stderr, "%04X: hawk status read low  | __%02x\n", cpu6_pc(), status);
		return status ;
	case 0xF148:		/* Bit 0 seems to be set while it is processing */
		return dsk_state != STATE_IDLE;
	default:
		fprintf(stderr, "%04X: Unknown hawk I/O read %04X\n",
			cpu6_pc(), addr);
		return 0xFF;
	}
}

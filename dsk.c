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

// F140 selected unit
static uint8_t dsk_selected_unit;

// F141/F142 selected sector
// bits are xxCC_CCCC_CCCH_SSSS
static uint16_t dsk_selected_sector;


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

static struct hawk_unit hawk[NUM_HAWK_UNITS];

void dsk_init(void)
{
	int i;
	char name[32];

	for (i = 0; i < NUM_HAWK_UNITS; i++) {
		memset(&hawk[i], 0, sizeof(hawk[i]));

		sprintf(name, "hawk%u.disk", i);
		hawk[i].fd = open(name, O_RDWR|O_BINARY);

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

/* F145
 * These all appear to be status signals directly from the Hawk unit.
 * The controller muxes this to currently selected unit
 */
uint8_t hawk_get_unit_status() {
	struct hawk_unit *u = &hawk[dsk_selected_unit];
	// Not known: addr_int, fault, seek_error

	return (u->addr_ack << 0) // Guess: address acknowledge
	     | (0           << 1)
	     | (0           << 2)
	     | (0           << 3)
	     | (u->ready    << 4) // Probally the ready signal from drive
	     | (u->on_cyl   << 5) // Head is on the correct cylinder
	     | (0           << 6)
	     | (u->wprotect << 7); // Write Protect bit
}

/* F144
 * These all appear to be controller status bits.
 */
static uint8_t hawk_get_controller_status() {
	// Not sure where these error bits go. Just put them everywhere.
	// lets just put it in all these remaining bits we suspect are errors
	uint8_t unk_error_bit = dsk_crc_error;

	return (dsk_busy      << 0) // command in progress
// WIPL ignores bit 1 after read, Bootstrap requires it to be zero after read
	     | (0             << 1) // either write error, or not an error
	     | (unk_error_bit << 2)
	     | (unk_error_bit << 3)
	     | (dsk_fmt_err   << 4) // Format Error (couldn't find preamble before address or data)
	     | (dsk_addr_err  << 5) // Address Error (address didn't match)
// Bootstrap loops forever unless dsk_timeout or hawk->on_cyl goes high
	     | (dsk_timeout   << 6) // Seek error line from drive
	     | (unk_error_bit << 7);
}

static void hawk_clear_controller_error() {
	dsk_crc_error = 0;
	dsk_addr_err = 0;
	dsk_fmt_err = 0;
	dsk_timeout = 0;
	hawk[dsk_selected_unit].addr_ack = 0;
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
		hawk_seek(&hawk[unit], cyl, head, sec);
	} else {
		// If the hawk unit wasn't ready, then the state machine will timeout
		dsk_timeout = 1;
	}

	// seeks are currently instant, so command completed
	dsk_busy = 0;
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
		hawk_set_dma(1);
		break;
	case 1:		/* Multi sector write - ditto */
		if (trace)
			fprintf(stderr, "%04X: hawk %i Write %f sectors\n", cpu6_pc(),
				dsk_selected_unit, cpu6_dma_count() / 400.0);
        hawk_set_dma(2);
		break;
	case 2:		/* Seek */

		dsk_seek(trace);
		break;
	case 3:		/* Return to Track Zero Sector (Recalibrate) */
		if (trace)
			fprintf(stderr, "%04X: hawk %i Return to Zero\n", cpu6_pc(),
				dsk_selected_unit);
		hawk_rtz(&hawk[dsk_selected_unit]);
		dsk_busy = 0;
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
	default:
		fprintf(stderr,
			"%04X: Unknown hawk I/O write %04X with %02X\n",
			cpu6_pc(), addr, val);
	}
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
		status = hawk_get_controller_status();
		if (trace)
			fprintf(stderr, "%04X: hawk controller status read | %02x\n", cpu6_pc(), status);
		return status;
	case 0xF145:
		status = hawk_get_unit_status();
		if (trace)
			fprintf(stderr, "%04X: hawk unit status read | %02x\n", cpu6_pc(), status);
		return status ;
	case 0xF148:		/* Bit 0 seems to be set while it is processing */
		return dsk_busy;
	default:
		fprintf(stderr, "%04X: Unknown hawk I/O read %04X\n",
			cpu6_pc(), addr);
		return 0xFF;
	}
}

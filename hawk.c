#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>

#include "cpu6.h"
#include "dma.h"
#include "hawk.h"

#ifndef O_BINARY
#define O_BINARY 0
#endif

/*
 *	CDC 9427H Hawk disk controller
 *	F140	unit select
 *	F141	}
 *	F142	}	C/H/S of some format
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
 *	This is roughly complete except that we don't know what actual
 *	error codes were used, and this ignores the unit select entirely
 *	at the moment.
 */

/* I've taken the number from the ceiling */
#define NUM_HAWK_UNITS 8

static uint8_t hawk_unit;
static uint8_t hawk_sech;
static uint8_t hawk_secl;

// All these "from Hawk unit" signals are muxed to selected unit.

// Ready
// From Hawk unit.
// High when:
//  - This disk cartridge is installed
//  - Spindle motor speed is correct
//  - heads loaded
//  - DC voltages within margin
//  - no fault condition exists
//  - unit selected
//  - terminator is present and has power
static uint8_t hawk_ready;

// On Cyl (On Cylinder)
// From Hawk unit.
// Set when the heads have finished seeking to the desired address.
// Also high when a seek error occurs
static uint8_t hawk_on_cyl;

// Sker (Seek Error)
// From Hawk unit
// Set when the unit was unable to complete a seek operation.
// A RTZS command from the controller clears the seek error
// condition and returns the heads to cylinder 00.
// On Cylinder is also asserted during seek error.
static uint8_t hawk_seek_error;

// Fault
// From Hawk unit
// Indicates that the unit has encounterd one or more fault conditions:
//  - more than one head selected
//  - read and write gates true at same time
//  - read and erase gates true at same time
//  - controller enabled only one of the write and erase gates
//  - low voltage
//  - selecting fixed heads on drive without fixed disk option
//  - Emergency retract
//
// Stays high until a RTZS operation.
static uint8_t hawk_fault;

// Unimplemented output signals from Hawk unit
// Some of them probally go in status.
//   Index (sector 0 pulse), Sector (one pulse per sector),
//   Address Interlock (bad address), Address Acknowledge (address accepted),
//   Wr State (Write Protect), Sector Address (upto 6 bits), Density.
//
// // See Page 25 of HAWK_9427_BP11_OCT80.pdf for details

// Busy
// From controller
// This is probally only set when it's doing (or waiting to do)
// a DMA operation. But the controller might also mix in the On Cylinder
// line while seeking.
static uint8_t hawk_dma_busy;

// Data Error
// This is just a stand in for error various read/write error conditions.
// Probally mostly coming from the controllers state machine. Sector not
// found, or CRC failed. Probally also includes the Fault line
static uint8_t hawk_data_error;


static int hawk_fd[NUM_HAWK_UNITS];

void hawk_init(void)
{
	int i;
	char name[32];

	for (i = 0; i < NUM_HAWK_UNITS; i++) {
		sprintf(name, "hawk%u.disk", i);
		/* We don't worry here is this works or not */
		hawk_fd[i] = open(name, O_RDWR|O_BINARY);
	}
}

static int hawk_get_fd(void)
{
	return hawk_unit < NUM_HAWK_UNITS ? hawk_fd[hawk_unit] : -1;
}

static uint16_t hawk_get_status() {
	// Not sure where data errors and fault goes,
	// lets just put it in all these remaining bits we suspect are errors
	uint8_t unk_error_bit = hawk_fault | hawk_data_error;

	return (0               << 0) // Potentially four bits of current sector?
	     | (0               << 1)
	     | (0               << 2)
	     | (0               << 3)
	     | (hawk_ready      << 4) // Probally the ready signal from drive
	     | (hawk_on_cyl     << 5) // Head is on the correct cylinder, line from drive
	     | (0               << 6)
	     | (0               << 7)
	     | (hawk_dma_busy   << 8) // Read/Write/Fmt command in progress
// WIPL ignores bit 9 after read, Bootstrap requires it to be zero after read
	     | (0               << 9) // either write error, or not an error
	     | (unk_error_bit  << 10)
	     | (unk_error_bit  << 11)
	     | (unk_error_bit  << 12)
	     | (unk_error_bit  << 13)
// Bootstrap loops forever unless either seek_error or on_track goes high
	     | (hawk_seek_error << 14) // Seek error line from drive
	     | (unk_error_bit  << 15);
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
static void hawk_position(void)
{
	unsigned sec = hawk_secl & 0x0F;
	unsigned head = !!(hawk_secl & 0x10);
	unsigned cyl = (hawk_sech << 3) | (hawk_secl >> 5);
	off_t offset = cyl;
	int fd = hawk_get_fd();

	offset *= 2;
	offset += head;
	offset *= 16;		/* According to Ken, 16 spt */
	offset += sec;
	offset *= 400;

	hawk_on_cyl = 0;
	if (fd != -1) {
		if (lseek(fd, offset, SEEK_SET) == -1) {
			fprintf(stderr, "hawk position failed (%d,%d,%d) = %lx.\n",
				cyl, head, sec, (long) offset);
			hawk_seek_error = 1;
		} else {
			// Successful seek
			hawk_on_cyl = 1;
		}
	} else {
		hawk_seek_error = 1;
	}
}

uint8_t hawk_read_next(void)
{
	int fd = hawk_get_fd();
	uint8_t c;

	if (fd == -1) {
		hawk_data_error = 1;
	}
	else if (read(fd, (void *)&c, 1) != 1) {
		fprintf(stderr, "hawk I/O error\n");
		hawk_data_error = 1;
	}
	return c;
}

void hawk_write_next(uint8_t c)
{
	int fd = hawk_get_fd();

	if (fd == -1) {
		hawk_data_error = 1;
	} else if (write(fd, (void *) &c, 1) != 1) {
		fprintf(stderr, "hawk I/O error\n");
		hawk_data_error = 1;
	}
}

void hawk_dma_done(void)
{
    hawk_set_dma(0);
	hawk_dma_busy = 0;

	// The Hawk disk unit will fault if a read/write is done off cylinder
	hawk_fault |= !hawk_on_cyl;
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
static void hawk_cmd(uint8_t cmd, unsigned trace)
{
	if (trace)
		fprintf(stderr, "%04X Hawk unit %02X command %02X\n", cpu6_pc(), hawk_unit, cmd);

	switch (cmd) {
	case 0:		/* Multi sector read  - 1 to 16 sectors */
		hawk_dma_busy = 1;
		hawk_set_dma(1);
		break;
	case 1:		/* Multi sector write - ditto */
		hawk_dma_busy = 1;
        hawk_set_dma(2);
		break;
	case 2:		/* Seek */
		// Guess. Seeks probally don't set busy
		hawk_position();
		break;
	case 3:		/* Return to Track Zero Sector (Recalibrate) */
		// Slams the heads into the rubber stops, and then seeks to the first track

		// According to documentation, The Hawk drive unit will clear any seek
		// errors and faults on RTZS
		hawk_seek_error = 0;
		hawk_fault = 0;

		// Guess, but chances are the controller also clears it's data errors on RTZS
		hawk_data_error = 0;

		// Guess. Seeks probally don't set busy

		hawk_position();
		break;
	case 4:		/* Format sector - Ken thinks but not sure */
	default:
		fprintf(stderr, "%04X: Unknown hawk command %02X\n",
			cpu6_pc(), cmd);
		hawk_dma_busy = 0;
		break;
	}
}

void hawk_write(uint16_t addr, uint8_t val, unsigned trace)
{
	switch (addr) {
	case 0xF140:
		hawk_unit = val;
		// On a real drive it's more complex. But for us, if
		// we have an image file for the unit, it's ready.
		hawk_ready = (hawk_get_fd() != -1);
		if (trace)
			fprintf(stderr, "Selected hawk unit %i\n", val);
		break;
	case 0xF141:
		hawk_sech = val;
		break;
	case 0xF142:
		hawk_secl = val;
		break;
		/* This seems to be a word. The code checks F144 bit 2 for
		   an error situation, and after the read F144 non zero for error */
	case 0xF144:
	case 0xF145:
		/* Guess.. it's done early in boot */
		hawk_data_error = 0;
		break;
	case 0xF148:
		hawk_cmd(val, trace);
		break;
	default:
		fprintf(stderr,
			"%04X: Unknown hawk I/O write %04X with %02X\n",
			cpu6_pc(), addr, val);
	}
}

uint8_t hawk_read(uint16_t addr, unsigned trace)
{
	uint8_t status;
	switch (addr) {
	case 0xF144:
		status = hawk_get_status() >> 8;
		if (trace)
			fprintf(stderr, "%04X: hawk status read high | %02x__\n", cpu6_pc(), status);
		return status;
	case 0xF145:
		status = hawk_get_status() & 0xff;
		if (trace)
			fprintf(stderr, "%04X: hawk status read low  | __%02x\n", cpu6_pc(), status);
		return status ;
	case 0xF148:		/* Bit 0 seems to be set while it is processing */
		return hawk_dma_busy;
	default:
		fprintf(stderr, "%04X: Unknown hawk I/O read %04X\n",
			cpu6_pc(), addr);
		return 0xFF;
	}
}

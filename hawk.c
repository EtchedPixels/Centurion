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

static uint8_t hawk_unit;
static uint8_t hawk_sech;
static uint8_t hawk_secl;
static uint8_t hawk_busy;
static uint8_t hawk_status;
static int hawk_fd;

void hawk_init(void)
{
       	/* We don't worry here is this works or not */
	hawk_fd = open("hawk.disk", O_RDWR|O_BINARY);
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

	offset *= 2;
	offset += head;
	offset *= 16;		/* According to Ken, 16 spt */
	offset += sec;
	offset *= 400;

	if (lseek(hawk_fd, offset, SEEK_SET) == -1) {
		fprintf(stderr, "hawk position failed (%d,%d,%d) = %lx.\n",
			cyl, head, sec, (long) offset);
		hawk_set_dma(0);
		hawk_busy = 0;
		hawk_status = 0xFE;
	}
}

uint8_t hawk_read_next(void)
{
	uint8_t c;
	if (read(hawk_fd, (void *) &c, 1) != 1) {
		hawk_status = 0xFE;	/* dunno but it does for error, completed */
		hawk_set_dma(0);
		hawk_busy = 0;
		fprintf(stderr, "hawk I/O error\n");
	}
	return c;
}

void hawk_write_next(uint8_t c)
{
	if (write(hawk_fd, (void *) &c, 1) != 1) {
		hawk_status = 0xFE;	/* dunno but it does for error, completed */
		hawk_set_dma(0);
		hawk_busy = 0;
		fprintf(stderr, "hawk I/O error\n");
	}
}

void hawk_dma_done(void)
{
	hawk_busy = 0;
	hawk_status = 0;
	hawk_set_dma(0);
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
static void hawk_cmd(uint8_t cmd)
{
	switch (cmd) {
	case 0:		/* Multi sector read  - 1 to 16 sectors */
		hawk_busy = 1;
		hawk_status = 1;	/* Busy */
                hawk_set_dma(1);
		hawk_position();
		break;
	case 1:		/* Multi sector write - ditto */
		hawk_busy = 1;
		hawk_status = 1;
                hawk_set_dma(2);
		hawk_position();
		break;
	case 2:		/* Seek */
		hawk_status = 0;
		hawk_busy = 0;
		break;
	case 3:		/* Restore */
		hawk_status = 0;
		hawk_busy = 0;
		break;
	case 4:		/* Format sector - Ken thinks but not sure */
	default:
		fprintf(stderr, "%04X: Unknown hawk command %02X\n",
			cpu6_pc(), cmd);
		hawk_busy = 0;
		hawk_status = 0xFE;
		break;
	}
}

void hawk_write(uint16_t addr, uint8_t val)
{
	switch (addr) {
	case 0xF140:
		hawk_unit = val;
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
		hawk_status = 0;
		break;
	case 0xF148:
		hawk_cmd(val);
		break;
	default:
		fprintf(stderr,
			"%04X: Unknown hawk I/O write %04X with %02X\n",
			cpu6_pc(), addr, val);
	}
}

uint8_t hawk_read(uint16_t addr)
{
	switch (addr) {
	case 0xF144:		/* Some kind of status - NZ is error */
		return hawk_status;
	case 0xF145:
		/* 0x10 is checked on the unit check, 0x20 is waited for
		   after the cmd 3. Presumably it's a seek complete or
		   similar */
		return 0x30;
	case 0xF148:		/* Bit 0 seems to be set while it is processing */
		return hawk_busy;
	default:
		fprintf(stderr, "%04X: Unknown hawk I/O read %04X\n",
			cpu6_pc(), addr);
		return 0xFF;
	}
}

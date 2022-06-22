/*
 *	A very minimal centurion system for testing
 */

#include <fcntl.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>

#include "centurion.h"
#include "console.h"
#include "cpu6.h"
#include "mux.h"

static unsigned finch;		/* Finch or original FDC */

volatile unsigned int emulator_done;

#define TRACE_MEM_RD	1
#define TRACE_MEM_WR	2
#define TRACE_MEM_REG	4
#define TRACE_CPU	8
#define TRACE_FDC	16
#define TRACE_CMD	32
#define TRACE_PARITY	64

unsigned int trace = 0;

unsigned int switches;

static unsigned diag = 0;

/* 18 bit address space it seems if the top mmu bit is not used. Allocate
   for the case it is for now */
static uint8_t mem[0x80000];
static uint8_t memclean[0x80000];

static uint8_t hexdigits;
static unsigned hexblank;
static unsigned hexdots[4];

static void hexdisplay(uint16_t addr, uint8_t val)
{
	const char *hexstr = "0123456789ABCDEF";
	uint8_t onoff = addr & 1;
	if (addr == 0xF110)
		hexdigits = val;
	else if (addr >= 0xF108) {
		addr -= 0xF108;
		addr >>= 1;
		hexdots[addr] = onoff;
	} else {
		hexblank = onoff;
	}
	if (hexblank) {
		printf("[OFF]\n");
		return;
	}
	printf("[");
	if (hexdots[0])
		printf("*");
	else
		printf(".");
	printf("%c", hexstr[hexdigits >> 4]);
	if (hexdots[1])
		printf("*");
	else
		printf(".");
	if (hexdots[2])
		printf("*");
	else
		printf(".");
	printf("%c", hexstr[hexdigits & 0x0F]);
	if (hexdots[3])
		printf("*");
	else
		printf(".");
	printf("]\n");
	fflush(stdout);
}

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
static unsigned hawk_dma;
static int hawk_fd;

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
		hawk_dma = 0;
		hawk_busy = 0;
		hawk_status = 0xFE;
	}
}

static uint8_t hawk_read_next(void)
{
	uint8_t c;
	if (read(hawk_fd, (void *) &c, 1) != 1) {
		hawk_status = 0xFE;	/* dunno but it does for error, completed */
		hawk_dma = 0;
		hawk_busy = 0;
		fprintf(stderr, "hawk I/O error\n");
	}
	return c;
}

static void hawk_write_next(uint8_t c)
{
	if (write(hawk_fd, (void *) &c, 1) != 1) {
		hawk_status = 0xFE;	/* dunno but it does for error, completed */
		hawk_dma = 0;
		hawk_busy = 0;
		fprintf(stderr, "hawk I/O error\n");
	}
}

static void hawk_dma_done(void)
{
	hawk_busy = 0;
	hawk_status = 0;
	hawk_dma = 0;
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
		hawk_dma = 1;
		hawk_status = 1;	/* Busy */
		hawk_position();
		break;
	case 1:		/* Multi sector write - ditto */
		hawk_busy = 1;
		hawk_dma = 2;
		hawk_status = 1;
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

static void hawk_write(uint16_t addr, uint8_t val)
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

static uint8_t hawk_read(uint16_t addr)
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

/*
 *	Floppy controller (or what we know of it)
 *
 *	"The CMD controller and Floppy Controller are AMD2901 based
 *	 controllers that move a command string into the controller under
 *	 DMA control  then the AMD2901 runs the command string then moves
 *	 the Read / Write data in or out of the controller under DMA control."
 *			-- Ken Romain
 *
 *	"The first floopy was 8 inch. All Centurion disk R/W are based on
 *	 400 ( 0x190 ) byte sectors to keep the old software from the Sykes
 *	 tape days still usable on the disk drive systems."
 *			-- Ken Romain
 *
 *	Write 43 to F800 to load a command
 *	Write 45 to enable data receive (needed even if no data)
 *	F801 is the in/out/busy bits, F800 the status where top bit = error
 *
 *	Commands
 *	81 01 82		Restore
 *	81 01 83 track		Seek
 *
 *	81 00 or 81 01		always starts
 *	82 XX			restore
 *	83 track		seek to track
 *	88 sec track len.w	read
 *
 *	Boot uses (seek to track 0 load a super long sector 0 ?)
 *	81 00 83 00 88 00 00 0F00
 *
 *	The hard disk controller seems close but not quite the same. The
 *	bootstrap uses something like
 *
 *	81 00 84 0F+unit 83 00 00 85 00 0190 01 0190 02 0190 ...
 */

#define ST_Fout		1
#define ST_Fin		2
#define ST_Busy		8

static uint8_t fd_buf[0x1000];
static unsigned fd_ptr;
static unsigned fd_dma;
static uint8_t fd_status;
static uint8_t fd_bits;

/* Assume ptr is a shared counter - but we don't actually know from
   what we have so far */

static void fdc_dma_in(uint8_t data)
{
	if (fd_ptr >= 0x1000) {
		fprintf(stderr, "%04X: overlong fdc data %02X\n", data,
			cpu6_pc());
		return;
	}
	fd_buf[fd_ptr++] = data;
}

static uint8_t fdc_dma_out(void)
{
	if (fd_ptr >= 0x1000) {
		fprintf(stderr, "%04X: overlong fdc command read\n",
			cpu6_pc());
		return 0xFF;
	}
	return fd_buf[fd_ptr++];
}

static void fdc_command_execute(uint8_t * p, int len)
{
	while (len-- > 0) {
		switch (*p++) {
		case 0x81:
			p++;	/* We don't know what this does */
			len--;
			break;
		case 0x82:
			fprintf(stderr, "restore.\n");
			break;
		case 0x83:
			fprintf(stderr, "seek %d\n", *p++);
			len--;
			break;
		case 0x84:
			fprintf(stderr, "set unit %d\n", *p++);
			len--;
			break;
		case 0x85:
			fprintf(stderr, "set head %d\n", *p++);
			len--;
		case 0x88:
			fprintf(stderr, "read %d,%d for %d bytes.\n",
				*p, p[1], p[2] << 8 | p[3]);
			p += 4;
			len -= 4;
			break;
		default:
			fprintf(stderr, "unknown command %02x\n", p[-1]);
			break;
		}
	}
}

/*
 *	The finch is a lot smarter
 *
 *	"A later rev. of the floppy controller (AMD2901 based) controlled
 *	 5-1/4 floppy (720KB) and CDC 5-1/4 Finch 24MB or 32MB hard drive,
 *	 letting us build a desktop Centurion Micro Plus system with CPU6,
 *	 128KB DRAM & 4 port MUX"
 *
 *	81 02 is now used (is this maybe a version check ?)
 *	82 is still restore
 *	83 takes two bytes and it's not clear what it seeks
 *	84 seems to be set unit
 *	85 possibly set head (as hard disk)
 *	8A is a table driven read, given tuples of sector, length
 *	   terminated FF
 *	FF is a terminator - in fact several test commands appear to DMA
 *			     over length blocks and rely on this.
 *
 */
static void finch_command_execute(uint8_t * p, int len)
{
	while (len-- > 0) {
		switch (*p++) {
		case 0x81:
			p++;	/* We don't know what this does */
			len--;
			break;
		case 0x82:
			fprintf(stderr, "restore.\n");
			break;
		case 0x83:
			fprintf(stderr, "seek %d\n", *p << 8 | p[1]);
			p += 2;
			len -= 2;
			break;
		case 0x84:
			fprintf(stderr, "set unit %d\n", *p++);
			len--;
			break;
		case 0x85:
			fprintf(stderr, "set head %d\n", *p++);
			len--;
		case 0x8A:
			/* Table driven read */
			while (*p != 0xFF) {
				fprintf(stderr, "read %d for %d bytes.\n",
					*p, (p[1] << 8) | p[2]);
				p += 3;
				len -= 3;
			}
			break;
		case 0xFF:	/* Seems to act as an end marker */
			break;
		default:
			fprintf(stderr, "unknown command %02x\n", p[-1]);
			break;
		}
	}
}

static void fdc_dma_in_done(void)
{
	unsigned i;
	if (fd_ptr > 0x0F00 && (trace & TRACE_FDC)) {
		fprintf(stderr, "fdcmd: %d\n\t", fd_ptr);
		for (i = 0x0F00; i < fd_ptr; i++) {
			fprintf(stderr, "%02X ", fd_buf[i]);
			if (((i & 15) == 15) && i != fd_ptr - 1)
				fprintf(stderr, "\n\t");
		}
		if (!finch)
			fdc_command_execute(fd_buf + 0x0F00,
					    fd_ptr - 0x0F01);
		else
			finch_command_execute(fd_buf + 0x0F00,
					      fd_ptr - 0x0F01);
		fprintf(stderr, "\n");
	}
	fd_bits = ST_Fout;
	fd_dma = 0;
	fd_status = 0;
}

static void fdc_dma_out_done(void)
{
	fd_bits = ST_Fout;
	fd_dma = 0;
}

static void fdc_write8(uint8_t data)
{
	if (trace & TRACE_FDC)
		fprintf(stderr, "fdc write %02X\n", data);
	switch (data) {
	case 0x00:		/* Mystery - reset state perhaps ? */
	case 0x01:		/* Used in the aux memory test */
	case 0x0F:		/* Used in the aux memory test */
		/* Status bits ?? */
		break;
	case 0x41:		/* used for reads */
	case 0x43:		/* used for seek etc */
		fd_bits = ST_Fin;	/* Fin not busy */
		fd_ptr = 0x0F00;
		fd_dma = 1;	/* Command in */
		fd_status = 0x80;	/*?? */
		break;
	case 0x44:		/* seems to be reading the command buffer back */
		fd_bits = ST_Busy | ST_Fout;	/* busy */
		fd_ptr = 0x0F00;
		fd_dma = 2;
		fd_status = 0x00;
		break;
	case 0x45:		/* data follow up */
		/* Should probably have ST_Busy set at this point ? */
		/* 1 or 2 ?? */
		fd_bits = ST_Fin | ST_Busy;	/* Should this be Fout or command based ? */
		fd_ptr = 0;
		fd_dma = 2;	/* Data out ? */
		fd_status = 0x00;	/* Seems to want top bit for error */
		/* Fake an error on track 5 */
		if (fd_buf[0x0F02] == 0x83 && fd_buf[0x0F03] == 0x05)
			fd_status = 0x80;
		break;
	case 0x46:		/* load data into aux memory */
		fd_bits = ST_Fin;
		fd_ptr = 0;
		fd_dma = 1;
		break;
	case 0x47:		/* retrieve data from aux memory */
		fd_bits = ST_Fout | ST_Busy;
		fd_ptr = 0;
		fd_dma = 2;
		break;
	default:
		fprintf(stderr, "%04X: unknown fdc cmd %02X.\n", cpu6_pc(),
			data);
		break;
	}
}

/*
 *	The CMD disk interface is remarkably similar but at F808
 *
 *	Commands this time are sometimes written with the pattern
 *
 *	Wait !busy
 *	41			}
 *	DMA command		} sometimes 43 cmd
 *	43
 *	wait
 *	check error
 *	45
 *	wait !busy
 *	error check
 *
 *	Commands observed
 *
 *	Seek test:
 *	81 00 84 10 83 XX XX FF
 *
 *	cycles down from 0336 to 0000, seems to use FFFF for park perhaps ?
 *
 *	Read test initial:
 *	81 00 84 00 83 00 00 85 00 01 90 01 01 90 02 01
 *	90 03 01 90 04 01 90 05 01 90 06 01 90 07 01 90
 *	08 01 90 09 01 90 0A 01 90 0B 01 90 0C 01 90 0D
 *	01 90 0E 01 90 0F 01 90 FF 00 00 00 00
 *
 *	The CMD with the Finch card follows the Finch pattern instead
 *
 *	81 00
 *	82
 *	83 seek (seems it might be little endian or some mix of track/head)
 *	84 set unit 0-F (Finch floppy uses 10-1F)
 *	85 sector length.w repeating until 0xFF
 *	(differs from the FD which seems to use 85 has head and 8A as
 *	the table driven read)
 *	FF terminator (FF 00 ?)
 */

static uint8_t cmdcmd[256];
static unsigned cmd_ptr;
static unsigned cmd_dma;
static uint8_t cmd_status;
static uint8_t cmd_bits;

static void cmd_dma_cmd_in(uint8_t data)
{
	if (cmd_ptr == 256) {
		fprintf(stderr, "%04X: overlong cmdc command %02X\n", data,
			cpu6_pc());
		return;
	}
	cmdcmd[cmd_ptr++] = data;
}

static uint8_t cmd_dma_cmd_out(void)
{
	if (cmd_ptr == 256) {
		fprintf(stderr, "%04X: overlong cmdc command read\n",
			cpu6_pc());
		return 0xFF;
	}
	return cmdcmd[cmd_ptr++];
}

static void cmd_dma_cmd_done(void)
{
	unsigned i;
	if (trace & TRACE_CMD) {
		fprintf(stderr, "cmdcmd: %d\n\t", cmd_ptr);
		for (i = 0; i < cmd_ptr; i++) {
			fprintf(stderr, "%02X ", cmdcmd[i]);
			if (((i & 15) == 15) && i != cmd_ptr - 1)
				fprintf(stderr, "\n\t");
		}
		fprintf(stderr, "\n");
	}
	cmd_bits = ST_Fout;	/* fin */
	cmd_dma = 0;
	cmd_status = 0;
	cmd_ptr = 0;
}

static void cmd_dma_cmd_out_done(void)
{
	cmd_bits = ST_Fout;	/* fin on */
	cmd_dma = 0;
}

/* Subtly different to the FDC or maybe the 41/43 divide is really the same
   but driven / observed differently */

static void cmd_write8(uint8_t data)
{
	if (trace & TRACE_CMD)
		fprintf(stderr, "cmd write %02X\n", data);
	switch (data) {
	case 0x00:		/* Mystery - reset state perhaps ? */
		cmd_bits = ST_Fout;	/* Fout not busy is expected */
		break;
	case 0x01:		/* Used in the aux memory test */
	case 0x0F:		/* Used in the aux memory test */
	case 0x41:		/* 43 41 45 is sometimes a pattern. */
		cmd_ptr = 0;
		break;
	case 0x43:		/* Run command ?? */
		cmd_bits = ST_Fin;	/* Fout not busy */
		cmd_ptr = 0;
		cmd_dma = 1;	/* Command in */
		cmd_status = 0x80;	/*?? */
		break;
	case 0x44:		/* seems to be reading the command buffer back */
		cmd_bits = ST_Busy | ST_Fout;	/* busy */
		cmd_ptr = 0;
		cmd_dma = 3;
		cmd_status = 0x00;
		break;
	case 0x45:		/* data follow up */
		cmd_bits = ST_Fout;	/* ?? suspect this depends on the command */
		cmd_ptr = 0;
		cmd_dma = 2;	/* Data out ? */
		cmd_status = 0x00;	/* Seems to want top bit for error */
		break;
	case 0x46:		/* load data into aux memory */
		fd_bits = ST_Fin;
		fd_ptr = 0;
		fd_dma = 1;
		break;
	case 0x47:		/* retrieve data from aux memory */
		fd_bits = ST_Fout | ST_Busy;
		fd_ptr = 0;
		fd_dma = 2;
		break;
	default:
		fprintf(stderr, "%04X: unknown cmd cmd %02X.\n", cpu6_pc(),
			data);
		break;
	}
}

static uint8_t io_read8(uint16_t addr)
{
	if (addr == 0xF800) {
		if (trace & TRACE_FDC)
			fprintf(stderr, "fd status %02X\n", fd_status);
		return fd_status;
	}
	if (addr == 0xF801) {
		if (trace & TRACE_FDC)
			fprintf(stderr, "fd bits %02X\n", fd_bits);
		return fd_bits;
	}
	if (addr == 0xF808) {
		if (trace & TRACE_CMD)
			fprintf(stderr, "cmd status %02X\n", cmd_status);
		return cmd_status;
	}
	if (addr == 0xF809) {
		if (trace & TRACE_CMD)
			fprintf(stderr, "cmd bits %02X\n", cmd_bits);
		return cmd_bits;
	}
	if (addr == 0xF110)
		return switches;
	if (addr >= 0xF140 && addr <= 0xF14F)
		return hawk_read(addr);
	if (addr >= 0xF200 && addr <= 0xF21F)
		return mux_read(addr);
	fprintf(stderr, "%04X: Unknown I/O read %04X\n", cpu6_pc(), addr);
	return 0;
}

static void io_write8(uint16_t addr, uint8_t val)
{
	if (addr == 0xF800) {
		fdc_write8(val);
		return;
	} else if (addr == 0xF808) {
		cmd_write8(val);
		return;
	} else if (addr >= 0xF106 && addr <= 0xF110) {
		hexdisplay(addr, val);
		return;
	} else if (addr >= 0xF140 && addr <= 0xF14F) {
		hawk_write(addr, val);
		return;
	} else if (addr >= 0xF200 && addr <= 0xF21F) {
		mux_write(addr, val);
		return;
	} else
		fprintf(stderr, "%04X: Unknown I/O write %04X %02X\n",
			cpu6_pc(), addr, val);
}

static uint32_t remap(uint32_t addr)
{
	addr &= 0x3FFFF;
	/* We need to fix up the fact the 1K diag RAM appear twice */
	if (diag && addr >= 0x0BC00 && addr <= 0x0BFFF)
		addr -= 0x400;
	return addr;
}

static uint8_t do_mem_read8(uint32_t addr, int dis)
{
	unsigned parity_off = 0;

	if (addr >= 0x3F000 && addr < 0x3FC00) {
		if (dis)
			return 0xFF;
		else
			return io_read8(addr & 0xFFFF);
	} else {
		if (diag && addr >= 0x8000)
			parity_off = 1;
		addr = remap(addr);
		if (addr >= 0x3F000)
			parity_off = 1;
		if (memclean[addr] || parity_off)
			return mem[addr];
		if (trace & TRACE_PARITY)
			fprintf(stderr, "PARITY\n");
		return mem[addr];
	}
}

uint8_t mem_read8(uint32_t addr)
{
	uint8_t r = do_mem_read8(addr, 0);
	if (trace & TRACE_MEM_RD)
		if (addr > 0xFF || (trace & TRACE_MEM_REG))
			fprintf(stderr, "%04X: %05X R %02X\n", cpu6_pc(),
				addr, r);
	return r;
}

uint8_t mem_read8_debug(uint32_t addr)
{
	return do_mem_read8(addr, 1);
}

void mem_write8(uint32_t addr, uint8_t val)
{
	if (trace & TRACE_MEM_WR)
		if (addr > 0xFF || (trace & TRACE_MEM_REG))
			fprintf(stderr, "%04X: %05X W %02X\n", cpu6_pc(),
				addr, val);
	if (addr >= 0x3F000 && addr < 0x3FC00) {
		io_write8(addr & 0xFFFF, val);
		return;
	}

	if (diag && addr >= 0x08000 && addr < 0x0B800) {
		fprintf(stderr, "Write to ROM at %04X\n", cpu6_pc());
		return;
	}
	if (addr >= 0x3FC00) {
		fprintf(stderr, "Write to ROM at %04X\n", cpu6_pc());
		return;
	}
	addr = remap(addr);
	memclean[addr] = 1;
	mem[addr] = val;
}

void halt_system(void)
{
	printf("System halted at %04X\n", cpu6_pc());
	emulator_done = 1;
}

static void load_rom(const char *name, uint32_t addr, uint16_t len)
{
	FILE *fp = fopen(name, "r");
	if (fp == NULL) {
		perror(name);
		exit(1);
	}
	if (fread(mem + addr, len, 1, fp) != 1) {
		fprintf(stderr, "%s: read error.\n", name);
		exit(1);
	}
	fclose(fp);
}

void usage(void)
{
	fprintf(stderr,
		"centurion [-l port] [-d] [-s switches] [-S diagswitches] [-t trace]\n");
	exit(1);
}

int main(int argc, char *argv[])
{
	int opt;
	unsigned port = 0;

	mux_init();

	while ((opt = getopt(argc, argv, "dFl:s:S:t:")) != -1) {
		switch (opt) {
		case 'd':
			diag = 1;
			break;
		case 'F':
			finch = 1;
			break;
		case 'l':
			port = atoi(optarg);
			break;
		case 's':
			/* CPU switches */
			cpu6_set_switches(atoi(optarg));
			break;
		case 'S':
			/* Diag switches */
			switches = atoi(optarg);
			break;
		case 't':
			trace = atoi(optarg);
			break;
		default:
			usage();
		}
	}
	if (optind < argc)
		usage();

	if (port == 0)
		tty_init();
	else
		net_init(port);
	load_rom("bootstrap_unscrambled.bin", 0x3FC00, 0x0200);
	if (diag) {
		load_rom("Diag_F1_Rev_1.0.BIN", 0x08000, 0x0800);
		load_rom("Diag_F2_Rev_1.0.BIN", 0x08800, 0x0800);
		load_rom("Diag_F3_Rev_1.0.BIN", 0x09000, 0x0800);
		load_rom("Diag_F4_1133CMD.BIN", 0x09800, 0x0800);
	}

	/* We don't worry here is this works or not */
	hawk_fd = open("hawk.disk", O_RDWR);

	cpu6_init();

	while (!emulator_done) {
		cpu6_execute_one(trace & TRACE_CPU);
		if (cpu6_halted())
			halt_system();
		/* Service DMA */
		if (hawk_dma == 1) {
			if (dma_read_cycle(hawk_read_next()))
				hawk_dma_done();
		}
		if (hawk_dma == 2) {
			if (dma_write_active())
				hawk_write_next(dma_write_cycle());
			else
				hawk_dma_done();
		}
		/* Floppy controller command host to controller */
		if (fd_dma == 1) {
			if (dma_write_active())
				fdc_dma_in(dma_write_cycle());
			else
				fdc_dma_in_done();
		}
		if (fd_dma == 2) {
			if (dma_read_cycle(fdc_dma_out()))
				fdc_dma_out_done();
		}
		if (cmd_dma == 1) {
			if (dma_write_active())
				cmd_dma_cmd_in(dma_write_cycle());
			else
				cmd_dma_cmd_done();
		}
		if (cmd_dma == 3) {
			if (dma_read_cycle(cmd_dma_cmd_out()))
				cmd_dma_cmd_out_done();
		}
		/* Update peripherals state */
		mux_poll();
	}
	return 0;
}

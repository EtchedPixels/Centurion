/*
 *	A very minimal centurion system for testing
 */

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <signal.h>
#include <termios.h>
#include <sys/select.h>
#include <unistd.h>
#include <fcntl.h>

#include "cpu6.h"

volatile unsigned int emulator_done;

#define TRACE_MEM	1
#define TRACE_MEM_REG	2
#define TRACE_CPU	4
#define TRACE_FDC	8
#define TRACE_CMD	16

unsigned int trace = 0;

unsigned int switches;

/* Utility functions for the mux */
static unsigned int check_chario(void)
{
	fd_set i, o;
	struct timeval tv;
	unsigned int r = 0;

	FD_ZERO(&i);
	FD_SET(0, &i);
	FD_ZERO(&o);
	FD_SET(1, &o);
	tv.tv_sec = 0;
	tv.tv_usec = 0;

	if (select(2, &i, NULL, NULL, &tv) == -1) {
		if (errno == EINTR)
			return 0;
		perror("select");
		exit(1);
	}
	if (FD_ISSET(0, &i))
		r |= 1;
	if (FD_ISSET(1, &o))
		r |= 2;
	return r;
}

static unsigned int next_char(void)
{
	char c;
	if (read(0, &c, 1) != 1) {
		fprintf(stderr, "(tty read without ready byte)\n");
		return 0xFF;
	}
	return c;
}

static uint8_t mem[65536];

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
 *	Each mux is a 6402 and what appears to be 3 bits of speed
 *	divider. The 6402 has 5 control lines
 *	PI	inhibit parity
 *	SBS	selects 1.5/2bit stop (v 1)
 *	CLS2/CLS1 set the length to 5 + their binary pair value
 *	EPE	set for even, clear for odd parity
 *
 *	We know C5 is 8N1 (and maybe sets the speed divider too)
 *	So presumably
 *	CLS2 CLS1 PI are set and SBS EPE clear. That implies that we've
 *	got 1 too many 1 bits so speed may be encoded. Possibly
 *	CLS2 CLS1 SBS EPE PI
 */

static uint8_t muxconf[16];


/* Bit 0 of control is char pending. The real system uses mark parity so
   we ignore that */
static void mux_write(uint16_t addr, uint8_t val)
{
	unsigned mux, data;
	addr &= 0xFF;
	mux = (addr >> 1) & 0x0F;
	data = addr & 1;

	if (!data) {
		muxconf[mux] = val;
		return;
	}

	if (mux != 0)
		return;

	val &= 0x7F;
	if (val != 0x0A && val != 0x0D
	    && (val < 0x20 || val == 0x7F))
		printf("[%02X]", val);
	else
		putchar(val);
	fflush(stdout);
}

static uint8_t mux_read(uint16_t addr)
{
	unsigned mux, data;
	unsigned ttystate;
	uint8_t ctrl = 0;

	addr &= 0xFF;
	mux = addr >> 1;
	data = addr & 1;


	if (mux != 0)
		return 0;

	if (data == 1)
		return next_char();

	ttystate = check_chario();
	if (ttystate & 1)
		ctrl |= 1;
	if (ttystate & 2)
		ctrl |= 2;
	return ctrl;
}

/*
 *	Hawk disk controller
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
	offset *= 16;
	offset += sec;
	offset *= 400;

	if (lseek(hawk_fd, offset, SEEK_SET) == -1) {
		fprintf(stderr, "hawk position failed (%d,%d,%d) = %lx.\n",
			cyl, head, sec, (long)offset);
		hawk_dma = 0;
		hawk_busy = 0;
		hawk_status = 0xFE;
	}
}

static uint8_t hawk_read_next(void)
{
	uint8_t c;
	if (read(hawk_fd, (void *)&c, 1) != 1) {
		hawk_status = 0xFE;	/* dunno but it does for error, completed */
		hawk_dma = 0;
		hawk_busy = 0;
		fprintf(stderr, "hawk I/O error\n");
	}
	return c;
}

static void hawk_write_next(uint8_t c)
{
	if (write(hawk_fd, (void *)&c, 1) != 1) {
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
 */
static void hawk_cmd(uint8_t cmd)
{
	switch(cmd) {
	case 0:	/* Multi sector read  - 1 to 16 sectors */
		hawk_busy = 1;
		hawk_dma = 1;
		hawk_status = 1;	/* Busy */
		hawk_position();
		break;
	case 1:	/* Multi sector write - ditto */
		hawk_busy = 1;
		hawk_dma = 2;
		hawk_status = 1;
		hawk_position();
		break;
	case 2:	/* Seek */
		hawk_status = 0;
		hawk_busy = 0;
		break;
	case 3:	/* Restore */
		hawk_status = 0;
		hawk_busy = 0;
		break;
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
	switch(addr) {
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
		fprintf(stderr, "%04X: Unknown hawk I/O write %04X with %02X\n",
			cpu6_pc(), addr, val);
	}
}

static uint8_t hawk_read(uint16_t addr)
{
	switch(addr) {
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

static uint8_t fdcmd[256];
static unsigned fd_ptr;
static unsigned fd_dma;
static uint8_t fd_status;
static uint8_t fd_bits;

static void fdc_dma_cmd_in(uint8_t data)
{
	if (fd_ptr == 256) {
		fprintf(stderr, "%04X: overlong fdc command %02X\n", data, cpu6_pc());
		return;
	}
	fdcmd[fd_ptr++] = data;
}

static uint8_t fdc_dma_cmd_out(void)
{
	if (fd_ptr == 256) {
		fprintf(stderr, "%04X: overlong fdc command read\n", cpu6_pc());
		return 0xFF;
	}
	return fdcmd[fd_ptr++];
}

static void fdc_dma_cmd_done(void)
{
	unsigned i;
	if (trace & TRACE_FDC) {
		fprintf(stderr, "fdcmd: %d\n\t", fd_ptr);
		for (i = 0; i < fd_ptr; i++) {
			fprintf(stderr, "%02X ", fdcmd[i]);
			if (((i & 15) == 15) && i != fd_ptr - 1)
				fprintf(stderr, "\n\t");
		}
		fprintf(stderr, "\n");
	}
	fd_bits = 1;	/* fin */
	fd_dma = 0;
	fd_status = 0;
}

static void fdc_dma_cmd_out_done(void)
{
	fd_bits = 2;	/* fout on */
	fd_dma = 0;
}

static void fdc_write8(uint8_t data)
{
	if (trace & TRACE_FDC)
		fprintf(stderr, "fdc write %02X\n", data);
	switch(data) {
	case 0x00:		/* Mystery - reset state perhaps ? */
		break;
	case 0x41:		/* used for reads */
	case 0x43:		/* used for seek etc */
		fd_bits = 2;	/* Fout not busy */
		fd_ptr = 0;
		fd_dma = 1;	/* Command in */
		fd_status = 0x80;	/*??*/
		break;
	case 0x44:		/* seems to be reading the command buffer back */
		fd_bits = 8;	/* busy */
		fd_ptr = 0;
		fd_dma = 3;
		fd_status = 0x00;
		break;
	case 0x45:		/* data follow up */
		fd_bits = 1;	/* ?? suspect this depends on the command */
		fd_ptr = 0;
		fd_dma = 2;	/* Data out ? */
		fd_status = 0x00;	/* Seems to want top bit for error */
		/* Fake an error on track 5 */
		if (fdcmd[2] == 0x83 && fdcmd[3] == 0x05)
			fd_status = 0x80;
		break;
	default:
		fprintf(stderr, "%04X: unknown fdc cmd %02X.\n", cpu6_pc(), data);
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
 */

static uint8_t cmdcmd[256];
static unsigned cmd_ptr;
static unsigned cmd_dma;
static uint8_t cmd_status;
static uint8_t cmd_bits;

static void cmd_dma_cmd_in(uint8_t data)
{
	if (cmd_ptr == 256) {
		fprintf(stderr, "%04X: overlong cmdc command %02X\n", data, cpu6_pc());
		return;
	}
	cmdcmd[cmd_ptr++] = data;
}

static uint8_t cmd_dma_cmd_out(void)
{
	if (cmd_ptr == 256) {
		fprintf(stderr, "%04X: overlong cmdc command read\n", cpu6_pc());
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
	cmd_bits = 1;	/* fin */
	cmd_dma = 0;
	cmd_status = 0;
	cmd_ptr = 0;
}

static void cmd_dma_cmd_out_done(void)
{
	cmd_bits = 2;	/* fout on */
	cmd_dma = 0;
}

/* Subtly different to the FDC or maybe the 41/43 divide is really the same
   but driven / observed differently */

static void cmd_write8(uint8_t data)
{
	if (trace & TRACE_CMD)
		fprintf(stderr, "cmd write %02X\n", data);
	switch(data) {
	case 0x00:		/* Mystery - reset state perhaps ? */
		cmd_bits = 1;	/* Fout not busy is expected */
		break;
	case 0x41:		/* 43 41 45 is sometimes a pattern. */
		cmd_ptr = 0;
		break;
	case 0x43:		/* Run command ?? */
		cmd_bits = 2;	/* Fout not busy */
		cmd_ptr = 0;
		cmd_dma = 1;	/* Command in */
		cmd_status = 0x80;	/*??*/
		break;
	case 0x44:		/* seems to be reading the command buffer back */
		cmd_bits = 8;	/* busy */
		cmd_ptr = 0;
		cmd_dma = 3;
		cmd_status = 0x00;
		break;
	case 0x45:		/* data follow up */
		cmd_bits = 1;	/* ?? suspect this depends on the command */
		cmd_ptr = 0;
		cmd_dma = 2;	/* Data out ? */
		cmd_status = 0x00;	/* Seems to want top bit for error */
		break;
	default:
		fprintf(stderr, "%04X: unknown cmd cmd %02X.\n", cpu6_pc(), data);
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

static uint16_t remap(uint16_t addr)
{
	/* We need to fix up the fact the 1K diag RAM appear twice */
	if (addr >= 0xBC00 && addr <= 0xBFFF)
		addr -= 0x400;
	return addr;
}

static uint8_t do_mem_read8(uint16_t addr, int dis)
{
	if (addr >= 0xF000 && addr < 0xFC00) {
		if (dis)
			return 0xFF;
		else
			return io_read8(addr);
	} else {
		addr = remap(addr);
		return mem[addr];
	}
}

uint8_t mem_read8(uint16_t addr)
{
	uint8_t r = do_mem_read8(addr, 0);
	if (trace & TRACE_MEM)
		if (addr > 0xFF || (trace & TRACE_MEM_REG))
			fprintf(stderr, "%04X: %04X R %02X\n", cpu6_pc(),
				addr, r);
	return r;
}

void mem_write8(uint16_t addr, uint8_t val)
{
	if (trace & TRACE_MEM)
		if (addr > 0xFF || (trace & TRACE_MEM_REG))
			fprintf(stderr, "%04X: %04X W %02X\n", cpu6_pc(),
				addr, val);
	if (addr >= 0xF000 && addr <= 0xFC00) {
		io_write8(addr, val);
		return;
	}

	if (addr >= 0x8000 && addr < 0xB800) {
		fprintf(stderr, "Write to ROM at %04X\n", cpu6_pc());
		return;
	}
	if (addr >= 0xFC00) {
		fprintf(stderr, "Write to ROM at %04X\n", cpu6_pc());
		return;
	}
	addr = remap(addr);
	mem[addr] = val;
}

static struct termios saved_term, term;

static void cleanup(int sig)
{
	tcsetattr(0, TCSADRAIN, &saved_term);
	emulator_done = 1;
}

static void exit_cleanup(void)
{
	tcsetattr(0, TCSADRAIN, &saved_term);
}

void halt_system(void)
{
	printf("System halted at %04X\n", cpu6_pc());
	emulator_done = 1;
}

static void load_rom(const char *name, uint16_t addr, uint16_t len)
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
	fprintf(stderr, "centurion [-s diagswitches] [-d debug]\n");
	exit(1);
}

int main(int argc, char *argv[])
{
	int opt;
	while ((opt = getopt(argc, argv, "d:s:S:")) != -1) {
		switch (opt) {
		case 'd':
			trace = atoi(optarg);
			break;
		case 's':
			/* CPU switches */
			cpu6_set_switches(atoi(optarg));
			break;
		case 'S':
			/* Diag switches */
			switches = atoi(optarg);
			break;
		default:
			usage();
		}
	}
	if (optind < argc)
		usage();

	if (tcgetattr(0, &term) == 0) {
		saved_term = term;
		atexit(exit_cleanup);
		signal(SIGINT, cleanup);
		signal(SIGQUIT, cleanup);
		signal(SIGPIPE, cleanup);
		term.c_lflag &= ~(ICANON | ECHO);
		term.c_cc[VMIN] = 0;
		term.c_cc[VTIME] = 1;
		term.c_cc[VINTR] = 0;
		term.c_cc[VSUSP] = 0;
		term.c_cc[VSTOP] = 0;
		tcsetattr(0, TCSADRAIN, &term);
	}
	load_rom("bootstrap_unscrambled.bin", 0xFC00, 0x0200);
	load_rom("Diag_F1_Rev_1.0.BIN", 0x8000, 0x0800);
	load_rom("Diag_F2_Rev_1.0.BIN", 0x8800, 0x0800);
	load_rom("Diag_F3_Rev_1.0.BIN", 0x9000, 0x0800);
	load_rom("Diag_F4_1133CMD.BIN", 0x9800, 0x0800);

	/* We don't worry here is this works or not */
	hawk_fd = open("hawk.disk", O_RDWR);

	while (!emulator_done) {
		cpu6_execute_one(trace & TRACE_CPU);
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
				fdc_dma_cmd_in(dma_write_cycle());
			else
				fdc_dma_cmd_done();
		}
		if (fd_dma == 3) {
			if (dma_read_cycle(fdc_dma_cmd_out()))
				fdc_dma_cmd_out_done();
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
		usleep(1);
	}
	return 0;
}

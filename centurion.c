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
 *	3:		Restore
 */

static uint8_t hawk_unit;
static uint8_t hawk_cyl;
static uint8_t hawk_sech;
static uint8_t hawk_busy;
static uint8_t hawk_status;
static unsigned hawk_dma;
static int hawk_fd;

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

static void hawk_dma_done(void)
{
	hawk_busy = 0;
	hawk_status = 0;
	hawk_dma = 0;
}

static void hawk_cmd(uint8_t cmd)
{
	switch(cmd) {
	case 0:	/* Multi sector read */
		hawk_busy = 1;
		hawk_dma = 1;
		hawk_status = 1;	/* Busy */
		break;
	case 3:
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
		hawk_cyl = val;
		break;
	case 0xF142:
		hawk_sech = val;
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

static uint8_t io_read8(uint16_t addr)
{
	if (addr == 0xF110)
		return switches;
	if (addr >= 0xF140 && addr <= 0xF14F)
		return hawk_read(addr);
	if (addr >= 0xF200 && addr <= 0xF21F)
		return mux_read(addr);
	return 0;
}

static void io_write8(uint16_t addr, uint8_t val)
{
	if (addr >= 0xF106 && addr <= 0xF110) {
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
		if (hawk_dma) {
			if (dma_read_cycle(hawk_read_next()))
				hawk_dma_done();
		}
	}
	return 0;
}

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
#include "dma.h"
#include "dsk.h"
#include "mux.h"
#include "cbin_load.h"
#include "scheduler.h"

static unsigned finch;		/* Finch or original FDC */

volatile unsigned int emulator_done;
static int64_t cpu_timestamp_ns = 0;

#define TRACE_MEM_RD	1
#define TRACE_MEM_WR	2
#define TRACE_MEM_REG	4
#define TRACE_CPU	8
#define TRACE_FDC	16
#define TRACE_CMD	32
#define TRACE_PARITY	64
#define TRACE_MUX	128
#define TRACE_DSK       256
#define TRACE_SCHEDULER 512

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

/* A crappy glue, remained from the old monolythic code, still sufficient to work.
 * A proper DMA API needs to be implemented i believe */
static unsigned hawk_dma;

void hawk_set_dma(unsigned mode)
{
	hawk_dma = mode;
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
		return dsk_read(addr, trace & TRACE_DSK);
	if (addr >= 0xF200 && addr <= 0xF21F)
		return mux_read(addr, trace & TRACE_MUX);
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
		dsk_write(addr, val, trace & TRACE_DSK);
		return;
	} else if (addr >= 0xF200 && addr <= 0xF21F) {
		mux_write(addr, val, trace & TRACE_MUX);
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

static uint8_t do_mem_read8(uint32_t addr, int debug)
{
	unsigned parity_off = 0;

	if (addr >= 0x3F000 && addr < 0x3FC00) {
		if (debug)
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
	// Extremely simple timing model where we assume each CPU memory
	// access takes exactly 3 cycles (600ns), and the microcode is
	// never doing things between memory accesses.
	cpu_timestamp_ns += 600;

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

uint16_t mem_read16_debug(uint32_t addr)
{
	return (do_mem_read8(addr, 1) << 8) | do_mem_read8(addr+1, 1);
}

static void mem_do_write8(uint32_t addr, uint8_t val)
{
	addr = remap(addr);
	memclean[addr] = 1;
	mem[addr] = val;
}

void mem_write8(uint32_t addr, uint8_t val)
{
	if (diag && addr >= 0x08000 && addr < 0x0B800) {
		fprintf(stderr, "%04X: Write to ROM [%05X]\n", cpu6_pc(), addr);
		return;
	}
	if (addr >= 0x3FC00) {
		fprintf(stderr, "%04X: Write to ROM [%05X]\n", cpu6_pc(), addr);
		return;
	}
	if (trace & TRACE_MEM_WR)
		if (addr > 0xFF || (trace & TRACE_MEM_REG))
			fprintf(stderr, "%04X: %05X W %02X\n", cpu6_pc(),
				addr, val);
	if (addr >= 0x3F000 && addr < 0x3FC00) {
		io_write8(addr & 0xFFFF, val);
		return;
	}
	mem_do_write8(addr, val);
}

void mem_write8_debug(uint32_t addr, uint8_t val)
{
	// a debugger is allowed to modify rom, but not IO
	if (addr >= 0x3F000 && addr < 0x3FC00) {
		return;
	}
	mem_do_write8(addr, val);
}

void mem_write16_debug(uint32_t addr, uint16_t val)
{
	mem_write8_debug(addr, val >> 8);
	mem_write8_debug(addr+1, val & 0xff);
}

int64_t get_current_time() {
	return cpu_timestamp_ns;
}

void advance_time(uint64_t nanoseconds) {
	cpu_timestamp_ns += nanoseconds;
}

void halt_system(void)
{
	printf("System halted at %04X\n", cpu6_pc());
	emulator_done = 1;
}

static void load_rom(const char *name, uint32_t addr, uint16_t len)
{
	FILE *fp = fopen(name, "rb");
	if (fp == NULL) {
		perror(name);
		exit(1);
	}
	if (len == 0) {
		fseek(fp, 0L, SEEK_END);
		len = ftell(fp);
		rewind(fp);
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
		"centurion [options] [bootfile]\n"
		"\n"
		"When supplied, bootfile will be loaded as centurion binary (default) OR raw binary\n"
		"\n"
		"Options:\n"
		" -b           bootfile is raw binary\n"
		" -A <addr>    bootfile will be loaded at offset <addr>\n"
		" -E <addr>    entry point for binary"
		" -d           emulate DIAG card\n"
		" -F           emulate a finch drive\n"
		" -l <port>    Listen for telnet on the given <port> number\n"
		" -s <value>   set CPU switches as a decimal value. Switch 1-4 are Sense\n"
		" -S <value>   set diag switches as decimal value (only effective with `-d`)\n"
		" -t <value>   enable enable system trace to stderr. See readme for values\n"
		" -T <value>   Exit after executing <value> instructions\n"
	);
	exit(1);
}

uint16_t parse_address(char *arg, char* arg_name) {
	char* end_ptr = NULL;

	uint16_t load_addr = strtoul(arg, &end_ptr, 16);
	if (load_addr < 0x100 || end_ptr == NULL) {
		fprintf(stderr, "%s address not valid\n", arg_name);
		exit(1);
	}
	return load_addr;
}

int main(int argc, char *argv[])
{
	int opt;
	unsigned binary = 0;
	unsigned port = 0;
	long long terminate_at = 0;
	long long instruction_count = 0;
	uint16_t load_addr = 0;
	uint16_t entry_addr = 0;
	char* boot_file = NULL;

	mux_init();

	while ((opt = getopt(argc, argv, "b::A:E:dFl:s:S:t:T:")) != -1) {
		switch (opt) {
		case 'b':
			binary = 1;
			break;
		case 'A':
			load_addr = parse_address(optarg, "Load");
			break;
		case 'E':
			entry_addr = parse_address(optarg, "Entry");
			break;
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
		case 'T':
			terminate_at = atol(optarg);
			break;
		default:
			usage();
		}
	}
	if (optind < argc)
		boot_file = argv[optind++];

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

	dsk_init();
	cpu6_init();

	if (boot_file != NULL) {
		if (binary) {
			if (load_addr == 0) {
				fprintf(stderr, "raw binary needs a load address\n");
				exit(1);
			}
			load_rom(boot_file, load_addr, 0);
			if (entry_addr == 0) {
				// by default, enter at first byte of binary
				entry_addr = load_addr;
			}
			printf("Raw Binary %s loaded to %04hx; entry at %04hx\n\n",
				boot_file, load_addr, entry_addr);
		} else {
			entry_addr = cbin_load(boot_file, load_addr);
		}
	}

	if (entry_addr != 0) {
		set_pc_debug(entry_addr);

		if (binary) {
			// Standard launch args from bootstrap ROM
			regpair_write_debug(S, 0x1000);   // Stack
		} else {
			// Standard launch args from WIPL:
			regpair_write_debug(S, 0xEA35);   // Stack
			regpair_write_debug(Z, 0);        // Disk Number
			regpair_write_debug(A, 0x00C5);   // AL= mux0 config?
		}
	}

	throttle_init();
	throttle_set_speed(1.0);

	while (!emulator_done) {
		cpu6_execute_one(trace & TRACE_CPU);
		if (cpu6_halted())
			halt_system();
		/* Service DMA */
		if (hawk_dma) {
			while(dma_write_active()) {
				// Advance time to next scheduler event
				int64_t next = scheduler_next();
				if (next == -1) {
					fprintf(stderr, "DMA stalled\n");
					exit(-1);
				}
				if (next > cpu_timestamp_ns)
					cpu_timestamp_ns = next;
				run_scheduler(cpu_timestamp_ns, trace & TRACE_SCHEDULER);
			}
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
		mux_poll(trace & TRACE_MUX);

		run_scheduler(cpu_timestamp_ns, trace & TRACE_SCHEDULER);
		throttle_emulation(cpu_timestamp_ns);

		instruction_count++;
		if (terminate_at && instruction_count >= terminate_at) {
			printf("\nTerminated after %lli instructions\n", instruction_count);
			if (trace)
				fprintf(stderr, "Terminated after %lli instructions\n", instruction_count);
			break;
		}
	}
	return 0;
}

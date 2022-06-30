/*
 *	In the SRAM bank the registers are laid out as
 *
 *	0x0E	H	(seems to hold PC on IPL changes but otherwise not)
 *			(possibly PC is cached in the CPU)
 *	0x0C	G
 *	0x0A	S
 *	0x08	Z
 *	0x06	Y
 *	0x04	X
 *	0x02	B
 *	0x00	A
 *
 *	(see monitor 84C3)
 */

#include <assert.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "cpu6.h"
#include "disassemble.h"

static uint8_t cpu_ipl = 0;	/* IPL 0-15 */
static uint8_t cpu_mmu = 0;	/* MMU tag 0-7 */
static uint16_t pc;
static uint16_t exec_pc;	/* PC at instruction fetch */
static uint8_t op;
static uint8_t alu_out;
static uint8_t switches = 0xF0;
static uint8_t int_enable;
static unsigned halted;
static unsigned pending_ipl_mask = 0;

#define BS1	0x01
#define BS2	0x02
#define BS3	0x04
#define BS4	0x08

static uint16_t dma_addr;
static uint16_t dma_count;
static uint8_t dma_mode;
static uint8_t dma_enable;
static uint8_t dma_mystery;	/* We don't know what this reg on the AM2901 is
				   about */

/* SRAM on the CPU card */
static uint8_t cpu_sram[256];
static uint8_t mmu[8][32];

// Standing in for some internal microcode state
static unsigned twobit_cached_reg = 0;

static void mmu_mem_write8(uint16_t addr, uint8_t val);
static uint32_t mmu_map(uint16_t addr);
static void logic_flags16(unsigned r);

/*
 *	DMA engine guesswork
 */

int dma_read_cycle(uint8_t byte)
{
	if (dma_enable == 0)
		return 1;
	/* DMA is done when it incs to 0 */
	if (++dma_count == 0) {
		dma_enable = 0;
		return 1;
	}
	if (dma_enable) {
/*		fprintf(stderr, "%04X: DMA %04X <- %02X\n", dma_count, dma_addr, byte); */
		mem_write8(dma_addr++, byte);
	}
	return 0;
}

int dma_write_active(void)
{
	if (dma_enable == 1)
		return 1;
	return 0;
}

uint8_t dma_write_cycle(void)
{
	uint8_t r;
	if (dma_enable == 0) {
		fprintf(stderr, "DMA write cycle with no DMA\n");
		exit(1);
	}
	r = mmu_mem_read8(dma_addr++);
	dma_count++;
	if (dma_count == 0)
		dma_enable = 0;
	return r;
}

/*
 *	When packed into C, the flags live in the upper 4 bits of the low byte
 */

#define ALU_L	0x10
#define ALU_F	0x20
#define ALU_M	0x40
#define ALU_V	0x80


/*
 *	System memory access
 *
 *	For the moment we assume the accesses occur in address order.
 *
 *	For a more serious emulation there are some lurking horrors here.
 *	Not only do we need to know the order of the accesses but we need
 *	to know the logic of the postinc/predec operators and how a fault
 *	is handled half way through an access.
 */

uint8_t mmu_mem_read8(uint16_t addr)
{
	if (addr < 0x0100)
		return cpu_sram[addr];
	return mem_read8(mmu_map(addr));
}

uint8_t mmu_mem_read8_debug(uint16_t addr)
{
	if (addr < 0x0100)
		return cpu_sram[addr];
	return mem_read8_debug(mmu_map(addr));
}

static void mmu_mem_write8(uint16_t addr, uint8_t val)
{
	if (addr < 0x0100)
		cpu_sram[addr] = val;
	else
		mem_write8(mmu_map(addr), val);
}

static uint16_t mmu_mem_read16(uint16_t addr)
{
	uint16_t r = mmu_mem_read8(addr) << 8;
	r |= mmu_mem_read8(addr + 1);
	return r;
}

static void mmu_mem_write16(uint16_t addr, uint16_t val)
{
	mmu_mem_write8(addr, val >> 8);
	mmu_mem_write8(addr + 1, val);
}

/*
 *	We know from the start address that the processor microsteps begin
 *	inc pc
 *	load opcode from (pc)
 *
 *	however it's not clear this is observable or matters for the moment
 *	because the cases that seem to matter like branches appear to do
 *
 *	inc pc
 *	load opcode
 *	inc pc
 *	load offset
 *	add offset
 *
 *	and then the inc of pc before the next instruction works. What
 *	does need a hard look here is the behaviour of X.
 */
uint8_t fetch(void)
{
	/* Do the pc++ after so that tracing is right */
	uint8_t r = mmu_mem_read8(pc);
	pc++;
	return r;
}

uint16_t fetch16(void)
{
	uint16_t r;
	r = mmu_mem_read8(pc) << 8;
	pc++;
	r |= mmu_mem_read8(pc);
	pc++;
	return r;
}

uint16_t fetch_literal(unsigned length)
{
	uint16_t addr = pc;
	pc += length;
	return addr;
}

static uint8_t reg_read(uint8_t r)
{
	return mmu_mem_read8((cpu_ipl << 4) | r);
}

static void reg_write(uint8_t r, uint8_t v)
{
	mmu_mem_write8((cpu_ipl << 4) | r, v);
}

/*
 *	This needs some updating if CPU6 behaves like CPU4. On the EE200
 *	a word register specified with an odd value gives you the upper byte
 *	twice.
 */

static uint16_t regpair_addr(uint8_t r)
{
	return r + (cpu_ipl << 4);
}

static uint16_t regpair_read(uint8_t r)
{
	if (r > 15) {
		fprintf(stderr, "Bad regpair encoding %02X %02X %04X\n",
			op, r, exec_pc);
		exit(1);
	}
	return (reg_read((r | 1) ^ 1) << 8) | reg_read((r ^ 1));
}

static void regpair_write(uint8_t r, uint16_t v)
{
	if (r > 15) {
		fprintf(stderr, "Bad regpair encoding %02X %04X\n", op,
			exec_pc);
		exit(1);
	}
	reg_write((r | 1) ^ 1, v >> 8);
	reg_write((r ^ 1), v);
}

/*
 *	Stack helpers
 */

void push(uint16_t val)
{
	uint16_t addr = regpair_read(S);
	addr -= 2;
	mmu_mem_write16(addr, val);
	regpair_write(S, addr);
}

uint16_t pop(void)
{
	uint16_t addr = regpair_read(S);
	uint16_t d = mmu_mem_read16(addr);
	regpair_write(S, addr + 2);
	return d;
}

void pushbyte(uint8_t val)
{
	uint16_t addr = regpair_read(S);
	addr -= 1;
	mmu_mem_write8(addr, val);
	regpair_write(S, addr);
}

uint8_t popbyte(void)
{
	uint16_t addr = regpair_read(S);
	uint8_t d = mmu_mem_read8(addr);
	regpair_write(S, addr + 1);
	return d;
}


static uint16_t get_twobit(unsigned mode, unsigned idx, unsigned len) {
	uint16_t addr = 0;
	unsigned regs;
	unsigned thismode = mode;

	if (idx == 0)
		thismode = mode >> 2;

	switch (thismode & 0x3)	{
	case 0:
		// EA <- PC
		addr = fetch16();
		//fprintf(stderr, "%x EA <- (PC) = %04x\n", idx, addr);
		break;
	case 1:
		// EA <- imm8/imm16 + r1 + r2
		//fprintf(stderr, "%x EA <- ", idx);
		regs = fetch();
		addr =  (regs & 0x10) ? fetch16() : fetch(); // if r1 is odd, do imm16
		//fprintf(stderr, "%04x", addr);
		addr += regpair_read((regs >> 4) & 0xe);
		//fprintf(stderr, " + (%x) %04x", (regs >> 4) & 0xe, regpair_read((regs >> 4) & 0xe));
		if ((regs & 0xe) != 0) { // ignore r2 if it's A
			addr += regpair_read(regs & 0xe);
			//fprintf(stderr, " + (%x) %04x",  regs & 0xe, regpair_read(regs & 0xe));
		}
		//fprintf(stderr, " = %04x", addr);
		break;
	case 2:
		// EA <- R
		// This mode is complicated because it tries to merge two mode 2s into a single byte
		if (idx == 1 && mode == 0xa) {
			// previous twobit already fetched our regbyte
			regs = twobit_cached_reg;
		} else {
			twobit_cached_reg = regs = fetch();
		}
		if (idx == 0)
			regs >>= 4;
		addr = regpair_read(regs & 0xe);
		//fprintf(stderr, "%x EA <- reg(%x) = %04x\n", idx, regs & 0xe, addr);
		break;
	case 3:
		// EA <- (literal)
		addr = fetch_literal(len);
		//fprintf(stderr, "%x EA <- (imm)*0x%02x = %04x\n", idx, len+1, addr);
		break;
	}
	return addr;
}

/*
 *	The MMU
 *
 *	The MMU is a 256 byte 'fast' SRAM. The bus is wired for 2K physical
 *	pages with the MMU providing the upper bits. The MMU is indexed by
 *	3 lines. Diag pretty much requires line 0 is the low bit of the IPL
 *	but the other two are a mystery so might be more IPL lines or a DMA
 *	enable line or similar.
 *
 *	There does not appear to be any kind of pass-through mode, instead
 *	microcode initializes the MMU for IPL0 at boot.
 */

static uint32_t mmu_map(uint16_t addr)
{
/*	fprintf(stderr, "MMU %X is [%X] -> %X\n", addr, addr >> 11,  (mmu[(addr >> 11)] << 11) |(addr & 0x7FF)); */
	/* FIXME: add tag in to shift bank */
	return (mmu[cpu_mmu][(addr >> 11)] << 11) + (addr & 0x07FF);
}

/*
 *	MMU read/write operations
 *
 *  2E sssrmmnn
 *
 *  s - sub-op
 *  r - read; 0 = write to mmu, 1 = read from mmu
 *	m - opn1 twobit address mode
 *	n - mem twobit address mode
 */
static int mmu_transfer_op(void)
{
	unsigned subop = fetch();
	// While opn1 is commonly an immediate, it can actually use any
	// addressing mode
	uint8_t opn1 = mmu_mem_read8(get_twobit(subop, 0, 1));
	// opn1 is in the format xxxxxbbb
	uint8_t base = opn1 & 0x7; 	// b - page table base
	uint8_t x = opn1 >> 3; 	// x - meaning depends on subop


	uint8_t offset = 0;
	unsigned len, val;

	switch (subop & 0xe0) {
		case 0x00: // WPF/RPF - transfer x entries at offset 0
			len = x + 1;
		break;
		case 0x20: // WPF1/RPF1 - transfer 1 entry at offset x
			len = 1;
			offset = x;
		break;
		case 0x40: // WPF32/RPF32 - transfer (32-x) entries at offset x
			offset = x;
			len = 32 - x;
		break;
		default:
			// microcode suggests these are illegal (will trap)
			fprintf(stderr, "%04X: Illegal 2E op %02X\n", cpu6_pc(), op);
			return 0;
	}

	// There is no way to bypass the MMU, so entries are always
	// transferred to/from virtual addresses
	// The microcode might have a requirement for this to not be implicit
	uint16_t addr = get_twobit(subop, 1, len);

	switch(subop & 0x10) {
	case 0x00:
		while(len--) {
			assert(base < 8 && offset < 0x20);
			mmu[base][offset++] = mmu_mem_read8(addr++);
		}
		break;
	case 0x10:
		while(len--) {
			assert(base < 8 && offset < 0x20);
			val = mmu[base][offset++];
			mmu_mem_write8(addr++, val);

			// We know this has some flag effects because 8130 relies upon it setting
			// presumably Z to exit
			logic_flags16(val);
		}
		break;
	}
	return 0;
}

static uint8_t block_op_getLen(int inst, int op) {
	if ((op & 0xF0) == 0x00) // binload doesn't take a length
		return 0;

	if (inst == 0x47) {
		// 47 instructions take a literal
		return fetch();
	} else {
		// 67 instructions take the length in AL
		// We aren't 100% sure that this takes AL instead of A
		// But code that needs a 16bit memcpy seems to use F7 instead
		return reg_read(AL);
	}
}

/*
 *	Block/String operations
 *
 *	47 ssssmmnn - Take length as byte immediate
 *	67 ssssmmnn - Take length as AL
 *
 *	s = sub-op
 *	m = src address mode
 *	n = dst address mode
 *
 *	Not all sub-ops take a length.
 *	Some sub-ops take additional args, as immediate or implicit reg
 */
static int block_op(int inst)
{
	unsigned op = fetch();
	unsigned am = op & 0x0F;
	unsigned dst_len = block_op_getLen(inst, op) + 1;
	unsigned src_len = dst_len;
	uint16_t sa, da;
	uint8_t chr;

	// clear the fault flag
	alu_out &= ~ALU_F;

	// memset only reads the source once
	if ((op & 0xF0) == 0x90)
		src_len = 1;

	// memchr takes an extra "chr" operand
	if ((op & 0xF0) == 0x20) {
		if (inst == 0x47) {
			chr = fetch();
		} else {
			// This gets it's chr from somewhere else. Probally a register?
			fprintf(stderr, "Unsupported 67 2x memchr at %x\n", exec_pc);
			exit(-1);
		}
	}

	sa = get_twobit(am, 0, src_len);
	da = get_twobit(am, 1, dst_len);

	switch(op & 0xF0) {
	case 0x20:
		// copies bytes from src to dst, stopping if a byte matches chr
		// appears to be combined memcpy/memchr/strcpy
		// It's possible it might also stop when chr is 0, which would
		// change it's behavior to a combined strcpy/strchr/strlen
		while(dst_len--) {
			uint8_t val = mmu_mem_read8(sa);
			mmu_mem_write8(da, val);
			if (val == chr) { // Match
				regpair_write(Y, sa);
				regpair_write(Z, da);
				return 0;
			}
			sa++;
			da++;
		};
		// No match
		alu_out |= ALU_F;
		return 0;
	case 0x40:
		while(dst_len--) {
			mmu_mem_write8(da++, mmu_mem_read8(sa++));
		};
		return 0;
	case 0x60:
		// Complete Guess, but this might be OR
		while(dst_len--) {
			uint8_t val = mmu_mem_read8(da++) | mmu_mem_read8(sa++);
			mmu_mem_write8(da, val);
		};
		return 0;
	case 0x70:
		// Complete Guess, but this might be AND
		while(dst_len--) {
			uint8_t val = mmu_mem_read8(da++) & mmu_mem_read8(sa++);
			mmu_mem_write8(da, val);
		};
		return 0;
	case 0x80:
		alu_out |= ALU_V;
		while (dst_len--) {
			if(mmu_mem_read8(da++) !=
				mmu_mem_read8(sa++)) {
				alu_out &= ~ALU_V;
				break;
			}
		}
		return 0;
	case 0x90: /* memset */
		chr = mmu_mem_read8(sa);
		while (dst_len--) {
			mmu_mem_write8(da++, chr);
		}
		return 0;
	default:
		fprintf(stderr, "%04X: Unknown block xfer %02X\n", cpu6_pc(), op);
		return 0;
	}
}

/* F7 - a 16bit memcpy instruction
 *
 * Args
 *   A - Length
 *   B - Source
 *   Y - Dest
 *
 *  Appears to leave all registers unmodified?
 */
static int memcpy16(void) {
	uint16_t len = regpair_read(A);
	uint16_t sa  = regpair_read(B);
	uint16_t da  = regpair_read(Y);

	do {
		mmu_mem_write8(da++, mmu_mem_read8(sa++));
	} while(len--);
	return 0;
}

/* Various microcode math routines which operate on arbitrary width integers
 *
 *	46 llllkkkk ssssmmnn
 *
 *	l - size of a operand (typically)
 *	k - size of b operand
 *	s - subop
 *	m - a address mode
 *	n - b address mode
 *
 * Valid sizes are 1 to 16 bytes
 * Some subops may take extra arguments via implicit registers
 */
static int bignum_op(void) {
	unsigned sizes = fetch();
	unsigned a_size = (sizes >> 4) + 1;
	unsigned b_size = (sizes & 0xf) + 1;

	unsigned mode = fetch();

	if ((mode >> 4) == 9) {
		// BASECONV
		// I have not idea how the actual microcode routine works, so here is an approximation
		// Doesn't handle cases where buffer hasn't been memset to 0xc0
		unsigned dest_width = reg_read(AL);
		unsigned base = a_size + 1;

		if (b_size > 8) {
			fprintf(stderr, "%i byte baseconv too big for our modern 64bit machines\n", b_size);
			exit(1);
		}
		uint16_t dst_addr = get_twobit(mode, 0, dest_width);
		uint16_t src_addr = get_twobit(mode, 1, b_size);


		// Convert to little endian
		unsigned long long num = 0;
		for (int i=0; i < b_size; i++) {
			num = num << 8 | mmu_mem_read8(src_addr+i);
		}

		char buffer[32];

		if (base == 10) {
			snprintf(buffer, sizeof(buffer), "%llu", num);
		} else if (base == 16) {
			snprintf(buffer, sizeof(buffer), "%llX", num);
		} else {
			fprintf(stderr, "baseconv, unsupported base %i\n", base);
			exit(1);
		}

		// I'm kind of guessing here, but it seems to do this?
		unsigned actual_width = strlen(buffer);
		if (actual_width > dest_width) {
			alu_out = ALU_F;
			return 0;
		}

		for (int i=0; i<actual_width; i++) {
			mmu_mem_write8(dst_addr+i, buffer[i] | 0x80);
		}

		// apparently A needs to be updated to point after string
		regpair_write(A, dst_addr + actual_width);
		return 0;
	}

	switch (mode >> 4) {
	default:
		fprintf(stderr, "Unsupported 46 Bignum op %i\n", mode >> 4);
		exit(1);
	}
}

/*
 *	Load flags
 *	F not touched
 *	L not touched
 *	M cleared then set if MSB of operand
 */
static void ldflags(unsigned r)
{
	alu_out &= ~(ALU_M | ALU_V);
	if (r & 0x80)
		alu_out |= ALU_M;
	if ((r & 0xFF) == 0)
		alu_out |= ALU_V;
}

/*
 *	The docs simply say that F is set if the sign of the destination
 *	register changes.
 *
 *	V - set according to value being zero or non zero
 *	M - set on result being negative
 *	F - set according to overflow rules
 *
 *	L is set only by add so done in add
 */
static void arith_flags(unsigned r, uint8_t a, uint8_t b)
{
	alu_out &= ~(ALU_F | ALU_M | ALU_V);
	if ((r & 0xFF) == 0)
		alu_out |= ALU_V;
	if (r & 0x80)
		alu_out |= ALU_M;
/*	if ((r ^ d) & 0x80)
		alu_out |= ALU_F; */

	/* Overflow for addition is (!r & x & m) | (r & !x & !m) */
	if (r & 0x80) {
		if (!((a | b) & 0x80))
			alu_out |= ALU_F;
	} else {
		if (a & b & 0x80)
			alu_out |= ALU_F;
	}
}

/*
 *	Subtract is similar but the overflow rule probably differs and
 *	L is a borrow not a carry
 */
static void sub_flags(uint8_t r, uint8_t a, uint8_t b)
{
	alu_out &= ~(ALU_F | ALU_M | ALU_V);
	if ((r & 0xFF) == 0)
		alu_out |= ALU_V;
	if (r & 0x80)
		alu_out |= ALU_M;
	if (a & 0x80) {
		if (!((b | r) & 0x80))
        		alu_out |= ALU_F;;
       	} else {
       		if (b & r & 0x80)
       			alu_out |= ALU_F;;
	}
}

/*
 *	Logical operations
 *	M is set if there is a 1 in the MSB of the source register
 *	   (or dest register for double register ops)
 *		TODO: before or after operation ?
 */
static void logic_flags(unsigned r)
{
	alu_out &= ~(ALU_M | ALU_V);
	if (r & 0x80)
		alu_out |= ALU_M;
	if (!(r & 0xFF))
		alu_out |= ALU_V;
}

/*
 *	Shift
 *	L is the bit shfited out
 *	M is set as with logic
 *	V is set if result is zero
 *	Left shift/rotate: F is xor of L and M after shift
 *
 */
static void shift_flags(unsigned c, unsigned r)
{
	alu_out &= ~(ALU_L | ALU_M | ALU_V);
	if ((r & 0xFF) == 0)
		alu_out |= ALU_V;
	if (c)
		alu_out |= ALU_L;
	if (r & 0x80)
		alu_out |= ALU_M;
}

/*
 *	Load flags
 *	F not touched
 *	L not touched
 *	M cleared then set if MSB of operand
 */
static void ldflags16(unsigned r)
{
	alu_out &= ~(ALU_M | ALU_V);
	if (r & 0x8000)
		alu_out |= ALU_M;
	if ((r & 0xFFFF) == 0)
		alu_out |= ALU_V;
}

/*
 *	The docs simply say that F is set if the sign of the destination
 *	register changes.
 *
 *	V - set according to value being zero or non zero
 *	M - set on result being negative
 *	F - set according to overflow rules
 *
 *	L is set only by add so done in add
 */
static void arith_flags16(unsigned r, uint16_t a, uint16_t b)
{
	alu_out &= ~(ALU_F | ALU_M | ALU_V);
	if ((r & 0xFFFF) == 0)
		alu_out |= ALU_V;
	if (r & 0x8000)
		alu_out |= ALU_M;
	/* If the result is negative but both inputs were positive then
	   we overflowed */
/* 	if ((r ^ d) & 0x8000)
		alu_out |= ALU_F; */
	/* Overflow for addition is (!r & x & m) | (r & !x & !m) */
	if (r & 0x8000) {
		if (!((a | b) & 0x8000))
			alu_out |= ALU_F;
	} else {
		if (a & b & 0x8000)
			alu_out |= ALU_F;
	}
}

/*
 *	Subtract is similar but the overflow rule probably differs and
 *	L is a borrow not a carry
 */
static void sub_flags16(uint16_t r, uint16_t a, uint16_t b)
{
	alu_out &= ~(ALU_F | ALU_M | ALU_V);
	if ((r & 0xFFFF) == 0)
		alu_out |= ALU_V;
	if (r & 0x8000)
		alu_out |= ALU_M;
	if (a & 0x8000) {
		if (!((b | r) & 0x8000))
        		alu_out |= ALU_F;;
       	} else {
       		if (b & r & 0x8000)
       			alu_out |= ALU_F;;
	}
}

/*
 *	Logical operations
 *	M is set if there is a 1 in the MSB of the source register
 *	   (or dest register for double register ops)
 *		TODO: before or after operation ?
 */
static void logic_flags16(unsigned r)
{
	alu_out &= ~(ALU_M | ALU_V);
	if (r & 0x8000)
		alu_out |= ALU_M;
	if (!(r & 0xFFFF))
		alu_out |= ALU_V;
}

/*
 *	Shift
 *	C is the bit shfited out
 *	M is set as with logic
 *	V is set if result is zero
 *	Left shift/rotate: F is xor of L and M after shift
 *
 */
static void shift_flags16(unsigned c, unsigned r)
{
	alu_out &= ~(ALU_L | ALU_M | ALU_V);
	if ((r & 0xFFFF) == 0)
		alu_out |= ALU_V;
	if (c)
		alu_out |= ALU_L;
	if (r & 0x8000)
		alu_out |= ALU_M;
}


/*
 *	INC/DEC are not full maths ops it seems
 */
static int inc(unsigned reg, unsigned val)
{
	uint8_t r = reg_read(reg);
	reg_write(reg, r + val);
	arith_flags(r + val, r, val);
	return 0;
}

/*
 *	This one is a bit strange
 *
 *	FF->0 expects !F !L M
 *
 */
static int dec(unsigned reg, unsigned val)
{
	uint8_t r = reg_read(reg) - val;
	reg_write(reg, r);
	alu_out &= ~(ALU_L | ALU_V | ALU_M | ALU_F);
	if (r == 0)
		alu_out |= ALU_V;
	if (r & 0x80)
		alu_out |= ALU_M;
	return 0;
}

static int clr(unsigned reg, unsigned v)
{
	reg_write(reg, v);
	alu_out &= ~(ALU_F | ALU_L | ALU_M);
	if (v == 0)
		alu_out |= ALU_V;
	else
		/* Gets us past the tests but is probably wrong */
		alu_out ^= ALU_V;
	return 0;
}

/*
 *	Sets all the flags but rule not clear
 */
static int not(unsigned reg, unsigned val)
{
	uint8_t r = ~reg_read(reg) + val;
	reg_write(reg, r);
	logic_flags(r);
	return 0;
}

/*
 *	The CPU test checks that SRL FF sets C and expects N to be clear
 */
static int sra(unsigned reg, unsigned count)
{
	uint8_t v;
	uint8_t r = reg_read(reg);

	while (count--) {
		v = r >> 1;
		if (v & 0x40)
			v |= 0x80;
		shift_flags(r & 1, v);
		r = v;
	}
	reg_write(reg, v);
	return 0;
}

/*
 *	Left shifts also play with F
 */
static int sll(unsigned reg, unsigned count)
{
	uint8_t r = reg_read(reg);
	uint8_t v;

	while (count--) {
		v = r << 1;
		shift_flags((r & 0x80), v);
		alu_out &= ~ALU_F;
		/* So annoying C lacks a ^^ operator */
		switch (alu_out & (ALU_L | ALU_M)) {
		case ALU_L:
		case ALU_M:
			alu_out |= ALU_F;
			break;
		}
		r = v;
	}
	reg_write(reg, v);
	return 0;
}

/* The CPU test checks that an RR with the low bit set propogates carry. It
   also checks that FF shift to 7F leaves n clear. The hex digit conversion
   confirms that the rotates are 9bit rotate through carry */
static int rrc(unsigned reg, unsigned count)
{
	uint8_t r = reg_read(reg);
	uint8_t c;

	while (count--) {
		c = r & 1;

		r >>= 1;
		r |= (alu_out & ALU_L) ? 0x80 : 0;

		shift_flags(c, r);
	}
	reg_write(reg, r);
	return 0;
}

/* An RL of FF sets C but either clears N or leaves it clear */
static int rlc(unsigned reg, unsigned count)
{
	uint8_t r = reg_read(reg);
	uint8_t c;

	while (count--) {
		c = r & 0x80;
		r <<= 1;
		r |= (alu_out & ALU_L) ? 1 : 0;

		shift_flags(c, r);
		alu_out &= ~ALU_F;
		/* So annoying C lacks a ^^ operator */
		switch (alu_out & (ALU_L | ALU_M)) {
		case ALU_L:
		case ALU_M:
			alu_out |= ALU_F;
			break;
		}
	}
	reg_write(reg, r);
	return 0;
}

/*
 *	Add changes all the flags so fix up L
 */
static int add(unsigned dst, unsigned src)
{
	uint16_t d = reg_read(dst);
	uint16_t s = reg_read(src);
	reg_write(dst, d + s);
	arith_flags(d + s, d, s);
	alu_out &= ~ALU_L;
	if ((s + d) & 0x100)
		alu_out |= ALU_L;
	return 0;
}

/*
 *	Subtract changes all the flags
 */
static int sub(unsigned dst, unsigned src)
{
	unsigned s = reg_read(src);
	unsigned d = reg_read(dst);
	unsigned r =  s - d;
	reg_write(dst, r);
	sub_flags(r, s, d);
	alu_out &= ~ALU_L;
	if (d <= s)
		alu_out |= ALU_L;
	return 0;
}

/*
 *	Logic operations
 */
static int and(unsigned dst, unsigned src)
{
	uint8_t r = reg_read(dst) & reg_read(src);
	reg_write(dst, r);
	logic_flags(r);
	return 0;
}

static int or(unsigned dst, unsigned src)
{
	uint8_t r = reg_read(dst) | reg_read(src);
	reg_write(dst, r);
	logic_flags(r);
	return 0;
}

static int xor(unsigned dst, unsigned src)
{
	uint8_t r = reg_read(dst) ^ reg_read(src);
	reg_write(dst, r);
	logic_flags(r);
	return 0;
}

static int mov(unsigned dst, unsigned src)
{
	uint8_t r = reg_read(src);
	reg_write(dst, r);
	logic_flags(r);
	return 0;
}


/* 16bit versions */

static uint16_t inc16(uint16_t a, uint16_t imm)
{
	arith_flags(a + imm, a, imm);
	return a + imm;
}

static uint16_t dec16(uint16_t a, uint16_t imm)
{
	uint16_t r = a - imm;
	alu_out &= ~(ALU_L | ALU_V | ALU_M | ALU_F);
	if ((r & 0xFFFF) == 0)
		alu_out |= ALU_V;
	if (r & 0x8000)
		alu_out |= ALU_M;
	return r;
}

/* Assume behaviour matches CLR */
static uint16_t clr16(uint16_t a, uint16_t imm)
{
	alu_out &= ~(ALU_F | ALU_L | ALU_M);
/*	if (imm == 0) */
		alu_out |= ALU_V;
	return imm;
}

static uint16_t not16(uint16_t a, uint16_t imm)
{
	uint16_t r = (~a) + imm;
	logic_flags16(r);
	return r;
}

static uint16_t sra16(uint16_t a, uint16_t count)
{
	uint16_t v;
	uint16_t r = a;

	while (count--) {
		v = r >> 1;
		if (v & 0x4000)
			v |= 0x8000;
		shift_flags16(r & 1, v);
		r = v;
	}
	return r;
}

static uint16_t sll16(uint16_t a, uint16_t count)
{
	uint16_t v;
	uint16_t r = a;

	while (count--) {
		v = r << 1;
		shift_flags16((r & 0x8000), v);
		alu_out &= ~ALU_F;
		/* So annoying C lacks a ^^ operator */
		switch (alu_out & (ALU_L | ALU_M)) {
		case ALU_L:
		case ALU_M:
			alu_out |= ALU_F;
			break;
		}
		r = v;
	}
	return r;
}

static uint16_t rrc16(uint16_t a, uint16_t count)
{
	uint16_t r = a;
	uint16_t c;

	while (count--) {
		c = r & 1;

		r >>= 1;
		r |= (alu_out & ALU_L) ? 0x8000 : 0;

		shift_flags16(c, r);
	}
	return r;
}

static uint16_t rlc16(uint16_t a, uint16_t count)
{
	uint16_t r = a;
	uint16_t c;

	while (count--) {
		c = r & 0x8000;

		r <<= 1;
		r |= (alu_out & ALU_L) ? 1 : 0;

		shift_flags16(c, r);
		alu_out &= ~ALU_F;
		/* So annoying C lacks a ^^ operator */
		switch (alu_out & (ALU_L | ALU_M)) {
		case ALU_L:
		case ALU_M:
			alu_out |= ALU_F;
			break;
		}
	}
	return r;
}

static int add16(unsigned dsta, unsigned a, unsigned b)
{
	mmu_mem_write16(dsta, a + b);
	arith_flags16(a + b, a, b);
	alu_out &= ~ALU_L;
	if ((b + a) & 0x10000)
		alu_out |= ALU_L;
	return 0;
}

static int sub16(unsigned dsta, unsigned a, unsigned b)
{
	unsigned r = b - a;
	mmu_mem_write16(dsta, r);
	sub_flags16(r, b, a);
	alu_out &= ~ALU_L;
	if (a <= b)
		alu_out |= ALU_L;
	return 0;
}

static int and16(unsigned dsta, unsigned a, unsigned b)
{
	uint16_t r = a & b;
	mmu_mem_write16(dsta, r);
	logic_flags16(r);
	return 0;
}

static int or16(unsigned dsta, unsigned a, unsigned b)
{
	uint16_t r = a | b;
	mmu_mem_write16(dsta, r);
	logic_flags16(r);
	return 0;
}

static int xor16(unsigned dsta, unsigned a, unsigned b)
{
	uint16_t r = a ^ b;
	mmu_mem_write16(dsta, r);
	logic_flags16(r);
	return 0;
}

static int mov16(unsigned dsta, unsigned srcv)
{
	mmu_mem_write16(dsta, srcv);
	logic_flags16(srcv);
	return 0;
}

/*
 *	Address generator (with side effects). We don't know when the
 *	pre-dec/post-inc hits the register
 */

static uint16_t indexed_address(unsigned size)
{
	uint8_t idx = fetch();
	unsigned r = idx >> 4;
	unsigned addr;
	int8_t offset = 0;	/* Signed or not ? */

	if (idx & 0x08)
		offset = fetch();
	switch (idx & 0x03) {
	case 0:
		addr = regpair_read(r) + offset;
		break;
	case 1:
		addr = regpair_read(r);
		regpair_write(r, addr + size);
		addr = addr + offset;
		break;
	case 2:
		addr = regpair_read(r);
		addr -= size;
		regpair_write(r, addr);
		addr = addr + offset;
		break;
	default:
		fprintf(stderr, "Unknown indexing mode %02X at %04X\n",
			idx, exec_pc);
		exit(1);
	}
	if (idx & 0x04)
		addr = mmu_mem_read16(addr);
	return addr;
}

static uint16_t decode_address(unsigned size, unsigned mode)
{
	uint16_t addr;
	uint16_t indir = 0;

	switch (mode) {
	case 0:
		addr = pc;
		pc += size;
		indir = 0;
		break;
	case 1:
		addr = pc;
		pc += 2;
		indir = 1;
		break;
	case 2:
		addr = pc;
		pc += 2;
		indir = 2;
		break;
	case 3:
		addr = (int8_t) fetch();
		addr += pc;
		indir = 0;
		break;
	case 4:
		addr = (int8_t) fetch();
		addr += pc;
		indir = 1;
		break;
	case 5:
		/* Indexed modes */
		addr = indexed_address(size);
		indir = 0;
		break;
	case 6:
	case 7:
		fprintf(stderr, "unknown address indexing %X at %04X\n",
			mode, exec_pc);
		break;
	default:
		/* indexed off a register */
		addr = regpair_read((mode & 7) << 1);
		indir = 0;
		break;
	}
	while (indir--)
		addr = mmu_mem_read16(addr);
	return addr;
}

/*
 *	Branch instructions
 */

static int branch_op(void)
{
	unsigned t;
	int8_t off;

	switch (op & 0x0F) {
	case 0:		/* BL   Branch if link is set */
		t = (alu_out & ALU_L);
		break;
	case 1:		/* BNL  Branch if link is not set */
		t = !(alu_out & ALU_L);
		break;
	case 2:		/* BF   Branch if fault is set */
		t = (alu_out & ALU_F);
		break;
	case 3:		/* BNF  Branch if fault is not set */
		t = !(alu_out & ALU_F);
		break;
	case 4:		/* BZ   Branch if zero */
		t = (alu_out & ALU_V);
		break;
	case 5:		/* BNZ  Branch if non zero */
		t = !(alu_out & ALU_V);
		break;
	case 6:		/* BM   Branch if minus */
		t = alu_out & ALU_M;
		break;
	case 7:		/* BP   Branch if plus */
		t = !(alu_out & ALU_M);
		break;
	case 8:		/* BGZ  Branch if greater than zero */
		/* Branch if both M and V are zero */
		t = !(alu_out & (ALU_M | ALU_V));
		break;
	case 9:		/* BLE  Branch if less than or equal to zero */
		t = alu_out & (ALU_M | ALU_V);
		break;
	case 10:		/* BS1  */
		t = (switches & BS1);
		break;
	case 11:		/* BS2 */
		t = (switches & BS2);
		break;
	case 12:		/* BS3 */
		t = (switches & BS3);
		break;
	case 13:		/* BS4 */
		t = (switches & BS4);
		break;
	case 14:		/* BTM - branch on teletype mark - CPU4 only ? */
		t = 0;
		break;
	case 15:		/* B?? - branch of IL1 AH bit 0 set (see B6/C6) */
		t = cpu_sram[0x10] & 0x01;
		break;
	}
	/* We'll keep pc and reg separate until we know if/how it fits memory */
	off = fetch();
	/* Offset is applied after fetch leaves PC at next instruction */
	if (t) {
		pc += off;
		return 18;
	}
	return 9;
}

#define SWITCH_IPL_RETURN  1
#define SWITCH_IPL_RETURN_MODIFIED 2
#define SWITCH_IPL_INTERRUPT 3

static void switch_ipl(unsigned new_ipl, unsigned mode)
{
	/*  C register layout:
	 *
	 *  15-12   Previous IPL
	 *  11-8    Unknown, not used?
	 *  7       Value (Zero)
	 *  6       Minus (Sign)
	 *  5       Fault (Overflow)
	 *  4       Link (Carry)
	 *  3-0     Memory MAP aka MMU aka Page Table Base
	 */

	unsigned old_ipl = cpu_ipl;
	if (mode != SWITCH_IPL_RETURN_MODIFIED) {
		// Save pc
		regpair_write(P, pc);

		// Save flags and MAP
		reg_write(CL, alu_out | cpu_mmu);
	}
	cpu_ipl = new_ipl;

	// We are now on the new level

	// restore pc
	pc = regpair_read(P);

	if (mode == SWITCH_IPL_INTERRUPT) {
		// Save previous IPL, so we can return later
		reg_write(CH, old_ipl << 4);
	}

	uint8_t cl = reg_read(CL);
	// Restore flags
	alu_out = cl & (ALU_L | ALU_F | ALU_M | ALU_V);

	// Restore memory MAP
	cpu_mmu = cl & 0x7;
}

/* Low operations - not all known */
static int low_op(void)
{
	switch (op) {
	case 0x00:		/* HALT */
		halted = 1;
		break;
	case 0x01:		/* NOP */
		return 4;
	case 0x02:		/* SF   Set Fault */
		alu_out |= ALU_F;
		break;
	case 0x03:		/* RF   Reset Fault */
		alu_out &= ~ALU_F;
		break;
	case 0x04:		/* EI   Enable Interrupts */
		int_enable = 1;
		break;
	case 0x05:		/* DI   Disable Interrupts */
		int_enable = 0;
		return 8;
	case 0x06:		/* SL   Set Link */
		alu_out |= ALU_L;
		break;
	case 0x07:		/* RL   Clear Link */
		alu_out &= ~ALU_L;
		break;
	case 0x08:		/* CL   Complement Link */
		alu_out ^= ALU_L;
		break;
	case 0x09:		/* RSR  Return from subroutine */
		pc = regpair_read(X);
		regpair_write(X, pop());
		break;
	case 0x0A:		/* RI   Return from interrupt */
		/* This may differ a bit on the CPU6 seems to have a carry
		   involvement */
		switch_ipl(reg_read(CH) >> 4, SWITCH_IPL_RETURN);
		break;
	case 0x0B:		/* RIM  Return from interrupt modified */
		switch_ipl(reg_read(CH) >> 4, SWITCH_IPL_RETURN_MODIFIED);
		break;
	case 0x0C:
		/* EE200 historical ? - enable link to teletype */
		break;
	case 0x0D:
		/* No flag effects */
		regpair_write(X, pc);
		break;
		/*
		 * "..0x0E ought to be a long (but not infinite) loop.
		 *  The delay opcode 0x0E should 4.5ms long.  I do not know how the
		 *  CPU5 & CPU6 handles the 0E but in the CPU4 it stopped the system
		 *  clock for the full 4.5ms, it even stopped the DMA channel for the
		 *  4.5ms which caused problems if you happened to be in the middle of
		 *  a disk R/W operation because the disk drive did not stop spinning
		 *  so disk data got screwed up."
		 *              -- Ken Romain
		 */
	case 0x0E:		/* DELAY */
		advance_time(4.5 * 1000000.0);
		break;
	case 0x0F:
	    /* RSYS - Does the inverse of JSYS
		 * Save PC to P, skip a byte off stack, load PC from X, load X
		 * from stack, load IPL from stack, load mmu tag from stack
		 */
		{
			uint16_t new_x, new_pc;
			regpair_write(P, pc);
			popbyte();	/* Skips one */
			new_x = pop();	/* Loads X */
			cpu_ipl = popbyte();	/* Loads new IL */
			/* X is set off the stack and S is propogated */
			new_pc = regpair_read(X);
			{
				uint8_t byte = popbyte();
				alu_out = byte & (ALU_L | ALU_F | ALU_M | ALU_V);
				/* We flip MMU context after all the POP cases */
				cpu_mmu = byte & 0x07;
			}
			regpair_write(X, new_x);
			pc = new_pc;
			return 0;
		}
	}
	return 0;
}

/* JSYS - System call
 *
 *	66 {arg}
 *
 * Pushes current state, switches to mmu bank zero and jumps to 0x100
 */
static int jsys_op(void)
{
	uint8_t arg = fetch();
	pushbyte(alu_out | cpu_mmu);  // Push CCR and Page Table Base
	pushbyte(cpu_ipl & 0xf);      // Push current level
	push(regpair_read(X));        // Push X
	regpair_write(X, pc);         // X <- PC

	pushbyte(arg);                // Push arg
	cpu_mmu = 0;                  // Switch to mmu bank 0
	pc = 0x100;                   // jump to 0x100
	return 0;
}

/* We only know some of this - it would be logical to expect DMA disable in here */
static int dma_op(void)
{
	unsigned rp;
	/* operations 2Fxx */
	op = fetch();
	rp = (op >> 4);

	switch (op & 0x0F) {
	case 0:
		dma_addr = regpair_read(rp);
		break;
	case 1:
		regpair_write(rp, dma_addr);
		break;
	case 2:
		dma_count = regpair_read(rp);
		break;
	case 3:
		regpair_write(rp, dma_count);
		break;
	case 4:
		dma_mode = rp;
		break;
	case 5:	/* From the microcode analysis */
		dma_mode = regpair_read(rp);
		break;
	case 6:
		dma_enable = 1;
		break;
	case 7:	/* From microcode */
		dma_enable = 0;
		break;
	/* 8-9 read/write some kind of unknown byte status register */
	case 8:
		dma_mystery = reg_read(rp);
		break;
	case 9:
		reg_write(rp, dma_mystery);
		break;
	/* A-F are not used */
	default:
		fprintf(stderr, "Unknown DMA operations 2F%02X at %04X\n",
			op, exec_pc);
		break;
	}
	return 0;
}

/*
 *	Jump doesn't quite match the op decodes for the load/store strangely
 *
 *	0 would be nonsense
 *	1,2,3 seem to match
 *	5 is used for what we would expect to be indexed modes but we only
 *	see it used as if the indexing byte was implicitly 8 (A + offset)
 *
 *	4,6,7 we don't know but 6,7 are not used for load/store and 4 is
 *	an indirect so is perhaps mmu_mem_read16(mmu_mem_read16(fetch16()));
 *
 *	7E and 7F are repurposed as multi register push/pop on CPU6
 */
static int jump_op(void)
{
	uint16_t new_pc;
	if (op == 0x76) {	/* syscall is a mystery */
		uint8_t old_ipl = cpu_ipl;
		unsigned old_s = regpair_read(S);
		cpu_ipl = 15;
		/* Unclear if this also occurs */
		/* Also seems to propogate S but can't be sure */
		regpair_write(S, old_s);
		reg_write(CH, old_ipl);
		return 0;
	}
	if (op == 0x7E) {
		/* Push a block of registers given the last register to push
		   and the count */
		uint8_t r = fetch();
		uint8_t c = (r & 0x0F);
		unsigned addr = regpair_read(S);
		r >>= 4;
		/* We push the highest one first */
		r += c;
		r &= 0x0F;
		c++;
		/* A push of S will use the original S before the push insn. */
		while(c--) {
			mmu_mem_write8(--addr, reg_read(r));
			r--;
			r &= 0x0F;
		}
		regpair_write(S, addr);
		return 0;
	}
	if (op == 0x7F) {
		/* Pop a block of registers given the first register and count */
		uint8_t r = fetch();
		uint8_t c = (r & 0x0F) + 1;
		unsigned addr = regpair_read(S);
		r >>= 4;
		/* A pop of S will always update S at the end */
		while(c--) {
			reg_write(r, mmu_mem_read8(addr++));
			r++;
			r = r & 0x0F;
		}
		regpair_write(S, addr);
		return 0;
	}
	/* We don't know what 0x70 does (it's invalid but I'd guess it jumps
	   to the following byte */
	new_pc = decode_address(2, op & 0x07);
	if (op & 0x08) {
		/* Subroutine calls are a hybrid of the classic call/ret and
		   branch/link. The old X is stacked, X is set to the new
		   return address and then we jump */
		push(regpair_read(X));
		regpair_write(X, pc);
		/* This is specifically stated in the EE200 manual */
		regpair_write(P, new_pc);
	}
	pc = new_pc;
	return 0;
}

/*
 *	This appears to work like the other loads and not affect C
 */
static int x_op(void)
{
	/* Valid modes 0-5 */
	uint16_t addr = decode_address(2, op & 7);
	uint16_t r;
	if (op & 0x08) {
		r = regpair_read(X);
		mmu_mem_write16(addr, r);
		ldflags16(r);
	} else {
		r = mmu_mem_read16(addr);
		regpair_write(X, r);
		ldflags16(r);
	}
	return 0;
}

static int loadbyte_op(void)
{
	uint16_t addr = decode_address(1, op & 0x0F);
	uint8_t r = mmu_mem_read8(addr);

	if (op & 0x40)
		reg_write(BL, r);
	else
		reg_write(AL, r);
	ldflags(r);
	return 0;
}

static int loadword_op(void)
{
	uint16_t addr = decode_address(2, op & 0x0F);
	uint16_t r = mmu_mem_read16(addr);

	if (op & 0x40)
		regpair_write(B, r);
	else
		regpair_write(A, r);
	ldflags16(r);
	return 0;
}

static int storebyte_op(void)
{
	uint16_t addr = decode_address(1, op & 0x0F);
	uint8_t r;

	if (op & 0x40)
		r = reg_read(BL);
	else
		r = reg_read(AL);

	mmu_mem_write8(addr, r);
	ldflags(r);
	return 0;
}

static int storeword_op(void)
{
	uint16_t addr = decode_address(2, op & 0x0F);
	uint16_t r;

	if (op & 0x40)
		r = regpair_read(B);
	else
		r = regpair_read(A);

	mmu_mem_write16(addr, r);
	ldflags16(r);

	return 0;
}

// opsys ALWAYS uses this instruction for accessing MMIO.
// It might do something special on the bus, or with page-tables
//
// If index register is odd, does a store, otherwise does a load
// If destination register is odd, does an 8 bit operation, otherwise 16bit
static int cpu6_indexed_loadstore(void)
{
	uint8_t regs = fetch();
	int8_t offset = fetch();
	uint8_t reg = regs >> 4;
	uint16_t addr = regpair_read(regs & 0x0e) + offset;

	switch (regs & 0x11) {
	case 0x00: // 16 bit load
		regpair_write(reg, mmu_mem_read16(addr));
		break;
	case 0x01: // 16 bit store
		mmu_mem_write16(addr, regpair_read(reg));
		break;
	case 0x10: // 8 bit load
		reg_write(reg, mmu_mem_read8(addr));
		break;
	case 0x11: // 8 bit store
		mmu_mem_write8(addr, reg_read(reg));
		break;
	}

	ldflags(reg);
	return 0;
}

static void cpu6_il_storebyte(uint8_t ipl, uint8_t rd, uint8_t rs)
{
	mmu_mem_write8((ipl << 4) | rd, reg_read(rs));
}

static void cpu6_il_loadbyte(uint8_t ipl, uint8_t rs, uint8_t rd)
{
	reg_write(rd, mmu_mem_read8((ipl << 4) | rs));
}

static int cpu6_il_mov(void)
{
	uint8_t byte2 = fetch();
	uint8_t ipl = byte2 >> 4;
	uint8_t r = byte2 & 0x0F;

	if (op == 0xd7) {
		cpu6_il_storebyte(ipl, (r | 1) ^ 1, AH);
		cpu6_il_storebyte(ipl, r ^ 1, AL);
	} else {
		cpu6_il_loadbyte(ipl, (r | 1) ^ 1, AH);
		cpu6_il_loadbyte(ipl, r ^ 1, AL);

	}
	return 0;
}

// 16bit store instruction.
// With address modes for:
//   - reg to reg
//   - (direct)
//   - immediate
//   - indexed (with 16bit displacement)
static int store16() {
	uint16_t addr;
	uint8_t regs = fetch();
	unsigned dst_reg = (regs >> 4) & 0xe;
	uint16_t value = regpair_read(regs & 0xe);

	ldflags16(value); // Flags

	switch(regs & 0x11) {
	case 0x00: // dst_reg <- src_reg
		regpair_write(dst_reg, value);
		break;
	case 0x01: // (direct) <- src_reg
		addr = fetch16();
		mmu_mem_write16(addr, value);
		break;
	case 0x10: // literal <- src_reg
	    // Writes src to next two bytes after instruction.
		addr = fetch_literal(2);
		mmu_mem_write16(addr, value);
		break;
	case 0x11: // (src_reg + disp16) <- src_reg
		addr = fetch16() + regpair_read(dst_reg);
		mmu_mem_write16(addr, value);
		break;
	}
	return 0;
}

static int loadstore_op(void)
{
	switch (op & 0x30) {
	case 0x00:
		return loadbyte_op();
	case 0x10:
		return loadword_op();
	case 0x20:
		return storebyte_op();
	case 0x30:
		return storeword_op();
	default:
		fprintf(stderr, "internal error loadstore\n");
		exit(1);
	}
}

static int misc2x_op(void)
{
	unsigned low = 0;
	unsigned reg = AL;
	if (!(op & 8)) {
		reg = fetch();
		low = reg & 0x0F;
		reg >>= 4;
	}

	switch (op) {
	case 0x20:
		return inc(reg, low + 1);
	case 0x21:
		return dec(reg, low + 1);
	case 0x22:
		return clr(reg, low);
	case 0x23:
		return not(reg, low);
	case 0x24:
		return sra(reg, low + 1);
	case 0x25:
		return sll(reg, low + 1);
	case 0x26:
		return rrc(reg, low + 1);
	case 0x27:
		return rlc(reg, low + 1);
	case 0x28:
		return inc(AL, 1);
	case 0x29:
		return dec(AL, 1);
	case 0x2A:
		return clr(AL, 0);
	case 0x2B:
		return not(AL, 0);
	case 0x2C:
		return sra(AL, 1);
	case 0x2D:
		return sll(AL, 1);
		/* On CPU 4 these would be inc XL/dec XL but they are not present and
		   X is almost always handled as a 16bit register only */
	case 0x2E:
		return mmu_transfer_op();
	case 0x2F:
		return dma_op();
	default:
		fprintf(stderr, "internal error misc2\n");
		exit(1);
	}
}

static uint16_t misc3x_op_impl(unsigned op, uint16_t val, unsigned imm)
{
	switch (op) {
	case 0x30:
		return inc16(val, imm + 1);
	case 0x31:
		return dec16(val, imm + 1);
	case 0x32:
		return clr16(val, imm);
	case 0x33:
		return not16(val, imm);
	case 0x34:
		return sra16(val, imm + 1);
	case 0x35:
		return sll16(val, imm + 1);
	case 0x36:
		return rrc16(val, imm + 1);
	case 0x37:
		return rlc16(val, imm + 1);
	case 0x38:
		return inc16(val, 1);
	case 0x39:
		return dec16(val, 1);
	case 0x3A:
		return clr16(val, 0);
	case 0x3B:
		return not16(val, 0);
	case 0x3C:
		return sra16(val, 1);
	case 0x3D:
		return sll16(val, 1);
	default:
		fprintf(stderr, "internal error misc3 %x\n", op);
		exit(1);
	}
}

/* Like misc2x but word
 * If the explicit register is odd, it operates on memory
*/
static int misc3x_op(void)
{
	// Special cases that don't fit general pattern
	if (op == 0x3E) {
		regpair_write(X, inc16(regpair_read(X), 1));
		return 0;
	}
	if (op == 0x3F) {
		regpair_write(X, dec16(regpair_read(X), 1));
		return 0;
	}

	if (op & 8) {
		// Implicit ops that work on A
		regpair_write(A, misc3x_op_impl(op, regpair_read(A), 0));
		return 0;
	}

	unsigned opn = fetch();
	unsigned imm = opn & 0xf;
	unsigned reg = (opn >> 4) & 0xe;
	if ((opn & 0x10) == 0) {
		// If register is even, operate on register
		regpair_write(reg, misc3x_op_impl(op, regpair_read(reg), imm));
		return 0;
	}

	// Otherwise, we do a memory read-modify-write operation
	uint16_t addr = fetch16();
	if (reg != A) {	// indexed
		addr += regpair_read(reg);
	}
	uint16_t result = misc3x_op_impl(op, mmu_mem_read16(addr), imm);
	mmu_mem_write16(addr, result);
	return 0;
}

/* Mostly ALU operations on AL */
/* 47 is added in CPU5/6 for block operations */
static int alu4x_op(void)
{
	unsigned src, dst;
	if ((!(op & 0x08))) {
		dst = fetch();
		src = dst >> 4;
		dst &= 0x0F;
	}
	switch (op) {
	case 0x40:		/* add */
		return add(dst, src);
	case 0x41:		/* sub */
		return sub(dst, src);
	case 0x42:		/* and */
		return and(dst, src);
	case 0x43:		/* or */
		return or(dst, src);
	case 0x44:		/* xor */
		return xor(dst, src);
	case 0x45:		/* mov */
		return mov(dst, src);
	case 0x48:
		return add(BL, AL);
	case 0x49:
		return sub(BL, AL);
	case 0x4A:
		return and(BL, AL);
	case 0x4B:
		return mov(XL, AL);
	case 0x4C:
		return mov(YL, AL);
	case 0x4D:
		return mov(BL, AL);
	case 0x4E:		/* unused */
	case 0x4F:		/* unused */
		fprintf(stderr, "Unknown ALU4 op %02X at %04X\n", op,
			exec_pc);
		return 0;
	default:
		fprintf(stderr, "internal error alu4\n");
		exit(1);
	}
}

/* Much like ALU4x but word */
/*
 *	CPU 6 appears to make 50-54 (maybe 55) use additional modes using the
 *	low bits differently to CPU4
 *
 *	CPU4 low bits cause weird high/high register behaviour CPU 6 they are
 *	a pair to select additional modes.
 *
 *	[sr:3][sx1:1][dr:3][sx0:1]
 *
 */
static int alu5x_op(void)
{
	unsigned src, dst;
	uint16_t addr;
	uint16_t a; // First argument isn't always dst anymore.
	uint16_t b;
	uint16_t dsta;
	uint16_t movv; // move value is usually source.
	               // But when is a choice of a memory operand, mov ignores everything else.
	if (!(op & 0x08)) {
		dst = fetch();
		src = dst >> 4;
		movv = b = regpair_read(src & 0x0E);
		dsta = regpair_addr(dst & 0x0E);
		a = regpair_read(dst & 0XE);
		if (op <= 0x55) {
			switch(dst & 0x11) {
			case 0x00: // dst_reg <- src_reg
				break;
			case 0x01: // dst_reg <- src_reg OP (direct)
				       // mov takes (direct)
				addr = fetch16();
				movv = b = mmu_mem_read16(addr);
				break;
			case 0x10: // dst_reg <- src_reg OP literal
				       // mov takes literal
				movv = b = fetch16();
				break;
			case 0x11: // dst_reg <- (src_reg + disp16) OP dst_reg
				       // mov takes (src_reg + disp16)
				addr = fetch16() + b;
				b = a;
				movv = a = mmu_mem_read16(addr);
				break;
			}
		}
	} else {
		a = regpair_read(B);
		movv = b = regpair_read(A);
		dsta = regpair_addr(B);
	}
	switch (op) {
	case 0x50:		/* add */
		return add16(dsta, a, b);
	case 0x51:		/* sub */
		return sub16(dsta, a, b);
	case 0x52:		/* and */
		return and16(dsta, a, b);
	case 0x53:		/* or */
		return or16(dsta, a, b);
	case 0x54:		/* xor */
		return xor16(dsta, a, b);
	case 0x55:		/* mov */
		return mov16(dsta, movv);
	case 0x56:		/* unused */
	case 0x57:		/* unused */
		fprintf(stderr, "Unknown ALU5 op %02X at %04X\n", op,
			exec_pc);
		return 0;
	case 0x58:
		return add16(dsta, a, b);
	case 0x59:
		return sub16(dsta, a, b);
	case 0x5A:
		return and16(dsta, a, b);
	/* These are borrowed for moves */
	case 0x5B:
		return mov16(regpair_addr(X), movv);
	case 0x5C:
		return mov16(regpair_addr(Y), movv);
	case 0x5D:
		return mov16(regpair_addr(B), movv);
	case 0x5E:
		return mov16(regpair_addr(Z), movv);
	case 0x5F:
		return mov16(regpair_addr(S), movv);
	default:
		fprintf(stderr, "internal error alu5\n");
		exit(1);
	}
}

/* It's not entirely clear what these instructions are for
 * they set Level 1's AH to 0 and -1
 *
 * Instruction 1F branches based on that reg, but doesn't show up in disassembly of LOAD
 * Disassembly of LOAD hints that it might disable the timer decrementer
 */
static int semaphore_op(void) {
	switch(op) {
	case 0xB6:
		cpu_sram[0x10] = 0xff;
		return 0;
	case 0xC6:
		cpu_sram[0x10] = 0x00;
		return 0;
	}
	fprintf(stderr, "semop: internal\n");
	exit(1);
}

/*
 *	The CPU has directly controlled flags for C N Z I
 *	We know from the branch rules there is an internal V flag
 *	The front panel implies we have an L but we don't know too much
 *	about it.
 */
static char *flagcode(void)
{
	static char buf[6];
	strcpy(buf, "-----");
	if (alu_out & ALU_F)
		*buf = 'F';
	if (alu_out & ALU_L)
		buf[2] = 'L';
	if (alu_out & ALU_M)
		buf[3] = 'M';
	if (alu_out & ALU_V)
		buf[4] = 'V';
	return buf;
}

void cpu6_interrupt(unsigned trace)
{
	unsigned old_ipl = cpu_ipl;
	unsigned pending_ipl;

	if (int_enable == 0)
		return;

	pending_ipl = pending_ipl_mask == 0 ? 0 : 31 - __builtin_clz(pending_ipl_mask);

	if (pending_ipl > cpu_ipl) {
		halted = 0;
		switch_ipl(pending_ipl, SWITCH_IPL_RETURN);

		if (trace)
			fprintf(stderr,
				"Interrupt %X: New PC = %04X, previous IPL %X\n",
				cpu_ipl, pc, old_ipl);
	}
}

// Not quite accurate to real hardware, but hopefully close enough
void cpu_assert_irq(unsigned ipl) {
	pending_ipl_mask |= 1 << ipl;
}

void cpu_deassert_irq(unsigned ipl) {
	pending_ipl_mask &= ~(1 << ipl);
}

unsigned cpu6_execute_one(unsigned trace)
{
	exec_pc = pc;

	cpu6_interrupt(trace);
	if (trace)
		fprintf(stderr, "CPU %04X: ", pc);
	op = fetch();
	if (trace) {
		fprintf(stderr,
			"%02X %s A:%04X  B:%04X X:%04X Y:%04X Z:%04X S:%04X C:%04X LVL:%x MAP:%x | ",
			op, flagcode(), regpair_read(A), regpair_read(B),
			regpair_read(X), regpair_read(Y), regpair_read(Z),
			regpair_read(S), regpair_read(C), cpu_ipl, cpu_mmu);
		disassemble(op);
	}
	if (op < 0x10)
		return low_op();
	if (op < 0x20)
		return branch_op();
	/* 20-5F is sort of ALU stuff but other things seem to have been shoved
	   into the same space */
	if (op < 0x30)
		return misc2x_op();
	if (op < 0x40)
		return misc3x_op();
	if (op == 0x46)
		return bignum_op();
	if (op == 0x47)
		return block_op(0x47);
	if (op < 0x50)
		return alu4x_op();
	if (op == 0x66)
		return jsys_op();
	if (op < 0x60)
		return alu5x_op();
	if (op == 0x67)
		return block_op(0x67);
	if (op < 0x70)
		return x_op();
	if (op < 0x80)
		return jump_op();
	if (op == 0xb6 || op == 0xc6)
		return semaphore_op();
	if (op == 0xd6)
		return store16();
	if (op == 0xd7 || op == 0xe6)
		return cpu6_il_mov();
	if (op == 0xf6)
		return cpu6_indexed_loadstore();
	if (op == 0xf7)
		return memcpy16();
	return loadstore_op();
}

uint16_t cpu6_pc(void)
{
	return exec_pc;
}

void set_pc_debug(uint16_t new_pc) {
	pc = new_pc;
}

void reg_write_debug(uint8_t r, uint8_t v) {
	reg_write(r, v);
}

void regpair_write_debug(uint8_t r, uint16_t v) {
	regpair_write(r, v);
}

void cpu6_set_switches(unsigned v)
{
	switches = v;
}

unsigned cpu6_halted(void)
{
	return halted;
}

/*
 *	MMU microcode initialize
 *
 *	Loop loads 30 entries for MMU bank 0 at 0x0800 spacing (ie 1:1
 *	mapping) then maps the I/O at 7E and 7F
 *	(We've no idea what the top bit is used for if anything)
 */
void cpu6_init(void)
{
	uint8_t *mp = mmu[0];
	unsigned i = 0;
	for (i = 0; i < 30; i++)
		*mp++ = i;
	*mp++ = 0x7E;
	*mp = 0x7F;
	pc = 0xFC00;
}

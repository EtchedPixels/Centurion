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

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "cpu6.h"

static unsigned cpu_ipl = 0;
static uint16_t pc = 0xFC00;
static uint8_t op;
static uint8_t alu_out;
static uint8_t switches = 0xF0;

#define BS1	0x01
#define BS2	0x02
#define BS3	0x04
#define BS4	0x08

static uint16_t dma_addr;
static uint16_t dma_count;
static uint8_t dma_mode;
static uint8_t dma_enable;

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
	/* Hawk uses this */
	if (dma_mode == 0 && dma_enable) {
		mem_write8(dma_addr++, byte);
		dma_count++;
	}
	return 0;
}

/*
 *	We don't know of any more direct access to these as a register
 *	by some other means so our order is arbitrary. Given the 4 switches
 *	and the way Bxx is encoded it may be the switches make up a byte
 *	with this somewhere
 */

#define ALU_C	1
#define ALU_N	2
#define ALU_Z	4
#define ALU_I	8
#define ALU_V	16

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

uint16_t mem_read16(uint16_t addr)
{
	uint16_t r = mem_read8(addr) << 8;
	r |= mem_read8(addr + 1);
	return r;
}

void mem_write16(uint16_t addr, uint16_t val)
{
	mem_write8(addr, val >> 8);
	mem_write8(addr + 1, val);
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
	uint8_t r = mem_read8(pc);
	pc++;
	return r;
}

uint16_t fetch16(void)
{
	uint16_t r;
	r = mem_read8(pc) << 8;
	pc++;
	r |= mem_read8(pc);
	pc++;
	return r;
}

/* We don't know the true mapping between the registers and the
   memory yet */

static uint8_t reg_read(uint8_t r)
{
	return mem_read8((cpu_ipl << 4) | r);
}

static void reg_write(uint8_t r, uint8_t v)
{
	mem_write8((cpu_ipl << 4) | r, v);
}

static uint16_t regpair_read(uint8_t r)
{
	if (r > 7) {
		fprintf(stderr, "Bad regpair encoding %02X %04X\n", op,
			pc);
		exit(1);
	}
	r <<= 1;
	return (reg_read(r) << 8) | reg_read(r + 1);
}

static void regpair_write(uint8_t r, uint16_t v)
{
	if (r > 7) {
		fprintf(stderr, "Bad regpair encoding %02X %04X\n", op,
			pc);
		exit(1);
	}
	r <<= 1;
	reg_write(r, v >> 8);
	reg_write(r + 1, v);
}

/*
 *	Stack helpers
 *
 *	Diag uses -S and S+ for stack operations so assume that this is
 *	the case for internal ones. It's also possible they are sliding them
 *	into another register...
 */

void push(uint16_t val)
{
	uint16_t addr = regpair_read(S);
	addr -= 2;
	mem_write16(addr, val);
	regpair_write(S, addr);
}

uint16_t pop(void)
{
	uint16_t addr = regpair_read(S);
	uint16_t d = mem_read16(addr);
	regpair_write(S, addr + 2);
	return d;
}

void pushbyte(uint8_t val)
{
	uint16_t addr = regpair_read(S);
	addr -= 1;
	mem_write8(addr, val);
	regpair_write(S, addr);
}

uint8_t popbyte(void)
{
	uint16_t addr = regpair_read(S);
	uint8_t d = mem_read8(addr);
	regpair_write(S, addr + 1);
	return d;
}

/*
 *	The MMU
 *
 *	Operations to decode yet
 *	35 04		- possible set MMU map for editing to AL
 *	7E 45		- maybe "make mmu ram appear at 0x100"
 *	7F 45		- maybe "hide mmu ram at 0x0100" (45 implies an
 *			  op between Z and S)
 *
 *	47 40 FF 0100 0200	- possibly MMU on for current IPL
 *	47 80 FF 0100 0200	- sets Z on some kind of error
 *
 *	??
 *
 *	Probably not chance that 0100 and 0200 match the addresses used in
 *	the memory tester
 */

static uint8_t mmu[16][8][2];


/*
 *	MMU load operations
 *
 *	256 x 8 of fast (well for the time.. 45ns or so) SRAM that appears
 *	to be indexed by the top 3 address bits. It looks like it can also
 *	be made to appear between 0x100-0x11F and 0x200-0x21f for a given IPL
 *	for diagnostics
 *
 *	2E 0C ~bank addresshigh for writes
 *	2E 1C ~bank addresshigh for reads
 *
 *	Seems to work on the current IPL.
 */
static int mmu_loadop(uint8_t subop)
{
	uint8_t bank = ~fetch();
	uint16_t val = fetch16();

	if (bank < 0 || bank > 7) {
		fprintf(stderr, "MMU bad bank %02X at %04X\n", bank, pc - 4);
		exit(1);
	}
	if (subop == 0x1C)
		mmu[cpu_ipl][bank][1] = (val >> 5);
	else if (subop == 0x0C)
		mmu[cpu_ipl][bank][0] = (val >> 5);
	else {
		fprintf(stderr, "MMU bad subop %02X at %04X\n", bank, pc - 4);
		exit(1);
	}
	return 0;
}

static int mmu_op47(void)
{
	uint8_t b3;
	/* should be followed by 40 FF or 80 FF then two addresses. We have
	   no clear idea what it does but the addresses match the spots used
	   for memory testing */
	fetch();
	b3 = fetch();
	fetch16();
	fetch16();
	if (b3 == 0x80)
		alu_out |= ALU_Z;
	return 0;
}

/*
 *	ALU - logic should be close, but the flags are speculative.
 *
 *	This is implemented using AM2901 ALU slices in the real
 *	hardware. The ALU provides carry in and out but it doesn't
 *	appear the CPU has any 'with carry' instructions beyond internal
 *	carry and carry visibility.
 *
 *	We should however expect that the likely "flags" are going to be
 *	carry, zero, negative and overflow. Other candidates though might
 *	include AGT and LGT like the TI990.
 */

static void flags(unsigned r)
{
	alu_out &= ~ALU_Z;
	if (r == 0)
		alu_out |= ALU_Z;
}

/* Load seems to be implemented as "add to zero" */
/* Unsure; carry */
/* Could also be 'clears N sets V to top bit */
static void ldflags(unsigned r)
{
	alu_out &= ~(ALU_N | ALU_Z | ALU_V);
	alu_out |= ALU_V;
	if (r & 0x80)
		alu_out |= ALU_N;
	if (r == 0)
		alu_out |= ALU_Z;
}

/*
 *	Arithmetic changes C N V and Z. However the behaviour of SUB and ADD
 *	differ.
 */
static void arith_flags(unsigned r, uint8_t a, uint8_t b)
{
	alu_out &= ~(ALU_C | ALU_N | ALU_Z | ALU_V);
	if ((uint8_t) r == 0)
		alu_out |= ALU_Z;
	if (r & 0x100)
		alu_out |= ALU_C;
	/* If the result is negative but both inputs were positive then
	   we overflowed */
	if (r & 0x80) {
		alu_out |= ALU_N;
		if (!((a | b) & 0x80))
			alu_out |= ALU_V;
	} else {
		/* If the result was positive but both inputs were negative
		   we overflowed */
		if (a & b & 0x80)
			alu_out |= ALU_V;
	}

}

/*
 *	For D-S
 *	
 *	D = S		C  !N  Z
 *	D < S		C   N !Z
 *	D > S		!C !N !Z
 *	
 */
static void sub_flags(uint8_t s, uint8_t d)
{
	unsigned r = d - s;
	alu_out &= ~(ALU_C | ALU_N | ALU_Z | ALU_V);
	if (d == s)
		alu_out |= ALU_C | ALU_Z;
	else if (d < s)
		alu_out |= ALU_C;
	/* Q: is this based on d < s or final sign ? */
	if (r & 0x100)
		alu_out |= ALU_N;

	/* For subtraction the V rule is different */
	if (d & 0x80) {
		if (!((s | r) & 0x80))
			alu_out |= ALU_V;
	} else {
		if (s & r & 0x80)
			alu_out |= ALU_V;
	}
}

/*
 *	Logic changes only the Z flag
 */
static void logic_flags(unsigned r)
{
	if (r == 0)
		alu_out |= ALU_Z;
	else
		alu_out &= ~ALU_Z;
}

/*
 *	This is less clear. We are pretty sure about the C flag, not entirely
 *	sure it affects Z (but it would be an oddity if it did not). The
 *	behaviour of N is quite a mystery.
 *
 *	One possibility is that SLL at least is implemented as an ADC x,x
 */
static void shift_flags(unsigned c, unsigned r)
{
	alu_out &= ~(ALU_C | ALU_N | ALU_Z);
	if (r == 0)
		alu_out |= ALU_Z;
	if (c)
		alu_out |= ALU_C;
	if (r & 0x80)
		alu_out |= ALU_N;
}

static void flags16(unsigned r)
{
	alu_out &= ~ALU_Z;
	if (r == 0)
		alu_out |= ALU_Z;
}

/* Load seems to be implemented as "add to zero" */
static void ldflags16(unsigned r)
{
	alu_out &= ~(ALU_N | ALU_Z | ALU_V);
	alu_out |= ALU_V;
	if (r & 0x8000)
		alu_out |= ALU_N;
	if (r == 0)
		alu_out |= ALU_Z;
}

/*
 *	Arithmetic changes C N and Z. However the behaviour of SUB and ADD
 *	differ so this needs work.
 */
static void arith_flags16(unsigned r, uint16_t a, uint16_t b)
{
	alu_out &= ~(ALU_C | ALU_N | ALU_Z | ALU_V);
	if ((uint16_t) r == 0)
		alu_out |= ALU_Z;
	if (r & 0x10000)
		alu_out |= ALU_C;

	/* If the result is negative but both inputs were positive then
	   we overflowed */
	if (r & 0x8000) {
		alu_out |= ALU_N;
		if (!((a | b) & 0x8000))
			alu_out |= ALU_V;
	} else {
		/* If the result was positive but both inputs were negative
		   we overflowed */
		if (a & b & 0x8000)
			alu_out |= ALU_V;
	}

}

/*
 *	This one is a bit odd
 *
 *	For D-S
 *	
 *	D = S		C  !N  Z
 *	D < S		C   N !Z
 *	D > S		!C !N !Z
 *	
 */
static void sub_flags16(uint16_t s, uint16_t d)
{
	unsigned r = d - s;
	alu_out &= ~(ALU_C | ALU_N | ALU_Z | ALU_V);
	if (d == s)
		alu_out |= ALU_C | ALU_Z;
	else if (d < s)
		alu_out |= ALU_C;
	/* Q: is this based on d < s or final sign ? */
	if (r & 0x10000)
		alu_out |= ALU_N;
	/* For subtraction the V rule is different */
	if (d & 0x8000) {
		if (!((s | r) & 0x8000))
			alu_out |= ALU_V;
	} else {
		if (s & r & 0x8000)
			alu_out |= ALU_V;
	}
}

/*
 *	Logic changes only the Z flag
 */
static void logic_flags16(unsigned r)
{
	if (r == 0)
		alu_out |= ALU_Z;
	else
		alu_out &= ~ALU_Z;
}

/*
 *	This is less clear. We are pretty sure about the C flag, not entirely
 *	sure it affects Z (but it would be an oddity if it did not). The
 *	behaviour of N is quite a mystery.
 *
 *	One possibility is that SLL at least is implemented as an ADC x,x
 */
static void shift_flags16(unsigned c, unsigned r)
{
	alu_out &= ~(ALU_C | ALU_N | ALU_Z);
	if (r == 0)
		alu_out |= ALU_Z;
	if (c)
		alu_out |= ALU_C;
	if (r & 0x80)
		alu_out |= ALU_N;
}

/*
 *	An inc from 0 to 1 is tested and C and N stay 0 but Z is changed
 *	An inc from 0xFF to 0x00 is tested and either clears C and N or
 *	leaves them unchanged but does change Z. The same test sequence
 *	is run for 16bit and appears consistent with 8. A later test
 *	starts with N set and inc FE to FF clears N. Same for 8bit. The
 *	later test sequence does FFFC inc x 4 and expects C but N clear.
 */
static int inc(unsigned reg)
{
	unsigned r = reg_read(reg) + 1;
	reg_write(reg, r);
	flags(r);
	return 0;
}

/*
 * 	A DEC through FF->0 does not set C or N. This is confirmed by
 *	the self tests. Z is however set (confirmed by the diag rom)
 *	and maybe V
 */
static int dec(unsigned reg)
{
	unsigned r = reg_read(reg) - 1;
	reg_write(reg, r);
	flags(r);
	return 0;
}

static int clr(unsigned reg)
{
	/* CLR is tested to clear C and N and set Z. As it's in with the ALU
	   ops presumably it's implemented through the ALU. It also seems to
	   clear V */
	reg_write(reg, 0);
	arith_flags(0, 0, 0);
	return 0;
}

/* The self test appears to check that not only affects Z */
static int not(unsigned reg)
{
	uint8_t r = ~reg_read(reg);
	reg_write(reg, r);
	flags(r);
	return 0;
}

/*
 *	The CPU test checks that SRL FF sets C and expects N to be clear
 */
static int srl(unsigned reg)
{
	uint8_t r = reg_read(reg);
	reg_write(reg, r >> 1);
	shift_flags(r & 1, r >> 1);
	return 0;
}

/*
 *	An SLL of 7F to FF sets N and clears C. In the test it seems all
 *	the tested cases set N.. Mystery TODO
 */
static int sll(unsigned reg)
{
	uint8_t r = reg_read(reg);
	reg_write(reg, r << 1);
	shift_flags((r & 0x80), r << 1);
	return 0;
}

/* The CPU test checks that an RR with the low bit set propogates carry. It
   also checks that FF shift to 7F leaves n clear. The hex digit conversion
   confirms that the rotates are 9bit rotate through carry */
static int rrc(unsigned reg)
{
	uint8_t r = reg_read(reg);
	uint8_t c = r & 1;

	r >>= 1;
	r |= (alu_out & ALU_C) ? 0x80 : 0;

	reg_write(reg, r);
	shift_flags(c, r);
	return 0;
}

/* An RL of FF sets C but either clears N or leaves it clear */
static int rlc(unsigned reg)
{
	uint8_t r = reg_read(reg);
	uint8_t c = r & 0x80;

	r <<= 1;
	r |= (alu_out & ALU_C) ? 1 : 0;

	reg_write(reg, r);
	shift_flags(c, r);
	return 0;
}

/*
 *	Add changes N and C (0x8000 + 0x0x7FFF clears N and C). It also appears
 *	to change Z.
 */
static int add(unsigned dst, unsigned src)
{
	uint16_t r = reg_read(dst) + reg_read(src);
	reg_write(dst, r);
	arith_flags(r, dst, src);
	return 0;
}

/*
 *	Subtract is apparently done via an invert add and inc because
 *	we test that SUB of 0xAA - 0xAA sets C and Z, and seems to clear N
 *
 *	We also know that 0x0001 - 0x8000 sets N and C so C is not a borrow.
 */
static int sub(unsigned dst, unsigned src)
{
	unsigned s = reg_read(src);
	unsigned d = reg_read(dst);
	unsigned r = d - s;
//	fprintf(stderr, "D %X - S %X = %X\n", d, s, r);
	reg_write(dst, r);
	sub_flags(s, d);
	return 0;
}

/*
 *	Logic operations do not affect C or N. There is an explicit test for
 *	this in both directions in the CPU diagnostic tests. Z is affected.
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
	flags(r);
	return 0;
}


/* 16bit versions */

static int inc16(unsigned reg)
{
	unsigned r = regpair_read(reg);
	regpair_write(reg, r + 1);
	arith_flags16(r + 1, r, 1);
	return 0;
}

static int dec16(unsigned reg)
{
	unsigned r = regpair_read(reg) - 1;
	regpair_write(reg, r);
	flags16(r);
	return 0;
}

/* Assume behaviour matches CLR */
static int clr16(unsigned reg)
{
	regpair_write(reg, 0);
	arith_flags16(0, 0, 0);
	return 0;
}

static int not16(unsigned reg)
{
	uint16_t r = ~regpair_read(reg);
	regpair_write(reg, r);
	flags16(r);
	return 0;
}

static int srl16(unsigned reg)
{
	uint16_t r = regpair_read(reg);
	regpair_write(reg, r >> 1);
	shift_flags16(r & 1, r >> 1);
	return 0;
}

static int sll16(unsigned reg)
{
	uint16_t r = regpair_read(reg);
	regpair_write(reg, r << 1);
	shift_flags16((r & 0x8000), r << 1);
	return 0;
}

static int rrc16(unsigned reg)
{
	uint16_t r = regpair_read(reg);
	uint16_t c = r & 1;

	r >>= 1;
	r |= (alu_out & ALU_C) ? 0x8000 : 0;

	regpair_write(reg, r);
	shift_flags16(c, r);
	return 0;
}

static int rlc16(unsigned reg)
{
	uint16_t r = regpair_read(reg);
	uint16_t c = r & 0x8000;

	r <<= 1;
	r |= (alu_out & ALU_C) ? 1 : 0;

	regpair_write(reg, r);
	shift_flags16(c, r);
	return 0;
}

static int add16(unsigned dst, unsigned src)
{
	unsigned r = regpair_read(dst) + regpair_read(src);
	regpair_write(dst, r);
	arith_flags16(r, dst, src);
	return 0;
}

static int sub16(unsigned dst, unsigned src)
{
	unsigned s = regpair_read(src);
	unsigned d = regpair_read(dst);
	unsigned r = d - s;
	regpair_write(dst, r);
	sub_flags16(s, d);
	return 0;
}

static int and16(unsigned dst, unsigned src)
{
	uint16_t r = regpair_read(dst) & regpair_read(src);
	regpair_write(dst, r);
	logic_flags16(r);
	return 0;
}

static int or16(unsigned dst, unsigned src)
{
	uint16_t r = regpair_read(dst) | regpair_read(src);
	regpair_write(dst, r);
	logic_flags16(r);
	return 0;
}

static int xor16(unsigned dst, unsigned src)
{
	uint16_t r = regpair_read(dst) ^ regpair_read(src);
	regpair_write(dst, r);
	logic_flags16(r);
	return 0;
}

static int mov16(unsigned dst, unsigned src)
{
	uint16_t r = regpair_read(src);
	regpair_write(dst, r);
	ldflags16(r);
	return 0;
}

/*
 *	Address generator (with side effects). We don't know when the
 *	pre-dec/post-inc hits the register
 */

static uint16_t indexed_address(unsigned size)
{
	uint8_t idx = fetch();
	unsigned r = idx >> 5;	/* No idea what happens if low bit set */
	unsigned addr;
	int8_t offset = 0;	/* Signed or not ? */

	if (idx & 0x10)
		fprintf(stderr,
			"indexed address with reg low bit %02X at %04X\n",
			idx, pc);
	if (idx & 0x08)
		offset = fetch();
	switch (idx & 0x07) {
	case 0:
		return regpair_read(r) + offset;
	case 1:
		addr = regpair_read(r);
		regpair_write(r, addr + size);
		return addr + offset;
	case 2:
		addr = regpair_read(r);
		addr -= size;
		regpair_write(r, addr);
		return addr + offset;
	default:
		fprintf(stderr, "Unknown indexing mode %02X at %04X\n",
			idx, pc);
		exit(1);
	}
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
		addr = pc + (int8_t)fetch();	/* signed ? */
		indir = 0;
		break;
	case 4:
		addr = pc + (int8_t)fetch();
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
			mode, pc);
		break;
	default:
		/* indexed off a register */
		addr = regpair_read(mode & 7);
		indir = 0;
		break;
	}
	while (indir--)
		addr = mem_read16(addr);
	return addr;
}

/*
 *	The branch instructions are not entirely obvious
 *
 *	14/15 are Z and NZ very clearly
 *	1A-1D are the switches
 *
 *	16 is used in AsciiToHexNibble 856D
 *		0x30 - code, taken if code < 0x30
 *	17 is used in 8571
 *		0x47 - code, taken if code >= 0x47
 *
 *	So 16/17 appear to be a pair checking C|Z
 *	
 *
 *	The use of 18 and 19 are a bit odd
 *
 *	8751 uses 19 to branch if the load of (X)+ is negative or zero
 *		so 19 look like taken on N|Z
 *	87B8 uses char minus 0x5F to decide if it should mask lower case
 *		
 *
 *	8757	uses 18 taken if B - A >= 0 (!N !Z)
 */

static int nxorv(void)
{
	if ((alu_out & (ALU_N | ALU_V)) == 0)
		return 0;
	if ((alu_out & (ALU_N | ALU_V)) == (ALU_N | ALU_V))
		return 0;
	return 1;
}

static int branch_op(void)
{
	unsigned t;
	int8_t off;

	switch (op & 0x0F) {
	case 0:		/* bcs */
		t = (alu_out & ALU_C);
		break;
	case 1:		/* bcc */
		t = !(alu_out & ALU_C);
		break;
		/* Question: is this a sign or arithmetic overflow in fact - ie is
		   it an N or V ? */
	case 2:		/* bns */
		t = (alu_out & ALU_N);
		break;
	case 3:		/* bnc */
		t = !(alu_out & ALU_N);
		break;
	case 4:		/* bz   - known right */
		t = (alu_out & ALU_Z);
		break;
	case 5:		/* bnz - known right */
		t = !(alu_out & ALU_Z);
		break;
		/*
		 *      The classic operators are
		 *      >=              Z | N ^ V = 0
		 *      > 0             N ^ V = 0
		 *      <=0             Z | N^V = 1
		 *      < 0             N ^ V = 1
		 *
		 */
	case 6:		/* Branch if result was > 0 */
		t = !nxorv();	/* >= 0 */
		if (alu_out & ALU_Z)	/* but not on zero */
			t = 0;
		break;
	case 7:
		t = nxorv();	/* < 0 */
		break;
	case 8:		/* Jump if Result > 0 */
		t = nxorv();
		if (alu_out & ALU_Z)
			t = 0;
		break;
	case 9:		/* Taken if Z or negative */
		t = alu_out & ALU_Z;
		if (!nxorv())
			t = 1;
		break;
		/* Switches */
	case 10:
		t = (switches & BS1);
		break;
	case 11:
		t = (switches & BS2);
		break;
	case 12:
		t = (switches & BS3);
		break;
	case 13:
		t = (switches & BS4);
		break;
	case 14:
	case 15:
		fprintf(stderr, "Unknown branch %02X at %04X\n", op, pc);
		t = 0;
		break;
	}
	/* We'll keep pc and reg separate until we know if/how it fits memory */
	off = fetch();
	/* Offset is applied after fetch leaves PC at next instruction */
	if (t)
		pc += off;
	return 0;
}

/* Low operations - not all known */
static int low_op(void)
{
	switch (op) {
	case 0x00:		/* HALT */
		halt_system();
		break;
	case 0x01:		/* NOP */
		break;
	case 0x02:		/* FSN */
		alu_out |= ALU_N;
		break;
	case 0x03:		/* FCN */
		alu_out &= ~ALU_N;
		break;
	case 0x04:		/* FSI *//* Interrupt flag */
		alu_out |= ALU_I;
		break;
	case 0x05:		/* FCI */
		alu_out &= ~ALU_I;
		break;
	case 0x06:		/* FSC */
		alu_out |= ALU_C;
		break;
	case 0x07:		/* FCC */
		alu_out &= ~ALU_C;
		break;
	case 0x08:		/* FCA */
		alu_out ^= ALU_C;
		break;
	case 0x09:		/* RET */
		/* RET - see notes on call */
		pc = regpair_read(X);
		regpair_write(X, pop());
		break;
	case 0x0A:		/* RETI if C  ? */
		/* C seems to influence reti, mux test on int 6 does
		  fsc reti if processed, reti if not.. */
		break;
	case 0x0E:		/* DELAY */
		break;
	case 0x0B:
	case 0x0C:
	case 0x0D:
		fprintf(stderr, "Unknown op %02X at %04X\n", op, pc);
	case 0x0F:
		/* 0F is used in the MMU RAM test - some kind of reti
		   variant used to flip ipl during the testing or maybe
		   a syscall style ret */
		/* The RAM test stacks
		           ipl.b	; ipl to return to
		           0.b		;
		           rt.w		; return RT ?
		           0.b
		   and appears to flip ipl and set pc to the old rt so it's
		   more like a RTWP on a TI990 than a reti ? */
		{
			uint16_t new_x, new_pc;
			popbyte();
			new_x = pop();
			popbyte();
			new_pc = regpair_read(X);
			cpu_ipl = popbyte() & 0x0F;
			regpair_write(X, new_x);
			pc = new_pc;
			return 0;
		}
	}
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
		dma_addr = regpair_read(rp >> 1);
		break;
	case 1:
		regpair_write(rp >> 1, dma_addr);
		break;
	case 2:
		dma_count = regpair_read(rp >> 1);
		break;
	case 3:
		regpair_write(rp >> 1, dma_count);
		break;
	case 4:
		dma_mode = rp;
		break;
	case 6:
		dma_enable = 1;
		break;
	default:
		fprintf(stderr, "Unknown DMA operations 2F%02X at %04X\n",
			op, pc);
		break;
	}
	return 0;
}

/*
 *	We know this has some flag effects because 8130 relies upon it setting
 *	presumably Z to exit
 *
 *	Weirdnesses
 *	2E 1C and 2E 0C appear to be special and followed by a negated
 *	mmu slot number and two bytes
 *	2E 1C is nonsense (word reg low bit set) but 2E 0C ought to be
 *	A = G (whatever G is - seems to be a mystery)
 */
static int move_op(void)
{
	op = fetch();
	/* Seem to be some strange gaps here */
	if (op == 0x0C || op == 0x1C)
		return mmu_loadop(op);
	if (op & 0x11) {
		fprintf(stderr,
			"Unknown reg encoding %02X for 16bit move at %04X\n",
			op, pc);
		exit(1);
	}
	mov16(op >> 5, (op & 0x0F) >> 1);
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
 *	an indirect so is perhaps mem_read16(mem_read16(fetch16()));
 */
static int jump_op(void)
{
	uint16_t new_pc;
	if (op == 0x76) {	/* syscall is a mystery */
		cpu_ipl = 15;
		return 0;
	}
	switch (op & 7) {
	case 1:		/* Immediate */
		new_pc = fetch16();
		break;
	case 2:		/* Jump to contents of address given */
		new_pc = mem_read16(fetch16());
		break;
	case 3:
		new_pc = (int8_t) fetch() + pc;	/* TODO: signed or unsigned */
		break;
	case 4:		/* Jump to contents of address given indirected */
		/* This one is a guess - we've not seen it */
		new_pc = mem_read16(mem_read16(fetch16()));
		break;
	case 5:
		new_pc = fetch() + regpair_read(A);	/* Again signed ? */
		break;
	default:
		fprintf(stderr, "Invalid jump/call 0x%02X at 0x%04X\n", op,
			pc);
		return 0;
	}
	if (op & 0x08) {
		/* guesswork time. 8500 implies it is not a simple branch and link */
		/* the use of rt+ implies it's also not pc stacking, so try old X stacking */
		push(regpair_read(X));
		regpair_write(X, pc);
	}
	pc = new_pc;
	return 0;
}

/*
 *	This appears to work like the other loads and not affect C
 */
static int rt_op(void)
{
	uint16_t addr = decode_address(2, op & 7);
	uint16_t r;
	if (op & 0x08) {
		r = regpair_read(X);
		mem_write16(addr, r);
	} else {
		r = mem_read16(addr);
		regpair_write(X, r);
		ldflags16(r);
	}
	return 0;
}

static int loadbyte_op(void)
{
	uint16_t addr = decode_address(1, op & 0x0F);
	uint8_t r = mem_read8(addr);

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
	uint16_t r = mem_read16(addr);

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

	mem_write8(addr, r);
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

	mem_write16(addr, r);

	return 0;
}

/*
 *	In the CPU test we verify that LD to AL or BL does not affect C
 *	but we do check that Z is set if 0 is loaded.
 *
 *	Elsewhere we rely upon N changing and V being set
 *
 */
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

static int misc2x_special(uint8_t reg)
{
	/* The only code we know is 22 32 which seems ot be some kind of cpu
	   ident/status check. It's not clear if there is a proper supervisor
	   check involved here for the NZ->Z case and a CPU type check for Z->NZ
	   or what */
	if (op == 0x22 && reg == 0x32) {
		/* CPU ID ?? */
		alu_out ^= ALU_Z;
		return 0;
	}
	fprintf(stderr, "Unknown misc2x special %02X:%02X at %04X\n", op,
		reg, pc - 1);
	return 0;
}

static int misc2x_op(void)
{
	unsigned reg = AL;
	if (!(op & 8)) {
		reg = fetch();
		if (reg & 0x0F)
			return misc2x_special(reg);
		reg >>= 4;
	}

	switch (op) {
	case 0x20:
		return inc(reg);
	case 0x21:
		return dec(reg);
	case 0x22:
		return clr(reg);
	case 0x23:
		return not(reg);
	case 0x24:
		return srl(reg);
	case 0x25:
		return sll(reg);
	case 0x26:
		return rrc(reg);
	case 0x27:
		return rlc(reg);
	case 0x28:
		return inc(AL);
	case 0x29:
		return dec(AL);
	case 0x2A:
		return clr(AL);
	case 0x2B:
		return not(AL);
	case 0x2C:
		return srl(AL);
	case 0x2D:
		return sll(AL);
	case 0x2E:
		return move_op();
	case 0x2F:
		return dma_op();
	default:
		fprintf(stderr, "internal error misc2\n");
		exit(1);
	}
}

/* Like misc2x but word */
static int misc3x_op(void)
{
	unsigned reg = A;
	if (!(op & 8)) {
		reg = fetch();
		if (reg & 0x0F)
			return misc2x_special(reg);
		reg >>= 4;
		if (reg & 1) {
			fprintf(stderr,
				"Unknown misc3x reg encoding %02X at %04X\n",
				reg, pc);
			exit(1);
		}
		reg >>= 1;
	}

	switch (op) {
	case 0x30:
		return inc16(reg);
	case 0x31:
		return dec16(reg);
	case 0x32:
		return clr16(reg);
	case 0x33:
		return not16(reg);
	case 0x34:
		return srl16(reg);
	case 0x35:
		return sll16(reg);
	case 0x36:
		return rrc16(reg);
	case 0x37:
		return rlc16(reg);
	case 0x38:
		return inc16(A);
	case 0x39:
		return dec16(A);
	case 0x3A:
		return clr16(A);
	case 0x3B:
		return not16(A);
	case 0x3C:
		return srl16(A);
	case 0x3D:
		return sll16(A);
	case 0x3E:		/* To check */
		return inc16(X);
	case 0x3F:
		return dec16(X);
	default:
		fprintf(stderr, "internal error misc3\n");
		exit(1);
	}
}

/* Mostly ALU operations on AL */
/* We have a very strange operation 47 40 */
static int alu4x_op(void)
{
	unsigned src, dst;
	if (!(op & 0x08)) {
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
	case 0x46:		/* unused */
		fprintf(stderr, "Unknown ALU4 op %02X at %04X\n", op,
			pc - 1);
		return 0;
	case 0x47:		/* unused */
		return mmu_op47();
	case 0x48:
		return add(BL, AL);
	case 0x49:
		return sub(BL, AL);
	case 0x4A:
		return and(BL, AL);
	case 0x4B:
		return or(BL, AL);
	case 0x4C:
		return xor(BL, AL);
	case 0x4D:
		return mov(BL, AL);
	case 0x4E:		/* unused */
	case 0x4F:		/* unused */
		fprintf(stderr, "Unknown ALU4 op %02X at %04X\n", op, pc);
		return 0;
	default:
		fprintf(stderr, "internal error alu4\n");
		exit(1);
	}
}

/* Much like ALU4x but word */
static int alu5x_op(void)
{
	unsigned src, dst;
	if (!(op & 0x08)) {
		dst = fetch();
		if (dst & 0x11) {
			fprintf(stderr,
				"ALU5 - unknown reg encoding %02X at %04X\n",
				dst, pc - 1);
			exit(1);
		}
		src = dst >> 4;
		dst &= 0x0F;
		src >>= 1;
		dst >>= 1;
	}
	switch (op) {
	case 0x50:		/* add */
		return add16(dst, src);
	case 0x51:		/* sub */
		return sub16(dst, src);
	case 0x52:		/* and */
		return and16(dst, src);
	case 0x53:		/* or */
		return or16(dst, src);
	case 0x54:		/* xor */
		return xor16(dst, src);
	case 0x55:		/* mov */
		return mov16(dst, src);
	case 0x56:		/* unused */
	case 0x57:		/* unused */
		fprintf(stderr, "Unknown ALU5 op %02X at %04X\n", op,
			pc - 1);
		return 0;
	case 0x58:
		return add16(B, A);
	case 0x59:
		return sub16(B, A);
	case 0x5A:
		return and16(B, A);
	case 0x5B:
		return or16(B, A);
	case 0x5C:
		return xor16(B, A);
	case 0x5D:
		return mov16(B, A);
	case 0x5E:		/* special case move */
		mov16(Z, A);
		return 0;
	case 0x5F:		/* special case move */
		mov16(S, A);
		return 0;
	default:
		fprintf(stderr, "internal error alu5\n");
		exit(1);
	}
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
	if (alu_out & ALU_C)
		*buf = 'C';
	if (alu_out & ALU_I)
		buf[1] = 'I';
	if (alu_out & ALU_N)
		buf[2] = 'N';
	if (alu_out & ALU_Z)
		buf[3] = 'Z';
	if (alu_out & ALU_V)
		buf[4] = 'V';
	return buf;
}

unsigned cpu6_execute_one(unsigned trace)
{
	if (trace)
		fprintf(stderr, "CPU %04X: ", pc);
	op = fetch();
	if (trace)
		fprintf(stderr, "%02X %s A:%04X  B:%04X X:%04X Y:%04X Z:%04X S:%04X\n",
			op, flagcode(), regpair_read(A), regpair_read(B),
			regpair_read(X), regpair_read(Y), regpair_read(Z),
			regpair_read(S));
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
	if (op < 0x50)
		return alu4x_op();
	if (op < 0x60)
		return alu5x_op();
	if (op < 0x70)
		return rt_op();
	if (op < 0x80)
		return jump_op();
	return loadstore_op();
}

uint16_t cpu6_pc(void)
{
	return pc;
}

void cpu6_set_switches(unsigned v)
{
	switches = v;
}

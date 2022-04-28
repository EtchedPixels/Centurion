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

static uint8_t cpu_ipl = 0;
static uint16_t pc = 0xFC00;
static uint8_t op;
static uint8_t alu_out;
static uint8_t switches = 0xF0;
static uint8_t int_enable;
static unsigned halted;
static unsigned pending_ipl;

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
	if (dma_enable) {
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
	r = mem_read8(dma_addr++);
	dma_count++;
	if (dma_count == 0)
		dma_enable = 0;
	return r;
}

/*
 *	We don't know how the flags are packed into C with the IPL
 *	but they appear to live in the low 4 bits and the lv in the
 *	low 4 bits of the upper byte
 */

#define ALU_L	1
#define ALU_M	2
#define ALU_F	4
#define ALU_V	8


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

static uint8_t reg_read(uint8_t r)
{
	return mem_read8((cpu_ipl << 4) | r);
}

static void reg_write(uint8_t r, uint8_t v)
{
	mem_write8((cpu_ipl << 4) | r, v);
}

/*
 *	This needs some updating if CPU6 behaves like CPU4. On the EE200
 *	a word register specified with an odd value gives you the upper byte
 *	twice.
 */
static uint16_t regpair_read(uint8_t r)
{
	if (r > 15) {
		fprintf(stderr, "Bad regpair encoding %02X %02X %04X\n", op, r,
			pc);
		exit(1);
	}
	return (reg_read((r | 1)^ 1) << 8) | reg_read((r ^ 1));
}

static void regpair_write(uint8_t r, uint16_t v)
{
	if (r > 15) {
		fprintf(stderr, "Bad regpair encoding %02X %04X\n", op,
			pc);
		exit(1);
	}
	reg_write((r | 1) ^ 1 , v >> 8);
	reg_write((r ^ 1), v);
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
		alu_out |= ALU_V;
	return 0;
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
static void arith_flags(unsigned r, uint8_t d)
{
	alu_out &= ~(ALU_F | ALU_M | ALU_V);
	if ((r & 0xFF) == 0)
		alu_out |= ALU_V;
	if (r & 0x80)
		alu_out |= ALU_M;
	if ((r ^ d) & 0x80)
		alu_out |= ALU_V;
}

/*
 *	Subtract is similar but the overflow rule probably differs and
 *	L is a borrow not a carry
 */
static void sub_flags(uint8_t r, uint8_t d)
{
	alu_out &= ~(ALU_F | ALU_M | ALU_V);
	if ((r & 0xFF) == 0)
		alu_out |= ALU_V;
	if (r & 0x80)
		alu_out |= ALU_M;
	if ((r ^ d) & 0x80)
		alu_out |= ALU_F;
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
	if ((r & 0xFFFF) == 0)
		alu_out |= ALU_V;
	if (c)
		alu_out |= ALU_L;
	if (r & 0x8000)
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
static void arith_flags16(unsigned r, uint16_t d)
{
	alu_out &= ~(ALU_F | ALU_M | ALU_V);
	if ((r & 0xFFFF) == 0)
		alu_out |= ALU_V;
	if (r & 0x8000)
		alu_out |= ALU_M;
	/* If the result is negative but both inputs were positive then
	   we overflowed */
	if ((r ^ d) & 0x8000)
		alu_out |= ALU_F;
}

/*
 *	Subtract is similar but the overflow rule probably differs and
 *	L is a borrow not a carry
 */
static void sub_flags16(uint16_t r, uint16_t d)
{
	alu_out &= ~(ALU_F | ALU_M | ALU_V);
	if ((r & 0xFFFF) == 0)
		alu_out |= ALU_V;
	if (r & 0x8000)
		alu_out |= ALU_M;
	if ((r ^ d) & 0x8000)
		alu_out |= ALU_F;
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
 *	Arithmetic flag but not link
 */
static int inc(unsigned reg)
{
	uint8_t r = reg_read(reg);
	reg_write(reg, r + 1);
	arith_flags(r + 1, r);
	return 0;
}

/*
 *	Does not affect L either
 */
static int dec(unsigned reg)
{
	uint8_t r = reg_read(reg);
	reg_write(reg, r - 1);
	sub_flags(r - 1, r);
	return 0;
}

static int clr(unsigned reg)
{
	reg_write(reg, 0);
	alu_out &= ~(ALU_F|ALU_L|ALU_M);
	alu_out |= ALU_V;
	return 0;
}

/*
 *	Sets all the flags but rule not clear
 */
static int not(unsigned reg)
{
	uint8_t r = ~reg_read(reg);
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

	while(count--) {
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

	while(count--) {
		v = r << 1;
		shift_flags((r & 0x80), v);
		alu_out &= ~ALU_F;
		/* So annoying C lacks a ^^ operator */
		switch(alu_out & (ALU_L | ALU_M)) {
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

	while(count--) {
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

	while(count--) {
		c = r & 0x80;
		r <<= 1;
		r |= (alu_out & ALU_L) ? 1 : 0;

		shift_flags(c, r);
		alu_out &= ~ALU_F;
		/* So annoying C lacks a ^^ operator */
		switch(alu_out & (ALU_L | ALU_M)) {
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
	uint16_t r = d + reg_read(src);
	reg_write(dst, r);
	arith_flags(r, d);
	alu_out &= ~ALU_L;
	if (r & 0x100)
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
	unsigned r = d - s;
	reg_write(dst, r);
	sub_flags(s - d, d);
	alu_out &= ~ALU_L;
	if (d < s)
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

static int inc16(unsigned reg)
{
	uint16_t r = regpair_read(reg);
	regpair_write(reg, r + 1);
	arith_flags16(r + 1, r);
	return 0;
}

static int dec16(unsigned reg)
{
	uint16_t r = regpair_read(reg);
	regpair_write(reg, r - 1);
	sub_flags16(r - 1, r);
	return 0;
}

/* Assume behaviour matches CLR */
static int clr16(unsigned reg)
{
	regpair_write(reg, 0);
	alu_out &= ~(ALU_F|ALU_L|ALU_M);
	alu_out |= ALU_V;
	return 0;
}

static int not16(unsigned reg)
{
	uint16_t r = ~regpair_read(reg);
	regpair_write(reg, r);
	logic_flags16(r);
	return 0;
}

static int sra16(unsigned reg, unsigned count)
{
	uint16_t v;
	uint16_t r = regpair_read(reg);

	while(count--) {
		v = r >> 1;
		if (v & 0x4000)
			v |= 0x8000;
		shift_flags16(r & 1, v);
		r = v;
	}
	regpair_write(reg, r);
	return 0;
}

static int sll16(unsigned reg, unsigned count)
{
	uint16_t v;
	uint16_t r = regpair_read(reg);

	while(count--) {
		v = r << 1;
		shift_flags16((r & 0x8000), v);
		alu_out &= ~ALU_F;
		/* So annoying C lacks a ^^ operator */
		switch(alu_out & (ALU_L | ALU_M)) {
		case ALU_L:
		case ALU_M:
			alu_out |= ALU_F;
			break;
		}
		r = v;
	}
	regpair_write(reg, r);
	return 0;
}

static int rrc16(unsigned reg, unsigned count)
{
	uint16_t r = regpair_read(reg);
	uint16_t c;

	while(count--) {
		c= r & 1;

		r >>= 1;
		r |= (alu_out & ALU_L) ? 0x8000 : 0;

		shift_flags16(c, r);
	}
	regpair_write(reg, r);
	return 0;
}

static int rlc16(unsigned reg, unsigned count)
{
	uint16_t r = regpair_read(reg);
	uint16_t c;

	while(count--) {
		c = r & 0x8000;

		r <<= 1;
		r |= (alu_out & ALU_L) ? 1 : 0;

		shift_flags16(c, r);
		alu_out &= ~ALU_F;
		/* So annoying C lacks a ^^ operator */
		switch(alu_out & (ALU_L | ALU_M)) {
		case ALU_L:
		case ALU_M:
			alu_out |= ALU_F;
			break;
		}
	}
	regpair_write(reg, r);
	return 0;
}

static int add16(unsigned dst, unsigned src)
{
	unsigned d = regpair_read(dst);
	unsigned r = d + regpair_read(src);
	regpair_write(dst, r);
	arith_flags16(r, d);
	alu_out &= ~ALU_L;
	if (r & 0x10000)
		alu_out |= ALU_L;
	return 0;
}

static int sub16(unsigned dst, unsigned src)
{
	unsigned s = regpair_read(src);
	unsigned d = regpair_read(dst);
	unsigned r = d - s;
	regpair_write(dst, r);
	sub_flags16(s - d, d);
	alu_out &= ~ALU_L;
	if (d < s)
		alu_out |= ALU_L;
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
	logic_flags16(r);
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
		addr =  addr + offset;
		break;
	default:
		fprintf(stderr, "Unknown indexing mode %02X at %04X\n",
			idx, pc);
		exit(1);
	}
	if (idx & 0x04)
		addr = mem_read16(addr);
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
		addr = regpair_read((mode & 7) << 1);
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

static int branch_op(void)
{
	unsigned t;
	int8_t off;

	switch (op & 0x0F) {
	case 0:		/* BL	Branch if link is set */
		t = (alu_out & ALU_L);
		break;
	case 1:		/* BNL	Branch if link is not set */
		t = !(alu_out & ALU_L);
		break;
	case 2:		/* BF 	Branch if fault is set */
		t = (alu_out & ALU_F);
		break;
	case 3:		/* BNF	Branch if fault is not set */
		t = !(alu_out & ALU_F);
		break;
	case 4:		/* BZ	Branch if zero */
		t = (alu_out & ALU_V);
		break;
	case 5:		/* BNZ	Branch if non zero */
		t = !(alu_out & ALU_V);
		break;
	case 6:		/* BM	Branch if minus */
		t = alu_out & ALU_M;
		break;
	case 7:		/* BP	Branch if plus */
		t = !(alu_out & ALU_M);
		break;
	case 8:		/* BGZ	Branch if greater than zero */
		/* Branch if both M and V are zero */
		t = !(alu_out & (ALU_M | ALU_V));
		break;
	case 9:		/* BLE	Branch if less than or equal to zero */
		t = alu_out & (ALU_M | ALU_V);
		break;
	case 10:	/* BS1  */
		t = (switches & BS1);
		break;
	case 11:	/* BS2 */
		t = (switches & BS2);
		break;
	case 12:	/* BS3 */
		t = (switches & BS3);
		break;
	case 13:	/* BS4 */
		t = (switches & BS4);
		break;
	case 14:	/* BTM - branch on teletype mark - CPU4 only ? */
		t = 0;
		break;
	case 15:	/* BEP - branch on even parity - CPU4 only ? */
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

/* Some of this is not clear */
static void reactivate_pri(void)
{
	/* Save flags */
	uint16_t c = regpair_read(C);
	c &= 0x0F;
	c |= alu_out;
	regpair_write(C, c);
	/* Saved IPL */
	cpu_ipl = (c >> 8) & 0x0F;
	pc = regpair_read(P);
	alu_out = regpair_read(C) & 0x0F;
}

/* Low operations - not all known */
static int low_op(void)
{
	switch (op) {
	case 0x00:		/* HALT */
		halted = 1;
		break;
	case 0x01:		/* NOP */
		break;
	case 0x02:		/* SF	Set Fault */
		alu_out |= ALU_F;
		break;
	case 0x03:		/* RF	Reset Fault */
		alu_out &= ~ALU_F;
		break;
	case 0x04:		/* EI	Enable Interrupts */
		int_enable = 1;
		break;
	case 0x05:		/* DI	Disable Interrupts */
		int_enable = 0;
		break;
	case 0x06:		/* SL	Set Link */
		alu_out |= ALU_L;
		break;
	case 0x07:		/* RL	Clear Link */
		alu_out &= ~ALU_L;
		break;
	case 0x08:		/* CL	Complement Link */
		alu_out ^= ALU_L;
		break;
	case 0x09:		/* RSR	Return from subroutine */
		pc = regpair_read(X);
		regpair_write(X, pop());
		break;
	case 0x0A:		/* RI	Return from interrupt */
		/* This may differ a bit on the CPU6 seems to have a carry
		   involvement */
		regpair_write(P, pc);
	case 0x0B:		/* RIM	Return from interrupt modified */
		reactivate_pri();
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
	 *		-- Ken Romain
	 */
	case 0x0E:		/* DELAY */
		break;
	case 0x0F:
		/* CPU6 specific ? */
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
	/* Guesswork at this point */
	fprintf(stderr, "Unknown 0x2E op %02X at %04X\n", op, pc);
	mov16(op >> 4, op & 0x0F);
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
		uint8_t old_ipl = cpu_ipl;
		cpu_ipl = 15;
		/* Unclear if this also occurs */
		reg_write(CH, old_ipl);
		return 0;
	}
	/* We don't know what 0x70 does (it's invalid but I'd guess it jumps
	   to the following byte */
	new_pc = decode_address(2, op & 0x07);
	if (op & 0x08) {
		/* guesswork time. 8500 implies it is not a simple branch and link */
		/* the use of rt+ implies it's also not pc stacking, so try old X stacking */
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
 *	It also seems (see diagnostics) not to affect the other flags unlike
 *	an A or B load. That makes sense given you want to pass flags
 *	and often finish with LD RT,(SP)+, RET which would trash them
 */
static int rt_op(void)
{
	/* Valid modes 0-5 */
	uint16_t addr = decode_address(2, op & 7);
	uint16_t r;
	if (op & 0x08) {
		r = regpair_read(X);
		mem_write16(addr, r);
		/* FIXME: FLAGS */
	} else {
		r = mem_read16(addr);
		regpair_write(X, r);
		/* FIXME: FLAGS */
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

	mem_write16(addr, r);
	ldflags16(r);

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

static int misc2x_special(uint8_t reg)
{
	/* The only code we know is 22 32 which seems ot be some kind of cpu
	   ident/status check. It's not clear if there is a proper supervisor
	   check involved here for the NZ->Z case and a CPU type check for Z->NZ
	   or what */
	if (op == 0x22 && reg == 0x32) {
		/* CPU ID ?? */
		alu_out ^= ALU_V;
		return 0;
	}
	fprintf(stderr, "Unknown misc2x special %02X:%02X at %04X\n", op,
		reg, pc - 1);
	return 0;
}

static int misc2x_op(void)
{
	unsigned count = 1;
	unsigned reg = AL;
	if (!(op & 8)) {
		reg = fetch();
		count = (reg & 0x0F) + 1;
		/* Hack until we understand what 22 32 is about */
		if (op == 0x22 && (reg & 0x0F))
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
		return sra(reg, count);
	case 0x25:
		return sll(reg, count);
	case 0x26:
		return rrc(reg, count);
	case 0x27:
		return rlc(reg, count);
	case 0x28:
		return inc(AL);
	case 0x29:
		return dec(AL);
	case 0x2A:
		return clr(AL);
	case 0x2B:
		return not(AL);
	case 0x2C:
		return sra(AL, 1);
	case 0x2D:
		return sll(AL, 1);
	/* On CPU 4 these would be inc XL/dec XL but they are not present and
	   X is almost always handled as a 16bit register only */
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
	unsigned count  = 0;
	unsigned reg = A;
	if (!(op & 8)) {
		reg = fetch();
		count = (reg & 0x0F) + 1;
		reg >>= 4;
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
		return sra16(reg, count);
	case 0x35:
		return sll16(reg, count);
	case 0x36:
		return rrc16(reg, count);
	case 0x37:
		return rlc16(reg, count);
	case 0x38:
		return inc16(A);
	case 0x39:
		return dec16(A);
	case 0x3A:
		return clr16(A);
	case 0x3B:
		return not16(A);
	case 0x3C:
		return sra16(A, 1);
	case 0x3D:
		return sll16(A, 1);
	case 0x3E:
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
	case 0x5B:	/* Microcode and 86D7 suggest not an or16 */
		/* return or16(B, A); */
		return mov16(X, A);
	/* These are borrowed for moves */
	case 0x5C:
		return mov16(Y, A);
	case 0x5D:
		return mov16(B, A);
	case 0x5E:
		return mov16(Z, A);
	case 0x5F:
		return mov16(S, A);
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
	unsigned old_ipl;

	if (int_enable == 0)
		return;
	if (pending_ipl > cpu_ipl) {
		old_ipl = cpu_ipl;
		halted = 0;
		regpair_write(P, pc);
		reg_write(CL, alu_out);
		cpu_ipl = pending_ipl;
		reg_write(CH, old_ipl);
		pc = regpair_read(P);
		alu_out = reg_read(CL);
		if (trace)
			fprintf(stderr, "Interrupt %X: New PC = %04X, previous IPL %X\n",
				cpu_ipl, pc, old_ipl);
	}
}

unsigned cpu6_execute_one(unsigned trace)
{
	cpu6_interrupt(trace);
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

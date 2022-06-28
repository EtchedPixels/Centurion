#include <assert.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include "centurion.h"
#include "console.h"
#include "cpu6.h"
#include "mux.h"

struct MuxUnit mux[NUM_MUX_UNITS];
 // 0 disables interrupts
 // LOAD writes 0 to F20A before transferring execution to a new binary
static unsigned char rx_ipl_request = 0;
static unsigned char tx_ipl_request = 0;
static int irq_cause = -1;
static uint32_t poll_count = 0;

// Set the initial state for all out ports
void mux_init(void)
{
	int i;

	for (i = 0; i < NUM_MUX_UNITS; i++) {
		mux[i].in_fd = -1;
		mux[i].out_fd = -1;
		mux[i].status = MUX_TX_READY;
		mux[i].lastc = 0xFF;
		mux[i].baud = 9600;
		mux[i].rx_ready_time = 0;
        mux[i].tx_done_time = 0;
	}
}

void mux_attach(unsigned unit, int in_fd, int out_fd)
{
	mux[unit].in_fd = in_fd;
	mux[unit].out_fd = out_fd;
}

/* Utility functions for the mux */
static unsigned int next_char(uint8_t unit)
{
	int r;
	unsigned char c;


	if (mux[unit].in_fd == -1) {
		return mux[unit].lastc;
	}

	r = read(mux[unit].in_fd, &c, 1);

	if (r == 0) {
		emulator_done = 1;
		return mux[unit].lastc;
	}

	if (r < 0) {
		/* Someone read the port when nothing there */
		if (errno == EAGAIN || errno == EWOULDBLOCK) {
			return mux[unit].lastc;
		}
		exit(1);
	}

	if (c == 0x7F) {
		/* Some terminals (like Cygwin) send DEL on Backspace */
		c = 0x08;
	}


	mux[unit].lastc = c;

	return c;
}

static int mux_assert_irq(unsigned unit, unsigned reason, unsigned trace)
{
	char ipl = reason ? tx_ipl_request : rx_ipl_request;

	if (!ipl)
		return 0;

	if (trace && irq_cause != (unit << 1 | reason))
		fprintf(stderr, "MUX%i: %s IRQ raised\n", unit, reason ? "TX" : "RX");

	// Cause is actually the lower 8 bits of unit that caused the interrupt
	// Though, TX interrupts have the lower bit set
	irq_cause = (unit << 1) | reason;
	cpu_assert_irq(ipl);

	return 1;
}

static void mux_unit_send(unsigned unit, uint8_t val) {
	if (!(mux[unit].status & MUX_TX_READY)) {
		fprintf(stderr, "Write to busy mux port\n");
	}
	mux[unit].status &= ~MUX_TX_READY;
	uint64_t symbol_time = (1000000000.0 / (double)mux[unit].baud);

	// it takes time for the send to complete
	mux[unit].tx_done_time = get_current_time() + (symbol_time * 10);

	if (mux[unit].out_fd == -1) {
		/* This MUX unit isn't connected to anything */
		return;
	}

	if (mux[unit].out_fd > 1) {
		val &= 0x7F;
		write(mux[unit].out_fd, &val, 1);
	} else {
		val &= 0x7F;
		if (val == 0x06) /* Cursor one position right */
			printf("\x1b[1C");
		else if (val != 0x08 && val != 0x0A && val != 0x0D
		    && (val < 0x20 || val == 0x7F))
			printf("[%02X]", val);
		else
			putchar(val);
		fflush(stdout);
	}
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
 *
 *	The upper half of the mux space appears to be controls
 *	0,1		MUX port 0
 *	2,3		MUX port 1
 *	4,5		MUX port 2
 *	6,7		MUX port 3
 *
 *	0A		Recv Interrupt level ?
 *  0B		Set to 0xe0 by OPSYS CRT driver initialization
 *	0E		Send Interrupt level?
 *			OPSYS sets this to the same value as 0A
 *	0F		read to check for interrupt - NZ = none
 *
 */

/* Bit 0 of control is char pending. The real system uses mark parity so
   we ignore that */
void mux_write(uint16_t addr, uint8_t val, uint32_t trace)
{
	unsigned card, unit, port, mode;

	// Decode address

	// Nibble 1 of the address is the card number
	// Each MUX4 board supports 4 ports.
	// There are apparently MUX8 cards, which might act as two cards?
	card = (addr >> 4) & 0xF;

	mode = addr & 0xf;
	if (mode > 7) {
		port = 0;
		unit = card * 4;
	} else {
		// Data or status
		mode &= 1;
		// Bits 1 and 2 are port
		port = (addr >> 1) & 0x3;
		unit = card * 4 + port;
	}

	if (unit > NUM_MUX_UNITS) {
		fprintf(stderr, "MUX%i: Write to disabled unit reg %x\n", unit, addr);
		return;
	}

	switch(mode) {
	case 0: // Status Reg
		if (trace)
			fprintf(stderr, "MUX%i: Status Write %x\n", unit, val);
		// TODO: Implement baud
		return;
	case 1: // Data Reg
		if (trace) {
			if ((val&0x7f) >= 0x20 && val != 0x7F && val != 0xff)
				fprintf(stderr, "MUX%i: Data Write %x ('%c')\n", unit, val, val & 0x7f);
			else
				fprintf(stderr, "MUX%i: Data Write %x\n", unit, val);
		}
		mux_unit_send(unit, val);
		return;
	case 0xA: // Set RX interrupt request level
		if (trace)
			fprintf(stderr, "MUX%i: RX level = %i\n", unit, val);
		rx_ipl_request = val;
		return;
	case 0xC:
		/* OPSYS kernel writes unit number (starting from 1)
		 * to this register and waits for the interrupt-driven write
		 * to complete. We suggest that this write forces a TX_READY interrupt
		 * on the given unit. Before doing so, the output routine actually
		 * waits for MUX_TX_READY bit to go high using a polled loop
		 */
		mux[val - 1].tx_done = 1;
		return;
	case 0xE: // Set TX interrupt request level
		if (trace)
			fprintf(stderr, "MUX%i: TX level = %i\n", unit, val);
		tx_ipl_request = val;
		return;
	case 8:
	case 0xB:
		// Written by CRT driver
		if (!trace)
			return; // Hide when not tracing
	default:
		fprintf(stderr, "\n%04X Write to unknown MUX register %x=%02x\n", cpu6_pc(), addr, val);
		return;
	}
}

uint8_t mux_read(uint16_t addr, uint32_t trace)
{
	unsigned card, port, unit, data, mode;

	data = 0;

	// It seems that all mux units share the same cause register via chaining
	if (addr == 0xf20f) {
		if (trace)
			fprintf(stderr, "MUX: InterruptCause Read: %02x\n",irq_cause);

		if (irq_cause & MUX_IRQ_TX) {
			// Reading this register is enough to clear the TX IRQ, but it seems
			// to not clear the RX IRQ, you actually have to read the data
			unsigned char unit = (irq_cause & MUX_UNIT_MASK) >> 1;
			mux[unit].tx_done = 0;

			if (trace)
				fprintf(stderr, "MUX%i: TX IRQ acknowledged\n", unit);
		}

		return irq_cause;
	}

	// Decode address

	// Nibble 1 of the address is the card number
	// Each MUX4 board supports 4 ports.
	// There are apparently MUX8 cards, which might act as two cards?
	card = (addr >> 4) & 0xF;


	mode = addr & 0xf;
	if (mode > 7) {
		unit = card*4;
	} else {
		// Bits 1 and 2 are port
		port = (mode >> 1) & 0x3;
		unit = card * 4 + port;
		mode = mode & 1;
	}

	if (unit > NUM_MUX_UNITS) {
		if (addr != 0xf20f)
			fprintf(stderr, "MUX%i: Read to disabled unit reg %x", unit, addr);
		return data;
	}

	switch (mode)
	{
	case 0x0: // Status register
		// Force CTS on
		data = mux[unit].status | MUX_CTS;
		if (trace)
			fprintf(stderr, "MUX%i: Status Read = %02x\n", unit, data);

		break;
	case 0x1:
		// Data register
		data = next_char(unit);
		mux[unit].status &= ~MUX_RX_READY;
		if (trace)
			fprintf(stderr, "MUX%i: Data Read = %02x ('%c')\n", unit, data, data);
		break;
	default:
		fprintf(stderr, "MUX%i: Unknown Register %x Read\n", unit, addr);
		break;
	}

	return data;
}

void mux_set_read_ready(unsigned unit, unsigned trace)
{
	assert(mux[unit].rx_ready_time == 0);

	// We need a delay here, otherwise interrupts would fire too fast.
	uint64_t symbol_time = (ONE_SECOND_NS / mux[unit].baud);
	mux[unit].rx_ready_time = get_current_time() + symbol_time * 10;
}

void mux_process_events(unsigned unit, unsigned trace) {
	uint64_t time = get_current_time();

	if (mux[unit].rx_ready_time && mux[unit].rx_ready_time <= time) {
		mux[unit].rx_ready_time = 0;
		mux[unit].status |= MUX_RX_READY;
		poll_count = 0;

		if (trace)
			fprintf(stderr, "MUX%i: RX_READY\n", unit);
	}

	if (mux[unit].tx_done_time && mux[unit].tx_done_time <= time) {
		mux[unit].tx_done_time = 0;
		mux[unit].status |= MUX_TX_READY;

		/* If a TX done interrupt is requested, it will be raised when the UART
			* switches from BUSY to READY state. The UART spends in READY state most
			* of the time, but the interrupt will eventually be acknowledged and
			* deasserted, so we store it as a separate status bit. On real HW it is
			* perhaps a part of the status register, but we don't know which one,
			* we haven't found any reads, so for now we keep it completely separate.
			*/
		if (tx_ipl_request)
			mux[unit].tx_done = 1;

		if (trace)
			fprintf(stderr, "MUX%i: TX_READY; TX_DONE = %d\n", unit, mux[unit].tx_done);
	}
}

void mux_poll(unsigned trace)
{
	int unit;

	for (unit = 0; unit < NUM_MUX_UNITS; unit++)
		mux_process_events(unit, trace);

	// Cheap speedhack, only check FDs sometimes
	if ((poll_count++ & 0xF) == 0)
		mux_poll_fds(mux, trace);

	cpu_deassert_irq(rx_ipl_request);
	cpu_deassert_irq(tx_ipl_request);

	/*
	 * Updates current IRQ state and chooses current irq_cause register value according to
	 * unit interrupt priorities. Each unit has two interrupts: RX and TX, and we enumerate
	 * them in order, starting from 0: RX0, TX0, RX1, TX1, etc. We consider the lowest
	 * number to have the highest priority, we aren't sure whether the real hardware
	 * does the same, but it's easy to reverse, if needed, by removing break statements.
	 */
	for (unit = 0; unit < NUM_MUX_UNITS; unit++) {
		if (mux[unit].status & MUX_RX_READY && mux_assert_irq(unit, MUX_IRQ_RX, trace))
			return;
		if (mux[unit].tx_done && mux_assert_irq(unit, MUX_IRQ_TX, trace))
			return;
	}

	if (trace && irq_cause >= 0)
		fprintf(stderr, "MUX: Last mux interrupt acknowledged\n");

	irq_cause = -1;
}

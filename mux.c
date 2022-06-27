#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include "centurion.h"
#include "console.h"
#include "cpu6.h"
#include "mux.h"

static struct MuxUnit mux[NUM_MUX_UNITS];
 // 0 disables interrupts
 // LOAD writes 0 to F20A before transferring execution to a new binary
static char rx_ipl_request = 0;
static char tx_ipl_request = 0;
static int irq_cause_unit = -1;
static int irq_cause_reason = -1;

// Set the initial state for all out ports
void mux_init(void)
{
	int i;

	for (i = 0; i < NUM_MUX_UNITS; i++) {
		mux[i].in_fd = -1;
		mux[i].out_fd = -1;
		mux[i].status = 0;
		mux[i].lastc = 0xFF;
		mux[i].baud = 9600;
		mux[i].tx_finish_time = 0;
		mux[i].rx_finish_time = 0;
	}
}

void mux_attach(unsigned unit, int in_fd, int out_fd)
{
	mux[unit].in_fd = in_fd;
	mux[unit].out_fd = out_fd;
}

/* Utility functions for the mux */
static unsigned int check_write_ready(uint8_t unit)
{
	int fd = mux[unit].out_fd;

	/* An unconnected port is always ready */
	return (fd == -1 || tty_check_writable(fd)) ? MUX_TX_READY : 0;
}

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

static void mux_assert_irq(unsigned unit, unsigned reason, unsigned trace)
{
	char ipl = reason ? tx_ipl_request : rx_ipl_request;
	if (!ipl)
		return;

	if (irq_cause_reason != -1)
		return; // Already pending interrupt

	int raised = cpu_assert_irq(ipl);
	if (raised) {
		irq_cause_unit = unit;
		irq_cause_reason = reason;

		if (trace && reason == MUX_IRQ_RX)
				fprintf(stderr, "MUX%i: RX IRQ raised\n", unit);

		if (trace && reason == MUX_IRQ_TX)
				fprintf(stderr, "MUX%i: TX IRQ raised\n", unit);
	}

}

static void mux_ack_irq(unsigned unit, unsigned reason, unsigned trace) {
	char ipl = reason ? tx_ipl_request : rx_ipl_request;
	if (!ipl)
		return;

	if (irq_cause_reason == reason && irq_cause_unit == unit) {
		irq_cause_unit = -1;
		irq_cause_reason = -1;
		cpu_deassert_irq(ipl);

		if (trace && reason == MUX_IRQ_RX)
			fprintf(stderr, "MUX%i: RX IRQ acknowledged\n", unit);

		if (trace && reason == MUX_IRQ_TX)
			fprintf(stderr, "MUX%i: TX IRQ acknowledged\n", unit);
	}
}

static void mux_unit_send(unsigned unit, uint8_t val) {
	uint64_t symbol_len = 1000000000.0 / (float)mux[unit].baud;

	mux[unit].status &= ~MUX_TX_READY;
	mux[unit].tx_finish_time = get_current_time() + symbol_len * 10;

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
	if (mode > 8) {
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
		mux_assert_irq(val - 1, MUX_IRQ_TX, trace);
		return;
	case 0xE: // Set TX interrupt request level
		if (trace)
			fprintf(stderr, "MUX%i: TX level = %i\n", unit, val);
		tx_ipl_request = val;
		return;
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
		if (irq_cause_unit == -1)
			return 0;

		card = irq_cause_unit / 4;
		port = irq_cause_unit % 4;

		// Cause is actually the lower 8 bits of unit that caused the interrupt
		// Though, TX interrupts have the lower bit set
		unsigned cause = (card << 4) | (port << 1) | irq_cause_reason;

		if (trace)
			fprintf(stderr, "MUX: InterruptCause Read: %02x\n", cause | 8);

		// Reading this register is enough to clear the TX IRQ, but it seems
		// to clear the RX IRQ, you actually have to read the data
		mux_ack_irq(irq_cause_unit, MUX_IRQ_TX, trace);

		return cause;
	}

	// Decode address

	// Nibble 1 of the address is the card number
	// Each MUX4 board supports 4 ports.
	// There are apparently MUX8 cards, which might act as two cards?
	card = (addr >> 4) & 0xF;


	mode = addr & 0xf;
	if (mode > 8) {
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
		data = mux[unit].status | check_write_ready(unit) | MUX_CTS;
		if (trace)
			fprintf(stderr, "MUX%i: Status Read = %02x\n", unit, data);

		break;
	case 0x1:
		// Data register
		data = next_char(port);
		mux[unit].status &= ~MUX_RX_READY;
		mux_ack_irq(unit, MUX_IRQ_RX, trace);
		if (trace)
			fprintf(stderr, "MUX%i: Data Read = %02x ('%c')\n", unit, data, data);
		break;
	default:
		fprintf(stderr, "MUX%i: Unknown Register %x Read\n", unit, addr);
		break;
	}

	return data;
}

static void set_read_ready(unsigned unit, unsigned trace)
{
	if ((mux[unit].status & MUX_RX_READY) == 0) {
		if (trace)
			fprintf(stderr, "MUX%i: Ready\n", unit);
		mux[unit].rx_finish_time = get_current_time() + (1000000000 / 30);
	}
	mux[unit].status |= MUX_RX_READY;
}

static void set_write_ready(unsigned unit, unsigned trace)
{
	mux[unit].status |= MUX_TX_READY;
	mux[unit].tx_finish_time = 0;
	mux_assert_irq(unit, MUX_IRQ_TX, trace);
}

static void check_tx_done(unsigned trace) {
	uint64_t time = get_current_time();
	for (unsigned unit = 0; unit < NUM_MUX_UNITS; unit++) {
		uint64_t finish = mux[unit].tx_finish_time;
		if (finish && finish <= time) {
			set_write_ready(unit, trace);
		}
	}
}

#ifdef WIN32

void mux_poll(unsigned trace)
{
	int unit;

	check_tx_done(trace);

	for (unit = 0; unit < NUM_MUX_UNITS; unit++) {
		if (mux[unit].status & MUX_RX_READY) {
			/* Do not waste time repetitively polling ports,
			 * which we know are ready
			 */
			mux_assert_irq(unit, MUX_IRQ_RX, trace);
			continue;
		}
		int fd = mux[unit].in_fd;

	    	if (fd != -1 && tty_check_readable(fd)) {
			set_read_ready(unit, trace);
		}
	}
}

#else

void mux_poll(unsigned trace)
{
	fd_set i;
	int max_fd = 0;
	int unit;

	check_tx_done(trace);

	FD_ZERO(&i);

	uint64_t time = get_current_time();

	for (unit = 0; unit < NUM_MUX_UNITS; unit++) {
		if (mux[unit].status & MUX_RX_READY) {
			/* Do not waste time repetitively polling ports,
			 * which we know are ready. But keep re-asserting the IRQ
			 * if configured
			 */
			mux_assert_irq(unit, MUX_IRQ_RX, trace);
			continue;
		}
		if (mux[unit].rx_finish_time > time) {
			// Block until the character has finished receiving
			continue;
		}
		int fd = mux[unit].in_fd;
	    	if (fd == -1)
			continue;
	    	FD_SET(fd, &i);
	    	if (fd >= max_fd)
	    		max_fd = fd + 1;
	}

	if (max_fd == 0 || select_wrapper(max_fd, &i, NULL) == -1) {
		return;
	}

	for (unit = 0; unit < NUM_MUX_UNITS; unit++) {
		int fd = mux[unit].in_fd;

		if (fd != -1 && FD_ISSET(fd, &i))
			set_read_ready(unit, trace);
	}
}

#endif

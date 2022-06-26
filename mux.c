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
static int cause_unit = 0;

// Set the initial state for all out ports
void mux_init(void)
{
	int i;

	for (i = 0; i < NUM_MUX_UNITS; i++) {
		mux[i].in_fd = -1;
		mux[i].out_fd = -1;
		mux[i].status = 0;
		mux[i].lastc = 0xFF;
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
		if (errno == EAGAIN || errno == EWOULDBLOCK)
			return mux[unit].lastc;
		perror("next_char");
		exit(1);
	}

	if (c == 0x7F) {
		/* Some terminals (like Cygwin) send DEL on Backspace */
		c = 0x08;
	}

	mux[unit].lastc = c;
	return c;
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
void mux_write(uint16_t addr, uint8_t val)
{
	unsigned unit, data;
	addr &= 0xFF;

	if (addr == 0x0A) {
		// Register 0x0A - set interrupt request level
		fprintf(stderr, "\nMux, RX Configured to LVL %x\n", val);
		rx_ipl_request = val;
		return;
	} else if (addr == 0xE) {
		fprintf(stderr, "\nMux, TX Configured to LVL %x\n", val);
		tx_ipl_request = val;
		return;
	} else if (addr >= NUM_MUX_UNITS * 2) {
		fprintf(stderr, "\nWrite to unknown MUX register %x=%02x\n", addr, val);
		// Any other out-of-band address, we don't know what to do with it
		return;
	}

	unit = (addr >> 1) & 0x0F;
	data = addr & 1;

	if (!data) {
		/* This is mode byte, we currently don't have anything
		 * to do with it.
		 * But if we wanted to operate on a real serial port, we'd need
		 * to change baud rate here.
		 * The code we have (diag test #6 and WIPL) loves to reset it every time.
		 * I have also never seen clearing register 0x0A; so i assume it also
		 * disables interrupt generation until requested explicitly.
		 */
		if (rx_ipl_request != 0) {
			cpu_deassert_irq(rx_ipl_request);
			rx_ipl_request = 0;
		}
		return;
	}

	uint64_t symbol_len = 1000000000.0 / 9600.0; // Hardcoded to 9600 baud

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

uint8_t mux_read(uint16_t addr)
{
	unsigned unit, data;

	addr &= 0xFF;

	if (addr == 0x0F) {
		/* Reading from 0x0F supposedly ACKs the interrupt */
		if (rx_ipl_request != 0) {
			cpu_deassert_irq(rx_ipl_request);
		}
		if (tx_ipl_request != 0) {
			cpu_deassert_irq(tx_ipl_request);
		}
		// Returns the mux unit ID that caused the interrupt
		// If we had multiple MUX4 boards, the board ID would be in the upper nibble
		return cause_unit << 1;
	} else if (addr >= NUM_MUX_UNITS * 2) {
		fprintf(stderr, "Read from unknown MUX register %x\n", addr);
		// Any other out-of-band address, we don't know what to do with it
		return 0;
	}

	unit = addr >> 1;
	data = addr & 1;

	if (data == 1) {
		/* Reading the data register resets RX_READY */
		mux[unit].status &= ~MUX_RX_READY;
		return next_char(unit);	/*( | 0x80; */
	}

	return mux[unit].status | check_write_ready(unit);
}

static void set_read_ready(unsigned unit)
{
	mux[unit].status |= MUX_RX_READY;
	if (rx_ipl_request)
		cpu_assert_irq(rx_ipl_request);
	// I've implemented this as just overwriting the last cause, which will cause issues
	// Would be interesting to see if the hardware has some kind of queue
	//
	// Ken mentions that sometimes key presses would be dropped under heavy loads
	cause_unit = unit;
}

static void set_write_ready(unsigned unit)
{
	mux[unit].status |= MUX_TX_READY;
	mux[unit].tx_finish_time = 0;
	if (tx_ipl_request)
		cpu_assert_irq(tx_ipl_request);
	cause_unit = unit;
}

static void check_tx_done() {
	uint64_t time = get_current_time();
	for (unsigned unit = 0; unit < NUM_MUX_UNITS; unit++) {
		uint64_t finish = mux[unit].tx_finish_time;
		if (finish && finish <= time) {
			set_write_ready(unit);
		}
	}
}

#ifdef WIN32

void mux_poll(void)
{
	int unit;

	check_tx_done();

	for (unit = 0; unit < NUM_MUX_UNITS; unit++) {
		if (mux[unit].status & MUX_RX_READY) {
			/* Do not waste time repetitively polling ports,
			 * which we know are ready
			 */
			continue;
		}
		int fd = mux[unit].in_fd;

	    	if (fd != -1 && tty_check_readable(fd)) {
			set_read_ready(unit);
		}
	}
}

#else

void mux_poll(void)
{
	fd_set i;
	int max_fd = 0;
	int unit;

	check_tx_done();

	FD_ZERO(&i);

	for (unit = 0; unit < NUM_MUX_UNITS; unit++) {
		if (mux[unit].status & MUX_RX_READY) {
			/* Do not waste time repetitively polling ports,
			 * which we know are ready
			 */
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
			set_read_ready(unit);
	}
}

#endif

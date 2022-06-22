#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include "centurion.h"
#include "cpu6.h"
#include "mux.h"

static struct MuxUnit mux[NUM_MUX_UNITS];
static char ipl_request = -1; // Let's suppose -1 = disabled

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

static int select_wrapper(int maxfd, fd_set* i, fd_set* o) {
	struct timeval tv;
	int rc;

	tv.tv_sec = 0;
	tv.tv_usec = 0;

	rc = select(maxfd, i, o, NULL, &tv);
	if (rc == -1 && errno != EINTR) {
		perror("select() failed in MUX");
		exit(1);
	}
	return rc;
}

/* Utility functions for the mux */
static unsigned int check_write_ready(uint8_t unit)
{
	int fd = mux[unit].out_fd;
	fd_set o;

        if (fd == -1) {
		/* An unconnected port is always ready */
                return MUX_TX_READY;
        }

	FD_ZERO(&o);
	FD_SET(fd, &o);
	if (select_wrapper(fd + 1, NULL, &o) == -1) {
		return 0;
	}
	return FD_ISSET(fd, &o) ? MUX_TX_READY : 0;
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

	if (c == 0x0A)
		fprintf(stderr, "Caught!\n");

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
 *	0A		Interrupt level ?
 *	0E		Cleared on IRQ test
 *	0F		read to check for interrupt - NZ = none
 *
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
		ipl_request = val;
		return;
	} else if (addr >= NUM_MUX_UNITS * 2) {
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
		if (ipl_request != -1) {
			cpu_deassert_irq(ipl_request);
			ipl_request = -1;
		}
		return;
	}

	if (mux[unit].out_fd == -1) {
		/* This MUX unit isn't connected to anything */
		return;
	}

	if (mux[unit].out_fd > 1) {
		val &= 0x7F;
		write(mux[unit].out_fd, &val, 1);
	} else {
		val &= 0x7F;
		if (val != 0x08 && val != 0x0A && val != 0x0D
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
		if (ipl_request != -1) {
			cpu_deassert_irq(ipl_request);
		}
		/* The F1 test also checks the value and ignores the interrupt
		 * (does not read the character) if nonzero value arrives. Perhaps
		 * this has to do with daisy-chaining, but we don't know; WIPL
		 * does not test the value
		 */
		return 0;
	} else if (addr >= NUM_MUX_UNITS * 2) {
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

void mux_poll(void)
{
	fd_set i;
	int max_fd = 0;
	int unit;

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
		if (fd == -1 || !FD_ISSET(fd, &i))
			continue;
		
		mux[unit].status |= MUX_RX_READY;
		if (ipl_request != -1)
			cpu_assert_irq(ipl_request);
	}
}

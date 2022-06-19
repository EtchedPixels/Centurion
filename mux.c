#include <errno.h>
#include <fcntl.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <termios.h>
#include <unistd.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <arpa/inet.h>

#include "centurion.h"
#include "mux.h"

static int max_fd;

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

static struct MuxUnit mux[NUM_MUX_UNITS];

// Set the initial state for all out ports
void mux_init(void)
{
        int i;

        for (i = 0; i < NUM_MUX_UNITS; i++) {
                mux[i].in_fd = -1;
                mux[i].out_fd = -1;
                mux[i].lastc = 0xFF;
        }
}

void tty_init(void)
{
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

        mux[0].in_fd = STDIN_FILENO;
        mux[0].out_fd = STDOUT_FILENO;
        max_fd = 2;
}

void net_init(unsigned short port)
{
	struct sockaddr_in sin;
        int sock_fd, io_fd;

	sock_fd = socket(AF_INET, SOCK_STREAM, 0);
	if (sock_fd == -1) {
		perror("socket");
		exit(1);
	}
	sin.sin_family = AF_INET;
	sin.sin_addr.s_addr = htonl(0x7F000001);
	sin.sin_port = htons(port);
	if (bind(sock_fd, (struct sockaddr *) &sin, sizeof(sin)) == -1) {
		perror("bind");
		exit(1);
	}
	/* TODO set reuseaddr */
	listen(sock_fd, 1);

	printf("[Waiting terminal connection...]\n");
	fflush(stdout);

	io_fd = accept(sock_fd, NULL, NULL);
	if (io_fd == -1) {
		perror("accept");
		exit(1);
	}
	close(sock_fd);
	fcntl(io_fd, F_SETFL, FNDELAY);
	max_fd = io_fd + 1;

        mux[0].in_fd = io_fd;
	mux[0].out_fd = io_fd;
}


/* Utility functions for the mux */
static unsigned int check_chario(uint8_t unit)
{
	fd_set i, o;
	struct timeval tv;
	unsigned int r = 0;

        if (mux[unit].in_fd == -1 || mux[unit].out_fd == -1) {
                return r;
        }

	FD_ZERO(&i);
	FD_SET(mux[unit].in_fd, &i);
	FD_ZERO(&o);
	FD_SET(mux[unit].out_fd, &o);
	tv.tv_sec = 0;
	tv.tv_usec = 0;

	if (select(max_fd, &i, &o, NULL, &tv) == -1) {
		if (errno == EINTR)
			return 0;
		perror("select");
		exit(1);
	}
	if (FD_ISSET(mux[unit].in_fd, &i))
		r |= 1;
	if (FD_ISSET(mux[unit].out_fd, &o))
		r |= 2;
	return r;
}

static unsigned int next_char(uint8_t unit)
{
	int r;

        if (mux[unit].in_fd == -1) {
                return mux[unit].lastc;
        }

	r = read(mux[unit].in_fd, &mux[unit].lastc, 1);

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
	return mux[unit].lastc;
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
	unit = (addr >> 1) & 0x0F;
	data = addr & 1;

	if (!data) {
                // This is mode byte, we currently don't have anything
                // to do with it.
                // But if we wanted to operate on a real serial port, we'd need
                // to change baud rate here.
		return;
	}

	if (unit >= NUM_MUX_UNITS || mux[unit].out_fd == -1)
		return;

	if (mux[unit].out_fd > 1) {
		val &= 0x7F;
		write(mux[unit].out_fd, &val, 1);
	} else {
		val &= 0x7F;
		if (val != 0x0A && val != 0x0D
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
	unsigned ttystate;
	uint8_t ctrl = 0;

	addr &= 0xFF;
	unit = addr >> 1;
	data = addr & 1;


	if (unit >= NUM_MUX_UNITS)
		return 0;

	if (data == 1)
		return next_char(unit);	/*( | 0x80; */

	ttystate = check_chario(unit);
	if (ttystate & 1)
		ctrl |= 1;
	if (ttystate & 2)
		ctrl |= 2;
	return ctrl;
}

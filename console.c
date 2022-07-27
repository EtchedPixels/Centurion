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
#include <time.h>

#include "centurion.h"
#include "console.h"
#include "mux.h"
#include "scheduler.h"

static struct termios saved_term, term;

static void exit_cleanup(void)
{
	tcsetattr(0, TCSADRAIN, &saved_term);
}

static void cleanup(int sig)
{
	tcsetattr(0, TCSADRAIN, &saved_term);
	emulator_done = 1;
}

void tty_init(void)
{
	if (tcgetattr(0, &term) == 0) {
		saved_term = term;
		atexit(exit_cleanup);
		signal(SIGINT, cleanup);
		signal(SIGQUIT, cleanup);
		signal(SIGPIPE, cleanup);
		term.c_iflag &= ~ICRNL;
		term.c_lflag &= ~(ICANON | ECHO);
		term.c_cc[VMIN] = 0;
		term.c_cc[VTIME] = 1;
		term.c_cc[VINTR] = 0;
		term.c_cc[VSUSP] = 0;
		term.c_cc[VSTOP] = 0;
		tcsetattr(0, TCSADRAIN, &term);
	}

        mux_attach(0, STDIN_FILENO, STDOUT_FILENO);
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

        mux_attach(0, io_fd, io_fd);
}

static int select_wrapper(int maxfd, fd_set* i, fd_set* o)
{
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

void mux_poll_fds(unsigned trace)
{
	fd_set i;
	int max_fd = 0;
	int unit;

	FD_ZERO(&i);

	for (unit = 0; unit < NUM_MUX_UNITS; unit++) {
		int ifd = mux_get_in_poll_fd(unit);

		if (ifd == -1)
			continue;
		FD_SET(ifd, &i);
		if (ifd >= max_fd)
			max_fd = ifd + 1;
	}

	if (max_fd > 0) {
	 	if (select_wrapper(max_fd, &i, NULL) == -1)
			return;
	}

	for (unit = 0; unit < NUM_MUX_UNITS; unit++) {
		int ifd = mux_get_in_fd(unit);

		if (ifd != -1 && FD_ISSET(ifd, &i))
			mux_set_read_ready(unit, trace);
	}
}


// Get's the current time from the OS (in nanoseconds)
uint64_t monotonic_time_ns() {
	struct timespec ts;
	clock_gettime(CLOCK_MONOTONIC, &ts);
	return ts.tv_sec * 1000000000ULL + ts.tv_nsec;
}


static uint64_t throttle_start_time;
static float throttle_speed;

void throttle_init() {
	throttle_start_time = monotonic_time_ns();
}

void throttle_set_speed(float speed) {
	throttle_speed = speed;
}

// Stall emulation if running faster than realtime
void throttle_emulation(uint64_t expected_time_ns) {
	uint64_t now = monotonic_time_ns();
	uint64_t adjusted_target = expected_time_ns / throttle_speed;
	int64_t delta_ns = (throttle_start_time + adjusted_target) - now;

	// We don't want to sleep if the delta is less than 5ms
	if (delta_ns > (5 * ONE_MILISECOND_NS)) {
		struct timespec delta;
		delta.tv_sec = delta_ns / 1000000000ULL;
		delta.tv_nsec = delta_ns % 1000000000ULL;

		// sometimes nanosecond returns early, so loop until it finishes
		while (nanosleep(&delta, &delta));
	} else if (delta_ns < -(50 * ONE_MILISECOND_NS)) {
		// If have lagged by too much, we forgive the time
		//throttle_start_time += delta_ns;
	}
}

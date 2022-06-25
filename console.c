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
#include "console.h"
#include "mux.h"

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

unsigned int tty_check_writable(int fd)
{
	fd_set o;

	FD_ZERO(&o);
	FD_SET(fd, &o);
	if (select_wrapper(fd + 1, NULL, &o) == -1) {
		return 0;
	}
	return FD_ISSET(fd, &o);
}

int select_wrapper(int maxfd, fd_set* i, fd_set* o)
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

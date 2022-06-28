#include <conio.h>
#include <fcntl.h>
#include <io.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <windows.h>

#include "console.h"
#include "mux.h"

static HANDLE hStdin, hStdout;
static DWORD saved_in_mode, saved_out_mode;

static void exit_cleanup(void)
{
        SetConsoleMode(hStdin, saved_in_mode);
        SetConsoleMode(hStdout, saved_out_mode);
}

static DWORD get_mode(HANDLE h)
{
        DWORD mode;

        if (GetConsoleMode(h, &mode))
                return mode;
        fprintf(stderr, "GetConsoleMode() failed!\n");
        fprintf(stderr, "Sorry, only native Windows console is supported, no mintty\n");
        exit(1);
}

static void set_mode(HANDLE h, DWORD mode)
{
        if (!SetConsoleMode(h, mode)) {
                fprintf(stderr, "SetConsoleMode() failed!\n");
                exit(1);
        }
}

void tty_init(void)
{
        hStdin  = (HANDLE)_get_osfhandle(STDIN_FILENO);
        hStdout = (HANDLE)_get_osfhandle(STDOUT_FILENO);
        saved_in_mode  = get_mode(hStdin);
        saved_out_mode = get_mode(hStdout);
        /* Disable canonical mode and Ctrl-C interception */
        set_mode(hStdin,  saved_in_mode & ~(ENABLE_ECHO_INPUT | ENABLE_LINE_INPUT | ENABLE_PROCESSED_INPUT));
        /* Enable VT-100 control codes on output */
        set_mode(hStdout, saved_out_mode | ENABLE_VIRTUAL_TERMINAL_PROCESSING);
        atexit(exit_cleanup);
        _setmode(STDIN_FILENO, O_BINARY);
        mux_attach(0, STDIN_FILENO, STDOUT_FILENO);
}

void net_init(unsigned short port)
{
        fprintf(stderr, "Network is not implemented yet on Win32\n");
        abort();
}

unsigned int tty_check_writable(int fd)
{
        return 1;
}

static int tty_check_readable(int fd)
{
        DWORD r;

        if (fd == STDIN_FILENO) {
                /*
                 * WaitForSingleObhect() sometimes signal something other than key input events.
                 * This may freeze the emulator because eventually the Centurion attempts to
                 * read a character which isn't there. Using _kbhit() works around that.
                 */
                return _kbhit();
        }

        r = WaitForSingleObject((HANDLE)_get_osfhandle(fd), 0);
        switch (r)
        {
        case WAIT_FAILED:
                fprintf(stderr, "WaitForMultipleObject() failed in MUX\n");
                exit(1);
        case WAIT_OBJECT_0:
                return 1;
        case WAIT_TIMEOUT:
                return 0;
        default:
                fprintf(stderr, "Unexpected WaitForMultipleObject() return value in MUX: %ld\n", r);
                exit(1);
        }
}

void mux_poll_fds(struct MuxUnit* mux, unsigned trace)
{
	int unit;

	for (unit = 0; unit < NUM_MUX_UNITS; unit++) {
		int ifd = mux[unit].in_fd;

		/* Do not waste time repetitively polling ports,
		 * which we know are ready.
		 */
		if (!(ifd == -1 || mux[unit].status & MUX_RX_READY)) {
                        if (tty_check_readable(ifd))
                                mux_set_read_ready(unit, trace);
		}
	}
}

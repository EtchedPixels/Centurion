#include <conio.h>
#include <fcntl.h>
#include <io.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <windows.h>

#include "console.h"
#include "mux.h"

HANDLE hStdin;
DWORD saved_mode;

static void exit_cleanup(void)
{
        SetConsoleMode(hStdin, saved_mode);
}

void tty_init(void)
{
        DWORD mode;

        hStdin = (HANDLE)_get_osfhandle(STDIN_FILENO);
        if (!GetConsoleMode(hStdin, &mode)) {
                fprintf(stderr, "GetConsoleMode() failed!\n");
                fprintf(stderr, "Sorry, only native Windows console is supported, no mintty\n");
                abort();
        }
        saved_mode = mode;
        atexit(exit_cleanup);
        mode &= ~(ENABLE_ECHO_INPUT | ENABLE_LINE_INPUT | ENABLE_PROCESSED_INPUT );
        if (!SetConsoleMode(hStdin, mode)) {
                fprintf(stderr, "SetConsoleMode() failed!\n");
                abort();
        }

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

int tty_check_readable(int fd)
{
        DWORD r;

        if (fd == STDIN_FILENO) {
                /*
                 * WaitForSingleObhect() sometimes signal something other than key input events.
                 * This may freeze the emulator because eventually the code attempts to
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

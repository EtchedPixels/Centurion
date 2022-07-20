#ifndef _TRACE_H
#define _TRACE_H

#include <stdio.h>

#define TRACE(...)                            \
	if (trace) {                          \
		fprintf(stderr, __VA_ARGS__); \
                fputc('\n', stderr);          \
        }

#define TRACE_PC(...)                                \
        if (trace) {                                 \
                fprintf(stderr, "%04X ", cpu6_pc()); \
		fprintf(stderr, __VA_ARGS__);        \
                fputc('\n', stderr);                 \
        }

#define WARN_PC(...)                                 \
        do {                                         \
                fprintf(stderr, "%04X ", cpu6_pc()); \
		fprintf(stderr, __VA_ARGS__);        \
                fputc('\n', stderr);                 \
        } while (0);

#endif

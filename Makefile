all: centurion

CFLAGS = -g3 -Wall -pedantic

centurion: centurion.o cpu6.o disassemble.o mux.o

centurion.o: centurion.c centurion.h cpu6.h

cpu6.o : cpu6.c cpu6.h

disassemble.o: disassemble.h cpu6.h

mux.o : centurion.h mux.h

clean:
	rm -f centurion *.o *~


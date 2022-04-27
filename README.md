# Centurion Emulator

An emulator the pieces we know so far of the Centurion Computer CPU-6 and
the diagnostics ROM images. It's good enough to run most of the diagnostics
and to pass the CPU self test.

The core of the processor is based upon the Eldorado Electrodata EE200, and
it is from this that most of the data is pulled.

There are differences however:

* CPU6 has an onboard DMA engine that uses instruction 2F
* CPU6 has an MMU that uses 2E to control some functionality
* There are MMU operations using 47 xx, 35 xx, 7E xx, 7F xx
* CPU6 has an 0x0F operation that is not in CPU4 and seems to be some kind of syscall/return
* CPU6 recognizes 0x76 as some kind of jump to IPL 15
* CPU4 has some branch and other ops for a bitbang terminal. It's not known if they remain in CPU4
* CPU6 toggles the V flag on 22 32. Unclear if this is a side effect of an invalid instruction used to test for CPU5 v CPU6 or something more

## Completed

* The known part of the instruction set
* Extrapolated behaviour for some of the other bits
* Framework to run the Diag board

## To Finish

* Interrupt model
* The various mystery instructions used in the CPU test and memory test


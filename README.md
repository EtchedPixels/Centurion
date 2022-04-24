# Centurion Emulator

An emulator the pieces we know so far of the Centurion Computer CPU-6 and
the diagnostics ROM images.

## Completed

* The known part of the instruction set
* Extrapolated behaviour for some of the other bits
* Framework to run the Diag board

## In Progress

* Finishing branch rules
* Figuring out the micro-sequencer ordering as observable for the RT register
* Flags

## To Begin

* Interrupt model
* Which registers map to which memory location
* The various mystery instructions used in the CPU test and memory test

## Compiling/Running

### Prerequisites

* Download ROM bin files from https://github.com/phire/centurion_isa/tree/main/roms<br>
    ```bootstrap_unscrambled.bin```<br>
	```Diag_F1_Rev_1.0.BIN```<br>
	```Diag_F2_Rev_1.0.BIN```<br>
	```Diag_F3_Rev_1.0.BIN```<br>
	```Diag_F4_1133CMD.BIN```
* Install C compiler with Make

### Compiling

* make

### Running

* For the Auxilary Tests Menu: ```./centurion -s 1 -S 13```

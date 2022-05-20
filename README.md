# Centurion Emulator

An emulator the pieces we know so far of the Centurion Computer CPU-6 and
the diagnostics ROM images. It's good enough to run most of the diagnostics
and to pass some of the CPU self test.

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
* V flag - seems not to agree with the EE200 manual ? try a full V rule


## Compiling/Running

### Prerequisites

* Download ROM bin files from https://github.com/phire/centurion_isa/tree/main/roms<br>
  * `bootstrap_unscrambled.bin`
  * `Diag_F1_Rev_1.0.BIN`
  * `Diag_F2_Rev_1.0.BIN`
  * `Diag_F3_Rev_1.0.BIN`
  * `Diag_F4_1133CMD.BIN`
* Install C compiler with Make

### Compiling

* make

### Running

To start the emulator, simply run `./centurion`. The system will then request a boot disk information.

The first time using the emulator, you may not have any *disks* to run. Therefore, you may instead want to run the auxiliary diagnostics ROM like so:

```
./centurion -d -s 1 -S 13
AUXILIARY TESTS

01=CPU INSTRUCTION TEST
02=CPU-6 MAPPING RAM TEST
03=ROM SELF TEST

04=CMD AUX MEMORY TEST
05=CMD SEEK TEST
06=CMD READ TEST
07=FLOPPY COMMAND BUFFER TEST
08=FLOPPY SEEK TEST
09=FLOPPY READ TEST
0A=ROM SELF TEST

0B=01133 CMD AUX MEMORY TEST
0C=01133 CMD SEEK TEST
0D=01133 CMD READ TEST
0E=FINCH AUX MEMORY TEST
0F=FINCH SEEK TEST
10=FINCH READ TEST
11=ROM SELF TEST


ENTER TEST NUMBER:_
```

## Test operating system

The test operating system is program `10` (0x0A) on the diag board. You can run it like so:

```
./centurion -d -s 1 -S 10
```

The test OS can be used as follows:

- enter `M` followed by a hex address
  - pressing `space` will print the value at that address, and increment the address
  - entering a hex value and pressing space will replace the value at that address and increment the address
- enter `G` followed by a hex address to start executing from that memory address 

For more info, view [the video from Usagi Electrics using the test OS](https://youtu.be/_j2L6nkO8MQ?list=PLnw98JPyObn0wJFdbcRDP7LMz8Aw2T97V&t=828).

## Command line options

The following options can be used when running the emulator:

- `-d` set the diag mode on
- `-F` emulate a finch drive
- `-l <port-number>` Listen for telnet on the given port number
- `-s <value>` set CPU switches as a decimal value. Switch 1 is *sense*
- `-S <value>` set diag switches as decimal value (only effective with `-d`)
- `-t <value>` enable system trace in terminal - See below

## System trace

The system trace outputs system IO to the terminal; useful for debugging. The `-t` option takes a value that is a [bitmask](https://en.wikipedia.org/wiki/Mask_(computing)) of the following:

- `1`: Memory
- `2`: Registers
- `4`: CPU
- `8`: FDC
- `16`: CMD

For example, in order to trace both *memory* and *registers*, set `-t 3`.

## Halting the emulator

To halt the emulator, simply press `Ctrl-\`, which will land you back on your terminal prompt.


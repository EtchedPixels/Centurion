#pragma once

#include <stdint.h>
#include "scheduler.h"

#define HAWK_NUM_CYLINDERS 406
#define HAWK_NUM_HEADS 2
#define HAWK_SECTS_PER_TRK 16 // Configurable, but Centurion used 16

#define HAWK_SECTOR_BYTES 400
#define HAWK_RAW_TRACK_BITS 62500 // Nominal, according to hawk manual
#define HAWK_RAW_SECTOR_BITS (HAWK_RAW_TRACK_BITS / HAWK_SECTS_PER_TRK)
#define HAWK_GAP_BITS 120
#define HAWK_SYNC_BITS 88

#define HAWK_ROTATION_NS (ONE_MILISECOND_NS * 25.0)
#define HAWK_BIT_NS (HAWK_ROTATION_NS / HAWK_RAW_TRACK_BITS)
#define HAWK_SECTOR_NS (HAWK_ROTATION_NS / HAWK_SECTS_PER_TRK)
#define HAWK_SECTOR_PULSE_NS (2000) // Complete guess

#define HAWK_DATACELL_DATA_BIT  0x01
#define HAWK_DATACELL_CLOCK_BIT 0x10

struct hawk_drive {
	struct event_t event;
	unsigned event_type;
	char event_name_string[16];

// Output signals
	// Ready
	// High when:
	//  - This disk cartridge is installed
	//  - Spindle motor speed is correct
	//  - heads loaded
	//  - DC voltages within margin
	//  - no fault condition exists
	//  - unit selected
	//  - terminator is present and has power
	uint8_t ready;

	// On Cyl (On Cylinder)
    // Cleared as soon as a seek begins.
	// Set when the heads have finished seeking to the desired address.
	// Also high when a seek error occurs
	uint8_t on_cyl;

	// Sker (Seek Error)
	// Set when the unit was unable to complete a seek operation.
	// A RTZS command from the controller clears the seek error
	// condition and returns the heads to cylinder 00.
	// On Cylinder is also asserted during seek error.
	uint8_t seek_error;

	// Fault
	// Indicates that the unit has encountered one or more fault conditions:
	//  - more than one head selected
	//  - read and write gates true at same time
	//  - read and erase gates true at same time
	//  - controller enabled only one of the write and erase gates
	//  - low voltage
	//  - selecting fixed heads on drive without fixed disk option
	//  - Emergency retract
	//
	// Stays high until a RTZS operation.
	uint8_t fault;

	// Address Acknowledge
	// Seek address accepted
	uint8_t addr_ack;

	// Address Interlock
    // cylinder address was larger than the number of cylinders on the disk
	uint8_t addr_int;

	// Write Protect
	// Either the unit's write protect switch is on, or the controller is
	// sending a write_inhibit signal to drive
	uint8_t wprotect;

	// Sector Pulse
	// high when head is at the start of a sector
	uint8_t sector_pulse;

	// Sector Address
	// The current sector under head
	uint8_t sector_addr;

	// Unimplemented output signals from Hawk unit
	// Some of them probably go in status.
	//   Index (sector 0 pulse), Density
	//
	// // See Page 25 of HAWK_9427_BP11_OCT80.pdf for details

	uint8_t seeking;

	// File handle of image file
	int fd_removable;
	int fd_fixed;

	// assigned drive number
	unsigned drive_num;

	unsigned selected; // removable or fixed

	// Datacells for current track
	// Wastefully store 1 bit per byte.
	// Bottom bit is actual data. Forth bit is "clock" signal, that will be one
	// for every data cell that contains data, and zero for data cells that
	// haven't been written.
	uint8_t datacells[HAWK_RAW_TRACK_BITS];

	int32_t data_ptr;
	int32_t head_pos;
	uint64_t rotation_offset;
};

void hawk_init(struct hawk_drive* unit, unsigned drive_num, int fd1, int fd2);
void hawk_setfd(struct hawk_drive* unit, unsigned fixed, int fd);
void hawk_seek(struct hawk_drive* unit, unsigned fixed, unsigned cyl, unsigned head);
void hawk_rtz(struct hawk_drive* unit, unsigned fixed);
int hawk_remaining_bits(struct hawk_drive* unit, uint64_t time);
void hawk_read_bits(struct hawk_drive* unit, int count, uint8_t *dest);
uint8_t hawk_read_byte(struct hawk_drive* unit);
uint16_t hawk_read_word(struct hawk_drive* unit);
void hawk_rewind(struct hawk_drive* unit, int count); // cheating
void hawk_wait_sector(struct hawk_drive* unit, unsigned sector);
int hawk_wait_sync(struct hawk_drive* unit);
void hawk_update(struct hawk_drive* unit, int64_t now);

// Callback to dsk
void dsk_hawk_changed(unsigned unit, int64_t time);
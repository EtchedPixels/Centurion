#pragma once

#include <stdint.h>

#define HAWK_NUM_CYLINDERS 406
#define HAWK_NUM_HEADS 2
#define HAWK_SECTS_PER_TRK 16 // Configurable, but Centurion used 16

#define HAWK_SECTOR_SIZE 400

struct hawk_unit {
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
	// From Hawk unit
	// Either the unit's write protect switch is on, or the controller is
	// sending a write_inhibit signal to drive
	uint8_t wprotect;

	// Unimplemented output signals from Hawk unit
	// Some of them probally go in status.
	//   Index (sector 0 pulse), Sector (one pulse per sector),
	//   Sector Address (upto 6 bits), Density.
	//
	// // See Page 25 of HAWK_9427_BP11_OCT80.pdf for details

	// File handle of image file
	int fd;

	// current cylinder << 1 | head
	uint16_t current_track;
};

void hawk_seek(struct hawk_unit* unit, unsigned cyl, unsigned head, unsigned sec);
void hawk_rtz(struct hawk_unit* unit);
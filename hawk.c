
#include "hawk.h"
#include "scheduler.h"

#include <assert.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#define HAWK_EVENT_NONE             0
#define HAWK_EVENT_SEEK_SUCCESS     1
#define HAWK_EVENT_SEEK_ERROR       2
#define HAWK_EVENT_ROTATE_SECTOR    3
#define HAWK_EVENT_ROTATE_SYNC      4

static void hawk_write_bits(struct hawk_drive* unit, int count, uint8_t* data);
static void hawk_set_bits(struct hawk_drive* unit, int count, uint8_t val);
static void hawk_erase_bits(struct hawk_drive* unit, int count);

static void hawk_event_callback(struct event_t* event, int64_t late_ns)
{
    // Event is the first member of the hawk_drive struct, so we can just cast it.
    struct hawk_drive* unit = (struct hawk_drive*)event;
    assert(offsetof(struct hawk_drive, event) == 0);

    switch (unit->event_type) {
    case HAWK_EVENT_SEEK_ERROR:
        unit->seek_error = 1;
        unit->seeking = 0;
        break;
    case HAWK_EVENT_SEEK_SUCCESS:
        unit->seeking = 0;
        break;
    default:
        break;
    }

    int64_t time = get_current_time() - late_ns;
    hawk_update(unit, time);

    if (unit->event_type == HAWK_EVENT_ROTATE_SECTOR)
        unit->data_ptr = unit->head_pos;

    unit->event_type = HAWK_EVENT_NONE;

    // notify DSK emulation that something happened
    dsk_hawk_changed(unit->drive_num, time);
}

// Reads entire track of data into host memory.
// Converts from 400 byte sectors, into raw bits with gaps, sync and format info
static int hawk_buffer_track(struct hawk_drive* unit, unsigned fixed, unsigned cyl, unsigned head) {
    off_t offset = ((cyl << 5) | (head << 4)) * HAWK_SECTOR_BYTES;
    uint8_t buffer[HAWK_SECTOR_BYTES];

    int fd = fixed ? unit->fd_fixed : unit->fd_removable;
    memset(unit->datacells, 0, sizeof(unit->datacells));

    // If we don't have a platter installed, the seek is going to complete anyway
    // There just won't be any data to read
    if (fd == -1)
        return 0;

    if (lseek(fd, offset, SEEK_SET) == -1) {
        fprintf(stderr, "hawk position failed (%d,%d,0) = %lx.\n",
            cyl, head, (long) offset);
        return 0;
    }

    for (int sector = 0; sector < HAWK_SECTS_PER_TRK; sector++) {
        unit->data_ptr = sector * HAWK_RAW_SECTOR_BITS;
        // ~120 bit gap, to compensate mechanical jitter
        hawk_erase_bits(unit, HAWK_GAP_BITS);

        // sync. 87 zeros, followed by a one
        hawk_set_bits(unit, HAWK_SYNC_BITS-1, 0);
        hawk_set_bits(unit, 1, 1);

        // sector address
        uint16_t addr = (cyl << 5) | (head << 4) | sector;
        uint16_t check_word = ~addr; // guess.
        uint8_t addr_data[4] = {
            (addr >> 8),
            addr & 0xff,
            (check_word >> 8),
            check_word & 0xff,
        };
        hawk_write_bits(unit, 32, addr_data);

        // second gap
        hawk_erase_bits(unit, HAWK_GAP_BITS);

        // another sync
        hawk_set_bits(unit, HAWK_SYNC_BITS-1, 0);
        hawk_set_bits(unit, 1, 1);

        // sector data
        if (read(fd, buffer, HAWK_SECTOR_BYTES) != HAWK_SECTOR_BYTES) {
            fprintf(stderr, "hawk read failed (%d,%d,%d).\n", cyl, head, sector);
            return 0;
        }
        hawk_write_bits(unit, HAWK_SECTOR_BYTES * 8, buffer);

        // CRC
        // TODO: proper CRC function
        uint8_t crc[2] = { 0xcc, 0xcc };
        hawk_write_bits(unit, 16, crc);

        // Trailer
        hawk_set_bits(unit, HAWK_GAP_BITS / 4, 0);
    }
    return 1;
}


void hawk_seek(struct hawk_drive* unit, unsigned fixed, unsigned cyl, unsigned head)
{
    if (unit->seeking)
        return;

    unit->seeking = 1;
    unit->addr_ack = 0;
    unit->addr_int = 0;

    unit->on_cyl = 0;
    unit->selected = fixed;

    // The hawk unit only has 9 lines for cylinder addr, so address really
    // should get masked.
    // OR, is DSK expected to throw an error before seeking?
    if (cyl >= HAWK_NUM_CYLINDERS) {
        // Tried to seek past end of disk
        unit->addr_int = 1;
        return;
    }

    off_t offset = (cyl << 5) | (head << 4) * HAWK_SECTOR_BYTES;
    offset *= HAWK_SECTOR_BYTES;

    // According to specs, the average track-to-track seek time is 7.5ms.
    // TODO: Accurate seek times
    unit->event.delta_ns = 7.5 * ONE_MILISECOND_NS;
    unit->event_type = HAWK_EVENT_SEEK_SUCCESS;

    if (unit->instant_read)
        unit->event.delta_ns = 0;

    // To simplify emulation, slurp the whole track into host memory
    hawk_buffer_track(unit, fixed, cyl, head);

    unit->addr_ack = 1;
    schedule_event(&unit->event);
}


void hawk_rtz(struct hawk_drive* unit, unsigned fixed)
{
    // According to manual, The Hawk drive unit will clear any seek
    // errors and faults on RTZS
    unit->seek_error = 0;
    unit->fault = 0;
    unit->seeking = 0;

    hawk_seek(unit, fixed, 0, 0);
}

void hawk_update(struct hawk_drive* unit, int64_t now) {
    // It's more of seek-complete than actually on_cyl.
    // Forced to zero as soon as a seek begins.
    // Gets set even if the seek errors out
    unit->on_cyl = unit->ready && !unit->seeking;

    uint64_t rotation = (now + unit->rotation_offset) % (uint64_t)HAWK_ROTATION_NS;

    unit->head_pos = rotation / HAWK_BIT_NS;
    unit->sector_addr = rotation / HAWK_SECTOR_NS;
    unit->sector_pulse = (rotation % (int64_t)HAWK_SECTOR_NS) < HAWK_SECTOR_PULSE_NS;
}

void hawk_init(struct hawk_drive *unit, unsigned drive_num, int fd1, int fd2) {
    memset(unit, 0, sizeof(struct hawk_drive));

    unit->event.callback = hawk_event_callback;
    snprintf(unit->event_name_string, sizeof(unit->event_name_string), "hawk%d_event", drive_num);
    unit->event.name = unit->event_name_string;

    unit->drive_num = drive_num;
    unit->wprotect = 1;

    hawk_setfd(unit, 0, fd1);
    hawk_setfd(unit, 1, fd2);

    // It's not actually possible to spin up a drive without a cartridge installed,
    // So if we have either image, it's ready.
    unit->ready = (fd1 != -1) || (fd2 != -1);

    if (unit->ready) {
        hawk_buffer_track(unit, 0, 0, 0);
        hawk_update(unit, 0);
    }
}

void hawk_setfd(struct hawk_drive* unit, unsigned fixed, int fd) {
    if (fixed)
        unit->fd_fixed = fd;
    else
        unit->fd_removable = fd;
}


int hawk_remaining_bits(struct hawk_drive* unit, uint64_t time) {
    hawk_update(unit, time);
    return unit->head_pos - unit->data_ptr;
}

void hawk_wait_sector(struct hawk_drive* unit, unsigned sector) {
    int64_t now = get_current_time();
    int64_t rotation = (now + unit->rotation_offset) % (uint64_t)HAWK_ROTATION_NS;
    int64_t desired_rotation = HAWK_SECTOR_NS * sector;

    int64_t delta = desired_rotation - rotation;
    if (delta < 0)
        delta += HAWK_ROTATION_NS;

    if (unit->instant_read) {
        // Just teleport the platter to the correct rotation
        unit->rotation_offset += delta;
        unit->rotation_offset %= (uint64_t)HAWK_ROTATION_NS;
        delta = 1000; // Small delta to allow scheduler to return to DMA loop
    }

    assert(unit->event_type == 0);

    unit->event.delta_ns = delta;
    unit->event_type = HAWK_EVENT_ROTATE_SECTOR;

    schedule_event(&unit->event);
}

int hawk_wait_sync(struct hawk_drive* unit) {
    if (unit->event_type != HAWK_EVENT_NONE)
        return 1;

    // Find the next one bit
    int32_t ptr = unit->data_ptr - 1;
    uint8_t bit;
    do {
        ptr = (ptr + 1) % HAWK_RAW_TRACK_BITS;
        bit = unit->datacells[ptr];
    } while (bit != (HAWK_DATACELL_CLOCK_BIT | HAWK_DATACELL_DATA_BIT));

    if (unit->instant_read) {
        // If we are doing instant reads, don't just wait for sync. Wait for
        // the end of the currently recorded section.
        while ((bit & HAWK_DATACELL_CLOCK_BIT) != 0) {
            ptr = (ptr + 1) % HAWK_RAW_TRACK_BITS;
            bit = unit->datacells[ptr];
        }
    }

    if (ptr <= unit->head_pos)
        return 0; // don't need to wait

    // Calculate rotation offset to it
    int64_t now = get_current_time();
    int64_t rotation = (now + unit->rotation_offset) % (uint64_t)HAWK_ROTATION_NS;
    int64_t desired_rotation = HAWK_BIT_NS * ptr;

    int64_t delta = (desired_rotation - rotation) % (uint64_t)HAWK_ROTATION_NS;

    if (unit->instant_read) {
        // Just teleport the platter to the correct rotation
        unit->rotation_offset += delta;
        unit->rotation_offset %= (uint64_t)HAWK_ROTATION_NS;

        // Don't need to wait
        hawk_update(unit, now);
        return 0;
    }

    // schedule event
    unit->event.delta_ns = delta;
    unit->event_type = HAWK_EVENT_ROTATE_SYNC;
    schedule_event(&unit->event);

    return 1;
}

void hawk_read_bits(struct hawk_drive* unit, int count, uint8_t *dest) {
    while (1) {
        uint8_t byte = 0;
        uint8_t bit;
        for (int shift = 7; shift >= 0; shift--) {
            do {
                bit = unit->datacells[unit->data_ptr++];
                unit->data_ptr %= HAWK_RAW_TRACK_BITS;
            } while (bit == 0); // skip over any erased bits
            bit &= HAWK_DATACELL_DATA_BIT;

            // This is somewhat realistic to real hardware. The data
            // and clock pluses have been split into separate signals by
            // the data recovery board's PLL, and that won't instantly
            // desync if the on-disk clock is missing.

            bit = bit << shift;
            byte |= bit;
            if (--count == 0) {
                *dest = byte;
                return;
            }
        }
        *dest++ = byte;
    }
}

uint8_t hawk_read_byte(struct hawk_drive* unit) {
    uint8_t byte = 0;
    hawk_read_bits(unit, 8, &byte);
    return byte;
}

uint16_t hawk_read_word(struct hawk_drive* unit) {
    // byteswap to little endian
    uint16_t word = hawk_read_byte(unit) << 8;
    word |= hawk_read_byte(unit);

    return word;
}

void hawk_rewind(struct hawk_drive* unit, int count) {
    unit->data_ptr = unit->data_ptr - count;
    if (unit->data_ptr < 0)
        unit->data_ptr += HAWK_RAW_TRACK_BITS;
}

static void hawk_write_bits(struct hawk_drive* unit, int count, uint8_t* data) {
    while (count > 0) {
        uint8_t byte = *(data++);
        for (int shift = 7; shift >= 0; shift--) {
            if (count-- == 0)
                return;

            uint8_t bit = ((byte >> shift) & 1) | HAWK_DATACELL_CLOCK_BIT;
            unit->datacells[unit->data_ptr++] = bit;
        }
    }
}

static void hawk_set_bits(struct hawk_drive* unit, int count, uint8_t val) {
    val = (val & 1) | HAWK_DATACELL_CLOCK_BIT;

    while (count--) {
        unit->datacells[unit->data_ptr++] = val;
    }
}

static void hawk_erase_bits(struct hawk_drive* unit, int count) {
    while (count--) {
        unit->datacells[unit->data_ptr++] = 0;
    }
}

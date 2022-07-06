
#include "hawk.h"
#include "scheduler.h"

#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

static void hawk_write_bits(struct hawk_unit* unit, int count, uint8_t* data);
static void hawk_set_bits(struct hawk_unit* unit, int count, uint8_t val);

struct seek_event_t {
    struct event_t event;
    struct hawk_unit *unit;
    unsigned seek_error;
};

static void hawk_seek_callback(struct event_t* event, int64_t late_ns);

static struct seek_event_t seek_event[8] = {{
    .event = {
        .name = "hawk0_seek",
        .callback = hawk_seek_callback,
    }
},{
    .event = {
        .name = "hawk1_seek",
        .callback = hawk_seek_callback,
    }
},{
    .event = {
        .name = "hawk2_seek",
        .callback = hawk_seek_callback,
    }
},{
    .event = {
        .name = "hawk3_seek",
        .callback = hawk_seek_callback,
    }
},{
    .event = {
        .name = "hawk4_seek",
        .callback = hawk_seek_callback,
    }
},{
    .event = {
        .name = "hawk5_seek",
        .callback = hawk_seek_callback,
    }
},{
    .event = {
        .name = "hawk6_seek",
        .callback = hawk_seek_callback,
    }
},{
    .event = {
        .name = "hawk7_seek",
        .callback = hawk_seek_callback,
    }
}
};

struct rotation_event_t {
    struct event_t event;
    struct hawk_unit *unit;
    unsigned in_process;
};

static void hawk_rotation_event_cb(struct event_t* event, int64_t late_ns);

static struct rotation_event_t rotation_event = {
    .event = {
        .name = "hawk_rotation",
        .callback = hawk_rotation_event_cb,
    }
};

static void hawk_seek_callback(struct event_t* event, int64_t late_ns)
{
    struct seek_event_t* e = (struct seek_event_t*)event;
    struct hawk_unit* unit = e->unit;

    unit->seek_error = e->seek_error;

    // It's more of seek-complete than actually on_cyl.
    // Forced to zero as soon as a seek begins.
    // Gets set even if the seek errors out
    unit->on_cyl = 1;
    unit->seeking = 0;

    // notify DSK emulation that something happened
    dsk_hawk_changed(unit->unit_num, get_current_time() - late_ns);
}

// Reads entire track of data into host memory.
// Converts from 400 byte sectors, into raw bits with gaps, sync and format info
static int hawk_buffer_track(struct hawk_unit* unit, unsigned cyl, unsigned head) {
    off_t offset = ((cyl << 5) | (head << 4)) * HAWK_SECTOR_BYTES;
    uint8_t buffer[HAWK_SECTOR_BYTES];

    if (lseek(unit->fd, offset, SEEK_SET) == -1) {
        fprintf(stderr, "hawk position failed (%d,%d,0) = %lx.\n",
            cyl, head, (long) offset);
        return 0;
    }

    for (int sector = 0; sector < HAWK_SECTS_PER_TRK; sector++) {
        int start = unit->data_ptr = sector * HAWK_RAW_SECTOR_BITS;
        // ~120 bit gap, to compensate mechanical jitter
        hawk_set_bits(unit, HAWK_GAP_BITS, 0);

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
        hawk_set_bits(unit, HAWK_GAP_BITS, 0);

        // another sync
        hawk_set_bits(unit, HAWK_SYNC_BITS-1, 0);
        hawk_set_bits(unit, 1, 1);

        // sector data
        if (read(unit->fd, buffer, HAWK_SECTOR_BYTES) != HAWK_SECTOR_BYTES) {
            fprintf(stderr, "hawk read failed (%d,%d,%d).\n", cyl, head, sector);
            return 0;
        }
        hawk_write_bits(unit, HAWK_SECTOR_BYTES * 8, buffer);

        // CRC
        // TODO: proper CRC function
        uint8_t crc[2] = { 0xcc, 0xcc };
        hawk_write_bits(unit, 16, crc);

        // Trailer
        hawk_set_bits(unit, HAWK_GAP_BITS / 2, 0);
        //fprintf(stderr, "Hawk: %i wrote %i bits to %i\n", sector, unit->data_ptr - start, start);
    }

    //fprintf(stderr, "Hawk track %d,%d loaded.\n", cyl, head);

    return 1;
}


void hawk_seek(struct hawk_unit* unit, unsigned cyl, unsigned head)
{
    // The hawk unit only has 9 lines for cylinder addr, so address really
    // should get masked.
    // OR, is DSK expected to throw an error before seeking?

    if (unit->seeking)
        return;

    unit->seeking = 1;
    unit->addr_ack = 0;
    unit->addr_int = 0;
    unit->current_track = cyl << 1 | head;

    unit->on_cyl = 0;


    off_t offset = (cyl << 5) | (head << 4) * HAWK_SECTOR_BYTES;
    offset *= HAWK_SECTOR_BYTES;

        if (cyl >= HAWK_NUM_CYLINDERS) {
        // Tried to seek past end of disk
        unit->addr_int = 1;
        return;
    }

    // According to specs, the average track-to-track seek time is 7.5ms.
    // TODO: Accurate seek times
    struct seek_event_t* e = &seek_event[unit->unit_num];
    e->event.delta_ns = 7.5 * ONE_MILISECOND_NS;
    e->event.callback = hawk_seek_callback;
    e->unit = unit;
    e->seek_error = 0;

    // To simplify emulation, slurp the whole track into host memory
    if (!hawk_buffer_track(unit, cyl, head)) {
        // According to manual, a Seek Error is generated if the carriage
        // goes beyond end of travel or on cyl is not present 0.5 seconds
        // after initiation of CA Strobe or RTZ

        // we will emulate our IO error as 500ms timeout
        e->seek_error = 1;
        e->event.delta_ns = 500 * ONE_MILISECOND_NS;
    }

    unit->addr_ack = 1;
    schedule_event(&e->event);
}


void hawk_rtz(struct hawk_unit* unit)
{
    // According to manual, The Hawk drive unit will clear any seek
    // errors and faults on RTZS
    unit->seek_error = 0;
    unit->fault = 0;
    unit->seeking = 0;

    hawk_seek(unit, 0, 0);
}

void hawk_update(struct hawk_unit* unit, int64_t now) {
    uint64_t rotation = (now + unit->rotation_offset) % (uint64_t)HAWK_ROTATION_NS;

    unit->head_pos = rotation / HAWK_BIT_NS;
    unit->sector_addr = rotation / HAWK_SECTOR_NS;
    unit->sector_pulse = (rotation % (int64_t)HAWK_SECTOR_NS) < HAWK_SECTOR_PULSE_NS;
}

int hawk_remaining_bits(struct hawk_unit* unit, uint64_t time) {
    hawk_update(unit, time);
    return unit->head_pos - unit->data_ptr;
}

static void hawk_rotation_event_cb(struct event_t* event, int64_t late_ns)
{
    assert(event == &rotation_event.event);
    rotation_event.in_process = 0;

    struct hawk_unit* unit = rotation_event.unit;

    int64_t time = get_current_time() - late_ns;
    hawk_update(unit, time);

     // Copy current head position to read/write pointer
    unit->data_ptr = unit->head_pos;
    dsk_hawk_changed(unit->unit_num, time);
}

void hawk_wait_sector(struct hawk_unit* unit, unsigned sector) {
    int64_t now = get_current_time();
    int64_t rotation = (now + unit->rotation_offset) % (uint64_t)HAWK_ROTATION_NS;
    int64_t desired_rotation = HAWK_SECTOR_NS * sector;

    int64_t delta = desired_rotation - rotation;
    if (delta < 0)
        delta += HAWK_ROTATION_NS;

    assert(rotation_event.in_process == 0);

    rotation_event.unit = unit;
    rotation_event.in_process = 1;
    rotation_event.event.delta_ns = delta;
    rotation_event.event.callback = hawk_rotation_event_cb;

    schedule_event(&rotation_event.event);
    // fprintf(stderr, "wait for %i, Scheduled rotation finished in %f ms\n", sector, (double)rotation_event.event.delta_ns / ONE_MILISECOND_NS);
}

void hawk_read_bits(struct hawk_unit* unit, int count, uint8_t *dest) {
    while (1) {
        uint8_t byte = 0;
        for (int shift = 7; shift >= 0; shift--) {
            uint8_t bit = unit->current_track_data[unit->data_ptr++];
            unit->data_ptr %= HAWK_RAW_TRACK_BITS;
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

uint8_t hawk_read_byte(struct hawk_unit* unit) {
    uint8_t byte = 0;
    hawk_read_bits(unit, 8, &byte);
    return byte;
}

uint16_t hawk_read_word(struct hawk_unit* unit) {
    // byteswap to little endian
    uint16_t word = hawk_read_byte(unit) << 8;
    word |= hawk_read_byte(unit);

    return word;
}

void hawk_rewind(struct hawk_unit* unit, int count) {
    unit->data_ptr = unit->data_ptr - count;
    if (unit->data_ptr < 0)
        unit->data_ptr += HAWK_RAW_TRACK_BITS;
}

static void hawk_write_bits(struct hawk_unit* unit, int count, uint8_t* data) {
    while (1) {
        uint8_t byte = *(data++);
        for (int shift = 7; shift >= 0; shift--) {
            if (count-- == 0)
                return;

            uint8_t bit = (byte >> shift) & 1;
            unit->current_track_data[unit->data_ptr++] = bit;
        }
    }
}

static void hawk_set_bits(struct hawk_unit* unit, int count, uint8_t val) {
    val = val & 1;

    while (count--) {
        unit->current_track_data[unit->data_ptr++] = val;
    }
}


#include "cpu6.h"
#include "cbin.h"

#include <stdio.h>
#include <stdlib.h>

static uint16_t read_word(struct cbin_record* record, size_t offset) {
    uint16_t word = record->data[offset] << 8;
    word = word | record->data[offset+1];
    return word;
}

// Type CBIN_DATA
static void load_data(uint16_t *load_offset, struct cbin_record* record) {
    if (record->addr == 0x004c && record->len > 0x1b) {
        // This appears to be a convention for the old table loader.
        // Seems it can't do multiple sectors, or fixups. So intended for
        // tape loading start with a single sector replacement loader,
        // loaded to 0x4c, and jump to it;
        // This new second stage loader completes the load.
        //
        // But newer loaders ignore it and just steal the load offset

        *load_offset = read_word(record, 0x1b);
        return;
    }

    // Load len bytes of data to addr
    for (int i = 0; i < record->len; i++) {
        mem_write8_debug(record->addr + i + *load_offset, record->data[i]);
    }
}

// Type CBIN_FIXUPS
static void apply_fixups(uint16_t load_offset, struct cbin_record* record) {
    uint16_t offset = load_offset + record->addr;
    uint16_t fixup_addr;
    uint16_t fixup_val;

    for (size_t i = 0; i < record->len; i += 2) {
        fixup_addr = read_word(record, i);

        fixup_val = mem_read16_debug(fixup_addr + load_offset);
        fixup_val += offset;
        mem_write16_debug(fixup_addr + load_offset, fixup_val);
    }
}

// Loads a Centurion binary file directly into memory
uint16_t cbin_load(const char *name, uint16_t load_offset) {
    uint16_t entry_addr = 0;
    struct cbin_record* record = NULL;

    cbin_state* cbin = cbin_open(name);

    while ((record = cbin_next_record(cbin))) {
        switch (record->type)
        {
        case CBIN_DATA:
            if (record->len == 0) {
                // a zero length data record is the entry address
                entry_addr = record->addr + load_offset;
            } else {
                load_data(&load_offset, record); // Might modify load_offset
            }
            break;
        case CBIN_FIXUPS:
            // Apply fixups
            if (record->len % 2 == 1){
                fprintf(stderr, "FIXUPS record must have even length");
                cbin_error(cbin);
            }
            apply_fixups(load_offset, record);
            break;
        default:
            fprintf(stderr, "unknown type %02x\n", record->type);
            cbin_error(cbin);
        }
    }
    // Finished loading
    if (entry_addr && cbin_finished(cbin)) {
        printf("Centurion Binary %s loaded; entry at %04hx\n\n", name, entry_addr);
        cbin_free(cbin);
        return entry_addr;
    }

    if (!cbin_errored(cbin) && entry_addr == 0) {
        fprintf(stderr, "Couldn't find entry point\n");
    }

    printf("Centurion Binary loading of %s failed\n", name);

    cbin_free(cbin);
    exit(1);
}

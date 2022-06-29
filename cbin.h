
#define SECTOR_SIZE 400

/* Centurion binary format
 *
 * Appears to be used for both executable and binary data files.
 * Supports relocation
 *
 */

enum cbin_type {
    CBIN_DATA   = 0x00,
    CBIN_FIXUPS = 0x01,

    // These aren't a full header, they are just a single byte type
    CBIN_END_SECTOR = 0x80, // Skips ahead to the next 400 byte sector
    CBIN_END_FILE = 0x84, // Finishes loading
};

struct cbin_record {
    uint8_t type;
    uint8_t len; // Might be limited to 0x78
    uint16_t addr;
    uint8_t data[SECTOR_SIZE];
};

typedef struct cbin_state cbin_state;

// Returns next record
// Returns a NULL pointer if no more records exist
struct cbin_record* cbin_next_record(cbin_state* cbin);

// Skips to next sector
void cbin_next_sector(cbin_state* cbin);

// Opens a cbin file
struct cbin_state* cbin_open(const char *filename);

// Cleans up internal state and frees the cbin struct
void cbin_free(cbin_state* cbin);

// Returns true if the load completed without error
unsigned cbin_finished(cbin_state* cbin);

// Returns true if an error was encountered while reading
unsigned cbin_errored(cbin_state* cbin);

// Set an error, print internal state
void cbin_error(cbin_state* cbin);


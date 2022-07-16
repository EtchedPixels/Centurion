#include <stdint.h>

void dsk_init(void);
unsigned get_hawk_dma_mode(void);

uint8_t dsk_read(uint16_t addr, unsigned trace);
void dsk_write(uint16_t addr, uint8_t val, unsigned trace);

uint8_t hawk_read_next(void);
void hawk_write_next(uint8_t c);
void hawk_dma_done(void);

#include <stdint.h>

void hawk_init(void);
unsigned get_hawk_dma_mode(void);

uint8_t hawk_read(uint16_t addr);
void hawk_write(uint16_t addr, uint8_t val);

uint8_t hawk_read_next(void);
void hawk_write_next(uint8_t c);
void hawk_dma_done(void);

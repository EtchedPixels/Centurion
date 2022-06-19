#include <inttypes.h>

void tty_init(void);
void net_init(unsigned short port);

void mux_write(uint16_t addr, uint8_t val);
uint8_t mux_read(uint16_t addr);

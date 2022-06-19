#include <inttypes.h>

#define NUM_MUX_UNITS 4

struct MuxUnit
{
        int in_fd;
        int out_fd;
        unsigned char lastc;
};

void mux_init(void);
void tty_init(void);
void net_init(unsigned short port);

void mux_write(uint16_t addr, uint8_t val);
uint8_t mux_read(uint16_t addr);

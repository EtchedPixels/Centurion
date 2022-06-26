#include <inttypes.h>

#define MUX0_BASE 0xf200
#define NUM_MUX_UNITS 4

struct MuxUnit
{
        int in_fd;
        int out_fd;
        unsigned char status;
        unsigned char lastc;
        uint64_t tx_finish_time;
};

/* Status register bits */
#define MUX_RX_READY (1 << 0)
#define MUX_TX_READY (1 << 1)

void mux_init(void);
void mux_attach(unsigned unit, int in_fd, int out_fd);
void tty_init(void);
void net_init(unsigned short port);
void mux_poll(void);

void mux_write(uint16_t addr, uint8_t val);
uint8_t mux_read(uint16_t addr);

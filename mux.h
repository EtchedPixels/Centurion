#include <inttypes.h>

#define MUX0_BASE 0xf200
#define NUM_MUX_UNITS 4

struct MuxUnit
{
        int in_fd;
        int out_fd;
        unsigned char status;
        unsigned char lastc;
        int baud;
        unsigned char tx_done;
};

/* Status register bits */
#define MUX_RX_READY (1 << 0)
#define MUX_TX_READY (1 << 1)
#define MUX_CTS      (1 << 5)

/* Interrupt status register bits */
#define MUX_IRQ_RX 0
#define MUX_IRQ_TX 1
#define MUX_UNIT_MASK 0x06

void mux_init(void);
void mux_attach(unsigned unit, int in_fd, int out_fd);
void mux_poll(unsigned trace);

void mux_write(uint16_t addr, uint8_t val, uint32_t trace);
uint8_t mux_read(uint16_t addr, uint32_t trace);

void mux_set_read_ready(unsigned unit, unsigned trace);
void mux_set_write_ready(unsigned unit, unsigned trace);

void mux_poll_fds(struct MuxUnit* mux, unsigned trace);

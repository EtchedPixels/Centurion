
#define AH		0
#define AL		1
#define BH		2
#define BL		3
#define XH		4
#define XL		5
#define YH		6
#define YL		7
#define ZH		8
#define ZL		9
#define SH		10
#define SL		11
#define GH		12
#define GL		13
#define PCL		14
#define PCH		15

#define A		0
#define B		1
#define X		2
#define Y		3
#define Z		4
#define S		5	/* Just a convention it seems */
#define GX		6	/* Flags ? */
#define PC		7	/* PC */

extern uint8_t mem_read8(uint16_t addr);
extern void mem_write8(uint16_t addr, uint8_t val);
extern void halt_system(void);
extern uint16_t cpu6_pc(void);
extern unsigned cpu6_execute_one(unsigned trace);
extern int dma_read_cycle(uint8_t data);
extern void cpu6_set_switches(unsigned switches);

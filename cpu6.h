#include <inttypes.h>

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
#define CH		12
#define CL		13
#define PH		14
#define PL		15

#define A		0
#define B		2
#define X		4
#define Y		6
#define Z		8
#define S		10	/* Just a convention it seems */
#define C		12	/* Flags ? */
#define P		14	/* PC */

#define ONE_SECOND_NS 1000000000.0

extern uint8_t mem_read8(uint32_t addr);
extern uint8_t mem_read8_debug(uint32_t addr);
extern uint8_t mmu_mem_read8(uint16_t addr);
extern uint8_t mmu_mem_read8_debug(uint16_t addr);
extern void mem_write8(uint32_t addr, uint8_t val);
extern void halt_system(void);
extern uint16_t cpu6_pc(void);
extern unsigned cpu6_execute_one(unsigned trace);
extern int dma_read_cycle(uint8_t data);
extern uint8_t dma_write_cycle(void);
extern int dma_write_active(void);
extern void cpu6_set_switches(unsigned switches);
extern unsigned cpu6_halted(void);
extern void cpu6_init(void);
extern void cpu_assert_irq(unsigned ipl);
extern void cpu_deassert_irq(unsigned ipl);
extern void advance_time(uint64_t nanoseconds);
extern uint64_t get_current_time();
#ifndef __IRQ_H_
#define __IRQ_H_

extern void eclic_irq20_handler(void);
extern void eclic_irq21_handler(void);
extern void eclic_irq22_handler(void);
extern void eclic_irq23_handler(void);

extern void eclic_irq50_handler(void);
#define CONCAT_STAGE_1(w, x, y, z) w ## x ## y ## z
#define CONCAT2(w, x) CONCAT_STAGE_1(w, x, , )
#define CONCAT3(w, x, y) CONCAT_STAGE_1(w, x, y, )
#define CONCAT4(w, x, y, z) CONCAT_STAGE_1(w, x, y, z)

/* Helper macros to build the IRQ handler and priority struct names */
#define IRQ_HANDLER(irqname) CONCAT3(eclic_irq, irqname, _handler)
/*
 * Macro to connect the interrupt handler "routine" to the irq number "irq" and
 * ensure it is enabled in the interrupt controller with the right priority.
 */
#define DECLARE_IRQ(irq, routine) DECLARE_IRQ_(irq, routine)
#define DECLARE_IRQ_(irq, routine)                    \
	void IRQ_HANDLER(irq)(void)				\
	{							\
		routine();				\
	}

/*IRQ_NUM define list*/
#define IRQ_NUM_MB_0	50
#define IRQ_NUM_MB_1	49
#define IRQ_NUM_MB_2	48
#define IRQ_NUM_MB_3	47

/*You can add other interrupts num here 46~19*/

/* gpio key/blutooth or others gpio irq useage*/
#define IRQ_NUM_GPIO3	23
#define IRQ_NUM_GPIO2	22
#define IRQ_NUM_GPIO1	21
#define IRQ_NUM_GPIO0	20

#endif

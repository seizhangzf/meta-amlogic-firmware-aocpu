

#ifndef __WAKEUP_H__
#define __WAKEUP_H__

/*use timerB to wakeup AP FSM*/
inline void wakeup_ap(void)
{
	uint32_t value;
	//uint32_t time_out = 20;

	/*set alarm timer*/
	REG32(FSM_TRIGER_SRC) = 1000;/*1ms*/

	value = REG32(FSM_TRIGER_CTRL);
	value &= ~((1 << 7) | (0x3) | (1 << 6));
	value |= ((1 << 7) | (0 << 6) | (0x3));
	REG32(FSM_TRIGER_CTRL) = value;
	vTaskDelay(1);
}

inline void clear_wakeup_trigger(void)
{
	REG32(FSM_TRIGER_SRC) = 0;
	REG32(FSM_TRIGER_CTRL) = 0;
}

inline void watchdog_reset_system(void)
{
        int i = 0;

        printf("enter watchdog_reset_system\n");
        while (1) {
                REG32(RESETCTRL_WATCHDOG_CTRL0) = 1 << 27 | 0 << 18;
                /* Decive GCC for waiting some cycles */
                for (i = 0; i < 100; i++) {
                        REG32(RESETCTRL_WATCHDOG_CTRL0);
                }
        }

}
#endif

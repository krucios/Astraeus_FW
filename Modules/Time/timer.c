/*
 * timer.c
 *
 *  Created on: 21 Jul 2016
 *      Author: S2400105
 */

#include "timer.h"

void timers_init(void) {
    timer_mss1_init(MSS_TIMER_PERIODIC_MODE, TIMER_PERIOD(MSS_TIM1_FREQ));
    timer_mss2_init(MSS_TIMER_PERIODIC_MODE, TIMER_PERIOD(MSS_TIM2_FREQ));
    timer_sys_init(TIMER_PERIOD(MSS_SYS_TIM_FFREQ));
}

void timer_mss1_init(mss_timer_mode_t mode, uint32_t load_value) {
    NVIC_SetPriority(Timer1_IRQn, 0x80u);
    MSS_TIM1_init(mode);
    MSS_TIM1_load_background(load_value);
}

void timer_mss1_start(void) {
    MSS_TIM1_start();
    MSS_TIM1_enable_irq();
}

void timer_mss1_stop(void) {
    MSS_TIM1_stop();
    MSS_TIM1_disable_irq();
}

void timer_mss2_init(mss_timer_mode_t mode, uint32_t load_value) {
    NVIC_SetPriority(Timer2_IRQn, 0x40u);
    MSS_TIM2_init(mode);
    MSS_TIM2_load_background(load_value);
}

void timer_mss2_start(void) {
    MSS_TIM2_start();
    MSS_TIM2_enable_irq();
}

void timer_mss2_stop(void) {
    MSS_TIM2_stop();
    MSS_TIM2_disable_irq();
}

void timer_sys_init(uint32_t load_value) {
    /*-------------------------------------------------------------------------
     * Initialize the system tick for 10mS operation or 1 tick every 100th of
     * a second and also make sure it is lower priority than the I2C IRQs.
     */
    NVIC_SetPriority(FabricIrq0_IRQn, 0x10u);
    NVIC_SetPriority(SysTick_IRQn, 0x11u);
    SysTick_Config(load_value);
}

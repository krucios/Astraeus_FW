/*
 * timer.h
 *
 *  Created on: 21 Jul 2016
 *      Author: S2400105
 */

#ifndef MODULES_TIME_TIMER_H_
#define MODULES_TIME_TIMER_H_

#include <drivers_config/sys_config/sys_config.h>
#include <drivers/mss_timer/mss_timer.h>

#define MSS_TIM1_FREQ         20    // in Hz
#define MSS_TIM2_FREQ         200   // in Hz
#define MSS_SYS_TIM_FFREQ     100   // in Hz

#define TIMER_PERIOD(freq)    (MSS_SYS_M3_CLK_FREQ / (freq))

void timers_init(void);

void timer_mss1_init(mss_timer_mode_t mode, uint32_t load_value);
void timer_mss1_start(void);
void timer_mss1_stop(void);

void timer_mss2_init(mss_timer_mode_t mode, uint32_t load_value);
void timer_mss2_start(void);
void timer_mss2_stop(void);

void timer_sys_init(uint32_t load_value);

#endif /* MODULES_TIME_TIMER_H_ */

/*
 * time.c
 *
 *  Created on: 20 Jul 2016
 *      Author: S2400105
 */

#include "time.h"

#include <drivers_config/sys_config/sys_config.h>

uint32_t sec = 0;
uint32_t msec = 0;

inline uint64_t usec() {
    return (uint64_t)(1000000 * sec)
         + (1000 * msec)
         + ((MSS_SYS_M3_CLK_FREQ / 20) - MSS_TIM1_get_current_value()) / 50;
         //  TimVal * TimPeriod (us) / TimReloadVal
}

void usec_service_routine() {
    msec += 50;
    if (msec >= 1000) {
        sec++;
        msec %= 1000;
    }
}


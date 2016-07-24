/*
 * time.h
 *
 *  Created on: 20 Jul 2016
 *      Author: S2400105
 */

#ifndef MODULES_TIME_TIME_H_
#define MODULES_TIME_TIME_H_

#include <stdint.h>
#include <mss_timer/mss_timer.h>

extern uint32_t sec;
extern uint32_t msec;

uint64_t usec();
void usec_service_routine();

#endif /* MODULES_TIME_TIME_H_ */

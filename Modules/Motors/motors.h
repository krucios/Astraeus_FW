/*
 * motors.h
 *
 *  Created on: 24 июля 2016 г.
 *      Author: kruci_000
 */

#ifndef MODULES_MOTORS_MOTORS_H_
#define MODULES_MOTORS_MOTORS_H_

#include <drivers/CorePWM/core_pwm.h>
#include <MC_hw_platform.h>

#define PWM_PRESCALE 1
#define PWM_PERIOD 1000

#define MOTORS_COUNT 4

extern pwm_instance_t  g_pwm;
extern uint16_t motors_pow[MOTORS_COUNT];

void motors_init();
void motors_set();
void motors_masked_set(uint8_t mask);
void motors_set_mask(uint8_t mask);

#endif /* MODULES_MOTORS_MOTORS_H_ */

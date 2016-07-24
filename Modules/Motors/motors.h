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

extern pwm_instance_t  g_pwm;

void motors_init();
void motors_set(int16_t m_power[4]);
void motors_masked_set(int16_t m_power[4], uint8_t mask);
void motors_set_mask(uint8_t mask);

#endif /* MODULES_MOTORS_MOTORS_H_ */

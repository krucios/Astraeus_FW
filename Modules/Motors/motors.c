/*
 * motors.c
 *
 *  Created on: 24 июля 2016 г.
 *      Author: kruci_000
 */

#include "motors.h"

pwm_instance_t g_pwm;
uint16_t motors_pow[4];
static uint8_t m_mask = 0;

void motors_init() {
    uint8_t i;

    PWM_init(&g_pwm, COREPWM_0_0, PWM_PRESCALE, PWM_PERIOD);
    PWM_enable(&g_pwm, PWM_1);

    for (i = 0; i < MOTORS_COUNT; i++) {
        motors_pow[i] = 0;
    }

    PWM_set_duty_cycle(&g_pwm, PWM_1, 0);
    PWM_enable(&g_pwm, PWM_2);
    PWM_set_duty_cycle(&g_pwm, PWM_2, 0);
    PWM_enable(&g_pwm, PWM_3);
    PWM_set_duty_cycle(&g_pwm, PWM_3, 0);
    PWM_enable(&g_pwm, PWM_4);
    PWM_set_duty_cycle(&g_pwm, PWM_4, 0);
}

void motors_set() {
    if (m_mask & (1 << 0))
        PWM_set_duty_cycle(&g_pwm, PWM_1, motors_pow[0]);
    else
        PWM_set_duty_cycle(&g_pwm, PWM_1, 0);
    if (m_mask & (1 << 1))
        PWM_set_duty_cycle(&g_pwm, PWM_2, motors_pow[1]);
    else
        PWM_set_duty_cycle(&g_pwm, PWM_2, 0);
    if (m_mask & (1 << 2))
        PWM_set_duty_cycle(&g_pwm, PWM_3, motors_pow[2]);
    else
        PWM_set_duty_cycle(&g_pwm, PWM_3, 0);
    if (m_mask & (1 << 3))
        PWM_set_duty_cycle(&g_pwm, PWM_4, motors_pow[3]);
    else
        PWM_set_duty_cycle(&g_pwm, PWM_4, 0);
}

void motors_masked_set(uint8_t mask) {
    if (mask & (1 << 0))
        PWM_set_duty_cycle(&g_pwm, PWM_1, motors_pow[0]);
    else
        PWM_set_duty_cycle(&g_pwm, PWM_1, 0);
    if (mask & (1 << 1))
        PWM_set_duty_cycle(&g_pwm, PWM_2, motors_pow[1]);
    else
        PWM_set_duty_cycle(&g_pwm, PWM_2, 0);
    if (mask & (1 << 2))
        PWM_set_duty_cycle(&g_pwm, PWM_3, motors_pow[2]);
    else
        PWM_set_duty_cycle(&g_pwm, PWM_3, 0);
    if (mask & (1 << 3))
        PWM_set_duty_cycle(&g_pwm, PWM_4, motors_pow[3]);
    else
        PWM_set_duty_cycle(&g_pwm, PWM_4, 0);
}

void motors_set_mask(uint8_t mask) {
    m_mask = mask;
}


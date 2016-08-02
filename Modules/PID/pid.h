/*
 * pid.h
 *
 *  Created on: 31 июля 2016 г.
 *      Author: kruci_000
 */

#ifndef MODULES_PID_PID_H_
#define MODULES_PID_PID_H_

#include <math.h>
#include <stdint.h>
#include <Modules/AHRS/MadgwickAHRS.h>

#define force_low_trottle 119
#define power_low_trottle 120

#define force_high_trottle 850
#define power_high_trottle 1000
#define loop_time 4 // freq of computing data in millisecond

#define Kp_d 256
#define Kd_d 256
#define Ki_d 1048576

extern volatile int16_t Kp_u, Kd_u;

void pid_update(float q0, float q1, float q2, float q3, float gx,
        float gy, float gz, uint16_t force, int16_t* pow);

#endif /* MODULES_PID_PID_H_ */

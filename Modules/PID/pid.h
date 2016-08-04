/*
 * pid.h
 *
 *  Created on: 31 ���� 2016 �.
 *      Author: kruci_000
 */

#ifndef MODULES_PID_PID_H_
#define MODULES_PID_PID_H_

#include <stdint.h>

#define PID_FORCE_MIN 119
#define PID_FORCE_MAX 850

#define PID_POWER_MIN 120
#define PID_POWER_MAX 1000

#define Kp_d 1
#define Kd_d 1024
#define Ki_d 1048576 // 2^^20

extern volatile int16_t Kp_u, Kd_u, Ki_u;
extern volatile uint16_t force;

/*
void pid_update(float q0, float q1, float q2, float q3, float gx,
        float gy, float gz, uint16_t force, int16_t* pow);
*/
void pid_update(int16_t* pow);

#endif /* MODULES_PID_PID_H_ */

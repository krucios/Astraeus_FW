/*
 * pid.c
 *
 *  Created on: 31 июля 2016 г.
 *      Author: kruci_000
 */

#include "pid.h"

#include <math.h>
#include <Helpers/conversion_defines.h>
#include <Modules/MPU6050/mpu6050.h>
#include <Modules/AHRS/MadgwickAHRS.h>
#include <Modules/MAVLink/common/mavlink.h>

volatile int16_t Kp_u = 16, Kd_u = 3, Ki_u = 0;
volatile uint16_t force = 0;

static int16_t Dtmp_r, Dtmp_p, Dtmp_y;
static int16_t Ptmp_r, Ptmp_p, Ptmp_y;
static int16_t Itmp_r, Itmp_p, Itmp_y;

static float    roll, pitch, yaw;
static int16_t  roll_deg, pitch_deg, yaw_deg;

inline void pid_update(int16_t* pow) {
    uint8_t i;
    int16_t sum_r, sum_p, sum_y;

    mavlink_quaternion_to_euler(q, &roll, &pitch, &yaw);
    roll_deg    = roll  * RAD_TO_DEG;
    pitch_deg   = pitch * RAD_TO_DEG;
    yaw_deg     = yaw   * DEG_TO_RAD;

    if (force < PID_FORCE_MIN) { // if force so low
        pow[0] = pow[1] = pow[2] = pow[3] = 0;
        return;
    } else if (force > PID_FORCE_MAX) // limit force if it so high for to prevent destabilization
        force = PID_FORCE_MAX;

    Ptmp_r  = (int16_t) ((int32_t)Kp_u * roll_deg    / Kp_d);
    Ptmp_p  = (int16_t) ((int32_t)Kp_u * pitch_deg   / Kp_d);
    Ptmp_y  = (int16_t) ((int32_t)Kp_u * yaw_deg     / Kp_d);
/*
    Itmp_r += (int16_t) ((int32_t)Ki_u * _ay     / Ki_d);
    Itmp_p += (int16_t) ((int32_t)Ki_u * _ax     / Ki_d);
    Itmp_y += (int16_t) ((int32_t)Ki_u * _az     / Ki_d);
*/
    Itmp_r += (int16_t) ((int32_t)Ki_u * roll_deg    / Ki_d);
    Itmp_p += (int16_t) ((int32_t)Ki_u * pitch_deg   / Ki_d);
    Itmp_y += (int16_t) ((int32_t)Ki_u * yaw_deg     / Ki_d);

    Dtmp_r  = (int16_t) ((int32_t)Kd_u * _gx     / Kd_d);
    Dtmp_p  = (int16_t) ((int32_t)Kd_u * _gy     / Kd_d);
    Dtmp_y  = (int16_t) ((int32_t)Kd_u * _gz     / Kd_d);

    sum_r = Ptmp_r + Itmp_r + Dtmp_r;
    sum_p = Ptmp_p - Itmp_p + Dtmp_p;
    sum_y = Ptmp_y + Itmp_y + Dtmp_y;

    pow[0] = force + sum_r - sum_p - sum_y;
    pow[1] = force + sum_r + sum_p + sum_y;
    pow[2] = force - sum_r - sum_p + sum_y;
    pow[3] = force - sum_r + sum_p - sum_y;

    for (i = 0; i < 4; i++) {
        if (pow[i] < PID_POWER_MIN)
            pow[i] = PID_POWER_MIN;
        else if (pow[i] > PID_POWER_MAX)
            pow[i] = PID_POWER_MAX;
    }

}


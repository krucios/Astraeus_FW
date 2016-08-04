/*
 * pid.c
 *
 *  Created on: 31 июля 2016 г.
 *      Author: kruci_000
 */

#include "pid.h"

#include <math.h>
#include <Modules/MPU6050/mpu6050.h>
#include <Modules/AHRS/MadgwickAHRS.h>
#include <Modules/MAVLink/common/mavlink.h>

volatile int16_t Kp_u = 1, Kd_u = 256, Ki_u = 64;
volatile uint16_t force = 0;

static int16_t Dtmp_r, Dtmp_p, Dtmp_y;
static int16_t Ptmp_r, Ptmp_p, Ptmp_y;
static int16_t Itmp_r, Itmp_p, Itmp_y;

static float roll, pitch, yaw;
/*
inline void pid_update(float q0, float q1, float q2, float q3, float gx,
        float gy, float gz, uint16_t force, int16_t* pow) {
*/
inline void pid_update(int16_t* pow) {
    uint8_t i;
    int16_t sum_r, sum_p, sum_y;

    mavlink_quaternion_to_euler(q, &roll, &pitch, &yaw);

    if (force < PID_FORCE_MIN) { // if force so low
        pow[0] = pow[1] = pow[2] = pow[3] = 0;
        return;
    } else if (force > PID_FORCE_MAX) // limit force if it so high for to prevent destabilization
        force = PID_FORCE_MAX;

    Ptmp_r  = (int16_t) ((int32_t)Kp_u * roll    / Kp_d);
    Ptmp_p  = (int16_t) ((int32_t)Kp_u * pitch   / Kp_d);
    Ptmp_y  = (int16_t) ((int32_t)Kp_u * yaw     / Kp_d);

    Itmp_r += (int16_t) ((int32_t)Ki_u * _ay     / Ki_d);
    Itmp_p += (int16_t) ((int32_t)Ki_u * _ax     / Ki_d);
    // TODO yaw integral part Itmp_y += (int16_t) (Ki_u * _az / Ki_d);

    Dtmp_r  = (int16_t) ((int32_t)Kd_u * _gx     / Kd_d);
    Dtmp_p  = (int16_t) ((int32_t)Kd_u * _gy     / Kd_d);
    Dtmp_y  = (int16_t) ((int32_t)Kd_u * _gz     / Kd_d);

    /*
    if (q0 > 0) { // если угол поворота >0.5 градуса, нужно стабилизировать по углам.
        // (q1, q2, q3) - вектор в локальной системе коордиант вокруг которого необходимо повернуть коптер
        float P_gain = (float) Kp_u / 16.0; //коэф усиления
        float angle = acos(q0) * P_gain; //угол поворота
        float invNorma = invSqrt(q1 * q1 + q2 * q2 + q3 * q3); // нормируем его функция должна неявно подтянуться из либы мажвика
        q1 *= invNorma;
        q2 *= invNorma;
        q3 *= invNorma;
        Ptmp_p = (int16_t) (q2 * angle);
        Ptmp_r = (int16_t) (q1 * angle);
        Ptmp_y = (int16_t) (q3 * angle);

    } else {
        Ptmp_p = 0;
        Ptmp_r = 0;
        Ptmp_y = 0;
    }
    */

    sum_r = Ptmp_r + Itmp_r + Dtmp_r;
    sum_p = Ptmp_p - Itmp_p + Dtmp_p;
    sum_y = Ptmp_y          + Dtmp_y; // + Itmp_y;

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


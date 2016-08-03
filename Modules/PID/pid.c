/*
 * pid.c
 *
 *  Created on: 31 июля 2016 г.
 *      Author: kruci_000
 */

#include "pid.h"
#include <Modules/MPU6050/mpu6050.h>

volatile int16_t Kp_u = 11, Kd_u = 11, Ki_u = 64;
// static int32_t Ki_u = 12;
// static int16_t P_lim = 200, I_lim = 50, D_lim = 200;
// static int16_t amplif = 6;
// static int32_t I_lim2;
// static int16_t Itmp_p = 87381, Itmp_r = -1310720, Itmp_y; // this val's get from telemetry after flying
static int16_t Dtmp_r, Dtmp_p, Dtmp_y;
static int16_t Ptmp_r, Ptmp_p, Ptmp_y;
static int16_t Itmp_r, Itmp_p, Itmp_y;
/*
inline void pid_update(float q0, float q1, float q2, float q3, float gx,
        float gy, float gz, uint16_t force, int16_t* pow) {
*/
inline void pid_update(float roll, float pitch, float yaw, float gx,
        float gy, float gz, uint16_t force, int16_t* pow) {
    uint8_t i;
    int16_t sum_r, sum_p, sum_y;

    if (force < force_low_trottle) { // if force so low
        pow[0] = pow[1] = pow[2] = pow[3] = 0;
        return;
    } else if (force > force_high_trottle) // limit force if it so high for to prevent destabilization
        force = force_high_trottle;

    Ptmp_r = (int16_t) (Kp_u * roll  / Kp_d);
    Ptmp_p = (int16_t) (Kp_u * pitch / Kp_d);
    Ptmp_y = (int16_t) (Kp_u * yaw   / Kp_d);

    Dtmp_r = (int16_t) (Kd_u * gx / Kd_d);
    Dtmp_p = (int16_t) (Kd_u * gy / Kd_d);
    Dtmp_y = (int16_t) (Kd_u * gz / Kd_d);

    Itmp_r += (int16_t) (Ki_u * _ay / Ki_d);
    Itmp_p += (int16_t) (Ki_u * _ax / Ki_d);
    // Itmp_y += (int16_t) (Ki_u * _az / Ki_d);

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

    sum_r = Dtmp_r + Ptmp_r + Itmp_r;
    sum_p = Dtmp_p + Ptmp_p - Itmp_p;
    sum_y = Dtmp_y + Ptmp_y; // + Itmp_y;

    pow[0] = force + sum_r - sum_p - sum_y;
    pow[1] = force + sum_r + sum_p + sum_y;
    pow[2] = force - sum_r - sum_p + sum_y;
    pow[3] = force - sum_r + sum_p - sum_y;

    for (i = 0; i < 4; i++) {
        if (pow[i] < power_low_trottle)
            pow[i] = power_low_trottle;
        else if (pow[i] > power_high_trottle)
            pow[i] = power_high_trottle;
    }

}


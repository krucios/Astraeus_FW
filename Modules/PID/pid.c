/*
 * pid.c
 *
 *  Created on: 31 июля 2016 г.
 *      Author: kruci_000
 */

#include "pid.h"

volatile int16_t Kp_u = 11, Kd_u = 11;
// static int32_t Ki_u = 12;
// static int16_t P_lim = 200, I_lim = 50, D_lim = 200;
// static int16_t amplif = 6;
// static int32_t I_lim2;
// static int16_t Itmp_p = 87381, Itmp_r = -1310720, Itmp_y; // this val's get from telemetry after flying
static int16_t Dtmp_p, Dtmp_r, Dtmp_y;
static int16_t Ptmp_p, Ptmp_r, Ptmp_y;

inline void pid_update(float q0, float q1, float q2, float q3, float gx,
        float gy, float gz, uint16_t force, int16_t* pow) {
    uint8_t i;

    if (force < force_low_trottle) { // if force so low
        pow[0] = pow[1] = pow[2] = pow[3] = 0;
        return;
    } else if (force > force_high_trottle) // limit force if it so high for to prevent destabilization
        force = force_high_trottle;

    Dtmp_p = (int16_t) (-1 * (int32_t) Kd_u * (int32_t) (gy) / Kd_d);
    Dtmp_r = (int16_t) (     (int32_t) Kd_u * (int32_t) (gx) / Kd_d);
    Dtmp_y = (int16_t) (-1 * (int32_t) Kd_u * (int32_t) (gz) / Kd_d);

    if (q0 > 0.99999f) { // если угол поворота >0.5 градуса, нужно стабилизировать по углам.
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

    int16_t sum_p, sum_r, sum_y;
    sum_p = Dtmp_p + Ptmp_p;
    sum_r = Dtmp_r + Ptmp_r;
    sum_y = (Dtmp_y + Ptmp_y) * 4;

    pow[0] = force - sum_p - sum_r + sum_y;
    pow[1] = force + sum_p - sum_r - sum_y;
    pow[2] = force + sum_p + sum_r + sum_y;
    pow[3] = force - sum_p + sum_r - sum_y;

    for (i = 0; i < 4; i++) {
        if (pow[i] < power_low_trottle)
            pow[i] = power_low_trottle;
        else if (pow[i] > power_high_trottle)
            pow[i] = power_high_trottle;
    }

}


/*
 * param_holder.h
 *
 *  Created on: 20 Jul 2016
 *      Author: S2400105
 */

#ifndef MODULES_PARAMETERS_HOLDER_PARAM_HOLDER_H_
#define MODULES_PARAMETERS_HOLDER_PARAM_HOLDER_H_

#include <Modules/MAVLink/common/mavlink.h>
#include <Modules/MAVLink/system.h>
#include <Modules/UART/uart.h>

#define ONBOARD_PARAM_COUNT 32
#define ONBOARD_PARAM_NAME_LENGTH 7
#define PARAM_AX        0
#define PARAM_AY        1
#define PARAM_AZ        2
#define PARAM_GX        3
#define PARAM_GY        4
#define PARAM_GZ        5
#define PARAM_MX        6
#define PARAM_MY        7
#define PARAM_MZ        8
#define PARAM_T         9
#define PARAM_AT        10
#define PARAM_AP        11

#define PARAM_P0        12 // Motor 0 power
#define PARAM_P1        13 // Motor 1 power
#define PARAM_P2        14 // Motor 2 power
#define PARAM_P3        15 // Motor 3 power

#define PARAM_RD        16 // Roll in degrees
#define PARAM_PD        17 // Pitch in degrees
#define PARAM_YD        18 // Yaw in degrees

#define PARAM_PR        19 // Proportional roll
#define PARAM_PP        20 // Proportional pitch
#define PARAM_PY        21 // Proportional yaw
#define PARAM_DR        22 // Derivative roll
#define PARAM_DP        23 // Derivative pitch
#define PARAM_DY        24 // Derivative yaw
#define PARAM_IR        25 // Integral roll
#define PARAM_IP        26 // Integral pitch
#define PARAM_IY        27 // Integral yaw

#define PARAM_SR        28 // Roll's sum
#define PARAM_SP        29 // Pitch's sum
#define PARAM_SY        30 // Yaw's sum
#define PARAM_FC        31 // Force

typedef struct {
    float   param       [ONBOARD_PARAM_COUNT];
    char    param_name  [ONBOARD_PARAM_COUNT]
                        [MAVLINK_MSG_PARAM_SET_FIELD_PARAM_ID_LEN];
} param_holder_t;

extern param_holder_t params;

void param_send(uint8_t index);
void param_queued_send_start(void);
void param_queued_send_routine(void);

#endif /* MODULES_PARAMETERS_HOLDER_PARAM_HOLDER_H_ */

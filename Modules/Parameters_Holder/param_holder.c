/*
 * param_holder.c
 *
 *  Created on: 20 Jul 2016
 *      Author: S2400105
 */

#include "param_holder.h"

param_holder_t params = {
        // .param      = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
        .param_name =
            {"ACCEL_X",
             "ACCEL_Y",
             "ACCEL_Z",
             "GYROS_X",
             "GYROS_Y",
             "GYROS_Z",
             "MAGNT_X",
             "MAGNT_Y",
             "MAGNT_Z",
             "TEMP_CS",
             "ALTITUD",
             "ABSPRES",
             "POWER_0",
             "POWER_1",
             "POWER_2",
             "POWER_3",
             "ROL_DEG",
             "PIT_DEG",
             "YAW_DEG",
             "ROLL__P",
             "PITCH_P",
             "YAW___P",
             "ROL___D",
             "PIT___D",
             "YAW___D",
             "ROL___I",
             "PIT___I",
             "YAW___I",
             "ROL_SUM",
             "PIT_SUM",
             "YAW_SUM",
             "FORCE  "
            }};

static uint16_t m_parameter_i = 0;

void param_send(uint8_t index) {
    mavlink_message_t message;
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    uint8_t len;

    mavlink_msg_param_value_pack(
        mavlink_system.sysid,
        mavlink_system.compid,
        &message,
        params.param_name[index],
        params.param[index],
        MAVLINK_TYPE_INT16_T,
        ONBOARD_PARAM_COUNT,
        index);
    len = mavlink_msg_to_send_buffer(buffer, &message);
    BT_send(buffer, len);
}

void param_queued_send_start(void) {
    m_parameter_i = 0;
}

void param_queued_send_routine(void) {
    if (m_parameter_i < ONBOARD_PARAM_COUNT) {
        param_send(m_parameter_i);
        m_parameter_i++;
    }
}

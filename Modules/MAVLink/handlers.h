/*
 * handlers.h
 *
 *  Created on: 20 Jul 2016
 *      Author: S2400105
 */

#ifndef MODULES_MAVLINK_HANDLERS_H_
#define MODULES_MAVLINK_HANDLERS_H_

#include "common/mavlink.h"
#include "mavlink_helpers.h"

#include <Modules/UART/uart.h>
#include <Modules/Motors/motors.h>
#include <Modules/Parameters_Holder/param_holder.h>
#include <Modules/PID/pid.h>

#include <stdio.h>

extern param_holder_t params;

void handle_mavlink_message(mavlink_message_t* msg) {
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint32_t len;

    switch (msg->msgid) {
    case MAVLINK_MSG_ID_COMMAND_LONG: {
        mavlink_command_long_t cmd;
        mavlink_msg_command_long_decode(msg, &cmd);

        if (is_sys_comp_match(cmd.target_system, cmd.target_component)) {
            switch (cmd.command) {
            case MAV_CMD_DO_MOTOR_TEST: {
                    uint8_t throttle = (uint8_t)cmd.param3 * 10;
                    motors_pow[0] = motors_pow[1] =
                    motors_pow[2] = motors_pow[3] = throttle;
                    uint8_t mask = 1 << (uint8_t)(cmd.param1 - 1);
                    motors_masked_set(mask);
                    delay((uint64_t)cmd.param4 * 1000000);
                    motors_masked_set(0); // 0 mask - disable all motors
                }
                break;
            case MAV_CMD_USER_1: {
                    uint8_t mask = 0;
                    if (cmd.param1 > 0) {
                        mask |= 1;
                    }
                    if (cmd.param2 > 0) {
                        mask |= 2;
                    }
                    if (cmd.param3 > 0) {
                        mask |= 4;
                    }
                    if (cmd.param4 > 0) {
                        mask |= 8;
                    }
                    motors_set_mask(mask);
                }
                break;
            case MAV_CMD_USER_2: {
                    force = cmd.param1;
                }
                break;
            case MAV_CMD_USER_3: {
                    Kp_u = cmd.param1;
                    Kd_u = cmd.param2;
                    Ki_u = cmd.param3;
                }
                break;
            case MAV_CMD_USER_4: {
                    MPU6050_calibration();
                }
                break;
            default: {
                    mavlink_message_t msg;
                    uint8_t buff[50];

                    sprintf(buff, "Unknown CMD: %d", cmd.command);

                    mavlink_msg_statustext_pack(mavlink_system.sysid,
                            mavlink_system.compid, &msg, MAV_SEVERITY_ERROR, buff);
                    mavlink_send_msg(&msg);
                }
            }
        }
    }
    break;
    case MAVLINK_MSG_ID_PARAM_REQUEST_LIST: {
        // Start sending parameters
        param_queued_send_start();
    }
    break;
    case MAVLINK_MSG_ID_PARAM_REQUEST_READ: {
        mavlink_param_request_read_t read;
        mavlink_msg_param_request_read_decode(msg, &read);

        // Check if this message is for this system
        if (is_sys_comp_match(read.target_system, read.target_component)) {
            char* key = (char*) read.param_id;
            if (!strcmp(key, params.param_name[read.param_index])) {
                mavlink_message_t message;
                uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
                uint8_t len;

                mavlink_msg_param_value_pack(mavlink_system.sysid,
                        mavlink_system.compid, &message,
                        params.param_name[read.param_index],
                        params.param[read.param_index], MAVLINK_TYPE_FLOAT,
                        ONBOARD_PARAM_COUNT, read.param_index);
                len = mavlink_msg_to_send_buffer(buffer, &message);
                BT_send(buffer, len);
            }
        }
    }
    break;
    case MAVLINK_MSG_ID_PARAM_SET: {
        mavlink_param_set_t set;
        mavlink_msg_param_set_decode(msg, &set);

        // Check if this message is for this system
        if (is_sys_comp_match(set.target_system, set.target_component)) {
            char* key = (char*) set.param_id;
            for (uint16_t i = 0; i < ONBOARD_PARAM_COUNT; i++) {
                if (!strcmp(key, params.param_name[i])) {
                    if (params.param[i] != set.param_value
                            && set.param_type == MAVLINK_TYPE_FLOAT) {
                        params.param[i] = set.param_value;
                        mavlink_message_t message;
                        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
                        uint8_t len;

                        mavlink_msg_param_value_pack(mavlink_system.sysid,
                                mavlink_system.compid, &message,
                                params.param_name[i], params.param[i],
                                MAVLINK_TYPE_FLOAT,
                                ONBOARD_PARAM_COUNT, i);
                        len = mavlink_msg_to_send_buffer(buffer, &message);
                        BT_send(buffer, len);
                    }
                }
            }
        }
    }
    break;
    }
}

#endif /* MODULES_MAVLINK_HANDLERS_H_ */

/*
 * main.c
 *
 *  Created on: 14 февр. 2016 г.
 *      Author: kruci_000
 */

#include <Modules/Time/time.h>
#include <Modules/Time/timer.h>
#include <Modules/UART/uart.h>
#include <Modules/I2C/i2c.h>
#include <Modules/MPU6050/mpu6050.h>
#include <Modules/HMC/hmc.h>
#include <Modules/MAVLink/common/mavlink.h>
#include <Modules/MAVLink/handlers.h>
#include <Modules/MAVLink/system.h>
#include <Modules/Parameters_Holder/param_holder.h>
#include <Modules/PID/pid.h>

#include <Helpers/sys_helper/sys_helper.h>

#include <hal.h>
#include <MC_hw_platform.h>
#include <m2sxxx.h>

#include <defines.h>

// TODO remove it
extern uint16_t force;

void setup(void);

int main(void) {
    // Modules initialisation
    setup();

    uint8_t rx_c;
    float roll, pitch, yaw;
    float q[4];
    uint16_t pow[4];

    mavlink_message_t msg;
    mavlink_status_t status;

    while (1) {
        delay(1000);
#ifdef MPU6050_ENABLED
        MPU6050_getScaledData(
                &params.param[PARAM_AX],
                &params.param[PARAM_AY],
                &params.param[PARAM_AZ],
                &params.param[PARAM_GX],
                &params.param[PARAM_GY],
                &params.param[PARAM_GZ],
                &params.param[PARAM_T]);
#endif // MPU6050_ENABLED
#ifdef HMC_ENABLED
        HMC_get_scaled_Data(&params.param[PARAM_MX],
                &params.param[PARAM_MY],
                &params.param[PARAM_MZ]);
#endif // HMC_ENABLED

        // pid_update(q0, q1, q2, q3,
        q[0] = q0;
        q[1] = q1;
        q[2] = q2;
        q[3] = q3;

        mavlink_quaternion_to_euler(q, &roll, &pitch, &yaw);

        pid_update(
                roll,
                pitch,
                yaw,
                params.param[PARAM_GX],
                params.param[PARAM_GY],
                params.param[PARAM_GZ],
                force,
                pow);

        motors_set(pow);

        if (BT_get_rx(&rx_c, 1)) {
            if (mavlink_parse_char(MAVLINK_COMM_0, rx_c, &msg, &status)) {
                handle_mavlink_message(&msg);
            }
            param_queued_send_routine();
        }
    }
    return 0;
}

void setup() {
    timers_init();
    uart_init();
    i2c_init();
    motors_init();
    mavlink_sys_update(MAV_MODE_AUTO_ARMED, MAV_STATE_STANDBY);
    timer_mss1_start();
#ifdef MPU6050_ENABLED
    MPU6050_init();
#ifdef MPU6050_SELFTEST
    if (!MPU6050_selfTest()) {
        mavlink_message_t msg;

        // TODO retcode into msg
        mavlink_msg_statustext_pack(
                mavlink_system.sysid,
                mavlink_system.compid,
                &msg,
                MAV_SEVERITY_CRITICAL,
                "MPU6050 SELFTEST FAILED");
        mavlink_send_msg(&msg);
    }
#endif // MPU6050_SELFTEST
    MPU6050_calibration();
#endif // MPU6050_ENABLED
#ifdef HMC_ENABLED
    HMC_init();
#ifdef HMC_SELFTEST
    if (!HMC_self_test()) {
        mavlink_message_t msg;

        // TODO retcode into msg
        mavlink_msg_statustext_pack(
                mavlink_system.sysid,
                mavlink_system.compid,
                &msg,
                MAV_SEVERITY_CRITICAL,
                "HMC SELFTEST FAILED");
        mavlink_send_msg(&msg);
    }
#endif // HMC_SELFTEST
#endif // HMC_ENABLED
    timer_mss2_start();
}


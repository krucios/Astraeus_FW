/*
 * timer_handlers.c
 *
 *  Created on: 21 Jul 2016
 *      Author: S2400105
 */

#include "time.h"
#include "timer.h"
#include "defines.h"

#include <Helpers/conversion_defines.h>

#include <Modules/I2C/i2c.h>
#include <Modules/Parameters_Holder/param_holder.h>
#include <Modules/MAVLink/common/mavlink.h>
#include <Modules/AHRS/MadgwickAHRS.h>
#include <Modules/MPU6050/mpu6050.h>
#include <Modules/HMC/hmc.h>

/*------------------------------------------------------------------------------
 * Service the I2C timeout functionality.
 */
void SysTick_Handler(void) {
    I2C_system_tick(&g_core_i2c, 10);
}
void FabricIrq0_IRQHandler(void) {
    I2C_isr(&g_core_i2c);
}

/*-----------------------------------------------------------------------------
 * Service MAVlink protocol message routine
 */
void Timer1_IRQHandler() {
    mavlink_message_t msg;

    mavlink_msg_heartbeat_pack(
            mavlink_system.sysid,
            mavlink_system.compid,
            &msg,
            MAV_TYPE_GROUND_ROVER,
            MAV_AUTOPILOT_GENERIC,
            mavlink_sys_mode,
            0,
            mavlink_sys_state);
    mavlink_send_msg(&msg);

    mavlink_msg_sys_status_pack(
            mavlink_system.sysid,
            mavlink_system.compid,
            &msg,
            MAV_SYS_STATUS_SENSOR_3D_GYRO
            | MAV_SYS_STATUS_SENSOR_3D_ACCEL
            | MAV_SYS_STATUS_SENSOR_3D_MAG
            | MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE
            | MAV_SYS_STATUS_SENSOR_GPS,
            MAV_SYS_STATUS_SENSOR_3D_GYRO
            | MAV_SYS_STATUS_SENSOR_3D_ACCEL
            | MAV_SYS_STATUS_SENSOR_3D_MAG,
            MAV_SYS_STATUS_SENSOR_3D_GYRO
            | MAV_SYS_STATUS_SENSOR_3D_ACCEL
            | MAV_SYS_STATUS_SENSOR_3D_MAG
            | MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE
            | MAV_SYS_STATUS_SENSOR_GPS,
            50,                             // TODO remove hardoded values
            0,
            -1,
            -1,
            0,
            0,
            0,
            0,
            0,
            0);
    mavlink_send_msg(&msg);

    mavlink_msg_autopilot_version_pack(
            mavlink_system.sysid,
            mavlink_system.compid,
            &msg,
            0, // No capabilities (MAV_PROTOCOL_CAPABILITY). TBD
            42,
            42,
            42,
            42,
            "DEADBEEF",
            "DEADBEEF",
            "DEADBEEF",
            42,
            42,
            42);
    mavlink_send_msg(&msg);

    mavlink_msg_highres_imu_pack(
            mavlink_system.sysid,
            mavlink_system.compid,
            &msg,
            usec(),
            params.param[PARAM_AX],
            params.param[PARAM_AY],
            params.param[PARAM_AZ],
            params.param[PARAM_GX],
            params.param[PARAM_GY],
            params.param[PARAM_GZ],
            params.param[PARAM_MX],
            params.param[PARAM_MY],
            params.param[PARAM_MZ],
            0,
            0,
            0,
            params.param[PARAM_T],
            (1 << 12) | ((1 << 9) - 1));
    mavlink_send_msg(&msg);

    mavlink_msg_attitude_quaternion_pack(
            mavlink_system.sysid,
            mavlink_system.compid,
            &msg,
            usec(),
            q0,
            q1,
            q2,
            q3,
            params.param[PARAM_GX] * DEG_TO_RAD,
            params.param[PARAM_GY] * DEG_TO_RAD,
            params.param[PARAM_GZ] * DEG_TO_RAD);
    mavlink_send_msg(&msg);

    MadgwickAHRSupdateIMU(params.param[PARAM_GX] * DEG_TO_RAD,
                          params.param[PARAM_GY] * DEG_TO_RAD,
                          params.param[PARAM_GZ] * DEG_TO_RAD,
                          params.param[PARAM_AX],
                          params.param[PARAM_AY],
                          params.param[PARAM_AZ]);

    usec_service_routine();

    MSS_TIM1_clear_irq();
}


/*-------------------------------------
 * Read values from sensors
 */
void Timer2_IRQHandler() {
#ifdef MPU6050_ENABLED
    MPU6050_getScaledData(&params.param[PARAM_AX],
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
    MSS_TIM2_clear_irq();
}

/*
 * system.h
 *
 *  Created on: 20 Jul 2016
 *      Author: S2400105
 */

#ifndef MODULES_MAVLINK_SYSTEM_H_
#define MODULES_MAVLINK_SYSTEM_H_

#include "mavlink_types.h"

extern mavlink_system_t mavlink_system;

extern uint8_t mavlink_sys_mode;
extern uint8_t mavlink_sys_state;

void mavlink_sys_update(uint8_t mode, uint8_t state);

#endif /* MODULES_MAVLINK_SYSTEM_H_ */

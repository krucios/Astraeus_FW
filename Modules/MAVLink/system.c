/*
 * system.c
 *
 *  Created on: 21 июля 2016 г.
 *      Author: kruci_000
 */

#include "system.h"

mavlink_system_t mavlink_system  = {.sysid = 20,
                                    .compid = 0};
uint8_t mavlink_sys_mode;
uint8_t mavlink_sys_state;

inline void mavlink_sys_update(uint8_t mode, uint8_t state) {
   mavlink_sys_mode = mode;
   mavlink_sys_state = state;
}



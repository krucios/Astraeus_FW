/*
 * uart.h
 *
 *  Created on: 20 Jul 2016
 *      Author: S2400105
 */

#ifndef MODULES_UART_UART_H_
#define MODULES_UART_UART_H_

#include <MC_hw_platform.h>
#include <drivers/CoreUARTapb/core_uart_apb.h>

#include "BT/bluetooth.h"

#define BAUD_VALUE_115200   26

void uart_init();

#endif /* MODULES_UART_UART_H_ */

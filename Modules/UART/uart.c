/*
 * uart.c
 *
 *  Created on: 21 июля 2016 г.
 *      Author: kruci_000
 */

#include "uart.h"

inline void uart_init() {
    BT_init(COREUARTAPB_0_0, BAUD_VALUE_115200);
}



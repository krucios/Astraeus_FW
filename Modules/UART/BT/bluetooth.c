/*
 * bluetooth.c
 *
 *  Created on: 21 июля 2016 г.
 *      Author: kruci_000
 */

#include "bluetooth.h"

UART_instance_t g_bt;

inline void BT_init(addr_t base_addr, uint16_t baud_val) {
    UART_init(&g_bt, base_addr, baud_val, (DATA_8_BITS | NO_PARITY));
}

inline void BT_send(const uint8_t* tx_buf, size_t size) {
    UART_send(&g_bt, tx_buf, size);
}

inline size_t BT_get_rx(uint8_t* rx_buffer, size_t buffer_size) {
    return UART_get_rx(&g_bt, rx_buffer, buffer_size);
}

inline void BT_polled_tx_string(const uint8_t* str) {
    UART_polled_tx_string(&g_bt, str);
}



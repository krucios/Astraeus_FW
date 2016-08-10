/*
 * bmp180.c
 *
 *  Created on: 11 рту. 2016 у.
 *      Author: kruci_000
 */

#include "bmp180.h"

struct bmp180_t bmp180;

static float _p0;

int8_t bus_write(uint8_t addr, uint8_t reg, uint8_t* data, uint8_t len) {
    return (uint8_t)i2c_write(addr, reg, data, len);
}

int8_t bus_read(uint8_t addr, uint8_t reg, uint8_t* data, uint8_t len) {
    return (uint8_t)i2c_read(addr, reg, data, len);
}

uint8_t BMP_init(void) {
    uint8_t result;

    bmp180.bus_write = bus_write;
    bmp180.bus_read = bus_read;
    bmp180.dev_addr = BMP180_I2C_ADDR;
    bmp180.delay_msec = delay_ms;

    result = bmp180_init(&bmp180);
    if (!result) {
        _p0 = BMP_get_preasure();
    }
    return result;
}

int16_t BMP_get_temperature() {
    return bmp180_get_temperature(bmp180_get_uncomp_temperature());
}

int32_t BMP_get_preasure() {
    return bmp180_get_pressure(bmp180_get_uncomp_pressure());
}

float BMP_get_altitude(float p) {
    float dp  = p / _p0;
    float dpd = powf(dp, 0.1902949571836346336822074215f);
    return 44330 * (1 - dpd);
}

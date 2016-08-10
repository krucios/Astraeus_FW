/*
 * bmp180.h
 *
 *  Created on: 11 рту. 2016 у.
 *      Author: kruci_000
 */

#ifndef MODULES_BMP180_BMP180_H_
#define MODULES_BMP180_BMP180_H_

#include "bmp180_driver.h"

#include <stdint.h>
#include <math.h>
#include <Modules/I2C/i2c.h>
#include <Helpers/sys_helper/sys_helper.h>

extern struct bmp180_t bmp180;

uint8_t BMP_init(void);

int16_t BMP_get_temperature();
int32_t BMP_get_preasure();
float   BMP_get_altitude(float p);

#endif /* MODULES_BMP180_BMP180_H_ */

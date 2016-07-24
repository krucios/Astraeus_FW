

#ifndef HMC_H_
#define HMC_H_

#include <Modules/I2C/i2c.h>
#include <Helpers/sys_helper/sys_helper.h>

#define HMC_SERIAL_ADDR 0x1E

#define HMC_CFG_A_REG  0x00
#define HMC_CFG_B_REG  0x01
#define HMC_MODE_REG   0x02
#define HMC_X_MSB_REG  0x03
#define HMC_X_LSB_REG  0x04
#define HMC_Z_MSB_REG  0x05
#define HMC_Z_LSB_REG  0x06
#define HMC_Y_MSB_REG  0x07
#define HMC_Y_LSB_REG  0x08
#define HMC_STATUS_REG 0x09
#define HMC_IDNT_A_REG 0x0A
#define HMC_IDNT_B_REG 0x0B
#define HMC_IDNT_C_REG 0x0C

#define HMC_MEASURE_CONTIN  0x0
#define HMC_MEASURE_SINGLE  0x1
#define HMC_MEASURE_IDLE    0x2

#define HMC_MODE_NORMAL     0x0
#define HMC_MODE_POS_BIAS   0x1
#define HMC_MODE_NEG_BIAS   0x2

#define HMC_SELFTEST_POS_5_MIN    243
#define HMC_SELFTEST_POS_5_MAX    575


#define HMC_CALIBRATION_SAMPLES_COUNT 1024

void HMC_init();
void HMC_calibration();

int8_t HMC_self_test();

void HMC_get_raw_Data(int16_t* mx, int16_t* my, int16_t* mz);
void HMC_get_Data(int16_t* mx, int16_t* my, int16_t* mz);
void HMC_get_scaled_Data(float* mx, float* my, float* mz);

// TODO implement it! int8_t HMC_setMesMode(uint8_t mode);
int8_t HMC_setMode(uint8_t mode);
int8_t HMC_setScale(float gauss);

#endif /* HMC_H_ */

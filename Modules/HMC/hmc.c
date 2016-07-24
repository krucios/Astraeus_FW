#include "hmc.h"

static int16_t hard_iron[3] = {0, 0, 0}; // X, Y, Z hard iron biases
static int16_t soft_n[3]    = {0, 0, 0}; // X, Y, Z soft iron numerators
static int16_t soft_d[3]    = {0, 0, 0}; // X, Y, Z soft iron denominators

float digital_resolution = 0.0;

void HMC_init() {
    uint8_t tx_len = 2;
    uint8_t tx_buf[tx_len];

    tx_buf[0] = HMC_CFG_A_REG;
    tx_buf[1] = 0x74; // Sample rate: 30 Hz and 8 averaged measurement per sample

    i2c_writeBytes(HMC_SERIAL_ADDR, tx_buf, tx_len);

    HMC_setScale(10.0f);
    HMC_setMode(HMC_MEASURE_CONTIN);

    HMC_calibration();
}

void HMC_calibration() {
    uint16_t i = 0;
    int16_t mag_max[3]  = {0x8000, 0x8000, 0x8000},
            mag_min[3]  = {0x7FFF, 0x7FFF, 0x7FFF},
            mag_temp[3] = {0, 0, 0};

    for(i = 0; i < HMC_CALIBRATION_SAMPLES_COUNT; i++) {
        HMC_get_raw_Data(&mag_temp[0], &mag_temp[1], &mag_temp[2]);
        for (int j = 0; j < 3; j++) {
            if(mag_temp[j] > mag_max[j]) mag_max[j] = mag_temp[j];
            if(mag_temp[j] < mag_min[j]) mag_min[j] = mag_temp[j];
        }
        delay(35000);  // at 30 Hz ODR, new mag data is available every 35 ms
    }

   // Get hard iron correction
    hard_iron[0]  = ((uint32_t)mag_max[0] + mag_min[0]) / 2;
    hard_iron[1]  = ((uint32_t)mag_max[1] + mag_min[1]) / 2;
    hard_iron[2]  = ((uint32_t)mag_max[2] + mag_min[2]) / 2;

   // Get soft iron correction estimate
    soft_d[0]  = (mag_max[0] - mag_min[0]) / 2;
    soft_d[1]  = (mag_max[1] - mag_min[1]) / 2;
    soft_d[2]  = (mag_max[2] - mag_min[2]) / 2;

    uint32_t avg = (soft_d[0] + soft_d[1] + soft_d[2]) / 3;

    soft_n[0] = avg;
    soft_n[1] = avg;
    soft_n[2] = avg;
}

int8_t HMC_self_test() {
    uint8_t retcode = 0;

    int16_t mx, my, mz;

    // TODO remove this copypaste
    uint8_t tx_len = 2;
    uint8_t tx_buf[tx_len];

    tx_buf[0] = HMC_CFG_A_REG;
    tx_buf[1] = 0x71; // Sample rate: 30 Hz and 8 averaged measurement per sample

    i2c_writeBytes(HMC_SERIAL_ADDR, tx_buf, tx_len);

    HMC_setScale(5.0);
    HMC_setMode(HMC_MEASURE_CONTIN);

    delay(6000); // Wait for 6 ms

    HMC_get_raw_Data(&mx, &my, &mz);

    if (mx <= HMC_SELFTEST_POS_5_MIN ||
        mx >= HMC_SELFTEST_POS_5_MAX) {
        retcode = 1;
    }
    if (my <= HMC_SELFTEST_POS_5_MIN ||
        my >= HMC_SELFTEST_POS_5_MAX) {
        retcode = 2;
    }
    if (mz <= HMC_SELFTEST_POS_5_MIN ||
        mz >= HMC_SELFTEST_POS_5_MAX) {
        retcode = 3;
    }

    return retcode;
}

void HMC_get_raw_Data(int16_t* mx, int16_t* my, int16_t* mz) {
    uint8_t rx_len = 6;
    uint8_t rx_buf[rx_len];

    uint8_t tx_len = 1;
    uint8_t tx_buf[tx_len];

    tx_buf[0] = HMC_X_MSB_REG;

    i2c_writeBytes(HMC_SERIAL_ADDR, tx_buf, tx_len); // transfer reg addr
    i2c_readBytes(HMC_SERIAL_ADDR, rx_buf, rx_len); // read all data regs

    // Because of wrong assembled sensor
    *mx = (rx_buf[2] << 8) | rx_buf[3];
    *my = (rx_buf[0] << 8) | rx_buf[1];
    *mz = (rx_buf[4] << 8) | rx_buf[5];
}

void HMC_get_Data(int16_t* mx, int16_t* my, int16_t* mz) {
    int16_t t_x, t_y, t_z;
    HMC_get_raw_Data(&t_x, &t_y, &t_z);

    t_x = t_x - hard_iron[0];
    t_y = t_y - hard_iron[1];
    t_z = t_z - hard_iron[2];

    *mx = ((int32_t)t_x * soft_n[0]) / soft_d[0];
    *my = ((int32_t)t_y * soft_n[1]) / soft_d[1];
    *mz = ((int32_t)t_z * soft_n[2]) / soft_d[2];
}

void HMC_get_scaled_Data(float* mx, float* my, float* mz) {
    int16_t t_x, t_y, t_z;
    HMC_get_Data(&t_x, &t_y, &t_z);

    *mx = digital_resolution * t_x;
    *my = digital_resolution * t_y;
    *mz = digital_resolution * t_z;
}

int8_t HMC_setMode(uint8_t mode) {
    int8_t error_code = 0;
    if (mode <= HMC_MEASURE_IDLE) {
        i2c_writeBytes(HMC_MODE_REG, &mode, 1);
    } else
        error_code = -1;
    return error_code;
}

// TODO accept predefined value instead of float
int8_t HMC_setScale(float gauss) {
    uint8_t reg_value = 0x00;
    int8_t error_code = 0;

    if (gauss <= 0.88) {
        reg_value = 0x00;
        digital_resolution = 0.73;
    } else if (gauss <= 1.3) {
        reg_value = 0x01;
        digital_resolution = 0.92;
    } else if (gauss <= 1.9) {
        reg_value = 0x02;
        digital_resolution = 1.22;
    } else if (gauss <= 2.5) {
        reg_value = 0x03;
        digital_resolution = 1.52;
    } else if (gauss <= 4.0) {
        reg_value = 0x04;
        digital_resolution = 2.27;
    } else if (gauss <= 4.7) {
        reg_value = 0x05;
        digital_resolution = 2.56;
    } else if (gauss <= 5.6) {
        reg_value = 0x06;
        digital_resolution = 3.03;
    } else if (gauss <= 8.1) {
        reg_value = 0x07;
        digital_resolution = 4.35;
    } else
        error_code = -1;

    if (!error_code) {
        // Setting is in the top 3 bits of the register.
        reg_value = reg_value << 5;
        i2c_writeBytes(HMC_CFG_B_REG, &reg_value, 1);
    }
    return error_code;
}


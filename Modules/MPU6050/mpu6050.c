#include "mpu6050.h"

#include <math.h>

static uint8_t cur_gyro_range;
static uint8_t cur_accel_range;

static int16_t c_ax, c_ay, c_az, c_gx, c_gy, c_gz, c_t;

void MPU6050_init() {
    MPU6050_setClockSource(MPU6050_CLOCK_PLL_XGYRO);
    MPU6050_setFullScaleGyroRange(MPU6050_GYRO_FS_1000);
    MPU6050_setFullScaleAccelRange(MPU6050_ACCEL_FS_4);
    MPU6050_setBypassMode(1);
    MPU6050_setSleepEnabled(0); // thanks to Jack Elston for pointing this one out!

    MPU6050_setDLPFMode(MPU6050_DLPF_BW_42);
}

uint8_t MPU6050_selfTest() {
    int16_t i;
    uint8_t retcode = 0;

    uint8_t a_test[3]; // X Y Z accel test values
    int16_t a_s[3]; //       accel values with self test
    int16_t a[3]; //       accle values without self test
    int16_t a_str[3]; //       accel self test response
    float ft_a[3]; //       accel factory trim value
    float ch_a[3]; //       accel change from factory trim

    uint8_t g_test[3]; // X Y Z gyro test values
    int16_t g_s[3]; //       gyro values with self test
    int16_t g[3]; //       gyro values without self test
    int16_t g_str[3]; //       gyro self test response
    float ft_g[3]; //       gyro factory trim value
    float ch_g[3]; //       gyro change from factory trim

    uint8_t prev_gyro_range = cur_gyro_range;
    uint8_t prev_accel_range = cur_accel_range;

    MPU6050_setFullScaleGyroRange(MPU6050_GYRO_FS_250);
    MPU6050_setFullScaleAccelRange(MPU6050_ACCEL_FS_2);

    uint8_t buff[4];
    buff[0] = MPU6050_SELF_TEST_X;

    /*-------------------------------*
     * Read SelfTest registers 13-16
     *-------------------------------*/
    i2c_writeBytes(MPU6050_ADDRESS, buff, 1); // Send start address
    i2c_readBytes(MPU6050_ADDRESS, buff, 4);  // Read all 4 self test registers

    a_test[0] = ((buff[0] & 0b1110000) >> 3) + ((buff[3] & 00110000) >> 4);
    a_test[1] = ((buff[1] & 0b1110000) >> 3) + ((buff[3] & 00001100) >> 2);
    a_test[2] = ((buff[2] & 0b1110000) >> 3) + (buff[3] & 00000011);

    for (i = 0; i < 3; i++) {
        g_test[i] = buff[i] & 0b00011111;
    }

    /*------------------------------*
     * Calculate Factory Trim Value
     *------------------------------*/
    for (i = 0; i < 3; i++) {
        if (a_test[i]) {
            ft_a[i] = 4096 * 0.34 * powf(0.92 / 0.34, (a_test[i] - 1) / 30.0);
        } else
            ft_a[i] = 0.0;
        if (g_test[i]) {
            ft_g[i] = 25 * 131 * powf(1.046, (float) (g_test[i] - 1));
        } else
            ft_g[i] = 0.0;
    }

    /*------------------------------*
     * Enable self test
     *------------------------------*/
    i2c_writeBits(MPU6050_ADDRESS, MPU6050_RA_ACCEL_CONFIG, 7, 3, 0b111);
    i2c_writeBits(MPU6050_ADDRESS, MPU6050_RA_GYRO_CONFIG, 7, 3, 0b111);

    MPU6050_getRawData(&a_s[0], &a_s[1], &a_s[2], &g_s[0], &g_s[1], &g_s[2],
            &i);

    /*------------------------------*
     * Disable self test
     *------------------------------*/
    i2c_writeBits(MPU6050_ADDRESS, MPU6050_RA_ACCEL_CONFIG, 7, 3, 0b000);
    i2c_writeBits(MPU6050_ADDRESS, MPU6050_RA_GYRO_CONFIG, 7, 3, 0b000);

    MPU6050_getRawData(&a[0], &a[1], &a[2], &g[0], &g[1], &g[2], &i);

    /*------------------------------*
     * Calculate change from factory trim
     *------------------------------*/
    for (i = 0; i < 3; i++) {
        a_str[i] = a_s[i] - a[i];
        ch_a[i] = (a_str[i] - ft_a[i]) / ft_a[i];
        if (ch_a[i] <= MPU6050_ACCEL_SELF_TEST_RESPONSE_MIN
                || ch_a[i] > MPU6050_ACCEL_SELF_TEST_RESPONSE_MAX) {
            retcode = 1;
        }
        g_str[i] = g_s[i] - g[i];
        ch_g[i] = (g_str[i] - ft_g[i]) / ft_g[i];
        if (ch_g[i] <= MPU6050_GYRO_SELF_TEST_RESPONSE_MIN
                || ch_g[i] > MPU6050_GYRO_SELF_TEST_RESPONSE_MAX) {
            retcode = 2;
        }
    }

    MPU6050_setFullScaleGyroRange(prev_gyro_range);
    MPU6050_setFullScaleAccelRange(prev_accel_range);

    return retcode;
}

void MPU6050_setClockSource(uint8_t source) {
    i2c_writeBits(MPU6050_ADDRESS,
    MPU6050_RA_PWR_MGMT_1,
    MPU6050_PWR1_CLKSEL_BIT,
    MPU6050_PWR1_CLKSEL_LENGTH, source);
}

void MPU6050_setFullScaleGyroRange(uint8_t range) {
    cur_gyro_range = range;
    i2c_writeBits(MPU6050_ADDRESS,
    MPU6050_RA_GYRO_CONFIG,
    MPU6050_GCONFIG_FS_SEL_BIT,
    MPU6050_GCONFIG_FS_SEL_LENGTH, range);
}

void MPU6050_setFullScaleAccelRange(uint8_t range) {
    cur_accel_range = range;
    i2c_writeBits(MPU6050_ADDRESS,
    MPU6050_RA_ACCEL_CONFIG,
    MPU6050_ACONFIG_AFS_SEL_BIT,
    MPU6050_ACONFIG_AFS_SEL_LENGTH, range);
}

void MPU6050_setSleepEnabled(uint8_t enabled) {
    i2c_writeBit(MPU6050_ADDRESS,
    MPU6050_RA_PWR_MGMT_1,
    MPU6050_PWR1_SLEEP_BIT, enabled);
}

void MPU6050_setBypassMode(uint8_t enabled) {
    i2c_writeBit(MPU6050_ADDRESS,
    MPU6050_RA_INT_PIN_CFG,
    MPU6050_INTCFG_I2C_BYPASS_EN_BIT, enabled);
}

void MPU6050_setDLPFMode(uint8_t mode) {
    i2c_writeBits(MPU6050_ADDRESS,
    MPU6050_RA_CONFIG,
    MPU6050_CFG_DLPF_CFG_BIT,
    MPU6050_CFG_DLPF_CFG_LENGTH, mode);
}

void MPU6050_calibration() {
    uint16_t i;

    int16_t _ax;
    int16_t _ay;
    int16_t _az;
    int16_t _gx;
    int16_t _gy;
    int16_t _gz;
    int16_t _t;

    MPU6050_getRawData(&c_ax, &c_ay, &c_az, &c_gx, &c_gy, &c_gz, &c_t);
    for (i = 0; i < MPU_CALIBRATION_SAMPLES_COUNT; i++) {
        MPU6050_getRawData(&_ax, &_ay, &_az, &_gx, &_gy, &_gz, &_t);
        c_ax = ((uint32_t) (c_ax + _ax)) / 2;
        c_ay = ((uint32_t) (c_ay + _ay)) / 2;
        c_az = ((uint32_t) (c_az + _az)) / 2;
        c_gx = ((uint32_t) (c_gx + _gx)) / 2;
        c_gy = ((uint32_t) (c_gy + _gy)) / 2;
        c_gz = ((uint32_t) (c_gz + _gz)) / 2;
        c_t = ((uint32_t) (c_t + _t)) / 2;
    }
}

void MPU6050_getRawData(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx,
        int16_t* gy, int16_t* gz, int16_t* t) {
    uint8_t tx_len = 1;
    uint8_t tx_buf[tx_len];

    tx_buf[0] = MPU6050_RA_ACCEL_XOUT_H;

    uint8_t rx_len = 14;
    uint8_t rx_buf[rx_len];

    i2c_writeBytes(MPU6050_ADDRESS, tx_buf, tx_len);
    i2c_readBytes(MPU6050_ADDRESS, rx_buf, rx_len);

    // Because of wrong assembled sensor
    *ax = (int16_t) (((rx_buf[0]) << 8) | rx_buf[1]);
    *ay = (int16_t) (((rx_buf[2]) << 8) | rx_buf[3]);
    *az = (int16_t) -(((rx_buf[4]) << 8) | rx_buf[5]);
    *t = (int16_t) (((rx_buf[6]) << 8) | rx_buf[7]);
    *gx = (int16_t) -(((rx_buf[8]) << 8) | rx_buf[9]);
    *gy = (int16_t) -(((rx_buf[10]) << 8) | rx_buf[11]);
    *gz = (int16_t) (((rx_buf[12]) << 8) | rx_buf[13]);
}

void MPU6050_getData(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx,
        int16_t* gy, int16_t* gz, int16_t* t) {
    MPU6050_getRawData(ax, ay, az, gx, gy, gz, t);

    if (USE_CALIBRATED_MASK & CALIBRATED_ACCX) {
        *ax -= c_ax;
    }
    if (USE_CALIBRATED_MASK & CALIBRATED_ACCY) {
        *ay -= c_ay;
    }
    if (USE_CALIBRATED_MASK & CALIBRATED_ACCZ) {
        *az -= c_az;
    }
    if (USE_CALIBRATED_MASK & CALIBRATED_GYROX) {
        *gx -= c_gx;
    }
    if (USE_CALIBRATED_MASK & CALIBRATED_GYROY) {
        *gy -= c_gy;
    }
    if (USE_CALIBRATED_MASK & CALIBRATED_GYROZ) {
        *gz -= c_gz;
    }
    if (USE_CALIBRATED_MASK & CALIBRATED_TEMP) {
        *t -= c_t;
    }
}

void MPU6050_getScaledData(float* ax, float* ay, float* az, float* gx,
        float* gy, float* gz, float* t) {
    int16_t _ax;
    int16_t _ay;
    int16_t _az;
    int16_t _gx;
    int16_t _gy;
    int16_t _gz;
    int16_t _t;

    float accel_max;
    float gyro_max;

    float accel_factor;
    float gyro_factor;

    MPU6050_getData(&_ax, &_ay, &_az, &_gx, &_gy, &_gz, &_t);

    switch (cur_accel_range) {
    case MPU6050_ACCEL_FS_2:
        accel_max = 2 * G_CONST;
        break;
    case MPU6050_ACCEL_FS_4:
        accel_max = 4 * G_CONST;
        break;
    case MPU6050_ACCEL_FS_8:
        accel_max = 8 * G_CONST;
        break;
    case MPU6050_ACCEL_FS_16:
        accel_max = 16 * G_CONST;
        break;
    default:
        accel_max = 0.0f;
    }

    switch (cur_gyro_range) {
    case MPU6050_GYRO_FS_250:
        gyro_max = 250.0f;
        break;
    case MPU6050_GYRO_FS_500:
        gyro_max = 500.0f;
        break;
    case MPU6050_GYRO_FS_1000:
        gyro_max = 1000.0f;
        break;
    case MPU6050_GYRO_FS_2000:
        gyro_max = 2000.0f;
        break;
    default:
        gyro_max = 0.0f;
        break;
    }

    accel_factor = accel_max / INT_LEAST16_MAX;
    gyro_factor = gyro_max / INT_LEAST16_MAX;

    *ax = _ax * accel_factor;
    *ay = _ay * accel_factor;
    *az = _az * accel_factor;

    *gx = _gx * gyro_factor;
    *gy = _gy * gyro_factor;
    *gz = _gz * gyro_factor;

    *t = (_t + 12420.2f) / 340; // According to formula from Register Map
}


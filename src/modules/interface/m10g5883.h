/**
 * @file m10g5883.h
 * @brief Driver interface for Walksnail M10G-5883 GNSS/Mag module
 */

#ifndef M10G5883_H_
#define M10G5883_H_

#include <stdint.h>
#include <stdbool.h>

// I2C Address
#define QMC5883L_I2C_ADDR       0x0D
#define M10_BAUDRATE_DEFAULT    115200

// QMC5883L Register Map
#define QMC5883L_REG_DATA_X_LSB 0x00
#define QMC5883L_REG_STATUS     0x06
#define QMC5883L_REG_TEMP_LSB   0x07
#define QMC5883L_REG_CONF_1     0x09
#define QMC5883L_REG_CONF_2     0x0A
#define QMC5883L_REG_PERIOD     0x0B
#define QMC5883L_REG_CHIP_ID    0x0D

// 로그용 데이터 구조체
typedef struct {
    uint8_t fixType;
    uint8_t numSV;
    int32_t lat;
    int32_t lon;
    int32_t hMSL;
    uint32_t hAcc;
    bool updated;
} m10g_gps_log_t;

typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} m10g_mag_log_t;

#endif /* M10G5883_H_ */
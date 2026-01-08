#ifndef __QMC5883L_H__
#define __QMC5883L_H__

#include <stdbool.h>
#include <stdint.h>
#include "i2cdev.h"

// QMC5883L I2C Address
#define QMC5883L_I2C_ADDR 0x0D

/**
 * @brief Initialize the QMC5883L sensor with PX4-style robust sequence
 * @param i2cPort The I2C port to use (usually I2C1_DEV)
 * @return true if initialization and validation successful
 */
bool qmc5883lInit(I2C_Dev *i2cPort);

/**
 * @brief Check if the sensor is initialized
 */
bool qmc5883lTestConnection(void);

/**
 * @brief Poll for new data based on DRDY status
 * @param x, y, z Pointers to store the raw axis data
 * @return true if new data was read successfully, false if not ready or error
 */
bool qmc5883lGetData(int16_t *x, int16_t *y, int16_t *z);

#endif // __QMC5883L_H__
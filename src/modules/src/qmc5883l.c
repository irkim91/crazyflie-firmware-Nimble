#include "qmc5883l.h"
#include "debug.h"
#include "FreeRTOS.h"
#include "task.h"

// --- Registers ---
#define QMC_REG_X_LSB       0x00
#define QMC_REG_X_MSB       0x01
#define QMC_REG_Y_LSB       0x02
#define QMC_REG_Y_MSB       0x03
#define QMC_REG_Z_LSB       0x04
#define QMC_REG_Z_MSB       0x05
#define QMC_REG_STATUS      0x06
#define QMC_REG_TEMP_LSB    0x07
#define QMC_REG_TEMP_MSB    0x08
#define QMC_REG_CONF_1      0x09
#define QMC_REG_CONF_2      0x0A
#define QMC_REG_PERIOD      0x0B

// --- Configuration Bits ---
// Mode: Continuous(01)
// ODR: 200Hz(11) -> PX4 Recommended
// RNG: 8G(01) -> Better for handling motor interference
// OSR: 512(00) -> High oversampling for less noise
// Binary: 00 01 11 01 = 0x1D
#define QMC_VAL_CONF_1      0x1D 

// Soft Reset: Bit 7 (0x80)
// Interrupt Disable: Bit 0 (0x01 is disable? No, QMC is 0 to disable usually, depends on datasheet version, generally 0x00 is fine implies Rollover enabled)
#define QMC_VAL_SOFT_RESET  0x80
#define QMC_VAL_NORMAL      0x00 

// Status Bits
#define QMC_STATUS_DRDY     0x01 // Data Ready
#define QMC_STATUS_OVL      0x02 // Overflow
#define QMC_STATUS_DOR      0x04 // Data Skipped

static I2C_Dev *i2cPort;
static bool isInit = false;

bool qmc5883lInit(I2C_Dev *i2cPortIn)
{
  i2cPort = i2cPortIn;
  isInit = false;

  // 1. Soft Reset (PX4 Logic: Always reset first)
  // 레지스터가 꼬여있을 수 있으므로 리셋을 먼저 수행합니다.
  i2cdevWriteByte(i2cPort, QMC5883L_I2C_ADDR, QMC_REG_CONF_2, QMC_VAL_SOFT_RESET);
  vTaskDelay(M2T(10)); // 10ms Wait for reset

  // 2. Control Register 2 Setting
  // Soft Reset 비트 해제 및 일반 모드 설정
  i2cdevWriteByte(i2cPort, QMC5883L_I2C_ADDR, QMC_REG_CONF_2, QMC_VAL_NORMAL);

  // 3. Set Period (FBR)
  // Recommended 0x01 for 200Hz ODR
  i2cdevWriteByte(i2cPort, QMC5883L_I2C_ADDR, QMC_REG_PERIOD, 0x01);

  // 4. Main Configuration (Rate, Range, Mode)
  i2cdevWriteByte(i2cPort, QMC5883L_I2C_ADDR, QMC_REG_CONF_1, QMC_VAL_CONF_1);

  // 5. Validation (Read Back)
  // 쓴 값이 제대로 들어갔는지 확인합니다.
  uint8_t checkVal;
  if (i2cdevReadByte(i2cPort, QMC5883L_I2C_ADDR, QMC_REG_CONF_1, &checkVal))
  {
      if (checkVal == QMC_VAL_CONF_1) {
          isInit = true;
          return true;
      } else {
          DEBUG_PRINT("QMC5883L Config Mismatch! Read: 0x%02X\n", checkVal);
      }
  } else {
      DEBUG_PRINT("QMC5883L I2C Read Error during Init\n");
  }

  return false;
}

bool qmc5883lTestConnection(void)
{
  return isInit;
}

bool qmc5883lGetData(int16_t *x, int16_t *y, int16_t *z)
{
  if (!isInit) return false;

  uint8_t status;
  
  // 1. Check Data Ready (PX4 Logic)
  // 무조건 읽지 않고, 데이터가 준비되었는지 먼저 확인합니다.
  if (i2cdevReadByte(i2cPort, QMC5883L_I2C_ADDR, QMC_REG_STATUS, &status))
  {
      if (status & QMC_STATUS_DRDY) 
      {
          uint8_t buf[6];
          // 2. Burst Read 6 Bytes (X, Y, Z)
          if (i2cdevRead(i2cPort, QMC5883L_I2C_ADDR, QMC_REG_X_LSB, 6, buf))
          {
            // 3. Convert Little Endian
            *x = (int16_t)(buf[1] << 8 | buf[0]);
            *y = (int16_t)(buf[3] << 8 | buf[2]);
            *z = (int16_t)(buf[5] << 8 | buf[4]);
            return true;
          }
      }
  }
  
  return false;
}
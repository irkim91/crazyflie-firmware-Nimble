/**
 * mti3_deck.c
 * Driver for Xsens MTi-3 AHRS Module connected via UART (Expansion Deck)
 */

#define DEBUG_MODULE "MTI3"

#include <stdint.h>
#include <string.h>
#include <math.h> // 수학 함수 사용

#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "deck.h"
#include "debug.h"
#include "uart1.h" // UART1 is on the expansion deck
#include "log.h"
#include "mti3_deck.h" 

// --- Configuration ---
#define MTI3_BAUDRATE 921600
#define XBUS_PREAMBLE 0xFA
#define XBUS_BID      0xFF
#define RAD2DEG       57.2957795131f

// --- [수정 1] 누락되었던 Enum 정의 복구 ---
typedef enum {
    WAIT_PREAMBLE,
    WAIT_BID,
    WAIT_MID,
    WAIT_LEN,
    WAIT_DATA,
    WAIT_CS
} XbusState_t;

// --- Internal Variables ---
static Mti3Data_t mti3Data;
static SemaphoreHandle_t dataMutex;
static bool isConnected = false;

// --- Helper Functions ---
static float parseFloat(const uint8_t* buffer) {
    uint32_t temp = 0;
    temp |= (uint32_t)buffer[0] << 24;
    temp |= (uint32_t)buffer[1] << 16;
    temp |= (uint32_t)buffer[2] << 8;
    temp |= (uint32_t)buffer[3];
    return *((float*)&temp);
}

static void quatToEuler(float q0, float q1, float q2, float q3, float* r, float* p, float* y) {
    // Roll (x-axis rotation)
    float sinr_cosp = 2.0f * (q0 * q1 + q2 * q3);
    float cosr_cosp = 1.0f - 2.0f * (q1 * q1 + q2 * q2);
    *r = atan2f(sinr_cosp, cosr_cosp) * RAD2DEG;

    // Pitch (y-axis rotation)
    float sinp = 2.0f * (q0 * q2 - q3 * q1);
    if (fabsf(sinp) >= 1.0f)
        *p = copysignf(90.0f, sinp);
    else
        *p = asinf(sinp) * RAD2DEG;

    // Yaw (z-axis rotation)
    float siny_cosp = 2.0f * (q0 * q3 + q1 * q2);
    float cosy_cosp = 1.0f - 2.0f * (q2 * q2 + q3 * q3);
    *y = atan2f(siny_cosp, cosy_cosp) * RAD2DEG;
}

// --- Interface Functions ---
bool mti3GetData(Mti3Data_t* outData) {
    if (!isConnected || dataMutex == NULL) return false;

    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(2)) == pdTRUE) {
        memcpy(outData, &mti3Data, sizeof(Mti3Data_t));
        mti3Data.isUpdated = false;
        xSemaphoreGive(dataMutex);
        return true;
    }
    return false;
}

bool mti3IsConnected(void) {
    return isConnected;
}

// --- Main Task ---
static void mti3Task(void* p) {
    uart1Init(MTI3_BAUDRATE);
    dataMutex = xSemaphoreCreateMutex();

    XbusState_t state = WAIT_PREAMBLE;
    
    // [수정 2] 'mid' 변수 제거 (사용하지 않아 에러 발생함)
    uint8_t length = 0;
    uint8_t checksum = 0;
    uint8_t rxByte;
    uint8_t dataBuffer[256];
    uint8_t dataIdx = 0;

    vTaskDelay(M2T(100));
    DEBUG_PRINT("MTi-3 Driver Started\n");

    while(1) {
        if (uart1GetDataWithTimeout(&rxByte, 10)) {
            
            if (state != WAIT_PREAMBLE) {
                checksum += rxByte;
            }

            switch(state) {
                case WAIT_PREAMBLE:
                    if (rxByte == XBUS_PREAMBLE) {
                        state = WAIT_BID;
                        checksum = 0;
                    }
                    break;

                case WAIT_BID:
                    // BID 확인 (0xFF)
                    state = (rxByte == XBUS_BID) ? WAIT_MID : WAIT_PREAMBLE;
                    break;

                case WAIT_MID:
                    // MID 확인 (0x36: MTData2)
                    state = (rxByte == 0x36) ? WAIT_LEN : WAIT_PREAMBLE;
                    break;

                case WAIT_LEN:
                    length = rxByte;
                    dataIdx = 0;
                    // 길이 유효성 체크
                    state = (length < 256) ? WAIT_DATA : WAIT_PREAMBLE;
                    break;

                case WAIT_DATA:
                    dataBuffer[dataIdx++] = rxByte;
                    if (dataIdx >= length) {
                        state = WAIT_CS;
                    }
                    break;

                case WAIT_CS:
                    if (checksum == 0) {
                        // Checksum Valid
                        if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(2)) == pdTRUE) {
                            uint8_t ptr = 0;
                            while (ptr < length) {
                                uint16_t dataId = (dataBuffer[ptr] << 8) | dataBuffer[ptr+1];
                                uint8_t dataLen = dataBuffer[ptr+2];
                                ptr += 3;

                                // Data ID 확인 및 파싱
                                switch (dataId & 0xF0F0) {
                                    case 0x2010: // Quaternion
                                        mti3Data.q0 = parseFloat(&dataBuffer[ptr]);
                                        mti3Data.q1 = parseFloat(&dataBuffer[ptr+4]);
                                        mti3Data.q2 = parseFloat(&dataBuffer[ptr+8]);
                                        mti3Data.q3 = parseFloat(&dataBuffer[ptr+12]);
                                        quatToEuler(mti3Data.q0, mti3Data.q1, mti3Data.q2, mti3Data.q3,
                                                    &mti3Data.roll, &mti3Data.pitch, &mti3Data.yaw);
                                        break;
                                    
                                    case 0x8040: // Rate of Turn (Gyro)
                                        mti3Data.gyr[0] = parseFloat(&dataBuffer[ptr]);
                                        mti3Data.gyr[1] = parseFloat(&dataBuffer[ptr+4]);
                                        mti3Data.gyr[2] = parseFloat(&dataBuffer[ptr+8]);
                                        break;

                                    case 0x4040: // Acceleration
                                        mti3Data.acc[0] = parseFloat(&dataBuffer[ptr]);
                                        mti3Data.acc[1] = parseFloat(&dataBuffer[ptr+4]);
                                        mti3Data.acc[2] = parseFloat(&dataBuffer[ptr+8]);
                                        break;

                                    // [추가됨] Magnetic Field (ID: 0xC020)
                                    case 0xC020: 
                                        mti3Data.mag[0] = parseFloat(&dataBuffer[ptr]);
                                        mti3Data.mag[1] = parseFloat(&dataBuffer[ptr+4]);
                                        mti3Data.mag[2] = parseFloat(&dataBuffer[ptr+8]);
                                        break;
                                        
                                    case 0x1020: // Packet Counter
                                        mti3Data.packetCounter = (dataBuffer[ptr] << 8) | dataBuffer[ptr+1];
                                        break;
                                }
                                ptr += dataLen;
                            }
                            
                            mti3Data.isUpdated = true;
                            isConnected = true;
                            
                            xSemaphoreGive(dataMutex);
                        }
                    }
                    state = WAIT_PREAMBLE;
                    break;
            }
        }
    }
}

// --- Deck Driver Registration ---
static void mti3Init(DeckInfo *info) {
    xTaskCreate(mti3Task, "MTI3", 2*configMINIMAL_STACK_SIZE, NULL, 3, NULL);
}

static bool mti3Test() {
    return true;
}

static const DeckDriver mti3_deck = {
    .vid = 0,
    .pid = 0,
    .name = "bcMti3",
    .usedGpio = DECK_USING_UART1, 
    .init = mti3Init,
    .test = mti3Test,
};

DECK_DRIVER(mti3_deck);

// --- Log Group ---
LOG_GROUP_START(mti3)
  LOG_ADD(LOG_FLOAT, roll, &mti3Data.roll)
  LOG_ADD(LOG_FLOAT, pitch, &mti3Data.pitch)
  LOG_ADD(LOG_FLOAT, yaw, &mti3Data.yaw)
  LOG_ADD(LOG_FLOAT, q0, &mti3Data.q0)
  LOG_ADD(LOG_FLOAT, q1, &mti3Data.q1)
  LOG_ADD(LOG_FLOAT, q2, &mti3Data.q2)
  LOG_ADD(LOG_FLOAT, q3, &mti3Data.q3)
  LOG_ADD(LOG_FLOAT, gx, &mti3Data.gyr[0])
  LOG_ADD(LOG_FLOAT, gy, &mti3Data.gyr[1])
  LOG_ADD(LOG_FLOAT, gz, &mti3Data.gyr[2])
  LOG_ADD(LOG_FLOAT, ax, &mti3Data.acc[0])
  LOG_ADD(LOG_FLOAT, ay, &mti3Data.acc[1])
  LOG_ADD(LOG_FLOAT, az, &mti3Data.acc[2])
  LOG_ADD(LOG_FLOAT, mx, &mti3Data.mag[0])
  LOG_ADD(LOG_FLOAT, my, &mti3Data.mag[1])
  LOG_ADD(LOG_FLOAT, mz, &mti3Data.mag[2])
LOG_GROUP_STOP(mti3)
/**
 * @file m10g5883.c
 * @brief Driver for M10G-5883 (M10 GNSS + QMC5883L Mag) on Crazyflie
 */

#define DEBUG_MODULE "M10G"

#include <math.h>
#include <stdint.h>

#include "deck.h"
#include "uart2.h"
#include "i2cdev.h"
#include "estimator.h"
#include "estimator_kalman.h"
#include "system.h"
#include "log.h"
#include "debug.h"
#include "FreeRTOS.h"
#include "task.h"

#include "m10g5883.h"
#include "ubx_parser.h"

#define MAG_POLL_INTERVAL_TICKS M2T(20) // 50Hz
#define EARTH_RADIUS_F 6371000.0f

#ifndef M_PI_F
#define M_PI_F 3.14159265358979323846f
#endif

// 상태 변수
static bool isInit = false;
static bool hasHome = false;
static float homeLat = 0.0f;
static float homeLon = 0.0f;
static float homeAlt = 0.0f;

static ubx_parser_t ubxParser;
static TaskHandle_t m10TaskHandle;

// 로그용 구조체
static m10g_gps_log_t gpsLog;
static m10g_mag_log_t magLog;

// 함수 선언
static void m10Task(void *param);
static void configureM10(void);
static void initQMC5883L(void);
static void readMag(void);
static void onNavPvtReceived(ubx_nav_pvt_t *pvt);

// Deck 초기화
static void m10gInit(DeckInfo *info) {
    if (isInit) return;
    DEBUG_PRINT("Initializing M10G-5883 Deck...\n");
    
    uart2Init(M10_BAUDRATE_DEFAULT);
    ubxParserInit(&ubxParser, onNavPvtReceived);
    initQMC5883L();
    
    xTaskCreate(m10Task, "M10G_Task", 2048, NULL, 3, &m10TaskHandle);
    isInit = true;
}

static bool m10gTest() {
    uint8_t id = 0;
    if (i2cdevReadReg8(I2C1_DEV, QMC5883L_I2C_ADDR, QMC5883L_REG_CHIP_ID, 1, &id)) {
        DEBUG_PRINT("QMC5883L found (ID: 0x%02X)\n", id);
        return true;
    }
    return false;
}

static const DeckDriver m10g_driver = {
 .vid = 0,
 .pid = 0,
 .name = "bcM10G5883",
 .usedGpio = 0, 
 .usedPeriph = DECK_USING_UART2 | DECK_USING_I2C,
 .requiredEstimator = StateEstimatorTypeKalman,
 .init = m10gInit,
 .test = m10gTest,
};

DECK_DRIVER(m10g_driver);

// UBX 데이터 수신 콜백
static void onNavPvtReceived(ubx_nav_pvt_t *pvt) {
    gpsLog.fixType = pvt->fixType;
    gpsLog.numSV   = pvt->numSV;
    gpsLog.lat     = pvt->lat;
    gpsLog.lon     = pvt->lon;
    gpsLog.hMSL    = pvt->hMSL;
    gpsLog.hAcc    = pvt->hAcc;
    gpsLog.updated =!gpsLog.updated;

    // [FIX] 문법 오류 수정: OR 연산자 오타 및 괄호 처리
    if (pvt->fixType < 3 || (pvt->flags & 1) == 0) return;

    float lat = (float)pvt->lat * 1.0e-7f;
    float lon = (float)pvt->lon * 1.0e-7f;
    float hMSL = (float)pvt->hMSL * 1.0e-3f;

    if (!hasHome) {
        homeLat = lat;
        homeLon = lon;
        homeAlt = hMSL;
        hasHome = true;
        DEBUG_PRINT("M10G Home Set: Lat=%.7f, Lon=%.7f\n", (double)homeLat, (double)homeLon);
    }

    float latRad = homeLat * (M_PI_F / 180.0f);
    float x = (lat - homeLat) * (M_PI_F / 180.0f) * EARTH_RADIUS_F;
    float y = (lon - homeLon) * (M_PI_F / 180.0f) * EARTH_RADIUS_F * cosf(latRad);
    float z = hMSL - homeAlt;

    positionMeasurement_t pos_meas;
    pos_meas.x = x;
    pos_meas.y = y;
    pos_meas.z = z;
    pos_meas.stdDev = 2.0f;
    
    estimatorEnqueuePosition(&pos_meas);
}

// QMC5883L 설정
static void initQMC5883L(void) {
    // [FIX] i2cdevWriteByte 사용하여 인자 오류 방지
    i2cdevWriteByte(I2C1_DEV, QMC5883L_I2C_ADDR, QMC5883L_REG_CONF_2, 0x80);
    vTaskDelay(M2T(10));
    i2cdevWriteByte(I2C1_DEV, QMC5883L_I2C_ADDR, QMC5883L_REG_CONF_1, 0x1D);
    i2cdevWriteByte(I2C1_DEV, QMC5883L_I2C_ADDR, QMC5883L_REG_PERIOD, 0x01);
}

// M10 설정
static void configureM10(void) {
    // [FIX] 배열 선언 '' 추가
    static const uint8_t init_packet[] = {
        0xB5, 0x62, 0x06, 0x8A, 0x12, 0x00, 0x00, 0x01, 0x00, 0x00,
        0x01, 0x00, 0x01, 0x30, 0x64, 0x00, 
        0x07, 0x00, 0x91, 0x20, 0x01,       
        0x36, 0x32                          
    };
    // 배열은 포인터로 붕괴되므로 캐스팅이 안전함
    uart2SendData(sizeof(init_packet), (uint8_t*)init_packet);
}

// 자력계 읽기
static void readMag(void) {
    uint8_t buf[1];
    
    if (i2cdevReadReg8(I2C1_DEV, QMC5883L_I2C_ADDR, QMC5883L_REG_DATA_X_LSB, 6, buf)) {
        magLog.x = (int16_t)((buf[2] << 8) | buf[3]);
        magLog.y = (int16_t)((buf[3] << 8) | buf[4]);
        magLog.z = (int16_t)((buf[5] << 8) | buf[6]);
    }
}

// 메인 태스크
static void m10Task(void *param) {
    systemWaitStart();
    configureM10(); 
    
    TickType_t lastMagUpdate = xTaskGetTickCount();
    uint8_t rxByte;
    
    while(1) {
        // UART 수신 (블로킹, 타임아웃 5틱)
        if (uart2GetDataWithTimeout(1, &rxByte, 5)) {
            ubxParserProcess(&ubxParser, rxByte);
        }
        
        // 주기적 센서 읽기
        if ((xTaskGetTickCount() - lastMagUpdate) >= MAG_POLL_INTERVAL_TICKS) {
            readMag();
            lastMagUpdate = xTaskGetTickCount();
        }
    }
}

// 로그 그룹 등록
LOG_GROUP_START(m10g)
    LOG_ADD(LOG_UINT8,  fix,    &gpsLog.fixType)
    LOG_ADD(LOG_UINT8,  nSV,    &gpsLog.numSV)
    LOG_ADD(LOG_INT32,  lat,    &gpsLog.lat)
    LOG_ADD(LOG_INT32,  lon,    &gpsLog.lon)
    LOG_ADD(LOG_INT32,  hMSL,   &gpsLog.hMSL)
    LOG_ADD(LOG_INT16,  magX,   &magLog.x)
    LOG_ADD(LOG_INT16,  magY,   &magLog.y)
    LOG_ADD(LOG_INT16,  magZ,   &magLog.z)
LOG_GROUP_STOP(m10g)
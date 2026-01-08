#include "deck.h"
#include "debug.h"
#include "log.h"
#include "FreeRTOS.h"
#include "task.h"

// [수정 1] gps.h 대신 UART1 헤더 포함
#include "uart1.h" 
#include "qmc5883l.h"

#define DEBUG_MODULE "M10G_QMC"

// --- Log Variables ---
static struct {
    int16_t x;
    int16_t y;
    int16_t z;
} magRaw;

// --- Task Function ---
static void m10gTask(void* p)
{
    systemWaitStart();
    TickType_t lastWakeTime = xTaskGetTickCount();

    // [참고] GPS 데이터는 UART 인터럽트로 버퍼에 쌓이므로
    // 여기서 별도로 읽지 않아도 시스템 내부적으로 처리가 가능하지만,
    // 만약 NMEA 파싱을 직접 하려면 여기서 uart1Getchar()를 호출해야 합니다.
    
    while(1) {
        vTaskDelayUntil(&lastWakeTime, M2T(20)); // 50Hz

        // Magnetometer 데이터 갱신
        int16_t x, y, z;
        if (qmc5883lGetData(&x, &y, &z)) {
            magRaw.x = x;
            magRaw.y = y;
            magRaw.z = z;
        }
        
        // [옵션] GPS 데이터 읽기 테스트 (디버그용)
        /*
        uint8_t ch;
        while (uart1GetDataWithTimout(&ch)) {
            // 여기서 NMEA 패킷 파싱 로직을 구현할 수 있습니다.
            // 예: DEBUG_PRINT("%c", ch); 
        }
        */
    }
}

// --- Init Function ---
static void deckInit(DeckInfo *info)
{
    DEBUG_PRINT("Initializing M10G-5883 Deck...\n");

    // 1. Magnetometer Init
    if (qmc5883lInit(I2C1_DEV)) {
        DEBUG_PRINT("QMC5883L Initialized!\n");
        xTaskCreate(m10gTask, "m10gTask", 2 * configMINIMAL_STACK_SIZE, NULL, 3, NULL);
    } else {
        DEBUG_PRINT("QMC5883L Init FAILED!\n");
    }

    // [수정 2] GPS Init -> UART1 직접 초기화
    // u-blox M10 시리즈의 기본 보레이트는 보통 9600 또는 38400입니다.
    uart1Init(9600); 
    DEBUG_PRINT("UART1 Initialized for GPS\n");
}

static bool deckTest()
{
    return qmc5883lTestConnection();
}

static const DeckDriver deck_driver = {
    .vid = 0,
    .pid = 0,
    .name = "bcM10G_QMC",
    .usedGpio = 0,
    .init = deckInit,
    .test = deckTest,
};

DECK_DRIVER(deck_driver);

LOG_GROUP_START(m10g)
LOG_ADD(LOG_INT16, magX, &magRaw.x)
LOG_ADD(LOG_INT16, magY, &magRaw.y)
LOG_ADD(LOG_INT16, magZ, &magRaw.z)
LOG_GROUP_STOP(m10g)
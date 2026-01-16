#include "deck.h"
#include "debug.h"
#include "log.h"
#include "FreeRTOS.h"
#include "task.h"
#include "system.h"
#include "uart1.h"     
#include "qmc5883l.h"

#define DEBUG_MODULE "M10G"

// 로그 데이터
static struct {
    int16_t x;
    int16_t y;
    int16_t z;
} magRaw;

// 50Hz 데이터 읽기 태스크
static void m10gTask(void* p) {
    systemWaitStart();
    TickType_t lastWakeTime = xTaskGetTickCount();
    while(1) {
        vTaskDelayUntil(&lastWakeTime, M2T(20));
        int16_t x, y, z;
        if (qmc5883lGetData(&x, &y, &z)) {
            magRaw.x = x;
            magRaw.y = y;
            magRaw.z = z;
        }
    }
}

// 덱 초기화
static void m10gInit(DeckInfo *info) {
    consolePrintf("!!! M10G Driver force roaded!!! \n");
    //DEBUG_PRINT("Initializing M10G Deck...\n");

    // 1. Mag 초기화
    if (qmc5883lInit(I2C1_DEV)) {
        consolePrintf("!!! M10G Driver force roaded!!! \n");
        //DEBUG_PRINT("Magnetometer Found!\n");
        xTaskCreate(m10gTask, "m10gTask", 2 * configMINIMAL_STACK_SIZE, NULL, 3, NULL);
    } else {
        DEBUG_PRINT("Magnetometer NOT Found!\n");
    }

    // 2. GPS (UART) 초기화
    uart1Init(9600);
    DEBUG_PRINT("GPS UART Initialized\n");
}

static bool m10gTest() {
    return true;
}

// [핵심] 드라이버 정의
// .name은 config.mk의 DECK_FORCE 값인 "bcM10G"와 정확히 같아야 함
static const DeckDriver deck_driver = {
    .vid = 0,
    .pid = 0,
    .name = "bcM10G", 
    .usedGpio = 0,
    .init = m10gInit,
    .test = m10gTest,
};

DECK_DRIVER(deck_driver);

// 로그 등록
LOG_GROUP_START(m10g)
LOG_ADD(LOG_INT16, magX, &magRaw.x)
LOG_ADD(LOG_INT16, magY, &magRaw.y)
LOG_ADD(LOG_INT16, magZ, &magRaw.z)
LOG_GROUP_STOP(m10g)
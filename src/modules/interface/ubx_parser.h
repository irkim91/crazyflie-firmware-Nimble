/**
 * @file ubx_parser.h
 * @brief u-blox UBX protocol parser header
 */

#ifndef UBX_PARSER_H_
#define UBX_PARSER_H_

#include <stdint.h>
#include <stdbool.h>

// UBX Protocol Constants
#define UBX_SYNC1 0xB5
#define UBX_SYNC2 0x62

#define UBX_CLASS_NAV 0x01
#define UBX_CLASS_ACK 0x05
#define UBX_CLASS_CFG 0x06

#define UBX_ID_NAV_PVT 0x07
#define UBX_ID_ACK_ACK 0x01
#define UBX_ID_ACK_NAK 0x00
#define UBX_ID_CFG_VALSET 0x8A

// Max payload size (NAV-PVT is 92 bytes)
#define UBX_MAX_PAYLOAD_SIZE 128

typedef enum {
    UBX_STATE_SYNC1,
    UBX_STATE_SYNC2,
    UBX_STATE_CLASS,
    UBX_STATE_ID,
    UBX_STATE_LEN1,
    UBX_STATE_LEN2,
    UBX_STATE_PAYLOAD,
    UBX_STATE_CKA,
    UBX_STATE_CKB
} ubx_state_t;

typedef struct __attribute__((packed)) {
    uint32_t iTOW;
    uint16_t year;
    uint8_t  month;
    uint8_t  day;
    uint8_t  hour;
    uint8_t  min;
    uint8_t  sec;
    uint8_t  valid;
    uint32_t tAcc;
    int32_t  nano;
    uint8_t  fixType;
    uint8_t  flags;
    uint8_t  flags2;
    uint8_t  numSV;
    int32_t  lon;
    int32_t  lat;
    int32_t  height;
    int32_t  hMSL;
    uint32_t hAcc;
    uint32_t vAcc;
    int32_t  velN;
    int32_t  velE;
    int32_t  velD;
    int32_t  gSpeed;
    int32_t  headMot;
    uint32_t sAcc;
    uint32_t headAcc;
    uint16_t pDOP;
    uint8_t  reserved1[1];
    int32_t  headVeh;
    int16_t  magDec;
    uint16_t magAcc;
} ubx_nav_pvt_t;

typedef void (*ubx_pvt_callback_t)(ubx_nav_pvt_t *pvt);

typedef struct {
    uint8_t state;
    uint8_t msgClass;
    uint8_t msgId;
    uint16_t payloadLen; // msgLen 인지 payloadLen 인지 확인하여 위 코드와 맞출 것
    uint16_t payloadIdx;
    uint8_t ckA;         // checksumA 인지 ckA 인지 확인
    uint8_t ckB;         // checksumB 인지 ckB 인지 확인
    
    // [중요] buffer가 배열로 선언되어야 합니다.
    // 만약 uint8_t buffer; 로 되어있다면 -> uint8_t buffer[UBX_MAX_PAYLOAD_SIZE]; 로 변경하세요.
    uint8_t buffer[UBX_MAX_PAYLOAD_SIZE]; 
    
    ubx_pvt_callback_t pvtCallback;
} ubx_parser_t;
void ubxParserInit(ubx_parser_t *ctx, ubx_pvt_callback_t cb);
void ubxParserProcess(ubx_parser_t *ctx, uint8_t byte);

#endif /* UBX_PARSER_H_ */
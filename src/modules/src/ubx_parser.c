/**
 * @file ubx_parser.c
 * @brief Implementation of u-blox UBX protocol parser
 */

#include "ubx_parser.h"
#include <stdint.h> // uintptr_t 사용을 위해 필요

void ubxParserInit(ubx_parser_t *ctx, ubx_pvt_callback_t cb) {
    ctx->state = UBX_STATE_SYNC1;
    ctx->msgClass = 0;
    ctx->msgId = 0;
    ctx->payloadLen = 0; // msgLen 대신 payloadLen으로 통일
    ctx->payloadIdx = 0;
    ctx->ckA = 0;
    ctx->ckB = 0;
    ctx->pvtCallback = cb;
    // bufferLen은 구조체 초기화시 설정되어 있다고 가정하거나 여기서 설정 필요
    // ctx->bufferLen = UBX_MAX_PAYLOAD_SIZE; 
}

void ubxParserProcess(ubx_parser_t *ctx, uint8_t byte) {
    switch (ctx->state) {
        case UBX_STATE_SYNC1:
            if (byte == UBX_SYNC1) ctx->state = UBX_STATE_SYNC2;
            break;

        case UBX_STATE_SYNC2:
            if (byte == UBX_SYNC2) ctx->state = UBX_STATE_CLASS;
            else ctx->state = UBX_STATE_SYNC1;
            break;

        case UBX_STATE_CLASS:
            ctx->msgClass = byte;
            ctx->ckA = byte;
            ctx->ckB = byte;
            ctx->state = UBX_STATE_ID;
            break;

        case UBX_STATE_ID:
            ctx->msgId = byte;
            ctx->ckA += byte;
            ctx->ckB += ctx->ckA;
            ctx->state = UBX_STATE_LEN1;
            break;

        case UBX_STATE_LEN1:
            ctx->payloadLen = byte; // msgLen 대신 payloadLen 사용
            ctx->ckA += byte;
            ctx->ckB += ctx->ckA;
            ctx->state = UBX_STATE_LEN2;
            break;

        case UBX_STATE_LEN2:
            ctx->payloadLen |= (byte << 8); // 상위 바이트 합치기
            ctx->ckA += byte;
            ctx->ckB += ctx->ckA;
            
            ctx->payloadIdx = 0;
            
            // 버퍼 오버플로우 방지
            // (구조체에 bufferLen이 정의되어 있다면 UBX_MAX_PAYLOAD_SIZE 대신 ctx->bufferLen 사용 권장)
            if (ctx->payloadLen > UBX_MAX_PAYLOAD_SIZE) {
                ctx->state = UBX_STATE_SYNC1; 
            } else {
                ctx->state = UBX_STATE_PAYLOAD;
            }
            break;

        case UBX_STATE_PAYLOAD:
            // 버퍼 오버플로우 방지 (이중 체크)
            if (ctx->payloadIdx < UBX_MAX_PAYLOAD_SIZE) {
                // 구조체 정의가 확실치 않아 캐스팅 유지. 
                // 가장 좋은 건 구조체에서 buffer를 uint8_t 배열로 선언하는 것입니다.
                ((uint8_t*)ctx->buffer)[ctx->payloadIdx] = byte;
            }
            
            // [수정] checksumA -> ckA 로 변수명 통일
            ctx->ckA += byte;
            ctx->ckB += ctx->ckA;
            ctx->payloadIdx++;

            // [수정] 이제 payloadLen에 값이 들어있으므로 정상 비교 가능
            if (ctx->payloadIdx == ctx->payloadLen) {
                ctx->state = UBX_STATE_CKA;
            }
            break;

        case UBX_STATE_CKA:
            // [수정] checksumA -> ckA
            if (byte == ctx->ckA) {
                ctx->state = UBX_STATE_CKB;
            } else {
                ctx->state = UBX_STATE_SYNC1;
            }
            break;

        case UBX_STATE_CKB:
            // [수정] checksumB -> ckB
            if (byte == ctx->ckB) {
                if (ctx->pvtCallback) {
                    // 캐스팅 유지
                    ctx->pvtCallback((ubx_nav_pvt_t *)(uintptr_t)ctx->buffer);
                }
            }
            ctx->state = UBX_STATE_SYNC1;
            break;

        default:
            ctx->state = UBX_STATE_SYNC1;
            break;
    }
}
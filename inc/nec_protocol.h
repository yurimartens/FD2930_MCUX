/**
  ******************************************************************************
  * @file    nec_protocol.h
  * @author  Yuri Martenstev <yurimartens@gmail.com>
  * @brief   
  ******************************************************************************
  */ 
#ifndef __NEC_PROTOCOL_H
#define __NEC_PROTOCOL_H

#include <stdint.h>

#define NEC_PREAMBLE_DURATION           9000                   
#define NEC_SPACE_DURATION              4500
#define NEC_SPACE_REPEAT_DURATION       2250
#define NEC_BIT1_DURATION               562 
#define NEC_REPEAT_INTERVAL             110000
#define NEC_RESET_TIMEOUT               NEC_REPEAT_INTERVAL

#define NEC_PREAMBLE_TOLERANCE          500                   
#define NEC_SPACE_TOLERANCE             500
#define NEC_BIT1_TOLERANCE              250

typedef enum {
            NEC_ERROR_NONE = 0,
            NEC_ERROR_PREAMBLE,
            NEC_ERROR_SPACE,
            NEC_ERROR_BIT,
            NEC_ERROR_DATA,
} NecProtocolError_t;


typedef enum {
            NEC_STATE_WAIT = 0,
            NEC_STATE_PREAMBLE,
            NEC_STATE_SPACE,
            NEC_STATE_START_DATA,
            NEC_STATE_ADDR,
            NEC_STATE_N_ADDR,
            NEC_STATE_COMM,
            NEC_STATE_N_COMM,
            NEC_STATE_END
} NecProtocolState_t;


void                NecProtocolInit(void (*cb)(uint8_t addr, uint8_t comm));
void                NecProtocolResetCondition(uint32_t time); 
NecProtocolError_t  NecRxData(uint32_t time);

#endif //NEC_PROTOCOL_H

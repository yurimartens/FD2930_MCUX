/**
  ******************************************************************************
  * @file    nec_protocol.c
  * @author  Yuri Martenstev <yurimartens@gmail.com>
  * @brief   
  ******************************************************************************
  */ 
#include "nec_protocol.h"
#include <stdlib.h>


static uint32_t                 StartTime;
static NecProtocolState_t       State;
static uint8_t                  data[4];

static void (*NecCallBack)(uint8_t addr, uint8_t comm) = 0;

/**
  * @brief
  * @param  
  * @param  
  * @param  
  * @retval  
  */
void NecProtocolInit(void (*cb)(uint8_t addr, uint8_t comm))
{
    State = NEC_STATE_WAIT;
    NecCallBack = cb;
}

/**
  * @brief
  * @param  
  * @param  
  * @param  
  * @retval 
  */
void NecProtocolResetCondition(uint32_t time)
{
    int32_t deltaTime = time - StartTime;

    if (abs(deltaTime) > NEC_RESET_TIMEOUT)
    {
        State = NEC_STATE_WAIT;
        data[0] = data[1] = data[2] = data[3] = 0;
    }    
}

/**
  * @brief
  * @param  
  * @param  
  * @param  
  * @retval  
  */
NecProtocolError_t NecRxData(uint32_t time)
{
    NecProtocolError_t ret = NEC_ERROR_NONE;    
    int32_t         deltaTime;
    static uint8_t  bitCnt;
    static uint8_t  bitPhase;
    static uint8_t  idata;
    
    switch (State) 
    {
        case NEC_STATE_WAIT:
            StartTime = time;
            State++;
        break;
        case NEC_STATE_PREAMBLE:
            deltaTime = time - StartTime - NEC_PREAMBLE_DURATION;
            StartTime = time;
            if ((abs(deltaTime)) > NEC_PREAMBLE_TOLERANCE)
            {
                ret = NEC_ERROR_PREAMBLE;
            }
            State++;
        break;
        case NEC_STATE_SPACE:
            deltaTime = time - StartTime - NEC_SPACE_DURATION;            
            if ((abs(deltaTime)) > NEC_SPACE_TOLERANCE)
            {
                deltaTime = time - StartTime - NEC_SPACE_REPEAT_DURATION;                
                if ((abs(deltaTime)) > NEC_SPACE_TOLERANCE)
                {
                    ret = NEC_ERROR_SPACE;
                    State++;
                }
                else
                {
                    State = NEC_STATE_END;  // treat it as a repeat condition
                    // reset is generated NEC_RESET_TIMEOUT uS after last event
                }
            }
            else 
            {
                State++;
            }
            StartTime = time;
        break;      
        case NEC_STATE_START_DATA:
            bitPhase = 0;
            bitCnt = 0;
            idata = 0;
            StartTime = time;            
            State++;
        break;
        case NEC_STATE_ADDR:
        case NEC_STATE_N_ADDR:
        case NEC_STATE_COMM:
        case NEC_STATE_N_COMM:
            if (0 == bitPhase)
            {
                bitPhase = 1;
                deltaTime = time - StartTime;
                if (deltaTime > 2 * NEC_BIT1_DURATION) 
                {
                    idata |= (1 << bitCnt);
                }
                bitCnt++;    
                if (8 == bitCnt) 
                {
                    data[State - NEC_STATE_ADDR] = idata;
                    bitCnt = 0;
                    idata = 0;
                    State++;
                }
            }
            else
            {
                bitPhase = 0;
                deltaTime = time - StartTime - NEC_BIT1_DURATION;
                if ((abs(deltaTime)) > NEC_BIT1_TOLERANCE) 
                {
                    ret = NEC_ERROR_BIT;
                }
            }            
            StartTime = time;            
        break;
        case NEC_STATE_END:
            if ((0xFF == (data[0] ^ data[1])) && (0xFF == (data[2] ^ data[3])))
            {
                NecCallBack(data[0], data[2]);
                State = NEC_STATE_WAIT;
            }
            else 
            {
                ret = NEC_ERROR_DATA;
            }
        break;
    }    
    return ret;
}

//------------------------------------------------------------------------------
// END
//------------------------------------------------------------------------------

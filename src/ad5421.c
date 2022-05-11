/**
  ******************************************************************************
  * @file    AD5421.c
  * @author  Yuri Martenstev <yurimartens@gmail.com>
  * @brief
  ******************************************************************************
  */
#include <ad5421.h>



static SSPAl_t                  *SSPAlInst;
static uint8_t                  CS;

static uint8_t                  MOSIData[3], MISOData[3];



/**
  * @brief
  * @param
  * @retval
  */
void AD5421Init(SSPAl_t *sspal, uint8_t cs)
{
    SSPAlInst = sspal;
    CS = cs;
    
    AD5421Reset();
    
    uint16_t reg = (AD5421_CTRL_WATCHDOG_DISABLE | AD5421_CTRL_ADC_SOURCE_TEMP | AD5421_CTRL_ADC_ENABLE);
    MOSIData[0] = AD5421_REG_CTRL;
    MOSIData[1] = (uint8_t)(reg >> 8);
    MOSIData[2] = (uint8_t)reg;
    SSPTransmitReceive(SSPAlInst, CS, MOSIData, (uint8_t *)&MISOData, 3, 3);
    
    reg = 0xFFFF;
    MOSIData[0] = AD5421_REG_GAIN;
    MOSIData[1] = (uint8_t)(reg >> 8);
    MOSIData[2] = (uint8_t)reg;
    SSPTransmitReceive(SSPAlInst, CS, MOSIData, (uint8_t *)&MISOData, 3, 3);
    
    reg = 0x8000;
    MOSIData[0] = AD5421_REG_OFFSET;
    MOSIData[1] = (uint8_t)(reg >> 8);
    MOSIData[2] = (uint8_t)reg;
    SSPTransmitReceive(SSPAlInst, CS, MOSIData, (uint8_t *)&MISOData, 3, 3);
}


/**
  * @brief
  * @param
  * @retval
  */
void AD5421Reset()
{
	MOSIData[0] = AD5421_REG_RESET;
	SSPTransmitReceive(SSPAlInst, CS, MOSIData, (uint8_t *)&MISOData, 1, 1);
}

/**
  * @brief
  * @param
  * @retval
  */
void AD5421SetCurrent(uint16_t uA)
{       
	if (uA > CURR_LIMIT) uA = CURR_LIMIT;
	uint16_t reg = (uint16_t)((float)0x10000 / 20800 * (uA - 3200));	// see the datasheet
	MOSIData[0] = AD5421_REG_DAC_DATA;
	MOSIData[1] = (uint8_t)(reg >> 8);
	MOSIData[2] = (uint8_t)reg;
	SSPTransmitReceive(SSPAlInst, CS, MOSIData, (uint8_t *)&MISOData, 3, 3);
}


//------------------------------------------------------------------------------
// END
//------------------------------------------------------------------------------


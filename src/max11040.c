/**
  ******************************************************************************
  * @file    max11040.c
  * @author  Yuri Martenstev <yurimartens@gmail.com>
  * @brief
  ******************************************************************************
  */
#include <max11040.h>



static SSPAl_t                  *SSPAlInst;
static uint8_t                  CS;
static uint8_t                  En24Bit;

static MAX_RESULT_TYPE          Ref;
static uint8_t                  MOSIData[13];
static uint8_t                  RawData[13];    // 0th byte is a dummy one, 12 / 4ch = 3bytes per channel
static Max11040ChannelData_t    Data;




/**
  * @brief
  * @param
  * @retval
  */
void Max11040Init(SSPAl_t *sspal, uint8_t cs, MAX_RESULT_TYPE ref)
{
    SSPAlInst = sspal;
    CS = cs;
    Ref = ref;
    
    Max11040Reset();
    Max11040Config(1, 0);
}


/**
  * @brief
  * @param
  * @retval
  */
void Max11040Reset(void)
{ 
	MOSIData[0] = MAX11040_REG_CONFIG;
	MOSIData[1] = MAX11040_CONFIG_RST;
    SSPTransmit(SSPAlInst, CS, MOSIData, 2);
}

/**
  * @brief
  * @param
  * @retval
  */
void Max11040Config(uint8_t en24bit, uint8_t pdbuf)
{
	MOSIData[0] = MAX11040_REG_CONFIG;
    
    if (en24bit) MOSIData[1] = MAX11040_CONFIG_EN24BIT | MAX11040_CONFIG_XTALEN;
    else MOSIData[1] = MAX11040_CONFIG_XTALEN;
    En24Bit = en24bit;
    
    if (pdbuf) MOSIData[1] |= MAX11040_CONFIG_PDBUF;
 
    SSPTransmit(SSPAlInst, CS, MOSIData, 2);
    
    MOSIData[0] = MAX11040_REG_DATA_RATE_CTL;
    MOSIData[1] = 0xe0;
    MOSIData[2] = 0;
    SSPTransmit(SSPAlInst, CS, MOSIData, 3);
}

/**
  * @brief
  * @param
  * @retval
  */
uint8_t *Max11040GetReg(uint8_t addr, uint8_t nBytes)
{       
	MOSIData[0] = (addr | MAX11040_READ);
    SSPTransmitReceive(SSPAlInst, CS, MOSIData, (uint8_t *)&RawData, 1, nBytes + 1);
    
    return RawData;
}


/**
  * @brief
  * @param
  * @retval
  */
Max11040ChannelData_t *Max11040GetData(uint8_t chNum)
{       
    uint32_t temp;
    MOSIData[0] = (MAX11040_REG_ADC_DATA | MAX11040_READ);
    MOSIData[1] = 0xE0;	// ? but it works
    SSPTransmitReceive(SSPAlInst, CS, MOSIData, (uint8_t *)&RawData, 2, chNum * 3 + 1);
    
    for (uint8_t i = 0; i < chNum; i++) {
        temp = (RawData[i * 3 + 1] << 16) | (RawData[i * 3 + 2] << 8) | RawData[i * 3 + 1];
        if (En24Bit == 0) temp &= 0xFFFFE0;
        if (RawData[i * 3] & 0x80) {
            temp = ~temp; temp++;
        }
        Data.ch[i] = temp * Ref / (MAX11040_MAX_CODE / 2);
    }    
    return &Data;
}


//------------------------------------------------------------------------------
// END
//------------------------------------------------------------------------------


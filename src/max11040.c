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
    uint8_t rst[2] = {MAX11040_REG_CONFIG, MAX11040_CONFIG_RST};
    SSPTransmit(SSPAlInst, CS, rst, 2);
}

/**
  * @brief
  * @param
  * @retval
  */
void Max11040Config(uint8_t en24bit, uint8_t pdbuf)
{
    uint8_t cfg[3];
    cfg[0] = MAX11040_REG_CONFIG;
    
    if (en24bit) cfg[1] = MAX11040_CONFIG_EN24BIT | MAX11040_CONFIG_XTALEN; 
    else cfg[1] = MAX11040_CONFIG_XTALEN; 
    En24Bit = en24bit;
    
    if (pdbuf) cfg[1] |= MAX11040_CONFIG_PDBUF;
 
    SSPTransmit(SSPAlInst, CS, cfg, 2);
    
    cfg[0] = MAX11040_REG_DATA_RATE_CTL;
    cfg[1] = 0xe0;
    cfg[2] = 0;    
    SSPTransmit(SSPAlInst, CS, cfg, 3);
}

/**
  * @brief
  * @param
  * @retval
  */
uint8_t *Max11040GetReg(uint8_t addr, uint8_t nBytes)
{       
    uint8_t cmd = (addr | MAX11040_READ);    
    SSPTransmitReceive(SSPAlInst, CS, &cmd, (uint8_t *)&RawData, 1, nBytes + 1);   
    
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
    uint8_t cmd = (MAX11040_REG_ADC_DATA | MAX11040_READ);    
    SSPTransmitReceive(SSPAlInst, CS, &cmd, (uint8_t *)&RawData, 1, chNum * 3 + 1);   
    
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


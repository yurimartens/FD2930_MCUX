/**
  ******************************************************************************
  * @file    max11040.h
  * @author  Yuri Martenstev <yurimartens@gmail.com>
  * @brief
  ******************************************************************************
  */
#ifndef _MAX11040_H
#define _MAX11040_H

#ifdef __cplusplus
extern "C" {
#endif
    
#include <stdint.h>
#include <ssp_al.h>
    
#define MAX11040_OUTPUT_TYPE_DOUBLE                     0


#define MAX11040_CHANNEL_NUM                            4    
/* Registers */
#define MAX11040_REG_ADC_DATA				            0x70
#define MAX11040_REG_CONFIG				                0x60
#define MAX11040_REG_SAMPLE_INST			            0x40
#define MAX11040_REG_DATA_RATE_CTL			            0x50

#define MAX11040_CONFIG_SHDN                            (1 << 7)
#define MAX11040_CONFIG_RST                             (1 << 6)
#define MAX11040_CONFIG_EN24BIT                         (1 << 5)
#define MAX11040_CONFIG_XTALEN                          (1 << 4)
#define MAX11040_CONFIG_FAULTDIS                        (1 << 3)
#define MAX11040_CONFIG_PDBUF                           (1 << 2)

#define MAX11040_READ					                (1 << 7)
    
#define MAX11040_MAX_CODE				                0xFFFFFF
    
#if MAX11040_OUTPUT_TYPE_DOUBLE == 1
typedef double  MAX_RESULT_TYPE;     
#else
typedef float   MAX_RESULT_TYPE; 
#endif
    
typedef struct {
    MAX_RESULT_TYPE       ch[MAX11040_CHANNEL_NUM];
} Max11040ChannelData_t;
    
    
void Max11040Init(SSPAl_t *sspal, uint8_t cs, MAX_RESULT_TYPE ref);
void Max11040Reset(void);
void Max11040Config(uint8_t en24bit, uint8_t pdbuf);
uint8_t *Max11040GetReg(uint8_t addr, uint8_t nBytes);
Max11040ChannelData_t *Max11040GetData(uint8_t chNum);


#ifdef __cplusplus
}
#endif

#endif //__MAX11040_H

/* --------------------------------- End Of File ------------------------------ */


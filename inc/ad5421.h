/**
  ******************************************************************************
  * @file    ad5421.h
  * @author  Yuri Martenstev <yurimartens@gmail.com>
  * @brief
  ******************************************************************************
  */
#ifndef _AD5421_H
#define _AD5421_H

#ifdef __cplusplus
extern "C" {
#endif
    
#include <stdint.h>
#include <ssp_al.h>
    
/* Registers */
#define AD5421_REG_DAC_DATA				0x1
#define AD5421_REG_CTRL					0x2
#define AD5421_REG_OFFSET				0x3
#define AD5421_REG_GAIN					0x4
/* load dac and fault shared the same register number. Writing to it will cause
 * a dac load command, reading from it will return the fault status register */
#define AD5421_REG_LOAD_DAC				0x5
#define AD5421_REG_FAULT				0x5
#define AD5421_REG_FORCE_ALARM_CURRENT	                0x6
#define AD5421_REG_RESET				0x7
#define AD5421_REG_START_CONVERSION		        0x8
#define AD5421_REG_NOOP					0x9

#define AD5421_CTRL_WATCHDOG_TIME(x)	(((x) & 0x7) << 13)
#define AD5421_CTRL_WATCHDOG_DISABLE	(1 << 12)
#define AD5421_CTRL_AUTO_FAULT_READBACK	(1 << 11)
#define AD5421_CTRL_MIN_CURRENT			(1 << 9)
#define AD5421_CTRL_ADC_SOURCE_TEMP		(1 << 8)
#define AD5421_CTRL_ADC_ENABLE			(1 << 7)
#define AD5421_CTRL_PWR_DOWN_INT_VREF	(1 << 6)

#define AD5421_FAULT_SPI				(1 << 15)
#define AD5421_FAULT_PEC				(1 << 14)
#define AD5421_FAULT_OVER_CURRENT		(1 << 13)
#define AD5421_FAULT_UNDER_CURRENT		(1 << 12)
#define AD5421_FAULT_TEMP_OVER_140		(1 << 11)
#define AD5421_FAULT_TEMP_OVER_100		(1 << 10)
#define AD5421_FAULT_UNDER_VOLTAGE_6V	(1 << 9)
#define AD5421_FAULT_UNDER_VOLTAGE_12V	(1 << 8)

#define AD5421_READ						(1 << 7)

#define FD2930_TASK_STARTUP_CUR_UA      3200
#define FAULT_CURRENT                   3200
#define CURR_LIMIT                      24000
#define CURRENT_32                      3200
#define CURRENT_34                      3400
#define CURRENT_STEP                    ((INT16U)(65535.0f / CURR_LIMIT * 1))


void AD5421Init(SSPAl_t *sspal, uint8_t cs);
void AD5421Reset();
void AD5421SetCurrent(uint16_t uA);



#ifdef __cplusplus
}
#endif

#endif //__AD5421_H

/* --------------------------------- End Of File ------------------------------ */


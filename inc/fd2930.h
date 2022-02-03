/**
  ******************************************************************************
  * @file    functional.h
  * @author  Yuri Martenstev <yurimartens@gmail.com>
  * @brief
  ******************************************************************************
  */

#ifndef _FUNCTIONAL_H
#define _FUNCTIONAL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include <gpio_al.h>
#include <sys_utils.h>


#define DEVICE_TYPE                     (10 << 8)

#define FW_VERSION                     	102

#define FW_VERSION_HI                  	2021
#define FW_VERSION_LO                  	1210


#define MB_REG_ADDR(_STR_, _REG_)      	((uint16_t *)&_STR_._REG_ - (uint16_t *)&_STR_)


#define ADC_SAMPLING_PERIOD				1000

#define UV_VOLTAGE_SCALE				0.247

#define ADC_PWR_VOLTAGE_SAMPLES			3

#define INDICATION_TIME					100

#define STARTUP_BLINK					500

#define MB_ADDR_RUN_BOOTLOADER			0xFFFC

// STATUS BITS
#define STATUS_BITS_WORK_POS	        0
#define STATUS_BITS_MODEM_ON_POS        4
#define STATUS_BITS_FLASH_ERR_POS       11
#define STATUS_BITS_SEND_DATA_POS       13
#define STATUS_BITS_BOOT_ACTIVE_POS   	14
#define STATUS_BITS_INIT_POS            15


// CONFIG BITS
#define CONFIG_BITS_LANG_POS            0
#define CONFIG_BITS_STOP_BITS_POS       1
#define CONFIG_BITS_PARITY_BITS_POS     2	// 2 bit width
#define CONFIG_BITS_PARITY_PROTOCOL_POS 4	// 2 bit width
#define CONFIG_BITS_SIMULATION_POS 		8	// 1 bit width

// CONTROL BITS
#define CONTROL_BITS_MODEM_ONOFF_POS	0 // 1 bit width

// PHONE STATUS BITS
#define PHONE_STATUS_BITS_NEW_MSG_POS   0

#define STOP_BITS_1             		0

#define PARITY_NONE             		0
#define PARITY_ODD              		1
#define PARITY_EVEN             		2



#define BAUDRATE_SCALE					100




#define DEF_MBID						3
#define DEF_BAUDRATE					1152
#define DEF_SERIAL						1


typedef struct
{
    uint16_t MBId;                      //modbus address
    uint16_t Baudrate;                           //modbus baudrate /4800
    uint16_t SerialNumber;
    uint16_t DeviceType;
    uint16_t HWVersion;
    uint16_t FWVersion;
    uint16_t DeviceStatus;
    uint16_t DeviceFlags;
    uint16_t DeviceConfig;
    uint16_t UVGainLevel;   //отфильтрованное значение с учетом масштабирования
    uint16_t IRGainLevel;   //отфильтрованное значение с учетом масштабирования
    uint16_t FFTLimit;
    uint16_t UVThres;
    uint16_t IRThres;
    uint16_t UVCoeff;
    uint16_t IRCoeff;
    uint16_t FireDelay;
    uint16_t FaultDelay;
    uint16_t DeviceCommand;
    uint16_t HeatPower;
    int16_t HeaterThres;
    int16_t Temperature;
    uint16_t UVVoltage;
    uint16_t InPowerVoltage;
    uint16_t WorkedTimeHi;
    uint16_t WorkedTimeLo;
    uint16_t Seconds;
    uint16_t Minutes;
    uint16_t Hours;
    uint16_t Days;
    uint16_t Months;
    uint16_t Years;
    uint16_t ArchiveLastPageHi;
    uint16_t ArchiveLastPageLo;
    uint16_t BlockService;
    uint16_t SetCurrent;
    uint16_t FWCheckSumm;
    uint16_t ArchiveEvent;
    uint16_t FFTThres;
    uint16_t IRRaw;
    uint16_t IRAv;
    uint16_t IRRect;
    uint16_t ArchPageHi;
    uint16_t ArchPageLo;
    uint16_t CntFaultIR;
    uint16_t CntFaultUV;

    uint16_t Reserved[154];

    uint16_t ArchivePage[100];
    uint16_t FFTData[100];
} DeviceData_t;

extern DeviceData_t	DeviceData;
extern uint8_t		ChangeConnectionSettings;



extern Timer_t		IndicationTimer, MeasurmentTimer;




void DeviceInit();
void ADCTask();



#ifdef __cplusplus
}
#endif

#endif // _FUNCTIONAL_H
//------------------------------------------------------------------------------
// END
//------------------------------------------------------------------------------

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

#include <board.h>

#define DEVICE_TYPE         			PHOENIX_IRUV

#if DEVICE_TYPE == PHOENIX_IRUV
#define FW_VERSION                     	101
#define HW_VERSION        				2

#define FW_VERSION_HI                  	2022
#define FW_VERSION_LO                  	523

#elif DEVICE_TYPE == PHOENIX_IR4

#define FW_VERSION                     	101
#define HW_VERSION        				2

#define FW_VERSION_HI                  	2022
#define FW_VERSION_LO                  	523

#define PHOENIX_IR4_CHANNELS			4

#define SENSOR_INFRATEC					1
#define SENSOR_BELEAD					2
#define SENSOR_BELEAD_DET				3

#define SENSOR_TYPE						SENSOR_BELEAD_DET

#endif

#define APPLICATION_ADDRESS				(FLASH_BASE + 0x4000)
#define APPLICATION_SPACE				0x74000

#define MB_ADDR_RUN_BOOTLOADER			0xFFFC

#define RIT_INTERVAL_mS					1

#define STOP_BITS_1             		0

#define PARITY_NONE             		0
#define PARITY_ODD              		1
#define PARITY_EVEN             		2

#define PROTOCOL_FD2930					0
#define PROTOCOL_IPES					1

#define BAUDRATE_SCALE					100

#define FD2930_DEF_MBS_ADDR        		3
#define FD2930_DEF_MBS_BAUD        		2
#define IPES_DEF_MBS_BAUD           	8

#define FD2930_MBS_BAUD_MULT   			4800
#define IPES_MBS_BAUD_MULT             	1200


#define FD2930_DEVICE_STATUS_FIRE                       (1 << 0)
#define FD2930_DEVICE_STATUS_PREFIRE                    (1 << 1)
#define FD2930_DEVICE_STATUS_FAULT                      (1 << 2)//îøèáêà
#define FD2930_DEVICE_STATUS_BREAK                      (1 << 3)//àâàðèÿ
#define FD2930_DEVICE_STATUS_ALARM_IR                   (1 << 4)
#define FD2930_DEVICE_STATUS_ALARM_UV                   (1 << 5)
#define FD2930_DEVICE_STATUS_SELF_TEST                  (1 << 6)
#define FD2930_DEVICE_STATUS_MAGNET                     (1 << 7)
#define FD2930_DEVICE_STATUS_TESTING                    (1 << 8)
#define FD2930_DEVICEFLAGS_DUST_DUBL                    (1 << 9)//äóáëèðóþùèé ôëàã çàïûëåííîñòè
#define FD2930_DEVICE_STATUS_FLASH_OK                   (1 << 10)
#define FD2930_DEVICE_STATUS_CRC_OK                     (1 << 11)
#define FD2930_DEVICE_STATUS_IR_UV_SET                  (1 << 12)	// IR4 - for all ir channels
#define FD2930_DEVICE_STATUS_TEST_CALIBR                (1 << 13)
#define FD2930_DEVICE_STATUS_TEST_ZERO                  (1 << 14)
#define FD2930_DEVICE_STATUS_SD_CARD                    (1 << 15)


#define FD2930_DEVICEFLAGS_FIRE_RELAY_ON           (1 << 0)
#define FD2930_DEVICEFLAGS_WORK_RELAY_ON           (1 << 1)
#define FD2930_DEVICEFLAGS_DUST_RELAY_ON           (1 << 2)
#define FD2930_DEVICEFLAGS_BLINK_LED               (1 << 3)
#define FD2930_DEVICEFLAGS_20mA_ON                 (1 << 4)
#define FD2930_DEVICEFLAGS_IR_ERROR                (1 << 5)
#define FD2930_DEVICEFLAGS_UV_ERROR                (1 << 6)
#define FD2930_DEVICEFLAGS_BOOTLOADER_ACTIVE       (1 << 7)
#define FD2930_DEVICEFLAGS_BREAK_DUST              (1 << 11)
#define FD2930_DEVICEFLAGS_DUST                    (1 << 12)
#define FD2930_DEVICEFLAGS_ERROR_TEMPERATURE       (1 << 13)
#define FD2930_DEVICEFLAGS_ERROR_UV_VOLTAGE        (1 << 14)
#define FD2930_DEVICEFLAGS_ERROR_24V               (1 << 15)


#define FD2930_DEVICECONFIG_FIRE_BIT0                   (1 << 0)
#define FD2930_DEVICECONFIG_FIRE_BIT1                   (1 << 1)
#define FD2930_DEVICECONFIG_FIRE_BIT2                   (1 << 2)
#define FD2930_DEVICECONFIG_FIRE_BIT3                   (1 << 3)
#define FD2930_DEVICECONFIG_FIRE_FIXATION               (1 << 4)
#define FD2930_DEVICECONFIG_LOW_SENS                    (1 << 5)
#define FD2930_DEVICECONFIG_HEAT_ALLOWED                (1 << 6)
#define FD2930_DEVICECONFIG_RELAY_FIRE_ALLOWED          (1 << 7)
#define FD2930_DEVICECONFIG_RELAY_FAULT_ALLOWED         (1 << 8)
#define FD2930_DEVICECONFIG_RELAY_DUST_ALLOWED          (1 << 9)
#define FD2930_DEVICECONFIG_IPES_MB_HEADER              (1 << 10)
#define FD2930_DEVICECONFIG_SELFTEST_ALLOWED            (1 << 11)
#define FD2930_DEVICECONFIG_DUST_TO_RELAY               (1 << 12)
#define FD2930_DEVICECONFIG_ERROR_TEMP_TO_RELAY         (1 << 13)
#define FD2930_DEVICECONFIG_ERROR_UV_VOLTAGE_TO_RELAY   (1 << 14)
#define FD2930_DEVICECONFIG_ERROR_24V_TO_RELAY          (1 << 15)

#define FD2930_DEFAULT_DEVICE_CONFIG    (FD2930_DEVICECONFIG_RELAY_FIRE_ALLOWED | FD2930_DEVICECONFIG_RELAY_FAULT_ALLOWED | FD2930_DEVICECONFIG_RELAY_DUST_ALLOWED | \
  FD2930_DEVICECONFIG_ERROR_TEMP_TO_RELAY | FD2930_DEVICECONFIG_ERROR_UV_VOLTAGE_TO_RELAY  \
    | FD2930_DEVICECONFIG_FIRE_BIT1 | FD2930_DEVICECONFIG_SELFTEST_ALLOWED | FD2930_DEVICECONFIG_DUST_TO_RELAY | FD2930_DEVICECONFIG_HEAT_ALLOWED)


#define FD2930_STATE_FLAG_START                 (1 << 0)
#define FD2930_STATE_FLAG_CHANGE_BAUDRATE       (1 << 1)
#define FD2930_STATE_FLAG_UPDATE_CURRENT        (1 << 2)
#define FD2930_STATE_FLAG_ERASE_ARCHIVE         (1 << 3)
#define FD2930_STATE_FLAG_FFT_START             (1 << 4)
#define FD2930_STATE_FLAG_FFT_ACTIVE            (1 << 5)
#define FD2930_STATE_FLAG_READ_ARCHIVE          (1 << 8)
#define FD2930_STATE_FLAG_READ_ARCHIVE_BIN      (1 << 9)
#define FD2930_STATE_FLAG_INIT_CURRENT	        (1 << 10)

#define MB_REG_ADDR(_STR_, _REG_)      			((uint16_t *)&_STR_._REG_ - (uint16_t *)&_STR_)


typedef enum
{
  FD2930_LED_OFF = 0,
  FD2930_LED_YELLOW,
  FD2930_LED_YELLOW_BLINKING,
  FD2930_LED_RED,
  FD2930_LED_RED_BLINKING,
  FD2930_LED_GREEN,
  FD2930_LED_BLUE,
} DeviceLEDState_t;

typedef struct {
	uint16_t Page[100];
} Archive_t;

#if DEVICE_TYPE == PHOENIX_IRUV
typedef struct
{
    uint16_t MBId;
    uint16_t Baudrate;
    uint16_t SerialNumber;
    uint16_t DeviceType;
    uint16_t HWVersion;
    uint16_t FWVersion;
    uint16_t Status;
    uint16_t Flags;
    uint16_t Config;
    uint16_t Reserve0;
    uint16_t AppAddrHi;
    uint16_t AppAddrLo;
    uint16_t FFTExceeded;
    uint16_t UVThresF;
    uint16_t IRThresF;
    uint16_t UVCoeff;		// 14
    uint16_t IRCoeff;		// 15
    uint16_t FireDelay;
    uint16_t FaultDelay;
    uint16_t Command;		// 18
    uint16_t HeatPower;
    int16_t HeaterThres;
    int16_t Temperature;
    uint16_t UVVoltage;
    uint16_t InPowerVoltage;
    uint16_t IRDACmd;
    uint16_t Reserved0;
    uint16_t Seconds;		// 26
    uint16_t Minutes;
    uint16_t Hours;
    uint16_t Days;
    uint16_t Months;
    uint16_t Years;
    uint16_t ArchLastPageHi;	// 32
    uint16_t ArchLastPageLo;	// 33
    uint16_t BlockService;
    uint16_t Current420;
    uint16_t FWCheckSumm;
    uint16_t ArchiveEvent;
    uint16_t FFTGain;
    uint16_t IRRaw;
    uint16_t IRAv;		// 40
    uint16_t IRRect;	// 41
    uint16_t ArchPageIdxHi;	// 42
    uint16_t ArchPageIdxLo;	// 43
    uint16_t IRTestFaultCnt;
    uint16_t UVTestFaultCnt;
    uint16_t StateFlags;
    uint16_t UVRaw;
    uint16_t UVThres;
    uint16_t IRThres;

    uint16_t UVNoise; // 50
    uint16_t UVNoiseTest;
	uint16_t IRNoise;
	uint16_t IRNoiseTest;
    uint16_t UVTestLevel;
	uint16_t IRTestLevel;
    uint16_t IRTroubleCnt;
	uint16_t IRTroubleCntPiece;
	uint16_t UVTroubleCnt;
	uint16_t UVTroubleCntPiece;
    uint16_t UVPickCnt;

    uint16_t Reserved1[139];	// 61

    Archive_t Archive;		// 200th MB addr
    uint16_t FFTData[100];
} DeviceData_t;

#elif DEVICE_TYPE == PHOENIX_IR4
typedef struct
{
    uint16_t MBId;                      //modbus address
    uint16_t Baudrate;                           //modbus baudrate /4800
    uint16_t SerialNumber;
    uint16_t DeviceType;
    uint16_t HWVersion;
    uint16_t FWVersion;
    uint16_t Status;
    uint16_t Flags;
    uint16_t Config;
    uint16_t IRGain[PHOENIX_IR4_CHANNELS];  	// 9   filtered and scaled
    uint16_t Correlation;	// 13
    uint16_t IRCoeff12;		// 14
    uint16_t IRCoeff34;		// 15
    uint16_t FireDelay;		// 16
    uint16_t FaultDelay;	// 17
    uint16_t Command;		// 18
    uint16_t HeatPower;		// 19
    int16_t HeaterThres;	// 20
    int16_t Temperature;	// 21
    uint16_t FFTExceeded;	// 22
    uint16_t InPowerVoltage;// 23
    uint16_t IRDACmd;		// 24
    uint16_t Rat1Thres;		// 25
    uint16_t Seconds;
    uint16_t Minutes;
    uint16_t Hours;
    uint16_t Days;
    uint16_t Months;
    uint16_t Years;
    uint16_t ArchLastPageHi;
    uint16_t ArchLastPageLo;
    uint16_t BlockService;
    uint16_t Current420;
    uint16_t FWCheckSumm;
    uint16_t ArchiveEvent;
    uint16_t FFTGain;
    uint16_t Rat2Thres;
    uint16_t Rat3Thres;		// 40
    uint16_t Rat13Thres;
    uint16_t ArchPageIdxHi;
    uint16_t ArchPageIdxLo;
    uint16_t CntFaultIR;
    uint16_t CntFaultUV;
    uint16_t StateFlags;
    uint16_t Res3;
    uint16_t Res4;
    uint16_t Res5;
    float    Rat1;			// 50
    float    Rat2;
    float    Rat3;
    float    Rat1_3;

    uint16_t IRNoise;		// 58
    uint16_t IRNoiseTest;
    uint16_t IRTestLevel;
    uint16_t IRTestFaultCnt;
    uint16_t IRTroubleCnt;
    uint16_t IRTroubleCntPiece;

    uint16_t Reserved1[142];	// 64

    Archive_t Archive;		// 200th MB addr
    uint16_t FFTData[PHOENIX_IR4_CHANNELS * FFT_OUTPUT_POINTS];
} DeviceData_t;
#endif



#ifdef __cplusplus
}
#endif

#endif // _FUNCTIONAL_H
//------------------------------------------------------------------------------
// END
//------------------------------------------------------------------------------

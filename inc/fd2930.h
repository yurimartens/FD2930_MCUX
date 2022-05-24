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

#include <lpc17xx_rtc.h>

#include <board.h>
#include <dsplib_app.h>

#define DEVICE_TYPE         			PHOENIX_IRUV

#if DEVICE_TYPE == PHOENIX_IRUV
#define FW_VERSION                     	305
#define HW_VERSION        				2

#define FW_VERSION_HI                  	2022
#define FW_VERSION_LO                  	524

#elif DEVICE_TYPE == PHOENIX_IR4

#define FW_VERSION                     	306
#define HW_VERSION        				2

#define FW_VERSION_HI                  	2022
#define FW_VERSION_LO                  	524

#define PHOENIX_IR4_CHANNELS			4

#define SENSOR_INFRATEC					1
#define SENSOR_BELEAD					2
#define SENSOR_BELEAD_DET				3

#define SENSOR_TYPE						SENSOR_BELEAD_DET

#endif

#define MB_REG_ADDR(_STR_, _REG_)      	((uint16_t *)&_STR_._REG_ - (uint16_t *)&_STR_)

#define MAX_REFERENCE_mV				2200

#define ADC_REFERENCE_mV				3300
#define ADC_RATE						10000

#define ADC_SAMPLING_PERIOD				1000

#define UV_VOLTAGE_SCALE				0.247

#define INDICATION_TIME					100

#define STARTUP_BLINK					500

#define APPLICATION_ADDRESS				0x00004000

#define MB_ADDR_RUN_BOOTLOADER			0xFFFC
#define MB_ADDR_LEAVE_TRANSPARENT_MODE  0xFFFA
#define MB_ADDR_ENTER_TRANSPARENT_MODE  0xFFFB

#define RIT_INTERVAL_mS					1

#define TIME_mS_TO_TICK(_T_)			(_T_ / RIT_INTERVAL_mS)
#define START_DELAY           			100//25//10   //стартовая задержка, в 100мс для установки дефолтных настроек модбас
#define AFTER_START_DELAY     			1000//250//100    //задержка перед инициализацией DAC 1000мс
#define DELAY_05S             			500//250//100    //задержка 1 секунда
#define DELAY_1S              			1000//250//100    //задержка 1 секунда
#define DELAY_2S              			2000//250//100    //задержка 2 секунды
#define DELAY_3S              			3000//250//100    //задержка 3 секунды
#define DELAY_5S              			5000//1250//500    //задержка 5 секунда
#define DELAY_8S              			8000//1250//500    //задержка 5 секунда
#define DELAY_10S             			10000//2500//1000    //задержка 10 секунда
#define DELAY_CHECK_FIRE_STATUS         10000//2500//1000    //задержка 10 секунда
#define DELAY_15S             			15000//2500//1000    //задержка 15 секунд
#define DELAY_20S             			20000//2500//1000    //задержка 20 секунд
#define DELAY_CHANNEL_CALIB   			180000
#define DELAY_RELAY_WORK				10
#define IR_CHANNEL_TIMING1				90
#define IR_CHANNEL_TIMING2				140

#define STOP_BITS_1             		0

#define PARITY_NONE             		0
#define PARITY_ODD              		1
#define PARITY_EVEN             		2

#define PROTOCOL_FD2930					0
#define PROTOCOL_IPES					1

#define BAUDRATE_SCALE					100

#define FD2930_NUMBER_FFT_CRIM_CHANNEL  20


#define FD2930_DEF_MBS_ADDR        		3
#define FD2930_DEF_MBS_BAUD        		2
#define IPES_DEF_MBS_BAUD           	8

#define FD2930_MBS_BAUD_MULT   			4800
#define IPES_MBS_BAUD_MULT             	1200

#define FD2930_PASSWORD                 5813

#define FD2930_DEFAULT_THRES_IR         400
#define FD2930_MIN_THRES_IR              40
#define FD2930_MAX_THRES_IR              2000
#define FD2930_MIN_RAT_THRES             1
#define FD2930_MAX_RAT_THRES             20
#define FD2930_DEFAULT_THRES_UV          400
#define FD2930_MIN_THRES_UV              100
#define FD2930_MAX_THRES_UV              2000
#define FD2930_DEFAULT_K_IR             40
#define FD2930_MIN_K_IR                 1
#define FD2930_MAX_K_IR                 100
#define FD2930_DEFAULT_K_UV             40
#define FD2930_MIN_K_UV                 1
#define FD2930_MAX_K_UV                 100
#define FD2930_DEFAULT_WAIT_FIRE        3
#define FD2930_MIN_WAIT_FIRE            1
#define FD2930_MAX_WAIT_FIRE            20
#define FD2930_DEFAULT_WAIT_FAULT       3
#define FD2930_MIN_WAIT_FAULT           1
#define FD2930_MAX_WAIT_FAULT           20
#define FD2930_DEFAULT_THRES_HEATER      20
#define FD2930_MIN_THRES_HEATER          0
#define FD2930_MAX_THRES_HEATER          50
#define FD2930_DEFAULT_HEATPOWER        50
#define FD2930_MIN_HEATPOWER            1
#define FD2930_MAX_HEATPOWER            100
#define FD2930_DEFAULT_GAIN_FFT         29
#define FD2930_MIN_GAIN_FFT             5
#define FD2930_MAX_GAIN_FFT             100
#define FD2930_W_TRL                    3600// 3600 1 ÷àñ
#define FD2930_W_TRL_P                  10800//800// 108003 ÷àñà

#define FD2930_DEFAULT_RAT1_THRES		9
#define FD2930_DEFAULT_RAT2_THRES		3
#define FD2930_DEFAULT_RAT3_THRES		3
#define FD2930_DEFAULT_RAT13_THRES		5

#define MAX_RAT							65535


#define TEMPERATURE_MAXIMUM             110
#define TEMPERATURE_MINIMUM             -55
#define VOLTAGE_UV_WORKING_MAXIMUM      600
#define VOLTAGE_UV_WORKING_MINIMUM      400
#define TEMPERATURE_FAULT_DELAY         100


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
//#define FD2930_DEVICECONFIG_ARCHIVE_ALLOWED             (1 << 10)
#define FD2930_DEVICECONFIG_IPES_MB_HEADER              (1 << 10)
#define FD2930_DEVICECONFIG_SELFTEST_IR_ALLOWED         (1 << 11)
#define FD2930_DEVICECONFIG_DUST_TO_RELAY               (1 << 12)
#define FD2930_DEVICECONFIG_ERROR_TEMP_TO_RELAY         (1 << 13)
#define FD2930_DEVICECONFIG_ERROR_UV_VOLTAGE_TO_RELAY   (1 << 14)
#define FD2930_DEVICECONFIG_SELFTEST_UV_ALLOWED         (1 << 15)

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

#define UV_PICK_LIMIT							10000
#define UV_PICK_WORK_AREA						500
#define IR_GAIN_UV_PICK							150

#define LOW_SENSE_FACTOR						1.7

#define IRDA_INDICATION_OFF_INTERVAL    		100
#define IRDA_INDICATION_BLUE_INTERVAL   		200

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

typedef enum {
    IRDA_INDICATION_OFF = 0,
    IRDA_INDICATION_OFF_WAIT,
    IRDA_INDICATION_BLUE,
    IRDA_INDICATION_BLUE_WAIT,
} IRDAIndicationState_t;

typedef enum
{
  FD2930_STATE_START1,                       //стартовая задержка, в 100мс для установки дефолтных настроек модбас
  FD2930_STATE_START2,                       //задержка перед инициализацией DAC 1000мс
  FD2930_STATE_START3,                       //успокоение аналогового тракта и цифрового фильтра
  FD2930_STATE_WORKING,                         //рабочий режим (дежурный режим)
  FD2930_STATE_SELFTEST,                        //самотестирование, дежурный режим, но данные с АЦП не обновляются
  FD2930_STATE_TEST,                            //режим тест, прибор не считывает данные, проверка реле и токового выхода
  FD2930_STATE_BREAK,                           //режим авария, блокировка всех реле
  FD2930_STATE_CHANNEL_CALIBR,                  //режим калибровки каналов, блокировка всех реле
  FD2930_STATE_TEST_CALIBR,                     //режим калибровка тестовых источников, блокировка всех реле
  FD2930_STATE_TEST_ZERO,                       //установка нуля тестовых источников
} DeviceState_t;


typedef enum
{
  FD2930_CONFIG_1= 1,
  FD2930_CONFIG_2,
  FD2930_CONFIG_3,
  FD2930_CONFIG_4,
  FD2930_CONFIG_5,
  FD2930_CONFIG_6,
  FD2930_CONFIG_7,
  FD2930_CONFIG_8,
} DeviceFireConfig_t;

typedef struct {
	uint16_t Page[100];
} Archive_t;

#if DEVICE_TYPE == PHOENIX_IRUV
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
    uint16_t UVGain;   //отфильтрованное значение с учетом масштабирования
    uint16_t IRGain;   //отфильтрованное значение с учетом масштабирования
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

extern DeviceData_t	DeviceData;
extern RTC_TIME_Type DeviceTime;
extern uint8_t		ChangeConnectionSettings;
extern uint8_t		Protocol;

extern DeviceState_t	DeviceState;

extern uint32_t 	ArchPageIdx, ArchLastPage;

extern Timer_t		MeasurmentTimer;

extern DeviceLEDState_t LEDState;
extern uint8_t 		IRDA_Command_Rcvd;
extern uint8_t 		IRDA_Command;

void DeviceInit();
void ADCTask();
void SDADCTask(double avCoeffIR, double avCoeffUV);
void FunctionalTaskBG();
void FunctionalTaskPeriodic();
uint8_t MBCallBack(uint16_t addr, uint16_t qty);
uint8_t MBPassCallBack(uint16_t addr, uint16_t qty);



#ifdef __cplusplus
}
#endif

#endif // _FUNCTIONAL_H
//------------------------------------------------------------------------------
// END
//------------------------------------------------------------------------------

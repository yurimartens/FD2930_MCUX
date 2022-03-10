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

#define DEVICE_TYPE         			1
#define FW_VERSION                     	301
#define HW_VERSION        				2

#define FW_VERSION_HI                  	2022
#define FW_VERSION_LO                  	310


#define MB_REG_ADDR(_STR_, _REG_)      	((uint16_t *)&_STR_._REG_ - (uint16_t *)&_STR_)

#define MAX_REFERENCE_mV				2200

#define ADC_REFERENCE_mV				3300
#define ADC_RATE						10000

#define ADC_SAMPLING_PERIOD				1000

#define UV_VOLTAGE_SCALE				0.247

#define INDICATION_TIME					100

#define STARTUP_BLINK					500

#define MB_ADDR_RUN_BOOTLOADER			0xFFFC

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

#define PROTOCOL_FD2930					0
#define PROTOCOL_IPES					1

#define BAUDRATE_SCALE					100

#define FD2930_NUMBER_FFT_CRIM_CHANNEL  20


#define FD2930_DEF_MBS_ADDR        		3
#define FD2930_DEF_MBS_BAUD        		2
#define IPES_DEF_MBS_BAUD           	8

#define FD2930_MBS_BAUD_MULT   			4800
#define IPES_MBS_BAUD_MULT             	1200

#define FD2930_DEFAULT_THRES_IR         400
#define FD2930_MIN_THRES_IR              100
#define FD2930_MAX_THRES_IR              2000
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
#define FD2930_DEVICE_STATUS_IR_UV_SET                  (1 << 12)
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

#define UV_PICK_LIMIT							10000
#define UV_PICK_WORK_AREA						500
#define IR_GAIN_UV_PICK							150

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
    uint16_t UVCoeff;
    uint16_t IRCoeff;
    uint16_t FireDelay;
    uint16_t FaultDelay;
    uint16_t Command;
    uint16_t HeatPower;
    int16_t HeaterThres;
    int16_t Temperature;
    uint16_t UVVoltage;
    uint16_t InPowerVoltage;
    uint32_t WorkedTime;
    uint16_t Seconds;
    uint16_t Minutes;
    uint16_t Hours;
    uint16_t Days;
    uint16_t Months;
    uint16_t Years;
    uint32_t ArchLastPage;
    uint16_t BlockService;
    uint16_t Current420;
    uint16_t FWCheckSumm;
    uint16_t ArchiveEvent;
    uint16_t FFTGain;
    uint16_t IRRaw;
    uint16_t IRAv;		// 40
    uint16_t IRRect;
    uint32_t ArchPageIdx;
    uint16_t CntFaultIR;
    uint16_t CntFaultUV;
    uint16_t StateFlags;
    uint16_t UVRaw;
    uint16_t UVThres;
    uint16_t IRThres;
    uint16_t Reserved0[150];	// 50

    Archive_t Archive;		// 200th MB addr
    uint16_t FFTData[100];
} DeviceData_t;

extern DeviceData_t	DeviceData;
extern RTC_TIME_Type DeviceTime;
extern uint8_t		ChangeConnectionSettings;
extern uint8_t			Protocol;

extern Timer_t		MeasurmentTimer;

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

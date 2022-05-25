/*
===============================================================================
 Name        : FD2930_MCUX.c
 Author      : yurimartens@gmail.com
 Version     :
 Copyright   :
 Description : main definition
===============================================================================
*/

#ifdef __USE_CMSIS
#include "LPC17xx.h"
#endif

#include <string.h>

#include <cr_section_macros.h>
#include <system_LPC17xx.h>
#include <lpc17xx_systick.h>
#include <lpc17xx_uart.h>
#include <lpc17xx_gpio.h>
#include <lpc17xx_pinsel.h>
#include <lpc17xx_nvic.h>

#include <lpc_types.h>

#include <fd2930.h>

#include <board.h>

#include <uart_al.h>
#include <modbus.h>

#include <sys_utils.h>
#include <flash_al.h>
#include <eeprom.h>

#include <bootloader.h>
#include <boot_mb_cmd.h>

#define UART_RX_BUF_SIZE		256
#define UART_TX_BUF_SIZE		256

#define FLASH_BUF_SIZE			256


Timer_t     InitTimer;

uint8_t 	Protocol;

UartAl_t    UartMBS;
uint8_t     RxBuf[UART_RX_BUF_SIZE];
uint8_t     TxBuf[UART_TX_BUF_SIZE];
Modbus_t    Modbus;


uint16_t	AddrOffset = 0;

uint8_t		MCUPageSizes[30];

uint8_t     FlashWriteBuf[FLASH_BUF_SIZE];

uint8_t		FlashError = 0;

DeviceData_t	DeviceData;
DeviceLEDState_t LEDState = FD2930_LED_OFF;

__STATIC_INLINE void MCUPinsConfiguration();
__STATIC_INLINE void MCUPeriphConfiguration();
__STATIC_INLINE void Uart1AndProtocolInit();
__STATIC_INLINE void MCUPageSizesInit();
__STATIC_INLINE void DeviceInit();
__STATIC_INLINE void HandleLEDs();



/**
  * @brief
  * @param
  * @retval
  */
uint8_t MBPassCallBack(uint16_t addr, uint16_t qty_data)
{
	uint8_t *data;
	static uint32_t start = 0;
	static uint16_t addrTemp;

	switch (UartMBS.RxBuf[1]) {
		case MODBUS_WRITE_REGISTER:	// used for commands
			switch (addr) {
				case BOOT_MB_CMD_ERASE_APP_SPACE:
					if (0 != BootEraseAppSpace()) {
						FlashError = 1;
					} else {
						FlashError = 0;
					}
					break;
				case BOOT_MB_CMD_SET_ADDR_OFFSET:
					AddrOffset = qty_data;
					ModbusSetHoldingRegs(&Modbus, (uint16_t *)(APPLICATION_ADDRESS + AddrOffset), (((APPLICATION_SPACE - AddrOffset) / 2) > 0xFFFF) ? 0xFFFF : (APPLICATION_SPACE - AddrOffset) / 2);
					break;
				case BOOT_MB_CMD_RUN_APPLICATION:
					{
					uint32_t devId = 0;
					if (0 == BootCheckAppCRC(APPLICATION_ADDRESS, APPLICATION_SPACE, &devId)) {
						if (DEVICE_TYPE == devId) {
							DeviceData.Flags &= ~FD2930_DEVICEFLAGS_BOOTLOADER_ACTIVE;
							EEPROMWrite((uint8_t *)&DeviceData, EEPROM_PAGE_SIZE);
							JumpToApplication(APPLICATION_ADDRESS);
						}
					}
					}
					break;
			}
			break;
		case MODBUS_WRITE_MULT_REG:	// used for transfer data
			data = UartMBS.RxBuf + 7;
			if (qty_data < 124) {
				SwapMBWords(data, qty_data);
				if (start == 0) {
					start = qty_data * 2;
					memcpy(FlashWriteBuf, data, start);
					addrTemp = addr;
				} else {
					memcpy(&FlashWriteBuf[start], data, qty_data * 2);
					start = 0;
					if (0 != BootWriteData(AddrOffset, addrTemp, FLASH_BUF_SIZE, FlashWriteBuf)) {
						FlashError = 1;
					}
				}
			}
			break;
	}
    return 0;
}


/**
  * @brief
  * @param
  * @retval
  */
int main(void)
{
	NVIC_SetVTOR(0x00000000);

	SystemInit();
	NVIC_SetPriorityGrouping(0x00);

	DeviceInit();

	if ((DeviceData.Flags & FD2930_DEVICEFLAGS_BOOTLOADER_ACTIVE) == 0) {
		uint32_t devId = 0;
		if (0 == BootCheckAppCRC(APPLICATION_ADDRESS, APPLICATION_SPACE, &devId)) {
			if (DEVICE_TYPE == devId) {
				JumpToApplication(APPLICATION_ADDRESS);
			}
		}
		DeviceData.Flags |= FD2930_DEVICEFLAGS_BOOTLOADER_ACTIVE;
		EEPROMWrite((uint8_t *)&DeviceData, EEPROM_PAGE_SIZE);
	}

	MCUPinsConfiguration();
	MCUPeriphConfiguration();

	Uart1AndProtocolInit();

	MCUPageSizesInit();
	BootloaderInit(MCUPageSizes, sizeof(MCUPageSizes), APPLICATION_ADDRESS, APPLICATION_SPACE);

	LEDState = FD2930_LED_BLUE_BLINKING;

	while (1) {
		if (FlashError) DeviceData.Status |= FD2930_DEVICE_STATUS_FLASH_OK;
		else DeviceData.Status &= ~FD2930_DEVICE_STATUS_FLASH_OK;
		ModbusIdle(&Modbus);
	}
	return 0;
}

/**
  * @brief
  * @param
  * @retval
  */
void SetDefaultParameters()
{
	DeviceData.Config = FD2930_DEFAULT_DEVICE_CONFIG;
	DeviceData.SerialNumber = 1;
	DeviceData.Status = FD2930_DEVICE_STATUS_BREAK;
	DeviceData.MBId = FD2930_DEF_MBS_ADDR;
	DeviceData.Baudrate = FD2930_DEF_MBS_BAUD;
	DeviceData.Flags = FD2930_DEVICEFLAGS_FIRE_RELAY_ON | FD2930_DEVICEFLAGS_WORK_RELAY_ON | FD2930_DEVICEFLAGS_DUST_RELAY_ON | FD2930_DEVICEFLAGS_BLINK_LED | FD2930_DEVICEFLAGS_20mA_ON;
}

/**
  * @brief
  * @param
  * @retval
  */
__STATIC_INLINE void DeviceInit()
{
	uint8_t write = 0;

	EEPROMInit();

	if (EEPROM_ERROR_EMPTY == EEPROMRead((uint8_t *)&DeviceData, EEPROM_PAGE_SIZE)) {
		SetDefaultParameters();
		write = 1;
	} else {
		if (DeviceData.Config == 0 || (DeviceData.MBId < 1 || DeviceData.MBId > 247) || \
		   (DeviceData.Baudrate != 1 && DeviceData.Baudrate != 2 && DeviceData.Baudrate != 4 && DeviceData.Baudrate != 8 && DeviceData.Baudrate != 12 && DeviceData.Baudrate != 16 && DeviceData.Baudrate != 24)) {
				SetDefaultParameters();
		}
		if (DeviceData.Config & FD2930_DEVICECONFIG_IPES_MB_HEADER) {
			Protocol = PROTOCOL_IPES;
		} else {
			Protocol = PROTOCOL_FD2930;
		}
		if (Protocol == PROTOCOL_IPES) {
			if (DeviceData.Baudrate != 1 && DeviceData.Baudrate != 2 && DeviceData.Baudrate != 4 && DeviceData.Baudrate != 8 && DeviceData.Baudrate != 16) {
				DeviceData.Baudrate = IPES_DEF_MBS_BAUD;
			}
		} else {
			if (DeviceData.Baudrate != 1 && DeviceData.Baudrate != 2 && DeviceData.Baudrate != 4 && DeviceData.Baudrate != 12 && DeviceData.Baudrate != 24) {
				DeviceData.Baudrate = FD2930_DEF_MBS_BAUD;
			}
		}
		DeviceData.Status = 0;
		DeviceData.DeviceType = DEVICE_TYPE;
		DeviceData.AppAddrHi = (APPLICATION_ADDRESS >> 16) & 0xFFFF;
		DeviceData.AppAddrLo = APPLICATION_ADDRESS & 0xFFFF;

		DeviceData.HWVersion = FW_VERSION_HI;
		DeviceData.FWVersion = FW_VERSION_LO;
	}
	if (write) EEPROMWrite((uint8_t *)&DeviceData, EEPROM_PAGE_SIZE);
}

/**
  * @brief
  * @param
  * @retval
  */
__STATIC_INLINE void MCUPageSizesInit()
{
	for (int p = 0; p < sizeof(MCUPageSizes); p++) {
		if (p < 16) MCUPageSizes[p] = 4;
		else MCUPageSizes[p] = 32;
	}
}

/**
  * @brief
  * @param
  * @retval
  */
void UART1_IRQHandler(void)
{
    UARTIsrHandler(&UartMBS);
    NVIC_ClearPendingIRQ(UART1_IRQn);
}


/**
* @brief 		SysTick interrupt handler
* @param		None
* @return 		None
*/
void SysTick_Handler(void)
{
	TimerDispatch(&Modbus.Timer);
	HandleLEDs();
}

/**
  * @brief
  * @param
  * @retval
  */
__STATIC_INLINE void MCUPeriphConfiguration(void)
{
    SYSTICK_InternalInit(1);
    SYSTICK_IntCmd(ENABLE);
    SYSTICK_Cmd(ENABLE);

    NVIC_SetPriority(SysTick_IRQn, 20); //NVIC_EnableIRQ(SysTick_IRQn); already enabled

    LPC_SC->PCLKSEL0 &= ~(3 << UART1_PCLK_OFFSET);
    LPC_SC->PCLKSEL0 |= PCLCK_U1_CLK_1;
    LPC_SC->PCONP |= PCONP_PCUART1;
}


/**
  * @brief
  * @param
  * @retval
  */
__STATIC_INLINE void MCUPinsConfiguration(void)
{
    PINSEL_CFG_Type PinCfg;

    PinCfg.Portnum = 0;
    PinCfg.Pinnum = PINSEL_PIN_15;
    PinCfg.OpenDrain = PINSEL_PINMODE_NORMAL;
    PinCfg.Funcnum = PINSEL_FUNC_1;
    PinCfg.Pinmode = PINSEL_PINMODE_PULLUP;
    PINSEL_ConfigPin(&PinCfg);  //TX485

    PinCfg.Portnum = 0;
    PinCfg.Pinnum = PINSEL_PIN_16;
    PinCfg.OpenDrain = PINSEL_PINMODE_NORMAL;
    PinCfg.Funcnum = PINSEL_FUNC_1;
    PinCfg.Pinmode = PINSEL_PINMODE_PULLUP;
    PINSEL_ConfigPin(&PinCfg);  //RX485

    PinCfg.Portnum = 0;
    PinCfg.Pinnum = PINSEL_PIN_20;
    PinCfg.OpenDrain = PINSEL_PINMODE_NORMAL;
    PinCfg.Funcnum = PINSEL_FUNC_1;
    PinCfg.Pinmode = PINSEL_PINMODE_PULLUP;
    PINSEL_ConfigPin(&PinCfg);  //RTS485

    PinCfg.Portnum = 2;
    PinCfg.Pinnum = PINSEL_PIN_1;
    PinCfg.OpenDrain = PINSEL_PINMODE_OPENDRAIN;
    PinCfg.Funcnum = PINSEL_FUNC_0;
    PinCfg.Pinmode = PINSEL_PINMODE_PULLUP;
    PINSEL_ConfigPin(&PinCfg);  //LED1
    GPIO_SetDir(2, 1 << PINSEL_PIN_1, 1);
    GPIO_SetValue(2, 1 << PINSEL_PIN_1);

    PinCfg.Portnum = 2;
    PinCfg.Pinnum = PINSEL_PIN_2;
    PinCfg.OpenDrain = PINSEL_PINMODE_OPENDRAIN;
    PinCfg.Funcnum = PINSEL_FUNC_0;
    PinCfg.Pinmode = PINSEL_PINMODE_PULLUP;
    PINSEL_ConfigPin(&PinCfg);  //LED2
    GPIO_SetDir(2, 1 << PINSEL_PIN_2, 1);
    GPIO_SetValue(2, 1 << PINSEL_PIN_2);

    PinCfg.Portnum = 2;
    PinCfg.Pinnum = PINSEL_PIN_3;
    PinCfg.OpenDrain = PINSEL_PINMODE_OPENDRAIN;
    PinCfg.Funcnum = PINSEL_FUNC_0;
    PinCfg.Pinmode = PINSEL_PINMODE_PULLUP;
    PINSEL_ConfigPin(&PinCfg);  //LED3
    GPIO_SetDir(2, 1 << PINSEL_PIN_3, 1);
    GPIO_SetValue(2, 1 << PINSEL_PIN_3);
}


/**
  * @brief
  * @param
  * @retval
  */
__STATIC_INLINE void Uart1AndProtocolInit()
{
	if (Protocol == PROTOCOL_IPES) {
		UARTInit(&UartMBS, (LPC_UART_TypeDef *)LPC_UART1, (DeviceData.MBId & 0xFF) * IPES_MBS_BAUD_MULT, UART_PARITY_NONE, UART_STOPBIT_1, UART_FLAG_RS485_MODE_ENABLED);
		ModbusInit(&Modbus, &UartMBS, (DeviceData.MBId >> 8) & 0xFF, (uint16_t *)(APPLICATION_ADDRESS + AddrOffset), (uint16_t *)&DeviceData, (((APPLICATION_SPACE - AddrOffset) / 2) > 0xFFFF) ? 0xFFFF : (APPLICATION_SPACE - AddrOffset) / 2, sizeof(DeviceData_t) / 2, NULL, MBPassCallBack);
	} else {
		UARTInit(&UartMBS, (LPC_UART_TypeDef *)LPC_UART1, DeviceData.Baudrate * FD2930_MBS_BAUD_MULT, UART_PARITY_NONE, UART_STOPBIT_1, UART_FLAG_RS485_MODE_ENABLED);
		ModbusInit(&Modbus, &UartMBS, DeviceData.MBId, (uint16_t *)(APPLICATION_ADDRESS + AddrOffset), (uint16_t *)&DeviceData, (((APPLICATION_SPACE - AddrOffset) / 2) > 0xFFFF) ? 0xFFFF : (APPLICATION_SPACE - AddrOffset) / 2, sizeof(DeviceData_t) / 2, NULL, MBPassCallBack);
	}
	UARTInitDMA(&UartMBS, LPC_GPDMA, -1, 1);
	UARTInitTxBuf(&UartMBS, TxBuf, sizeof(TxBuf));
	UARTInitRxBuf(&UartMBS, RxBuf, sizeof(RxBuf));

	NVIC_SetPriority(UART1_IRQn, 18);
	NVIC_EnableIRQ(UART1_IRQn);
}


/**
  * @brief
  * @param
  * @retval
  */
__STATIC_INLINE void HandleLEDs()
{
	static uint16_t l = 0, redtime = 0, greentime = 0, blink_counter = 0;

	switch (LEDState) {
		case FD2930_LED_OFF:
			GPIO_SET_PIN(LPC_GPIO2, LED1);
			GPIO_SET_PIN(LPC_GPIO2, LED2);
			GPIO_SET_PIN(LPC_GPIO2, LED3);
		break;
		case FD2930_LED_YELLOW:
			GPIO_SET_PIN(LPC_GPIO2, LED2);
			GPIO_SET_PIN(LPC_GPIO2, LED3);
			if (l) {
				GPIO_RESET_PIN(LPC_GPIO2, LED1);
				GPIO_SET_PIN(LPC_GPIO2, LED3);
				if (!redtime) {
					l = 0;
					redtime = 13;
				} else {
					redtime--;
				}
			} else {
				GPIO_RESET_PIN(LPC_GPIO2, LED3);
				GPIO_SET_PIN(LPC_GPIO2, LED1);
				if (!greentime) {
					l = 1;
					greentime = 1;
				} else {
					greentime--;
				}
			}
	    break;
		case FD2930_LED_YELLOW_BLINKING:
			GPIO_SET_PIN(LPC_GPIO2, LED2);
			if (blink_counter < 500) {
				if (l) {
					GPIO_RESET_PIN(LPC_GPIO2, LED1);
					GPIO_SET_PIN(LPC_GPIO2, LED3);
					if (!redtime) {
						l = 0;
						redtime = 13;
					} else {
						redtime--;
					}
				} else {
					GPIO_RESET_PIN(LPC_GPIO2, LED3);
					GPIO_SET_PIN(LPC_GPIO2, LED1);
					if (!greentime) {
						l = 1;
						greentime = 1;
					} else {
						greentime--;
					}
				}
				blink_counter++;
			} else
				if (blink_counter < 1000) {
					GPIO_SET_PIN(LPC_GPIO2, LED1);
					GPIO_SET_PIN(LPC_GPIO2, LED3);
					blink_counter++;
				} else {
					blink_counter = 0;
				}
		break;
		case FD2930_LED_RED:
			GPIO_RESET_PIN(LPC_GPIO2, LED1);
			GPIO_SET_PIN(LPC_GPIO2, LED2);
			GPIO_SET_PIN(LPC_GPIO2, LED3);
	    break;
		case FD2930_LED_RED_BLINKING:
			GPIO_SET_PIN(LPC_GPIO2, LED2);
			GPIO_SET_PIN(LPC_GPIO2, LED3);
			if (redtime >= 100) {
				GPIO_TOGGLE_PIN(LPC_GPIO2, LED1);
				redtime = 0;
			} else {
				redtime++;
			}
	    break;
		case FD2930_LED_GREEN:
			GPIO_RESET_PIN(LPC_GPIO2, LED3);
			GPIO_SET_PIN(LPC_GPIO2, LED1);
			GPIO_SET_PIN(LPC_GPIO2, LED2);
	    break;
		case FD2930_LED_BLUE:
			GPIO_RESET_PIN(LPC_GPIO2, LED2);
			GPIO_SET_PIN(LPC_GPIO2, LED1);
			GPIO_SET_PIN(LPC_GPIO2, LED3);
	    break;
		case FD2930_LED_BLUE_BLINKING:
			GPIO_SET_PIN(LPC_GPIO2, LED1);
			GPIO_SET_PIN(LPC_GPIO2, LED3);
			if (redtime >= 1000) {
				GPIO_TOGGLE_PIN(LPC_GPIO2, LED2);
				redtime = 0;
			} else {
				redtime++;
			}
			break;
	}
}

//------------------------------------------------------------------------------
// end
//------------------------------------------------------------------------------


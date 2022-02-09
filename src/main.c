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

#include <cr_section_macros.h>
#include <system_LPC17xx.h>
#include <lpc17xx_systick.h>
#include <lpc17xx_ssp.h>
#include <lpc17xx_uart.h>
#include <lpc17xx_timer.h>
#include <lpc17xx_rit.h>
#include <lpc17xx_gpio.h>
#include <lpc17xx_pinsel.h>


#include <lpc_types.h>

#include <fd2930.h>

#include <board.h>

#include <uart_al.h>
#include <ssp_al.h>
#include <uart_al.h>
#include <adc_al.h>
#include <max11040.h>
#include <modbus.h>

#include <sys_utils.h>



UartAl_t    Uart;
uint8_t     RxBuf[256];
uint8_t     TxBuf[256];
Modbus_t    Modbus;

SSPAl_t     SSP;
uint8_t     OutBuf[10];
uint8_t     InBuf[16];




__STATIC_INLINE void MCUPinsConfiguration(void);
__STATIC_INLINE void MCUPeriphConfiguration(void);
__STATIC_INLINE void Uart1AndProtocolInit();



int main(void) {

	SystemInit();
	NVIC_SetPriorityGrouping(0x00);

	MCUPinsConfiguration();
	MCUPeriphConfiguration();

	DeviceInit();

	ADCInit(LPC_ADC, ADC_RATE, ADC_REFERENCE_mV);

	Uart1AndProtocolInit();

	SSPInit(&SSP, LPC_SSP1, 1000000, SSP_CPHA_FIRST, SSP_CPOL_LO);
	SSPInitCSPin(&SSP, 0, LPC_GPIO0, SSEL1);
	SSPInitTxBuf(&SSP, OutBuf, sizeof(OutBuf));
	SSPInitRxBuf(&SSP, InBuf, sizeof(InBuf));

	Max11040Init(&SSP, 0, 2200);	// ref in mV

	uint8_t *reg = Max11040GetReg(MAX11040_REG_CONFIG, 1);

	//reg = Max11040GetReg(MAX11040_REG_ADC_DATA, 1);
	//reg = Max11040GetReg(MAX11040_REG_CONFIG, 1);



	//IRPortInit();

	//AppTimerInit();  //запуск по таймеру рабочего цикла



	TimerInit(&IndicationTimer, 0);
	TimerReset(&IndicationTimer, 20000);

	while (1) {

		ModbusIdle(&Modbus);
		ADCTask();
		FunctionalTaskBG();

	}
	return 0;
}

/**
  * @brief
  * @param
  * @retval
  */
void UART1_IRQHandler(void)
{
    UARTIsrHandler(&Uart);
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
	TimerDispatch(&IndicationTimer);
	TimerDispatch(&MeasurmentTimer);
}

/**
  * @brief
  * @param
  * @retval
  */
void RIT_IRQHandler(void)
{
	RIT_GetIntStatus(LPC_RIT);
	RTC_GetFullTime(LPC_RTC, &DeviceTime);
    NVIC_ClearPendingIRQ(RIT_IRQn);

    FunctionalTaskPeriodic();

    LPC_WDT->WDFEED = 0xAA; LPC_WDT->WDFEED = 0x55;
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

    NVIC_SetPriority(SysTick_IRQn, 17); //NVIC_EnableIRQ(SysTick_IRQn); already enabled

    LPC_SC->PCLKSEL0 &= ~(3 << UART1_PCLK_OFFSET);
    LPC_SC->PCLKSEL0 |= PCLCK_U1_CLK_1;
    LPC_SC->PCONP |= (PCONP_PCUART1 | PCONP_PCGPDMA);
    NVIC_SetPriority(UART1_IRQn, 18);
    NVIC_EnableIRQ(UART1_IRQn);
    NVIC_ClearPendingIRQ(UART1_IRQn);

    LPC_SC->PCONP |= PCONP_PCAD;
    NVIC_ClearPendingIRQ(ADC_IRQn);

    RIT_Init(LPC_RIT);
    RIT_TimerConfig(LPC_RIT, RIT_INTERVAL_mS);
    NVIC_SetPriority(RIT_IRQn, 20);
    NVIC_EnableIRQ(RIT_IRQn);
    NVIC_ClearPendingIRQ(RIT_IRQn);

    RTC_Init(LPC_RTC);
    NVIC_DisableIRQ(RTC_IRQn);
    RTC_ResetClockTickCounter(LPC_RTC);
    RTC_Cmd(LPC_RTC, ENABLE);
    RTC_CalibCounterCmd(LPC_RTC, DISABLE);
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
    PinCfg.Pinnum = PINSEL_PIN_6;
    PinCfg.OpenDrain = PINSEL_PINMODE_NORMAL;
    PinCfg.Funcnum = PINSEL_FUNC_0;
    PinCfg.Pinmode = PINSEL_PINMODE_PULLUP;
    PINSEL_ConfigPin(&PinCfg);    //SSEL1
    GPIO_SetDir(0, 1 << PINSEL_PIN_6, 1);
    GPIO_SetValue(0, 1 << PINSEL_PIN_6);

    PinCfg.Portnum = 0;
    PinCfg.Pinnum = PINSEL_PIN_7;
    PinCfg.OpenDrain = PINSEL_PINMODE_NORMAL;
    PinCfg.Funcnum = PINSEL_FUNC_2;
    PinCfg.Pinmode = PINSEL_PINMODE_PULLUP;
    PINSEL_ConfigPin(&PinCfg);  //SCK1

    PinCfg.Portnum = 0;
    PinCfg.Pinnum = PINSEL_PIN_8;
    PinCfg.OpenDrain = PINSEL_PINMODE_NORMAL;
    PinCfg.Funcnum = PINSEL_FUNC_2;
    PinCfg.Pinmode = PINSEL_PINMODE_PULLUP;
    PINSEL_ConfigPin(&PinCfg);  //MISO1

    PinCfg.Portnum = 0;
    PinCfg.Pinnum = PINSEL_PIN_9;
    PinCfg.OpenDrain = PINSEL_PINMODE_NORMAL;
    PinCfg.Funcnum = PINSEL_FUNC_2;
    PinCfg.Pinmode = PINSEL_PINMODE_PULLUP;
    PINSEL_ConfigPin(&PinCfg);  //MOSI1

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

    PinCfg.Portnum = 0;
    PinCfg.Pinnum = PINSEL_PIN_11;
    PinCfg.OpenDrain = PINSEL_PINMODE_NORMAL;
    PinCfg.Funcnum = PINSEL_FUNC_0;
    PinCfg.Pinmode = PINSEL_PINMODE_PULLUP;
    PINSEL_ConfigPin(&PinCfg);      //IR_PORT

    PinCfg.Portnum = 0;
    PinCfg.Pinnum = PINSEL_PIN_23;
    PinCfg.OpenDrain = PINSEL_PINMODE_NORMAL;
    PinCfg.Funcnum = PINSEL_FUNC_1;
    PinCfg.Pinmode = PINSEL_PINMODE_TRISTATE;
    PINSEL_ConfigPin(&PinCfg);    //adc0

    PinCfg.Portnum = 0;
    PinCfg.Pinnum = PINSEL_PIN_24;
    PinCfg.OpenDrain = PINSEL_PINMODE_NORMAL;
    PinCfg.Funcnum = PINSEL_FUNC_1;
    PinCfg.Pinmode = PINSEL_PINMODE_TRISTATE;
    PINSEL_ConfigPin(&PinCfg);      //adc1

    PinCfg.Portnum = 0;
    PinCfg.Pinnum = PINSEL_PIN_25;
    PinCfg.OpenDrain = PINSEL_PINMODE_NORMAL;
    PinCfg.Funcnum = PINSEL_FUNC_1;
    PinCfg.Pinmode = PINSEL_PINMODE_TRISTATE;
    PINSEL_ConfigPin(&PinCfg);      //adc2

    PinCfg.Portnum = 1;
    PinCfg.Pinnum = PINSEL_PIN_23;
    PinCfg.OpenDrain = PINSEL_PINMODE_NORMAL;
    PinCfg.Funcnum = PINSEL_FUNC_3;
    PinCfg.Pinmode = PINSEL_PINMODE_PULLUP;
    PINSEL_ConfigPin(&PinCfg);      //MISO0

    PinCfg.Portnum = 1;
    PinCfg.Pinnum = PINSEL_PIN_24;
    PinCfg.OpenDrain = PINSEL_PINMODE_NORMAL;
    PinCfg.Funcnum = PINSEL_FUNC_3;
    PinCfg.Pinmode = PINSEL_PINMODE_PULLUP;
    PINSEL_ConfigPin(&PinCfg);      //MOSI0

    PinCfg.Portnum = 1;
    PinCfg.Pinnum = PINSEL_PIN_20;
    PinCfg.OpenDrain = PINSEL_PINMODE_NORMAL;
    PinCfg.Funcnum = PINSEL_FUNC_3;
    PinCfg.Pinmode = PINSEL_PINMODE_PULLUP;
    PINSEL_ConfigPin(&PinCfg);      //SCK0

    PinCfg.Portnum = 1;
    PinCfg.Pinnum = PINSEL_PIN_21;
    PinCfg.OpenDrain = PINSEL_PINMODE_NORMAL;
    PinCfg.Funcnum = PINSEL_FUNC_3;
    PinCfg.Pinmode = PINSEL_PINMODE_PULLUP;
    PINSEL_ConfigPin(&PinCfg);      //SSEL0

    PinCfg.Portnum = 1;
    PinCfg.Pinnum = PINSEL_PIN_22;
    PinCfg.OpenDrain = PINSEL_PINMODE_NORMAL;
    PinCfg.Funcnum = PINSEL_FUNC_0;
    PinCfg.Pinmode = PINSEL_PINMODE_PULLUP;
    PINSEL_ConfigPin(&PinCfg);      //SYNC
    GPIO_SetDir(1, 1 << PINSEL_PIN_22, 1);
    GPIO_SetValue(1, 1 << PINSEL_PIN_22);


    PinCfg.Portnum = 2;
    PinCfg.Pinnum = PINSEL_PIN_0;
    PinCfg.OpenDrain = PINSEL_PINMODE_NORMAL;
    PinCfg.Funcnum = PINSEL_FUNC_0;
    PinCfg.Pinmode = PINSEL_PINMODE_PULLUP;
    PINSEL_ConfigPin(&PinCfg);  //heaton
    GPIO_SetDir(2, 1 << PINSEL_PIN_0, 1);
    GPIO_SetValue(2, 1 << PINSEL_PIN_0);

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

    PinCfg.Portnum = 2;
    PinCfg.Pinnum = PINSEL_PIN_4;
    PinCfg.OpenDrain = PINSEL_PINMODE_NORMAL;
    PinCfg.Funcnum = PINSEL_FUNC_0;
    PinCfg.Pinmode = PINSEL_PINMODE_PULLUP;
    PINSEL_ConfigPin(&PinCfg);  //IR_TEST
    GPIO_SetDir(2, 1 << PINSEL_PIN_4, 1);
    GPIO_SetValue(2, 1 << PINSEL_PIN_4);

    PinCfg.Portnum = 2;
    PinCfg.Pinnum = PINSEL_PIN_5;
    PinCfg.OpenDrain = PINSEL_PINMODE_NORMAL;
    PinCfg.Funcnum = PINSEL_FUNC_0;
    PinCfg.Pinmode = PINSEL_PINMODE_PULLUP;
    PINSEL_ConfigPin(&PinCfg);  //UV_TEST
    GPIO_SetDir(2, 1 << PINSEL_PIN_5, 1);
    GPIO_SetValue(2, 1 << PINSEL_PIN_5);

    PinCfg.Portnum = 2;
    PinCfg.Pinnum = PINSEL_PIN_6;
    PinCfg.OpenDrain = PINSEL_PINMODE_NORMAL;
    PinCfg.Funcnum = PINSEL_FUNC_0;
    PinCfg.Pinmode = PINSEL_PINMODE_PULLUP;
    PINSEL_ConfigPin(&PinCfg);  //R_FIRE
    GPIO_SetDir(2, 1 << PINSEL_PIN_6, 1);
    GPIO_SetValue(2, 1 << PINSEL_PIN_6);

    PinCfg.Portnum = 2;
    PinCfg.Pinnum = PINSEL_PIN_7;
    PinCfg.OpenDrain = PINSEL_PINMODE_NORMAL;
    PinCfg.Funcnum = PINSEL_FUNC_0;
    PinCfg.Pinmode = PINSEL_PINMODE_PULLUP;
    PINSEL_ConfigPin(&PinCfg);  //R_WORK
    GPIO_SetDir(2, 1 << PINSEL_PIN_7, 1);
    GPIO_SetValue(2, 1 << PINSEL_PIN_7);

    PinCfg.Portnum = 2;
    PinCfg.Pinnum = PINSEL_PIN_8;
    PinCfg.OpenDrain = PINSEL_PINMODE_NORMAL;
    PinCfg.Funcnum = PINSEL_FUNC_0;
    PinCfg.Pinmode = PINSEL_PINMODE_PULLUP;
    PINSEL_ConfigPin(&PinCfg);  //R_DUST
    GPIO_SetDir(2, 1 << PINSEL_PIN_8, 1);
    GPIO_SetValue(2, 1 << PINSEL_PIN_8);

    PinCfg.Portnum = 4;
    PinCfg.Pinnum = PINSEL_PIN_29;
    PinCfg.OpenDrain = PINSEL_PINMODE_NORMAL;
    PinCfg.Funcnum = PINSEL_FUNC_0;
    PinCfg.Pinmode = PINSEL_PINMODE_PULLUP;
    PINSEL_ConfigPin(&PinCfg);  //HALL1
    GPIO_SetDir(4, 1 << PINSEL_PIN_29, 0);

    PinCfg.Portnum = 4;
    PinCfg.Pinnum = PINSEL_PIN_28;
    PinCfg.OpenDrain = PINSEL_PINMODE_NORMAL;
    PinCfg.Funcnum = PINSEL_FUNC_0;
    PinCfg.Pinmode = PINSEL_PINMODE_PULLUP;
    PINSEL_ConfigPin(&PinCfg);  //HALL2
    GPIO_SetDir(4, 1 << PINSEL_PIN_28, 0);
}


/**
  * @brief
  * @param
  * @retval
  */
__STATIC_INLINE void Uart1AndProtocolInit()
{
	UARTInit(&Uart, (LPC_UART_TypeDef *)LPC_UART1, 9600, UART_PARITY_NONE, UART_STOPBIT_1, UART_FLAG_RS485_MODE_ENABLED);
	UARTInitDMA(&Uart, LPC_GPDMA, -1, 1);
	UARTInitTxBuf(&Uart, TxBuf, sizeof(TxBuf));
	UARTInitRxBuf(&Uart, RxBuf, sizeof(RxBuf));

	ModbusInit(&Modbus, &Uart, 3, (uint16_t *)&DeviceData, (uint16_t *)&DeviceData, sizeof(DeviceData_t) / 2, sizeof(DeviceData_t) / 2, 0, 0);
}
//------------------------------------------------------------------------------
// end
//------------------------------------------------------------------------------


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
#include <lpc17xx_nvic.h>



#include <lpc_types.h>

#include <fd2930.h>

#include <board.h>

#include <uart_al.h>
#include <ssp_al.h>
#include <uart_al.h>
#include <adc_al.h>
#include <max11040.h>
#include <modbus.h>
#include <ad5421.h>

#include <sys_utils.h>
#include <flash_al.h>
#include <pwm_al.h>

#include <log_app.h>

#include <nec_protocol.h>



UartAl_t    Uart;
uint8_t     RxBuf[256];
uint8_t     TxBuf[256];
Modbus_t    Modbus;

SSPAl_t     SSPADC;
uint8_t     ADCOutBuf[10];
uint8_t     ADCInBuf[16];

SSPAl_t     SSPSD420;
uint8_t     SD420OutBuf[10];
uint8_t     SD420InBuf[16];



__STATIC_INLINE void MCUPinsConfiguration(void);
__STATIC_INLINE void MCUPeriphConfiguration(void);
__STATIC_INLINE void RUNPeriodicTasks();
__STATIC_INLINE void Uart1AndProtocolInit();
__STATIC_INLINE void SSP0AndModulesInit();
__STATIC_INLINE void SSP1AndModulesInit();



/**
  * @brief
  * @param
  * @retval
  */
void IRCallBack(uint8_t addr, uint8_t comm)
{
    IRDA_Command = comm;        // modbus output
    IRDA_Command_Rcvd = 1;      // led indication
}

/**
  * @brief
  * @param
  * @retval
  */
void IRPortInit(void)
{
    TIM_TIMERCFG_Type TIM_ConfigStruct;

    TIM_ConfigStruct.PrescaleOption = TIM_PRESCALE_USVAL;
    TIM_ConfigStruct.PrescaleValue = 1;
    TIM_Init(LPC_TIM0, TIM_TIMER_MODE, &TIM_ConfigStruct);

    TIM_Cmd(LPC_TIM0, TIM_ENABLE);
    NecProtocolInit(IRCallBack);

    PINSEL_CFG_Type PinCfg;
    PinCfg.Portnum = 0;
    PinCfg.Pinnum = PINSEL_PIN_11;
    PinCfg.OpenDrain = PINSEL_PINMODE_NORMAL;
    PinCfg.Funcnum = PINSEL_FUNC_0;
    PinCfg.Pinmode = PINSEL_PINMODE_PULLUP;

    PINSEL_ConfigPin(&PinCfg);

    GPIO_IntCmd(0, IR_PORT_PIN, 0);
    GPIO_IntCmd(0, IR_PORT_PIN, 1);
    NVIC_SetPriority(EINT3_IRQn, 19);
    NVIC_EnableIRQ(EINT3_IRQn);
}

/**
  * @brief
  * @param
  * @retval
  */
void EINT3_IRQHandler(void)
{
    GPIO_ClearInt(0, IR_PORT_PIN);
    NecRxData(LPC_TIM0->TC);
}


/**
  * @brief
  * @param
  * @retval
  */
int main(void) {

	__enable_irq();

	NVIC_SetVTOR(0x4000);

	SystemInit();
	NVIC_SetPriorityGrouping(0x00);

	MCUPinsConfiguration();
	MCUPeriphConfiguration();

	DeviceInit();

	Uart1AndProtocolInit();
	SSP0AndModulesInit();
	SSP1AndModulesInit();

	RUNPeriodicTasks();	// Must be called after SSP1 init due to SDADC activity in RIT ISR

	ADCInit(LPC_ADC, ADC_RATE, ADC_REFERENCE_mV);

	PWM1Init(1000); // 1kHz

	IRPortInit();

	if (LogAppInit()) {
		SSP0AndModulesInit();	// reinit, in fatfs speed is slow
	}

	while (1) {
		if (ChangeConnectionSettings) {
			if (Modbus.State == mbsWAIT) {
				ChangeConnectionSettings = 0;
				Uart1AndProtocolInit();
			}
		}
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
	static uint32_t div = 0;

	static IRDAIndicationState_t IRDAIndicatorState = IRDA_INDICATION_OFF;
	static uint32_t sysTick = 0, startTick = 0;
	static DeviceLEDState_t LEDStatePrev;

	TimerDispatch(&Modbus.Timer);
	TimerDispatch(&MeasurmentTimer);

	if (div < 10) {
		div++;
	} else {
		div = 0;
		disk_timerproc();   //for SD card
	}

	sysTick++;
	if (IRDA_Command_Rcvd) {
		switch (IRDAIndicatorState) {
			case IRDA_INDICATION_OFF:
				LEDStatePrev = LEDState;
				LEDState = FD2930_LED_OFF;
				startTick = sysTick;
				IRDAIndicatorState++;
				break;
			case IRDA_INDICATION_OFF_WAIT:
				if ((sysTick - startTick) > IRDA_INDICATION_OFF_INTERVAL){
					IRDAIndicatorState++;
				}
	            break;
			case IRDA_INDICATION_BLUE:
				LEDState = FD2930_LED_BLUE;
				startTick = sysTick;
				IRDAIndicatorState++;
				break;
			case IRDA_INDICATION_BLUE_WAIT:
				if ((sysTick - startTick) > IRDA_INDICATION_BLUE_INTERVAL){
					IRDAIndicatorState = IRDA_INDICATION_OFF;
					IRDA_Command_Rcvd = 0;
					LEDState = LEDStatePrev;
				}
	            break;
		}
	}
	NecProtocolResetCondition(LPC_TIM0->TC);
}

/**
  * @brief
  * @param
  * @retval
  */
void RIT_IRQHandler(void)
{
	RIT_GetIntStatus(LPC_RIT);
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

    NVIC_SetPriority(SysTick_IRQn, 20); //NVIC_EnableIRQ(SysTick_IRQn); already enabled

    LPC_SC->PCLKSEL1 &= ~(3 << SSP0_PCLK_OFFSET);
    LPC_SC->PCLKSEL1 |= PCLCK_SSP0_CLK_2;

    LPC_SC->PCLKSEL0 &= ~(3 << SSP1_PCLK_OFFSET);
    LPC_SC->PCLKSEL0 |= PCLCK_SSP1_CLK_1;

    LPC_SC->PCLKSEL0 &= ~(3 << UART1_PCLK_OFFSET);
    LPC_SC->PCLKSEL0 |= PCLCK_U1_CLK_1;

    LPC_SC->PCONP |= PCONP_PCAD;
    NVIC_ClearPendingIRQ(ADC_IRQn);

    RIT_Init(LPC_RIT);
    RIT_TimerConfig(LPC_RIT, RIT_INTERVAL_mS);
    NVIC_SetPriority(RIT_IRQn, 19);
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
__STATIC_INLINE void RUNPeriodicTasks()
{
	NVIC_EnableIRQ(RIT_IRQn);
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
    PinCfg.Funcnum = PINSEL_FUNC_0;
    PinCfg.Pinmode = PINSEL_PINMODE_PULLUP;
    PINSEL_ConfigPin(&PinCfg);      //SSEL0
    GPIO_SetDir(1, 1 << PINSEL_PIN_21, 1);
    GPIO_SetValue(1, 1 << PINSEL_PIN_21);

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
    PinCfg.OpenDrain = PINSEL_PINMODE_OPENDRAIN;
    PinCfg.Funcnum = PINSEL_FUNC_1;
    PinCfg.Pinmode = PINSEL_PINMODE_TRISTATE;
    PINSEL_ConfigPin(&PinCfg);  //heaton

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
	if (Protocol == PROTOCOL_IPES) {
		UARTInit(&Uart, (LPC_UART_TypeDef *)LPC_UART1, (DeviceData.MBId & 0xFF) * IPES_MBS_BAUD_MULT, UART_PARITY_NONE, UART_STOPBIT_1, UART_FLAG_RS485_MODE_ENABLED);
		ModbusInit(&Modbus, &Uart, (DeviceData.MBId >> 8) & 0xFF, (uint16_t *)&DeviceData, (uint16_t *)&DeviceData, sizeof(DeviceData_t) / 2, sizeof(DeviceData_t) / 2, MBCallBack, MBPassCallBack);
	} else {
		UARTInit(&Uart, (LPC_UART_TypeDef *)LPC_UART1, DeviceData.Baudrate * FD2930_MBS_BAUD_MULT, UART_PARITY_NONE, UART_STOPBIT_1, UART_FLAG_RS485_MODE_ENABLED);
		ModbusInit(&Modbus, &Uart, DeviceData.MBId, (uint16_t *)&DeviceData, (uint16_t *)&DeviceData, sizeof(DeviceData_t) / 2, sizeof(DeviceData_t) / 2, MBCallBack, MBPassCallBack);
	}

	UARTInitDMA(&Uart, LPC_GPDMA, -1, 1);
	UARTInitTxBuf(&Uart, TxBuf, sizeof(TxBuf));
	UARTInitRxBuf(&Uart, RxBuf, sizeof(RxBuf));

	NVIC_SetPriority(UART1_IRQn, 18);
	NVIC_EnableIRQ(UART1_IRQn);
}

/**
  * @brief
  * @param
  * @retval
  */
__STATIC_INLINE void SSP0AndModulesInit()
{
	SSPInit(&SSPSD420, LPC_SSP0, SCLK_FAST, SSP_CPHA_FIRST, SSP_CPOL_HI);	// HI means lo sck in idle
	SSPInitCSPin(&SSPSD420, 0, LPC_GPIO1, SYNC);
	SSPInitCSPin(&SSPSD420, 1, LPC_GPIO1, SSEL0);
	SSPInitTxBuf(&SSPSD420, SD420OutBuf, sizeof(SD420OutBuf));
	SSPInitRxBuf(&SSPSD420, SD420InBuf, sizeof(SD420InBuf));
	/*
	 * Init of ad5421 is in RIT state machine
	 */
}

/**
  * @brief
  * @param
  * @retval
  */
__STATIC_INLINE void SSP1AndModulesInit()
{
	SSPInit(&SSPADC, LPC_SSP1, 5000000, SSP_CPHA_FIRST, SSP_CPOL_LO);
	SSPInitCSPin(&SSPADC, 0, LPC_GPIO0, SSEL1);
	SSPInitTxBuf(&SSPADC, ADCOutBuf, sizeof(ADCOutBuf));
	SSPInitRxBuf(&SSPADC, ADCInBuf, sizeof(ADCInBuf));

	Max11040Init(&SSPADC, 0, MAX_REFERENCE_mV);	// ref in mV
}
//------------------------------------------------------------------------------
// end
//------------------------------------------------------------------------------


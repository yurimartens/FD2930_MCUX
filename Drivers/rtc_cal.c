/* Martin Thomas, 7/2010, 3BSD license */

#include "lpc17xx_rtc.h"
#include "rtc_cal.h"
#include "LPC17xx.h"


/* # of bits used in the Calibration value */
#define CALIBRATION_VALUE_SIZE	17
/* If the actual frequency is within +/- of this value,
	calibration should be disabled because of overflow */
#define MAX_DELTA_FREQUENCY_VALUE	(RTC_IDEAL_FREQ / ((1 << CALIBRATION_VALUE_SIZE) - 1))

void AppRTCTaskInit(void)
{
//  RTC clock output to pin 1.27 (43)
//  PINCON->PINSEL3 |= (1 << 22);
//  SC->CLKOUTCFG = 0x104;
  
  //if(!(RTC->CCR & RTC_CCR_CLKEN))  //���� ���� �� ����� �� ������
  if(1)
  {
    /* Initialize RTC  */
    // Init RTC module
    RTC_Init(LPC_RTC);
    
    /* Disable RTC interrupt */
    NVIC_DisableIRQ(RTC_IRQn);
    NVIC_SetPriority(RTC_IRQn, 0);
    
    /* Enable rtc (starts increase the tick counter and second counter register) */
    RTC_ResetClockTickCounter(LPC_RTC);
    RTC_Cmd(LPC_RTC, ENABLE);
    RTC_CalibCounterCmd(LPC_RTC, DISABLE);
    
    /* Set current time for RTC */
    // Current time is 14:00:00PM, 2011-09-13
    RTC_SetTime (LPC_RTC, RTC_TIMETYPE_HOUR, 0);
    RTC_SetTime (LPC_RTC, RTC_TIMETYPE_MINUTE, 0);
    RTC_SetTime (LPC_RTC, RTC_TIMETYPE_SECOND, 0);
    RTC_SetTime (LPC_RTC, RTC_TIMETYPE_MONTH, 1);
    RTC_SetTime (LPC_RTC, RTC_TIMETYPE_YEAR, 2011);
    RTC_SetTime (LPC_RTC, RTC_TIMETYPE_DAYOFMONTH, 1);
    
    /* Setting Timer calibration
    * Calibration value =  5s;
    * Direction = Forward calibration
    * So after each 5s, calibration logic can periodically adjust the time counter by
    * incrementing the counter by 2 instead of 1
    */
    //    RTC_CalibConfig(RTC, 5400, RTC_CALIB_DIR_FORWARD);
//    RTC_CalibCounterCmd(LPC_RTC, ENABLE);
  }

  /////////my////////////////////////////////////////////
  //RTC_GetFullTime (LPC_RTC, &fd2930_time); //��������� ������� �����
}

void AppRTCTaskGetTime(void)
{
	/////////my////////////////////////////////////////////
  //RTC_GetFullTime (LPC_RTC, &fd2930_time);
}




/*****************************************************************************
** Function name:		rtc_cal_config
**
** Descriptions:		Function calculates the required calibration value and
**						direction values for the calibration register.
**						If the actual_rtc_frequency is within 
**						+/- (32.768kHz / (2^17 - 1)) == +/- 0.250 Hz, then disable
**						the auto calibration due to RTCAL overflow.
**						Else, save the proper LPC_RTC calibration value and direction.
**
** parameters:			The actual LPC_RTC oscillating frequency should be passed
**						in mHz (milli Hz).
** 						
** Returned value:		None
**
*******************************************************************************/
/* This code is from NXP's example-code for application note AN10849.
 * Copyright(C) 2008(?), NXP Semiconductor, 2009(?).05.28 version 1.00 preliminary version"
 * For further information please read the AppNote available from nxp.com.
 * Adapted for the NXP driver-library and this project by Martin Thomas 7/2010 */

void rtc_cal_config(uint32_t actual_rtc_frequency)
{
	uint32_t calibration_value;
	int32_t rtc_frequency_delta;

	rtc_frequency_delta = (RTC_IDEAL_FREQ - actual_rtc_frequency);

	RTC_CalibCounterCmd(LPC_RTC, DISABLE);

	/* Check for valid LPC_RTC frequency */
	/* Disable calibration if the correction value is +/- .250Hz offset
	 It would otherwise cause an overflow in RTCCAL */
	if ((rtc_frequency_delta <= MAX_DELTA_FREQUENCY_VALUE)
	    && (rtc_frequency_delta >= -MAX_DELTA_FREQUENCY_VALUE)) {

		RTC_CalibCounterCmd(LPC_RTC, DISABLE);

		/* Determine the calibration direction CALDIR and CALVAL*/
	} else if (rtc_frequency_delta > MAX_DELTA_FREQUENCY_VALUE) {

		/* Forward direction */
		calibration_value = RTC_IDEAL_FREQ / (RTC_IDEAL_FREQ - actual_rtc_frequency);
		RTC_CalibConfig(LPC_RTC, calibration_value, RTC_CALIB_DIR_FORWARD);
		RTC_CalibCounterCmd(LPC_RTC, ENABLE);

	} else {

		/* Backward direction */
		calibration_value = RTC_IDEAL_FREQ / (actual_rtc_frequency - RTC_IDEAL_FREQ);
		RTC_CalibConfig(LPC_RTC, calibration_value, RTC_CALIB_DIR_BACKWARD);
		RTC_CalibCounterCmd(LPC_RTC, ENABLE);

	}

	return;
}

void rtc_cal_init(RTC_TIME_Type *pFullTime)
{
	RTC_Init(LPC_RTC);
	LPC_RTC->RTC_AUX = RTC_AUX_RTC_OSCF;
	rtc_cal_config(RTC_ACTUAL_FREQ);
	RTC_SetFullTime(LPC_RTC, pFullTime);
	RTC_Cmd(LPC_RTC, ENABLE);
}

void rtc_cal_settime(RTC_TIME_Type *pFullTime)
{
	RTC_Cmd(LPC_RTC, DISABLE);
	RTC_SetFullTime(LPC_RTC, pFullTime);
	RTC_Cmd(LPC_RTC, ENABLE);
}

uint8_t rtc_cal_gettime(RTC_TIME_Type *pFullTime)
{
	uint32_t ct0, ct1;

	// adapted from ChaN's FatFs LPC23xx example:
	do {
		ct0 = LPC_RTC->CTIME0;
		ct1 = LPC_RTC->CTIME1;
	} while (ct0 != LPC_RTC->CTIME0 || ct1 != LPC_RTC->CTIME1);

	pFullTime->SEC   = ct0 & 63;;
	pFullTime->MIN   = (ct0 >> 8) & 63;;
	pFullTime->HOUR  = (ct0 >> 16) & 31;
	pFullTime->DOM   = ct1 & 31;
	pFullTime->MONTH = (ct1 >> 8) & 15;;
	pFullTime->YEAR  = (ct1 >> 16) & 4095;

	return (LPC_RTC->RTC_AUX & RTC_AUX_RTC_OSCF) ? 1 : 0;
}


/**
  ******************************************************************************
  * @file    functional.c
  * @author  Yuri Martenstev <yurimartens@gmail.com>
  * @brief
  ******************************************************************************
  */
#include <fd2930.h>

#include <adc_al.h>


DeviceData_t	DeviceData;
Timer_t			IndicationTimer, MeasurmentTimer;



/**
  * @brief
  * @param
  * @retval
  */
void DeviceInit()
{
	TimerInit(&MeasurmentTimer, 0);
	TimerReset(&MeasurmentTimer, ADC_SAMPLING_PERIOD);
}

/**
  * @brief
  * @param
  * @retval
  */
void ADCTask()
{
	if (TimerIsOverflow(&MeasurmentTimer)) {
		TimerReset(&MeasurmentTimer, ADC_SAMPLING_PERIOD);
		DeviceData.Temperature = (ADCGetDataSingleChannel(1) * 1000 - 750) / 10 + 25;
		DeviceData.UVVoltage = (uint16_t)(ADCGetDataSingleChannel(2) * 1000 * UV_VOLTAGE_SCALE);
	}
}

    
//------------------------------------------------------------------------------
// end
//------------------------------------------------------------------------------



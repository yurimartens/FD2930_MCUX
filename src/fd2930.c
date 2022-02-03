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

RTC_TIME_Type 	DeviceTime;
Timer_t			IndicationTimer, MeasurmentTimer;

uint16_t			CntTemperatureFault;



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
		DeviceData.Temperature = (ADCGetDataSingleChannel(1) - 750) / 10 + 25;
		DeviceData.UVVoltage = (uint16_t)(ADCGetDataSingleChannel(2) * UV_VOLTAGE_SCALE);
		DeviceData.InPowerVoltage = 24;

		if (DeviceData.Temperature > TEMPERATURE_MAXIMUM || DeviceData.Temperature < TEMPERATURE_MINIMUM) {
		    if (CntTemperatureFault < TEMPERATURE_FAULT_DELAY) {CntTemperatureFault++;};
		    if (CntTemperatureFault >= TEMPERATURE_FAULT_DELAY) {
		    	if (!(DeviceData.Flags & FD2930_DEVICEFLAGS_ERROR_TEMPERATURE)) {;}//push_flash_command(FLASH_WRITE_EVENT, EVENT_T_FAULT, MBS.buffer);
		    	DeviceData.Flags |= FD2930_DEVICEFLAGS_ERROR_TEMPERATURE;
		    }
		} else {
		    if ((DeviceData.Flags & FD2930_DEVICEFLAGS_ERROR_TEMPERATURE)) {;}//push_flash_command(FLASH_WRITE_EVENT, EVENT_T_NORMAL, MBS.buffer);
		    DeviceData.Flags &= ~FD2930_DEVICEFLAGS_ERROR_TEMPERATURE;
		    CntTemperatureFault = 0;
		}
		if (DeviceData.UVVoltage > VOLTAGE_UV_WORKING_MAXIMUM || DeviceData.UVVoltage < VOLTAGE_UV_WORKING_MINIMUM) {
		    if (!(DeviceData.Flags & FD2930_DEVICEFLAGS_ERROR_UV_VOLTAGE)) {;}//push_flash_command(FLASH_WRITE_EVENT, EVENT_HV_FAULT, MBS.buffer);
		    DeviceData.Flags |= FD2930_DEVICEFLAGS_ERROR_UV_VOLTAGE;
		} else {
		    if ((DeviceData.Flags & FD2930_DEVICEFLAGS_ERROR_UV_VOLTAGE)) {;}//push_flash_command(FLASH_WRITE_EVENT, EVENT_HV_NORMAL, MBS.buffer);
		    DeviceData.Flags &= ~FD2930_DEVICEFLAGS_ERROR_UV_VOLTAGE;
		}
	}
}


/**
  * @brief
  * @param
  * @retval
  */
void FunctionalTask()
{
	if (DeviceData.StateFlags & FD2930_STATE_FLAG_UPDATE_CURRENT) {
		//Set420(DeviceData.current);
		DeviceData.StateFlags &= ~FD2930_STATE_FLAG_UPDATE_CURRENT;
    }
    if (DeviceData.StateFlags & FD2930_STATE_FLAG_FFT_START) {
    	DeviceData.StateFlags &= ~FD2930_STATE_FLAG_FFT_START;
    	DeviceData.StateFlags |= FD2930_STATE_FLAG_FFT_ACTIVE;
    	//iTest_SinusoidInput();
    }
}

    
//------------------------------------------------------------------------------
// end
//------------------------------------------------------------------------------



/**
  ******************************************************************************
  * @file    functional.c
  * @author  Yuri Martenstev <yurimartens@gmail.com>
  * @brief
  ******************************************************************************
  */
#include <fd2930.h>

#include <adc_al.h>
#include <max11040.h>


DeviceData_t	DeviceData;
DeviceState_t	DeviceState;

RTC_TIME_Type 	DeviceTime;
Timer_t			IndicationTimer, MeasurmentTimer;

uint16_t		CntTemperatureFault;

uint16_t		UVNoise;



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
void FunctionalTaskBG()
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

/**
  * @brief
  * @param
  * @retval
  */
void FunctionalTaskPeriodic()
{
	SDADCTask();
	switch (DeviceState) {
		case FD2930_STATE_STARTING1:

		break;
		case FD2930_STATE_STARTING2:

		break;
	}
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


#define EWMA_FILTER(_PREV_, _CURR_, _COEFF_)				(_COEFF_ * _CURR_ + (1 - _COEFF_) * _PREV_)

/**
  * @brief
  * @param
  * @retval
  */
void SDADCTask(float avCoeffIR, float avCoeffUV)
{
	Max11040ChannelData_t *res = Max11040GetData(2);
	DeviceData.IRRaw = res->ch[0];
	DeviceData.UVRaw = res->ch[1];

	if (DeviceData.UVRaw < UVNoise / 10) {
		if (UVPickCnt < UV_PICK_LIMIT) UVPickCnt++;
		else UVPickCnt = 0;
	}

	if ((UVPickCnt > UV_PICK_WORK_AREA) || (UVPickCnt == 0) || (DeviceData.IRGain > IR_GAIN_UV_PICK)) {
		DeviceData.UVGain = EWMA_FILTER(DeviceData.UVGain, DeviceData.UVRaw, avCoeffUV);
	}

	if (DeviceData.IRRaw > DeviceData.IRAv) DeviceData.IRRec = DeviceData.IRRaw - DeviceData.IRAv;
	else DeviceData.IRRec = DeviceData.IRAv - DeviceData.IRRec;
	DeviceData.IRGain = EWMA_FILTER(DeviceData.IRGain, DeviceData.IRRaw, avCoeffIR);

	if (!(DeviceData.StateFlags & FD2930_STATE_FLAG_FFT_ACTIVE)) {

	}
}

    
//------------------------------------------------------------------------------
// end
//------------------------------------------------------------------------------



/**
  ******************************************************************************
  * @file    functional.c
  * @author  Yuri Martenstev <yurimartens@gmail.com>
  * @brief
  ******************************************************************************
  */
#include <fd2930.h>

#include <adc_al.h>
#include <pwm_al.h>
#include <max11040.h>
#include <eeprom.h>
#include <Modbus.h>
#include <ad5421.h>


DeviceData_t	DeviceData;
DeviceState_t	DeviceState;

uint16_t		DeviceStatusTemp;

RTC_TIME_Type 	DeviceTime;
Timer_t			MeasurmentTimer;

uint16_t		CntTemperatureFault;

uint16_t		UVNoise, UVNoiseTest, IRNoise, IRNoiseTest;
uint16_t		UVTestLevel, IRTestLevel, UVTestFaultCnt, IRTestFaultCnt;

uint16_t		IRThresF, UVThresF;

uint16_t		IRTroubleCnt, IRTroubleCntPiece, UVTroubleCnt, UVTroubleCntPiece;

uint16_t 		UVPickCnt;
uint16_t 		SelfTestCnt, SelfTest, BadSelfTest;

uint16_t 		CheckFireStatusCnt;
uint16_t 		AutorecoveryCnt;

uint16_t 		FFTCnt;

double 			IR, UV, IRRaw, UVRaw, IRAv, IRRect;

int16_t 		*FFTInputData  = (int16_t *)0x2007C000; /* AHB SRAM0 */
int16_t 		*FFTOutputData = (int16_t *)0x20080000; /* AHB SRAM1 */

uint16_t		CriminalFFTChannelNumber;

DeviceLEDState_t LEDState = FD2930_LED_YELLOW;

uint8_t 		IRDA_Command_Rcvd;

uint8_t			Protocol;
uint8_t 		ChangeConnectionSettings;

float 			HeatPowerInst = 1.0;

extern Modbus_t Modbus;
extern SSPAl_t  SSPSD420;


__STATIC_INLINE void CheckFireStatus();
__STATIC_INLINE void SetFireStatus();
__STATIC_INLINE void ResetFireStatus();
__STATIC_INLINE void SetDeviceStatus();
__STATIC_INLINE void SetLEDs();
__STATIC_INLINE void SetRelays();
__STATIC_INLINE void SetHeater();
__STATIC_INLINE void HandleLEDs();
__STATIC_INLINE void HandleRelayWork();
__STATIC_INLINE void HandleIRTestChannel();

void SetDefaultParameters();
void SetDefaultMBParameters();
void UpdateOutputs();


/**
  * @brief
  * @param
  * @retval
  */
void DeviceInit()
{
	uint8_t write = 0;
	EEPROMInit();

	AutorecoveryCnt = 0;
	DeviceData.Status = 0;
	DeviceData.FWVersion = FW_VERSION;
	DeviceData.HWVersion = HW_VERSION;
	DeviceData.DeviceType = DEVICE_TYPE;

	if (EEPROM_ERROR_EMPTY == EEPROMRead((uint8_t *)&DeviceData, EEPROM_PAGE_SIZE)) {
		SetDefaultParameters();
		AutorecoveryCnt = 1; //здесь же запустим счетчик автовосстановления
		write = 1;
	} else {
		if (DeviceData.Config == 0 || (DeviceData.MBId < 1 || DeviceData.MBId > 247) || \
			(DeviceData.HeatPower > FD2930_MAX_HEATPOWER || DeviceData.HeatPower < FD2930_MIN_HEATPOWER) || \
			(DeviceData.Baudrate != 1 && DeviceData.Baudrate != 2 && DeviceData.Baudrate != 4 && DeviceData.Baudrate != 8 && DeviceData.Baudrate != 12 && DeviceData.Baudrate != 16 && DeviceData.Baudrate != 24)) {
			SetDefaultParameters();
			write = 1;
		}
		if (Protocol == PROTOCOL_IPES) {
			if (DeviceData.Baudrate != 1 && DeviceData.Baudrate != 2 && DeviceData.Baudrate != 4 && DeviceData.Baudrate != 8 && DeviceData.Baudrate != 16) {
				DeviceData.Baudrate = IPES_DEF_MBS_BAUD;
				write = 1;
			}
		} else {
			if (DeviceData.Baudrate != 1 && DeviceData.Baudrate != 2 && DeviceData.Baudrate != 4 && DeviceData.Baudrate != 12 && DeviceData.Baudrate != 24) {
				DeviceData.Baudrate = FD2930_DEF_MBS_BAUD;
				write = 1;
			}
		}
		if (DeviceData.FFTThres > FD2930_MAX_GAIN_FFT || DeviceData.FFTThres < FD2930_MIN_GAIN_FFT) {DeviceData.FFTThres = FD2930_DEFAULT_GAIN_FFT; write = 1;}
		if (DeviceData.IRThres > FD2930_MAX_THRES_IR || DeviceData.IRThres < FD2930_MIN_THRES_IR) {DeviceData.IRThres = FD2930_DEFAULT_THRES_IR; write = 1;}
		if (DeviceData.UVThres > FD2930_MAX_THRES_UV || DeviceData.UVThres < FD2930_MIN_THRES_UV) {DeviceData.UVThres = FD2930_DEFAULT_THRES_UV; write = 1;}
		if (DeviceData.IRCoeff > FD2930_MAX_K_IR || DeviceData.IRCoeff < FD2930_MIN_K_IR) {DeviceData.IRCoeff = FD2930_DEFAULT_K_IR; write = 1;}
		if (DeviceData.UVCoeff > FD2930_MAX_K_UV || DeviceData.UVCoeff < FD2930_MIN_K_UV) {DeviceData.UVCoeff = FD2930_DEFAULT_K_UV; write = 1;}
		if (DeviceData.FaultDelay > FD2930_MAX_WAIT_FAULT || DeviceData.FaultDelay < FD2930_MIN_WAIT_FAULT) {DeviceData.FaultDelay = FD2930_DEFAULT_WAIT_FAULT; write = 1;}
		if (DeviceData.FireDelay > FD2930_MAX_WAIT_FIRE || DeviceData.FireDelay < FD2930_MIN_WAIT_FIRE) {DeviceData.FireDelay = FD2930_DEFAULT_WAIT_FIRE; write = 1;}
	}

	if (write) EEPROMWrite((uint8_t *)&DeviceData, EEPROM_PAGE_SIZE);

	DeviceData.Status |= (FD2930_DEVICE_STATUS_CRC_OK | FD2930_DEVICE_STATUS_FLASH_OK);// | FD2930_DEVICE_STATUS_TESTING);

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
		AD5421SetCurrent(DeviceData.Current420);
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
	static uint32_t cnt = 0, cnt24 = 0, postFireCnt = 0;
	static uint64_t selfTestCnt = 0;
	double avCoeffIR = 0.05, avCoeffUV = 0.05;

	HandleLEDs();
	HandleRelayWork();
	HandleIRTestChannel();

	if (CheckFireStatusCnt < TIME_mS_TO_TICK(DELAY_CHECK_FIRE_STATUS)) {
		CheckFireStatusCnt++;
		GPIO_SET_PIN(LPC_GPIO2, IR_TEST);
		GPIO_SET_PIN(LPC_GPIO2, UV_TEST);
		GPIO_SET_PIN(LPC_GPIO0, UV_TEST2);
		avCoeffIR = 0.05, avCoeffUV = 0.05;
	} else {
		avCoeffIR = 0.0014, avCoeffUV = 0.001;
	}
	if (DeviceState & FD2930_STATE_SELFTEST) {
		avCoeffIR = 0.002, avCoeffUV = 0.002;
	}
	SDADCTask(avCoeffIR, avCoeffUV);

	switch (DeviceState) {
		case FD2930_STATE_START1:
			if (cnt < TIME_mS_TO_TICK(START_DELAY)) {
				cnt++;
				if ((!GPIO_READ_PIN(LPC_GPIO4, HALL1)) || (!GPIO_READ_PIN(LPC_GPIO4, HALL2))) {
					cnt = 0;
					SetDefaultMBParameters();
					DeviceState++;
				}
			} else {
				cnt = 0;
				DeviceState++;
			}
		break;
		case FD2930_STATE_START2:
			if (cnt < TIME_mS_TO_TICK(AFTER_START_DELAY)) {
				cnt++;
			} else {
				cnt = 0;
				AD5421Init(&SSPSD420, 0);
				DeviceData.Current420 = FD2930_TASK_STARTUP_CUR_UA;
				AD5421SetCurrent(FD2930_TASK_STARTUP_CUR_UA);
				DeviceState++;
			}
		break;
		case FD2930_STATE_START3:
			if (cnt < (TIME_mS_TO_TICK(DELAY_10S) + TIME_mS_TO_TICK(DELAY_1S * (DeviceData.MBId % 16)))) {
				cnt++;
			} else {
				cnt = 0;
				if ((DeviceData.Status & FD2930_DEVICE_STATUS_IR_UV_SET) && (DeviceData.Status & FD2930_DEVICE_STATUS_TEST_CALIBR) && (DeviceData.Status & FD2930_DEVICE_STATUS_TEST_ZERO)) {
					if (DeviceData.Config & FD2930_DEVICECONFIG_SELFTEST_ALLOWED) {
						DeviceState = FD2930_STATE_SELFTEST;
						SelfTest++;
						GPIO_RESET_PIN(LPC_GPIO2, (UV_TEST));
						GPIO_RESET_PIN(LPC_GPIO0, (UV_TEST2));
					} else {
						DeviceState = FD2930_STATE_WORKING;
						////push_flash_command(FLASH_WRITE_EVENT, EVENT_NORMAL_EVENT, MBS.buffer);
					}
				} else {
					DeviceState = FD2930_STATE_BREAK;
				}
				UpdateOutputs();
			}
		break;
		case FD2930_STATE_SELFTEST:
			DeviceData.Status |= FD2930_DEVICE_STATUS_SELF_TEST;
			if (SelfTestCnt < TIME_mS_TO_TICK(DELAY_5S)) {
				SelfTestCnt++; // время самотестирования //на 5-ой секунде выносим вердикт, выходим в деж режим
			} else {
				SelfTestCnt = 0;
		        switch ((DeviceFireConfig_t)(DeviceData.Config & 0x000F)) {
			        case FD2930_CONFIG_1:
			        case FD2930_CONFIG_2:
			        case FD2930_CONFIG_3:
			        case FD2930_CONFIG_4:
			        	if ((DeviceData.UVGain - UVNoiseTest) < 0.7 * UVTestLevel) UVTestFaultCnt++;
			        	if ((DeviceData.IRGain - IRNoiseTest) < 0.7 * IRTestLevel) IRTestFaultCnt++;

			        	if (((DeviceData.IRGain - IRNoiseTest) < 0.5 * IRTestLevel) || ((DeviceData.UVGain - UVNoiseTest) < 3000)) { // если чувствительность ИК меньше 40% и УФ меньше 40% то ставим флаг аварийной запыленности запыленности
			        		if ((DeviceData.Temperature > 0) && (DeviceData.Temperature < 70) && (HeatPowerInst < 20)) {// если температура не выходит за рамки и подогрев не в разогреве то  ставим флаг // просто если идет резкий разогрев, то ИК сенсор тупит и теряет чувствительность из за чего тест не проходит
			        			if (SelfTest < 14) {//если не прошло 14 тестов - 2 часа с включения
			        				DeviceData.Flags |= FD2930_DEVICEFLAGS_BREAK_DUST;
			        				DeviceData.Flags |= FD2930_DEVICEFLAGS_DUST;// этот флаг тоже поставим , потому что если уж не прошли очень плохие тесты, то плохие тоже не прошли. что бы выключалось реле неисправность
			        			} else { //прошло 14 тестов - 2 часа с включения
			        				if (BadSelfTest >= 3) {//реагируем на три плохих теста подряд
			        					DeviceData.Flags |= FD2930_DEVICEFLAGS_BREAK_DUST;
			        					DeviceData.Flags |= FD2930_DEVICEFLAGS_DUST;// этот флаг тоже поставим , потому что если уж не прошли очень плохие тесты, то плохие тоже не прошли. что бы выключалось реле неисправность
			        				}
			        			}
			        		}
			        	}
			        	if (((DeviceData.IRGain - IRNoiseTest) < 0.7 * IRTestLevel) || ((DeviceData.UVGain - UVNoiseTest) < 3000)) { // если чувствительность ИК меньше 70% и УФ меньше 70% и  && (DeviceData.temperature < 60) то ставим флаг запыленности
			        		if ((DeviceData.Temperature > 0) && (DeviceData.Temperature < 70) && (HeatPowerInst < 20)) { // если температура не выходит за рамки и подогрев не в разогреве то  ставим флаг // просто если идет резкий разогрев, то ИК сенсор тупит и теряет чувствительность из за чего тест не проходит
			        			if (BadSelfTest < 10) BadSelfTest++;
			        			if (SelfTest < 14) { //если не прошло 14 тестов - 2 часа с включения
			        				DeviceData.Flags |= FD2930_DEVICEFLAGS_DUST;
			        				DeviceData.Status |= FD2930_DEVICEFLAGS_DUST_DUBL;//дублируем запыленность в регистре состояния
			        			} else { //прошло 14 тестов - 2 часа с включения
			        				if (BadSelfTest >= 3) { //реагируем на три плохих теста подряд
			        					DeviceData.Flags |= FD2930_DEVICEFLAGS_DUST;
			        					DeviceData.Status |= FD2930_DEVICEFLAGS_DUST_DUBL;//дублируем запыленность в регистре состояния
			        				}
			        			}
			        		}
			            }
			        	if (((DeviceData.IRGain - IRNoiseTest) >= 0.7 * IRTestLevel) && ((DeviceData.UVGain - UVNoiseTest) >= 0.5 * UVTestLevel)) { // если чувствительность ИК больше 40% и УФ больше 40% в то снимаем флаги
			        		BadSelfTest = 0;
			        		DeviceData.Flags &= ~FD2930_DEVICEFLAGS_DUST;
			        		DeviceData.Status &= ~FD2930_DEVICEFLAGS_DUST_DUBL;//дублируем запыленность в регистре состояния
			        		DeviceData.Flags &= ~FD2930_DEVICEFLAGS_BREAK_DUST;
			        	}
			       break;
			       case FD2930_CONFIG_5:// в одноканальных ИК режимах не проверяем УФ канал
			       case FD2930_CONFIG_6:// в одноканальных ИК режимах не проверяем УФ канал
			    	   if (((DeviceData.IRGain - IRNoiseTest) < 0.5 * IRTestLevel)) { // если чувствительность ИК меньше 40% то ставим флаг аварийной запыленности запыленности
			    		   if ((DeviceData.Temperature > 0) && (DeviceData.Temperature < 50) && (HeatPowerInst < 20)) { // если температура не выходит за рамки и подогрев не в разогреве то  ставим флаг // просто если идет резкий разогрев, то ИК сенсор тупит и теряет чувствительность из за чего тест не проходит
			    			   DeviceData.Flags |= FD2930_DEVICEFLAGS_BREAK_DUST;
			    			   DeviceData.Flags |= FD2930_DEVICEFLAGS_DUST;// этот флаг тоже поставим , потому что если уж не прошли очень плохие тесты, то плохие тоже не прошли. что бы выключалось реле неисправность
			    		   }
			    	   }
			    	   if (((DeviceData.IRGain - IRNoiseTest) < 0.7 * IRTestLevel)) { // если чувствительность ИК меньше 70% и  && (DeviceData.temperature < 60) то ставим флаг запыленности
			    		   if ((DeviceData.Temperature > 0) && (DeviceData.Temperature < 50) && (HeatPowerInst < 20)) { //дублируем запыленность в регистре состояния// если температура не выходит за рамки и подогрев не в разогреве то  ставим флаг
			    			   DeviceData.Flags |= FD2930_DEVICEFLAGS_DUST;
			    			   DeviceData.Status |= FD2930_DEVICEFLAGS_DUST_DUBL;
			    		   }                                  // просто если идет резкий разогрев, то ИК сенсор тупит и теряет чувствительность из за чего тест не проходит
			    	   } else {
			    		   DeviceData.Flags &= ~FD2930_DEVICEFLAGS_DUST;
			    		   DeviceData.Status &= ~FD2930_DEVICEFLAGS_DUST_DUBL;//дублируем запыленность в регистре состояния
			    		   DeviceData.Flags &= ~FD2930_DEVICEFLAGS_BREAK_DUST;
			    	   }
			       break;
			       case FD2930_CONFIG_7:// в одноканальном УФ режиме не проверяем ИК канал
			    	   if ((DeviceData.UVGain - UVNoiseTest) < 3000) { // если чувствительность УФ меньше 40% то ставим флаг аварийной запыленности запыленности
			    		   if ((DeviceData.Temperature > 0) && (DeviceData.Temperature < 50) && (HeatPowerInst < 20)) { // если температура не выходит за рамки и подогрев не в разогреве то  ставим флаг // просто если идет резкий разогрев, то ИК сенсор тупит и теряет чувствительность из за чего тест не проходит
			    			   DeviceData.Flags |= FD2930_DEVICEFLAGS_BREAK_DUST;
			    			   DeviceData.Flags |= FD2930_DEVICEFLAGS_DUST;// этот флаг тоже поставим , потому что если уж не прошли очень плохие тесты, то плохие тоже не прошли. что бы выключалось реле неисправность
			    		   }
			    	   }
			    	   if ((DeviceData.UVGain - UVNoiseTest) < 0.7 * UVTestLevel) { // если чувствительность УФ меньше 70% и  && (DeviceData.temperature < 60) то ставим флаг запыленности
			    		   if ((DeviceData.Temperature > 0) && (DeviceData.Temperature < 50) && (HeatPowerInst < 20)) {
			    			   DeviceData.Flags |= FD2930_DEVICEFLAGS_DUST;
			    			   DeviceData.Status |= FD2930_DEVICEFLAGS_DUST_DUBL;
			    		   }//дублируем запыленность в регистре состояния// если температура не выходит за рамки и подогрев не в разогреве то  ставим флаг
			    		   // просто если идет резкий разогрев, то ИК сенсор тупит и теряет чувствительность из за чего тест не проходит
			    	   } else { // если чувствительность УФ больше 40% в то снимаем флаги
			    		   DeviceData.Flags &= ~FD2930_DEVICEFLAGS_DUST;
			    		   DeviceData.Status &= ~FD2930_DEVICEFLAGS_DUST_DUBL;//дублируем запыленность в регистре состояния
			    		   DeviceData.Flags &= ~FD2930_DEVICEFLAGS_BREAK_DUST;
			    	   }
			       break;
			       case FD2930_CONFIG_8:// в режиме КПГ ничего не проверяем, снимаем флаги
			    	   DeviceData.Flags &= ~FD2930_DEVICEFLAGS_DUST;
			    	   DeviceData.Status &= ~FD2930_DEVICEFLAGS_DUST_DUBL;//дублируем запыленность в регистре состояния
			    	   DeviceData.Flags &= ~FD2930_DEVICEFLAGS_BREAK_DUST;
			       break;
		        }
		        GPIO_SET_PIN(LPC_GPIO2, IR_TEST);
		        GPIO_SET_PIN(LPC_GPIO2, UV_TEST);
		        GPIO_SET_PIN(LPC_GPIO0, UV_TEST2);
		        DeviceState = FD2930_STATE_WORKING;
		        DeviceData.Status &= ~FD2930_DEVICE_STATUS_SELF_TEST;
		        ////push_flash_command(FLASH_WRITE_EVENT, EVENT_NORMAL_EVENT, MBS.buffer);
		        CheckFireStatusCnt = 0; //счетчик для разрешения анализа пожара, после процессов калибровки и самотестирования
			}
		break;
		case FD2930_STATE_WORKING:
		    if (CheckFireStatusCnt >= TIME_mS_TO_TICK(DELAY_CHECK_FIRE_STATUS)) CheckFireStatus();          //анализ данных каналов, спустя задержку после процессов самотестирования и калибровки
		    SetFireStatus();

		    if (cnt < TIME_mS_TO_TICK(DELAY_1S)) {
		    	cnt++;
		    } else {
		    	cnt = 0;
		        //push_live_data();
		    	if (DeviceData.Status & FD2930_DEVICE_STATUS_FIRE) postFireCnt = 10;// счетчик мертвого времени после сброса пожара, что бы плавающие пороги не подпрыгивали от остаточных сигналов в ИК и УФ
		    	else if (postFireCnt > 0) postFireCnt--;

		    	if (DeviceData.IRGain > (0.5 * DeviceData.IRThres) && DeviceData.UVGain > (0.5 * DeviceData.UVThres)) selfTestCnt = 0;

		    	if ((DeviceData.Config & 0x0F) <= 4) { // , только для двухканальных режимов
		    		if ((DeviceData.UVGain < 150) && (postFireCnt == 0) && (UVPickCnt < 1)) {// если на уф канале все спокойно то вычисляем уровень шума в ИК и прибавляем часть его к установленному порогу
		    			IRNoise = IRNoise + ((DeviceData.IRGain * 100) - (IRNoise * 100)) / 1000;// усредненное значение шума для плавающего порога
		    			IRThresF = DeviceData.IRThres + 0.7 * IRNoise;
		    		}
		    		if (IRThresF > 1000) IRThresF = 1000;
		    		if (postFireCnt > 0) {
		    			IRNoise = 0;
		    			IRThresF = DeviceData.IRThres;
		    		}
		    	} else {
		    		IRNoise = 0;
		    		IRThresF = DeviceData.IRThres;
		    	}
		    	if (DeviceData.IRGain > (0.7 * DeviceData.IRThres)) {
		    		if (DeviceData.UVGain < 150 && postFireCnt == 0) {
		    			if (IRTroubleCnt < FD2930_W_TRL) IRTroubleCnt++; // 1 час
		    			if (IRTroubleCntPiece < FD2930_W_TRL_P) IRTroubleCntPiece++; // 3 часов
		    			if (((DeviceData.Config & 0xf) == 5) || ((DeviceData.Config & 0xf) == 7) || ((DeviceData.Config & 0xf) == 8)) {
		    				IRTroubleCnt = 0;
		    				IRTroubleCntPiece = 0;
		    			} // в одноканальных ИК+FFT и УФ и КПГ режимах нет смысла проверять помеху по ИК
		    		}
		        } else {
		        	IRTroubleCnt = 0;
		        }
		    	if (IRTroubleCnt == FD2930_W_TRL || IRTroubleCntPiece == FD2930_W_TRL_P) DeviceData.Flags |= FD2930_DEVICEFLAGS_IR_ERROR; //3600 10800

		    	if ((DeviceData.Config & 0xf) <= 4) { // , только для двухканальных режимов
		    		if (DeviceData.IRGain < 150 && postFireCnt == 0) { // если на ИК канале все спокойно то вычисляем уровень шума в УФ и прибавляем часть его к установленному порогу,
		    			UVNoise = UVNoise + ((DeviceData.UVGain * 100) - (UVNoise * 100)) / 1000;// усредненное значение шума для плавающего порога
		    			UVThresF = DeviceData.UVThres + 0.7 * UVNoise;
		    		}
		    		if (UVThresF > 1000) UVThresF = 1000;
		    		if (postFireCnt > 0) {
		    			UVNoise = 0;
		    			UVThresF = DeviceData.UVThres;
		    		}
		    	} else {
		    		UVNoise = 0;
		    		UVThresF = DeviceData.UVThres;
		    	}

		    	if (DeviceData.UVGain > (0.7 * DeviceData.UVThres)) {
		    		if (DeviceData.IRGain < 150 && postFireCnt == 0) {
		    			if (UVTroubleCnt < FD2930_W_TRL) UVTroubleCnt++;;// 1 час
		    			if (UVTroubleCntPiece < FD2930_W_TRL_P) UVTroubleCntPiece++;;// 3 часов
		    			if (((DeviceData.Config & 0xf) == 5) || ((DeviceData.Config & 0xf) == 6) || ((DeviceData.Config & 0xf) == 8)) {
		    				UVTroubleCnt = 0;
		    				UVTroubleCntPiece = 0;
		    			}// в одноканальных ИК режимах и КПГ нет смысла проверять помеху по УФ
		    		}
		        } else {
		        	UVTroubleCnt = 0;
		        }
		        if (UVTroubleCnt == FD2930_W_TRL || UVTroubleCntPiece == FD2930_W_TRL_P) DeviceData.Flags |= FD2930_DEVICEFLAGS_UV_ERROR;;//3600 10800

		        if (DeviceData.IRGain < (0.7 * DeviceData.IRThres) && IRTroubleCntPiece < FD2930_W_TRL_P) {
		        	IRTroubleCnt = 0;
		        	DeviceData.Flags &= ~FD2930_DEVICEFLAGS_IR_ERROR;
		        }
		        if (DeviceData.UVGain < (0.7 * DeviceData.UVThres) && UVTroubleCntPiece < FD2930_W_TRL_P) {
		        	UVTroubleCnt = 0;
		        	DeviceData.Flags &= ~FD2930_DEVICEFLAGS_UV_ERROR;
		        }

		        RTC_GetFullTime(LPC_RTC, &DeviceTime);
		        UpdateOutputs();

		        if ((!GPIO_READ_PIN(LPC_GPIO4, HALL1)) || (!GPIO_READ_PIN(LPC_GPIO4, HALL2))) {
		        	if (!(DeviceData.Status & FD2930_DEVICE_STATUS_MAGNET)) {
		        		DeviceData.Status |= FD2930_DEVICE_STATUS_MAGNET;
		        		////push_flash_command(FLASH_WRITE_EVENT, EVENT_HALL_ON, MBS.buffer);
		        	}
		        	ResetFireStatus(); //сброс зафиксированного состояния пожар
		        } else {
		        	if (DeviceData.Status & FD2930_DEVICE_STATUS_MAGNET) {
		        		DeviceData.Status &= ~FD2930_DEVICE_STATUS_MAGNET;
		        		////push_flash_command(FLASH_WRITE_EVENT, EVENT_HALL_OFF, MBS.buffer);
		        	}
		    	}
		        GPIO_SET_PIN(LPC_GPIO2, (UV_TEST));
		        GPIO_SET_PIN(LPC_GPIO0, (UV_TEST2)); //turn off UV test source
		        GPIO_SET_PIN(LPC_GPIO2, IR_TEST);
		    }

		    if (DeviceData.Flags & FD2930_DEVICEFLAGS_DUST) { // если запыленность неаварийная тестируемся  чаще
		    	if (selfTestCnt < TIME_mS_TO_TICK((3 * 60 * DELAY_1S))) {
		    		selfTestCnt++;//яя// тестируемся каждые три минуты
		    	} else {
		    		selfTestCnt = 0;
		    		DeviceData.WorkedTime += 30;
		    		if (cnt24 < 48) {
		    			cnt24++;
		    		} else {
						cnt24 = 0;
						//write_parameters();
		    		}
		    		if (DeviceData.Config & FD2930_DEVICECONFIG_SELFTEST_ALLOWED) {
		    			GPIO_RESET_PIN(LPC_GPIO2, (UV_TEST));
		    			GPIO_RESET_PIN(LPC_GPIO0, (UV_TEST2));
		    			DeviceState = FD2930_STATE_SELFTEST; //начинаем самотестирование
		    			DeviceData.Status |= FD2930_DEVICE_STATUS_SELF_TEST;
		    		}
		    	}
		    } else {// иначе в дежурном режиме когда все хорошо, тестируемся каждые 10 мин. Требует Транснефть
		    	if (selfTestCnt < TIME_mS_TO_TICK((10 * 60 * DELAY_1S))) {
		    		selfTestCnt++;//яя
		    	} else {
		    		selfTestCnt = 0;
		    		DeviceData.WorkedTime += 30;
		    		if (cnt24 < 48) {
		    			cnt24++;
		    		} else {
		    			cnt24 = 0;
		    			//write_parameters();
		    		}
		    		if (DeviceData.Config & FD2930_DEVICECONFIG_SELFTEST_ALLOWED) {
		    			GPIO_RESET_PIN(LPC_GPIO2, (UV_TEST));
		    			GPIO_RESET_PIN(LPC_GPIO0, (UV_TEST2));
		    			DeviceState = FD2930_STATE_SELFTEST; //начинаем самотестирование
		    			if (SelfTest < 14) SelfTest++;
		    			DeviceData.Status |= FD2930_DEVICE_STATUS_SELF_TEST;
		    		}
		    	}
		    }
		break;
		case FD2930_STATE_CHANNEL_CALIBR:
			if (SelfTestCnt < TIME_mS_TO_TICK(DELAY_CHANNEL_CALIB)) {
				SelfTestCnt++;
			} else {
				SelfTestCnt = 0;
		        DeviceState = FD2930_STATE_WORKING;
		        GPIO_SET_PIN(LPC_GPIO2, IR_TEST);
		    }
		break;
		case FD2930_STATE_TEST_ZERO:
		    if (SelfTestCnt < TIME_mS_TO_TICK(DELAY_05S)) {
		    	SelfTestCnt++;
		    } else {
		    	SelfTestCnt = 0;
		    	IRNoiseTest = 100;//DeviceData.IRGain;
		    	UVNoiseTest = 200;//DeviceData.UVGain;
		    	if (!(DeviceData.Status & FD2930_DEVICE_STATUS_TEST_ZERO)) {
		    		DeviceData.Status |= FD2930_DEVICE_STATUS_TEST_ZERO;
		    		////push_flash_command(FLASH_WRITE_EVENT, EVENT_SET_ZERO, MBS.buffer);
		    	}
		    	//write_parameters();
		    	DeviceState = FD2930_STATE_BREAK;
		    	GPIO_SET_PIN(LPC_GPIO2, (UV_TEST));
		    	GPIO_SET_PIN(LPC_GPIO0, (UV_TEST2)); //turn off UV test source
		    	GPIO_SET_PIN(LPC_GPIO2, IR_TEST);
		    }
		break;
		case FD2930_STATE_TEST_CALIBR:
			if (SelfTestCnt < TIME_mS_TO_TICK(DELAY_5S)) {
				SelfTestCnt++;
			} else {
				SelfTestCnt = 0;
				IRTestLevel = DeviceData.IRGain - IRNoiseTest;
				UVTestLevel = DeviceData.UVGain - UVNoiseTest;
				if (!(DeviceData.Status & FD2930_DEVICE_STATUS_TEST_CALIBR)) {
					DeviceData.Status |= FD2930_DEVICE_STATUS_TEST_CALIBR;
					////push_flash_command(FLASH_WRITE_EVENT, EVENT_CALIBR1, MBS.buffer);
				}
				//write_parameters();
				DeviceState = FD2930_STATE_WORKING;
				////push_flash_command(FLASH_WRITE_EVENT, EVENT_NORMAL_EVENT, MBS.buffer);
				GPIO_SET_PIN(LPC_GPIO2, (UV_TEST));
				GPIO_SET_PIN(LPC_GPIO0, (UV_TEST2)); //turn off UV test source
				GPIO_SET_PIN(LPC_GPIO2, IR_TEST);
				DeviceData.IRGain = 0;
				IR = 0;
				DeviceData.IRAv = 1250;
				DeviceData.IRRaw = 1250;
				CheckFireStatusCnt = 0;
		    }
		break;
		case FD2930_STATE_TEST:
			if (cnt < TIME_mS_TO_TICK(DELAY_1S)) {
				cnt++;
			} else {
				cnt = 0;
				RTC_GetFullTime(LPC_RTC, &DeviceTime);
				UpdateOutputs();
			}
		    break;
		case FD2930_STATE_BREAK:
		    if (cnt < TIME_mS_TO_TICK(DELAY_1S)) {
		    	cnt++;
		    } else {
		    	cnt = 0;
		    	RTC_GetFullTime(LPC_RTC, &DeviceTime);
		    	if (AutorecoveryCnt > 0 && AutorecoveryCnt < 30) AutorecoveryCnt++;
		    	if (AutorecoveryCnt == 2) {
		    		if (!(DeviceData.Status & FD2930_DEVICE_STATUS_IR_UV_SET)) {
		    			////push_flash_command(FLASH_WRITE_EVENT, EVENT_CALIBR2, MBS.buffer);
		    			DeviceData.Status |= FD2930_DEVICE_STATUS_IR_UV_SET;
		    		}
		    		//write_parameters();
		    	}
		    	if (AutorecoveryCnt == 10) {
		            DeviceState = FD2930_STATE_TEST_ZERO;
		            DeviceData.Status &= ~FD2930_DEVICE_STATUS_TEST_ZERO;
		            DeviceData.Status &= ~FD2930_DEVICE_STATUS_TEST_CALIBR;
		            GPIO_RESET_PIN(LPC_GPIO2, (UV_TEST));
		            GPIO_RESET_PIN(LPC_GPIO0, (UV_TEST2));
		            //write_parameters();
		    	}
		    	if (AutorecoveryCnt == 20) {
		    		DeviceState = FD2930_STATE_TEST_CALIBR;
		            DeviceData.Status &= ~FD2930_DEVICE_STATUS_TEST_CALIBR;
		            GPIO_RESET_PIN(LPC_GPIO2, (UV_TEST));
		            GPIO_RESET_PIN(LPC_GPIO0, (UV_TEST2));
		            //write_parameters();
		            //AutorecoveryCnt=0; ?? было так
		    	}
		    	UpdateOutputs();
		    }
		    if (selfTestCnt < TIME_mS_TO_TICK(3 * 60 * DELAY_1S)) {
		    	selfTestCnt++;
		    } else {
		    	selfTestCnt = 0;
		    	if ((DeviceData.Config & FD2930_DEVICECONFIG_SELFTEST_ALLOWED) && (DeviceData.Status & FD2930_DEVICE_STATUS_TEST_CALIBR)) {
		    		GPIO_RESET_PIN(LPC_GPIO2, (UV_TEST));
		    		GPIO_RESET_PIN(LPC_GPIO0, (UV_TEST2));
		    		DeviceState = FD2930_STATE_SELFTEST;
		    		DeviceData.Status |= FD2930_DEVICE_STATUS_SELF_TEST;
		    	}
		    }
		break;
	}
}

/**
  * @brief
  * @param
  * @retval
  */
void UpdateOutputs()
{
	DeviceData.StateFlags |= FD2930_STATE_FLAG_UPDATE_CURRENT;
	SetDeviceStatus();
	SetLEDs();
	SetRelays();
	SetHeater();
}

/**
  * @brief
  * @param
  * @retval
  */
__STATIC_INLINE void CheckFireStatus()
{
	const uint32_t channel_fire_delay = 3000;//750;//300;
	const uint32_t between_channel_fire_delay = 23000;//5750;//2300;
	static uint32_t between_channel_fire_counter = 0, between_channel_prefire_counter = 0;
	uint8_t flag_IR_prefire = 0, flag_UV_prefire = 0, flag_UV_inv = 0;
	static uint8_t flag_IR_fire_2ch = 0, flag_UV_fire_2ch = 0, flag_IR_prefire_2ch = 0, flag_UV_prefire_2ch = 0;
	static uint32_t counter_IR_fire = 0, counter_UV_fire = 0;
	static uint32_t counter_IR_prefire = 0, counter_UV_prefire = 0;
	static uint32_t counter_UV_inv = 0;

	if (DeviceData.Config & FD2930_DEVICECONFIG_LOW_SENS) {
		if (DeviceData.IRGain > 1.7 * DeviceData.IRThres) {
			flag_IR_prefire = 1;
			if (!(DeviceData.Status & FD2930_DEVICE_STATUS_ALARM_IR)) {
				//push_flash_command(FLASH_WRITE_EVENT, EVENT_IR_HIGH, MBS.buffer);
				DeviceData.Status |= FD2930_DEVICE_STATUS_ALARM_IR;
			}
		} else {
			if (DeviceData.IRGain > 1.7 * 0.7 * DeviceData.IRThres) {
				flag_IR_prefire = 1;
				if (DeviceData.Status & FD2930_DEVICE_STATUS_ALARM_IR) {
					//push_flash_command(FLASH_WRITE_EVENT, EVENT_IR_NORMAL, MBS.buffer);
					DeviceData.Status &= ~FD2930_DEVICE_STATUS_ALARM_IR;
				}
			} else {
				flag_IR_prefire = 0;
				flag_IR_fire_2ch = 0;
				if (DeviceData.Status & FD2930_DEVICE_STATUS_ALARM_IR) {
					//push_flash_command(FLASH_WRITE_EVENT, EVENT_IR_NORMAL, MBS.buffer);
					DeviceData.Status &= ~FD2930_DEVICE_STATUS_ALARM_IR;
				}
			}
		}
		if (DeviceData.UVGain > 1.7 * UVThresF) {
			flag_UV_prefire = 1;
			if (!(DeviceData.Status & FD2930_DEVICE_STATUS_ALARM_UV)) {
				DeviceData.Status |= FD2930_DEVICE_STATUS_ALARM_UV;
				//push_flash_command(FLASH_WRITE_EVENT, EVENT_UV_HIGH, MBS.buffer);
			}
		} else {
			if (DeviceData.UVGain > 1.7 * 0.7 * UVThresF) {
				flag_UV_prefire = 1;
				if (DeviceData.Status & FD2930_DEVICE_STATUS_ALARM_UV) {
					DeviceData.Status &= ~FD2930_DEVICE_STATUS_ALARM_UV;
					//push_flash_command(FLASH_WRITE_EVENT, EVENT_UV_NORMAL, MBS.buffer);
				}
			} else {
				flag_UV_prefire = 0;
				flag_UV_fire_2ch = 0;
				if (DeviceData.Status & FD2930_DEVICE_STATUS_ALARM_UV) {
					DeviceData.Status &= ~FD2930_DEVICE_STATUS_ALARM_UV;
					//push_flash_command(FLASH_WRITE_EVENT, EVENT_UV_NORMAL, MBS.buffer);
				}
			}
		}
	} else {
		if (DeviceData.IRGain > IRThresF) {
			flag_IR_prefire = 1;
			if (!(DeviceData.Status & FD2930_DEVICE_STATUS_ALARM_IR)) {
				//push_flash_command(FLASH_WRITE_EVENT, EVENT_IR_HIGH, MBS.buffer);
				DeviceData.Status |= FD2930_DEVICE_STATUS_ALARM_IR;
			}
		} else {
			if (DeviceData.IRGain > 0.7 * IRThresF) {
				flag_IR_prefire = 1;
				if (DeviceData.Status & FD2930_DEVICE_STATUS_ALARM_IR) {
					//push_flash_command(FLASH_WRITE_EVENT, EVENT_IR_NORMAL, MBS.buffer);
					DeviceData.Status &= ~FD2930_DEVICE_STATUS_ALARM_IR;
				}
			} else {
				flag_IR_prefire = 0;
				flag_IR_fire_2ch = 0;
				if (DeviceData.Status & FD2930_DEVICE_STATUS_ALARM_IR) {
					//push_flash_command(FLASH_WRITE_EVENT, EVENT_IR_NORMAL, MBS.buffer);
					DeviceData.Status &= ~FD2930_DEVICE_STATUS_ALARM_IR;
				}
			}
		}
		if (DeviceData.UVGain > UVThresF) {
			flag_UV_prefire = 1;
			if (!(DeviceData.Status & FD2930_DEVICE_STATUS_ALARM_UV)) {
				DeviceData.Status |= FD2930_DEVICE_STATUS_ALARM_UV;
				//push_flash_command(FLASH_WRITE_EVENT, EVENT_UV_HIGH, MBS.buffer);
			}
		} else {
			if (DeviceData.UVGain > 0.7 * UVThresF) {
				flag_UV_prefire = 1;
				if (DeviceData.Status & FD2930_DEVICE_STATUS_ALARM_UV) {
					DeviceData.Status &= ~FD2930_DEVICE_STATUS_ALARM_UV;
					//push_flash_command(FLASH_WRITE_EVENT, EVENT_UV_NORMAL, MBS.buffer);
				}
			} else {
				flag_UV_prefire = 0;
				flag_UV_fire_2ch = 0;
				if (DeviceData.Status & FD2930_DEVICE_STATUS_ALARM_UV) {
					DeviceData.Status &= ~FD2930_DEVICE_STATUS_ALARM_UV;
					//push_flash_command(FLASH_WRITE_EVENT, EVENT_UV_NORMAL, MBS.buffer);
				}
			}
		}
	}

	if (DeviceData.UVGain < DeviceData.UVThres) flag_UV_inv = 1;
	else flag_UV_inv = 0;

	switch((DeviceFireConfig_t)(DeviceData.Config & 0xf)) {
		case FD2930_CONFIG_1:
			if (DeviceData.Status & FD2930_DEVICE_STATUS_ALARM_IR) {
				counter_IR_fire++;
				counter_IR_prefire++;
			} else {
				if (flag_IR_prefire) {
					counter_IR_fire = 0;
					flag_IR_fire_2ch = 0;
					counter_IR_prefire++;
				} else {
					counter_IR_fire = 0;
					flag_IR_fire_2ch = 0;
					flag_IR_prefire_2ch = 0;
					counter_IR_prefire = 0;
					between_channel_fire_counter = 0;
				}
			}
			if (DeviceData.Status & FD2930_DEVICE_STATUS_ALARM_UV) {
				counter_UV_fire++;
				counter_UV_prefire++;
			} else {
				if (flag_UV_prefire) {
					counter_UV_fire = 0;
					flag_UV_fire_2ch = 0;
					counter_UV_prefire++;
				} else {
					counter_UV_fire = 0;
					flag_UV_fire_2ch = 0;
					flag_UV_prefire_2ch = 0;
					counter_UV_prefire = 0;
					between_channel_fire_counter = 0;
				}
			}

			if (flag_IR_fire_2ch) {
				if (between_channel_fire_counter < channel_fire_delay) {
					if (counter_UV_fire >= channel_fire_delay) {
						counter_UV_fire = channel_fire_delay;
						if (CriminalFFTChannelNumber > FD2930_NUMBER_FFT_CRIM_CHANNEL) DeviceStatusTemp |= FD2930_DEVICE_STATUS_FIRE;
						else DeviceStatusTemp &= ~FD2930_DEVICE_STATUS_FIRE;
						between_channel_fire_counter = 0;
					} else {
						between_channel_fire_counter++;
					}
				} else {
					if (between_channel_fire_counter < between_channel_fire_delay) {
						between_channel_fire_counter++;
					} else {
						if (counter_UV_fire >= channel_fire_delay) {
							counter_UV_fire = channel_fire_delay;
							if (CriminalFFTChannelNumber > FD2930_NUMBER_FFT_CRIM_CHANNEL) DeviceStatusTemp |= FD2930_DEVICE_STATUS_FIRE;
							else DeviceStatusTemp &= ~FD2930_DEVICE_STATUS_FIRE;
						} else {
							between_channel_fire_counter = channel_fire_delay;
						}
					}
				}
			} else {
				if (flag_UV_fire_2ch) {
					if (between_channel_fire_counter < channel_fire_delay) {
						if (counter_IR_fire >= channel_fire_delay) {
							counter_IR_fire = channel_fire_delay;
							if (CriminalFFTChannelNumber > FD2930_NUMBER_FFT_CRIM_CHANNEL) DeviceStatusTemp |= FD2930_DEVICE_STATUS_FIRE;    //âêëþ÷àåì ïîæàð
							else DeviceStatusTemp &= ~FD2930_DEVICE_STATUS_FIRE;
							between_channel_fire_counter = 0;
						} else {
							between_channel_fire_counter++;
						}
					} else {
						if (between_channel_fire_counter < between_channel_fire_delay) {
							between_channel_fire_counter++;
						} else {
							if (counter_IR_fire >= channel_fire_delay) {
								counter_IR_fire = channel_fire_delay;
								if (CriminalFFTChannelNumber > FD2930_NUMBER_FFT_CRIM_CHANNEL) DeviceStatusTemp |= FD2930_DEVICE_STATUS_FIRE;    //âêëþ÷àåì ïîæàð
								else DeviceStatusTemp &= ~FD2930_DEVICE_STATUS_FIRE;
							} else {
								between_channel_fire_counter = channel_fire_delay;  //æäåì åùå 20 ñåêóíä
							}
						}
					}
				} else {
					if ((!counter_IR_fire && !counter_UV_fire) || (CriminalFFTChannelNumber < FD2930_NUMBER_FFT_CRIM_CHANNEL)) DeviceStatusTemp &= ~FD2930_DEVICE_STATUS_FIRE;
					if (counter_IR_fire >= channel_fire_delay) {
						counter_IR_fire = channel_fire_delay;
						flag_IR_fire_2ch = 1;
					}
					if (counter_UV_fire >= channel_fire_delay) {
						counter_UV_fire = channel_fire_delay;
						flag_UV_fire_2ch = 1;
					}
				}
			}

			if (flag_IR_prefire_2ch) {
				if (between_channel_prefire_counter < channel_fire_delay) {
					if (counter_UV_prefire >= channel_fire_delay) {
						counter_UV_prefire = channel_fire_delay;
						DeviceStatusTemp |= FD2930_DEVICE_STATUS_PREFIRE;
						between_channel_prefire_counter = 0;
					} else {
						between_channel_prefire_counter++;
					}
				} else {
					if (between_channel_prefire_counter < between_channel_fire_delay) {
						between_channel_prefire_counter++;
					} else {
						if (counter_UV_prefire >= channel_fire_delay) {
							counter_UV_prefire = channel_fire_delay;
							DeviceStatusTemp |= FD2930_DEVICE_STATUS_PREFIRE;
						} else {
							between_channel_prefire_counter = channel_fire_delay;
						}
					}
				}
			} else
				if (flag_UV_prefire_2ch) {
					if (between_channel_prefire_counter < channel_fire_delay) {
						if (counter_IR_prefire >= channel_fire_delay) {
							counter_IR_prefire = channel_fire_delay;
							DeviceStatusTemp |= FD2930_DEVICE_STATUS_PREFIRE;    //âêëþ÷àåì ïîæàð
							between_channel_prefire_counter = 0;
						} else {
							between_channel_prefire_counter++;
						}
					} else {
						if (between_channel_prefire_counter < between_channel_fire_delay) {
							between_channel_prefire_counter++;        //æäåì åùå 20 ñåêóíä
						} else {
							if (counter_IR_prefire >= channel_fire_delay) {
								counter_IR_prefire = channel_fire_delay;
								DeviceStatusTemp |= FD2930_DEVICE_STATUS_PREFIRE;    //âêëþ÷àåì ïîæàð
							} else {
								between_channel_prefire_counter = channel_fire_delay;  //æäåì åùå 20 ñåêóíä
							}
						}
					}
				} else {
					if (!counter_IR_prefire && !counter_UV_prefire) DeviceStatusTemp &= ~FD2930_DEVICE_STATUS_PREFIRE;
					if (counter_IR_prefire >= channel_fire_delay) {
						counter_IR_prefire = channel_fire_delay;
						flag_IR_prefire_2ch = 1;
					}
					if (counter_UV_prefire >= channel_fire_delay) {
						counter_UV_prefire = channel_fire_delay;
						flag_UV_prefire_2ch = 1;
					}
				}
		break;
		case FD2930_CONFIG_2:
			if (DeviceData.Status & FD2930_DEVICE_STATUS_ALARM_IR) {
				counter_IR_fire++;
				counter_IR_prefire++;
			} else {
				if (flag_IR_prefire) {
					counter_IR_fire = 0;
					flag_IR_fire_2ch = 0;
					counter_IR_prefire++;
				} else {
					counter_IR_fire = 0;
					flag_IR_fire_2ch = 0;
					flag_IR_prefire_2ch = 0;
					counter_IR_prefire = 0;
					between_channel_fire_counter = 0;
				}
			}
			if (DeviceData.Status & FD2930_DEVICE_STATUS_ALARM_UV) {
				counter_UV_fire++;
				counter_UV_prefire++;
			} else {
				if (flag_UV_prefire) {
					counter_UV_fire = 0;
					flag_UV_fire_2ch = 0;
					counter_UV_prefire++;
				} else {
					counter_UV_fire = 0;
					flag_UV_fire_2ch = 0;
					flag_UV_prefire_2ch = 0;
					counter_UV_prefire = 0;
					between_channel_fire_counter = 0;
				}
			}

			if (flag_IR_fire_2ch) {
				if (between_channel_fire_counter < channel_fire_delay) {
					if (counter_UV_fire >= channel_fire_delay) {
						counter_UV_fire = channel_fire_delay;
						DeviceStatusTemp |= FD2930_DEVICE_STATUS_FIRE;    //âêëþ÷àåì ïîæàð
						between_channel_fire_counter = 0;
					} else {
						between_channel_fire_counter++;
					}
				} else {
					if (between_channel_fire_counter < between_channel_fire_delay) {
						between_channel_fire_counter++;
					} else {
						if (counter_UV_fire >= channel_fire_delay) {
							counter_UV_fire = channel_fire_delay;
							DeviceStatusTemp |= FD2930_DEVICE_STATUS_FIRE;    //âêëþ÷àåì ïîæàð
						} else {
							between_channel_fire_counter = channel_fire_delay;  //æäåì åùå 20 ñåêóíä
						}
					}
				}
			} else
				if (flag_UV_fire_2ch) {
					if (between_channel_fire_counter < channel_fire_delay) {
						if (counter_IR_fire >= channel_fire_delay) {
							counter_IR_fire = channel_fire_delay;
							DeviceStatusTemp |= FD2930_DEVICE_STATUS_FIRE;    //âêëþ÷àåì ïîæàð
							between_channel_fire_counter = 0;
						} else {
							between_channel_fire_counter++;
						}
					} else {
						if (between_channel_fire_counter < between_channel_fire_delay) {
							between_channel_fire_counter++;
						} else {
							if (counter_IR_fire >= channel_fire_delay) {
								counter_IR_fire = channel_fire_delay;
								DeviceStatusTemp |= FD2930_DEVICE_STATUS_FIRE;    //âêëþ÷àåì ïîæàð
							} else {
								between_channel_fire_counter = channel_fire_delay;  //æäåì åùå 20 ñåêóíä
							}
						}
					}
				} else {
					if (!counter_IR_fire && !counter_UV_fire) DeviceStatusTemp &= ~FD2930_DEVICE_STATUS_FIRE;
					if (counter_IR_fire >= channel_fire_delay) {
						counter_IR_fire = channel_fire_delay;
						flag_IR_fire_2ch = 1;
					}
					if (counter_UV_fire >= channel_fire_delay) {
						counter_UV_fire = channel_fire_delay;
						flag_UV_fire_2ch = 1;
					}
				}

			if (flag_IR_prefire_2ch) {
				if (between_channel_prefire_counter < channel_fire_delay) {
					if (counter_UV_prefire >= channel_fire_delay) {
						counter_UV_prefire = channel_fire_delay;
						DeviceStatusTemp |= FD2930_DEVICE_STATUS_PREFIRE;
						between_channel_prefire_counter = 0;
					} else {
						between_channel_prefire_counter++;
					}
				} else {
					if (between_channel_prefire_counter < between_channel_fire_delay) {
						between_channel_prefire_counter++;
					} else {
						if (counter_UV_prefire >= channel_fire_delay) {
							counter_UV_prefire = channel_fire_delay;
							DeviceStatusTemp |= FD2930_DEVICE_STATUS_PREFIRE;
						} else {
							between_channel_prefire_counter = channel_fire_delay;
						}
					}
				}
			} else {
				if (flag_UV_prefire_2ch) {
					if (between_channel_prefire_counter < channel_fire_delay) {
						if (counter_IR_prefire >= channel_fire_delay) {
							counter_IR_prefire = channel_fire_delay;
							DeviceStatusTemp |= FD2930_DEVICE_STATUS_PREFIRE;    //âêëþ÷àåì ïîæàð
							between_channel_prefire_counter = 0;
						} else {
							between_channel_prefire_counter++;
						}
					} else {
						if (between_channel_prefire_counter < between_channel_fire_delay) {
							between_channel_prefire_counter++;        //æäåì åùå 20 ñåêóíä
						} else {
							if (counter_IR_prefire >= channel_fire_delay) {
								counter_IR_prefire = channel_fire_delay;
								DeviceStatusTemp |= FD2930_DEVICE_STATUS_PREFIRE;    //âêëþ÷àåì ïîæàð
							} else {
								between_channel_prefire_counter = channel_fire_delay;  //æäåì åùå 20 ñåêóíä
							}
						}
					}
				} else {
					if (!counter_IR_prefire && !counter_UV_prefire) DeviceStatusTemp &= ~FD2930_DEVICE_STATUS_PREFIRE;
					if (counter_IR_prefire >= channel_fire_delay) {
						counter_IR_prefire = channel_fire_delay;
						flag_IR_prefire_2ch = 1;
					}
					if (counter_UV_prefire >= channel_fire_delay) {
						counter_UV_prefire = channel_fire_delay;
						flag_UV_prefire_2ch = 1;
					}
				}
			}
		break;
		case FD2930_CONFIG_3:
			if (DeviceData.Status & FD2930_DEVICE_STATUS_ALARM_IR) {
				counter_IR_fire++;
				counter_IR_prefire++;
			} else {
				if (flag_IR_prefire) {
					counter_IR_fire = 0;
					counter_IR_prefire++;
				} else {
					counter_IR_fire = 0;
					counter_IR_prefire = 0;
				}
			}
			if (DeviceData.Status & FD2930_DEVICE_STATUS_ALARM_UV) {
				counter_UV_fire++;
				counter_UV_prefire++;
			} else {
				if (flag_UV_prefire) {
					counter_UV_fire = 0;
					counter_UV_prefire++;
				} else {
					counter_UV_fire = 0;
					counter_UV_prefire = 0;
				}
			}
			if ((!counter_IR_fire && !counter_UV_fire) || (CriminalFFTChannelNumber < FD2930_NUMBER_FFT_CRIM_CHANNEL)) DeviceStatusTemp &= ~FD2930_DEVICE_STATUS_FIRE;
			if (!counter_IR_prefire && !counter_UV_prefire) DeviceStatusTemp &= ~FD2930_DEVICE_STATUS_PREFIRE;
			if (counter_IR_fire >= channel_fire_delay) counter_IR_fire = channel_fire_delay;
			if (counter_IR_prefire >= channel_fire_delay) counter_IR_prefire = channel_fire_delay;
			if (counter_UV_fire >= channel_fire_delay) counter_UV_fire = channel_fire_delay;
			if (counter_UV_prefire >= channel_fire_delay) counter_UV_prefire = channel_fire_delay;
			if ((counter_IR_fire >= channel_fire_delay) && (counter_UV_fire >= channel_fire_delay)) {
				if (CriminalFFTChannelNumber > FD2930_NUMBER_FFT_CRIM_CHANNEL) DeviceStatusTemp |= FD2930_DEVICE_STATUS_FIRE;
			}
			if ((counter_IR_prefire >= channel_fire_delay) && (counter_UV_prefire >= channel_fire_delay)) DeviceStatusTemp |= FD2930_DEVICE_STATUS_PREFIRE;
		break;
		case FD2930_CONFIG_4:
			if (DeviceData.Status & FD2930_DEVICE_STATUS_ALARM_IR) {
				counter_IR_fire++;
				counter_IR_prefire++;
			} else {
				if (flag_IR_prefire) {
					counter_IR_fire = 0;
					counter_IR_prefire++;
				} else {
					counter_IR_fire = 0;
					counter_IR_prefire = 0;
				}
			}
			if (DeviceData.Status & FD2930_DEVICE_STATUS_ALARM_UV) {
				counter_UV_fire++;
				counter_UV_prefire++;
			} else {
				if (flag_UV_prefire) {
					counter_UV_fire = 0;
					counter_UV_prefire++;
				} else {
					counter_UV_fire = 0;
					counter_UV_prefire = 0;
				}
			}
			if (!counter_IR_fire && !counter_UV_fire) DeviceStatusTemp &= ~FD2930_DEVICE_STATUS_FIRE;
			if (!counter_IR_prefire && !counter_UV_prefire) DeviceStatusTemp &= ~FD2930_DEVICE_STATUS_PREFIRE;
			if (counter_IR_fire >= channel_fire_delay) counter_IR_fire = channel_fire_delay;
			if (counter_IR_prefire >= channel_fire_delay) counter_IR_prefire = channel_fire_delay;
			if (counter_UV_fire >= channel_fire_delay) counter_UV_fire = channel_fire_delay;
			if (counter_UV_prefire >= channel_fire_delay) counter_UV_prefire = channel_fire_delay;
			if ((counter_IR_fire >= channel_fire_delay) && (counter_UV_fire >= channel_fire_delay)) DeviceStatusTemp |= FD2930_DEVICE_STATUS_FIRE;
			if ((counter_IR_prefire >= channel_fire_delay) && (counter_UV_prefire >= channel_fire_delay)) DeviceStatusTemp |= FD2930_DEVICE_STATUS_PREFIRE;
		break;
		case FD2930_CONFIG_5:
			if (DeviceData.Status & FD2930_DEVICE_STATUS_ALARM_IR) {
				counter_IR_fire++;
				counter_IR_prefire++;
			} else {
				if (flag_IR_prefire) {
					counter_IR_fire = 0;
					counter_IR_prefire++;
				} else {
					counter_IR_fire = 0;
					counter_IR_prefire = 0;
				}
			}
			if (!counter_IR_fire || (CriminalFFTChannelNumber < FD2930_NUMBER_FFT_CRIM_CHANNEL)) {
				DeviceStatusTemp &= ~FD2930_DEVICE_STATUS_FIRE;
			} else {
				if (counter_IR_fire >= channel_fire_delay) {
					counter_IR_fire = channel_fire_delay;
					if (CriminalFFTChannelNumber > FD2930_NUMBER_FFT_CRIM_CHANNEL) DeviceStatusTemp|= FD2930_DEVICE_STATUS_FIRE;
				}
			}
			if (!counter_IR_prefire) {
				DeviceStatusTemp &= ~FD2930_DEVICE_STATUS_PREFIRE;
			} else {
				if (counter_IR_prefire >= channel_fire_delay) {
					counter_IR_prefire = channel_fire_delay;
					DeviceStatusTemp |= FD2930_DEVICE_STATUS_PREFIRE;
				}
			}
		break;
		case FD2930_CONFIG_6:
			if (DeviceData.Status & FD2930_DEVICE_STATUS_ALARM_IR) {
				counter_IR_fire++;
				counter_IR_prefire++;
			} else {
				if (flag_IR_prefire) {
					counter_IR_fire = 0;
					counter_IR_prefire++;
				} else {
					counter_IR_fire = 0;
					counter_IR_prefire = 0;
				}
			}
			if (!counter_IR_fire) {
				DeviceStatusTemp &= ~FD2930_DEVICE_STATUS_FIRE;
			} else {
				if (counter_IR_fire >= channel_fire_delay) {
					counter_IR_fire = channel_fire_delay;
					DeviceStatusTemp|= FD2930_DEVICE_STATUS_FIRE;
				}
			}
			if (!counter_IR_prefire) {
				DeviceStatusTemp &= ~FD2930_DEVICE_STATUS_PREFIRE;
			} else {
				if (counter_IR_prefire >= channel_fire_delay) {
					counter_IR_prefire = channel_fire_delay;
					DeviceStatusTemp |= FD2930_DEVICE_STATUS_PREFIRE;
				}
			}
			break;
		case FD2930_CONFIG_7:
			if (DeviceData.Status & FD2930_DEVICE_STATUS_ALARM_UV) {
				counter_UV_fire++;
				counter_UV_prefire++;
			} else {
				if (flag_UV_prefire) {
					counter_UV_fire = 0;
					counter_UV_prefire++;
				} else {
					counter_UV_fire = 0;
					counter_UV_prefire = 0;
				}
			}
			if (!counter_UV_fire) {
				DeviceStatusTemp &= ~FD2930_DEVICE_STATUS_FIRE;
			} else {
				if (counter_UV_fire >= channel_fire_delay) {
					counter_UV_fire = channel_fire_delay;
					DeviceStatusTemp |= FD2930_DEVICE_STATUS_FIRE;
				}
			}
			if (!counter_UV_prefire) {
				DeviceStatusTemp &= ~FD2930_DEVICE_STATUS_PREFIRE;
			} else {
				if (counter_UV_prefire >= channel_fire_delay) {
					counter_UV_prefire = channel_fire_delay;
					DeviceStatusTemp |= FD2930_DEVICE_STATUS_PREFIRE;
				}
			}
			break;
		case FD2930_CONFIG_8:
			if (flag_UV_inv) {
				counter_UV_inv++;
				DeviceData.Status |= FD2930_DEVICE_STATUS_ALARM_UV;
			} else {
				counter_UV_inv = 0;
				DeviceData.Status &= ~FD2930_DEVICE_STATUS_ALARM_UV;
			}
			if (!counter_UV_inv) {
				DeviceStatusTemp &= ~FD2930_DEVICE_STATUS_FIRE;
				DeviceStatusTemp &= ~FD2930_DEVICE_STATUS_PREFIRE;
			} else {
				if (counter_UV_inv >= channel_fire_delay) {
					counter_UV_inv = channel_fire_delay;
					DeviceStatusTemp |= FD2930_DEVICE_STATUS_FIRE;
					DeviceStatusTemp |= FD2930_DEVICE_STATUS_PREFIRE;
				}
			}
		break;
	}
}

/**
  * @brief
  * @param
  * @retval
  */
__STATIC_INLINE void SetFireStatus()
{
	if (DeviceStatusTemp & FD2930_DEVICE_STATUS_FIRE) {
		if (!(DeviceData.Status & FD2930_DEVICE_STATUS_FIRE)) {
			DeviceData.Status |= FD2930_DEVICE_STATUS_FIRE;
			//push_flash_command(FLASH_WRITE_EVENT, EVENT_FIRE_ON, MBS.buffer);
		}
	}
	if (DeviceStatusTemp & FD2930_DEVICE_STATUS_PREFIRE) DeviceData.Status |= FD2930_DEVICE_STATUS_PREFIRE;
	if (!(DeviceData.Config & FD2930_DEVICECONFIG_FIRE_FIXATION)) {
		if (!(DeviceStatusTemp & FD2930_DEVICE_STATUS_FIRE)) {
			if (DeviceData.Status & FD2930_DEVICE_STATUS_FIRE) {
				DeviceData.Status &= ~FD2930_DEVICE_STATUS_FIRE;
				//push_flash_command(FLASH_WRITE_EVENT, EVENT_FIRE_OFF, MBS.buffer);
			}
		}
		if (!(DeviceStatusTemp & FD2930_DEVICE_STATUS_PREFIRE)) DeviceData.Status &= ~FD2930_DEVICE_STATUS_PREFIRE;
	}
}

/**
  * @brief
  * @param
  * @retval
  */
__STATIC_INLINE void ResetFireStatus()
{
	if (!(DeviceStatusTemp & FD2930_DEVICE_STATUS_FIRE)) {
		if (DeviceData.Status & FD2930_DEVICE_STATUS_FIRE) {
			DeviceData.Status &= ~FD2930_DEVICE_STATUS_FIRE;
			//push_flash_command(FLASH_WRITE_EVENT, EVENT_FIRE_OFF, MBS.buffer);
		}
	}
	if (!(DeviceStatusTemp & FD2930_DEVICE_STATUS_PREFIRE)) DeviceData.Status &= ~FD2930_DEVICE_STATUS_PREFIRE;
}

/**
  * @brief
  * @param
  * @retval
  */
__STATIC_INLINE void SetDeviceStatus()
{
	switch(DeviceState) {
		case FD2930_STATE_TEST:
			break;
		default:
			if (DeviceData.Config & DeviceData.Flags & 0xf000) {
				if (!(DeviceData.Status & FD2930_DEVICE_STATUS_FAULT)) {
					DeviceData.Status |= FD2930_DEVICE_STATUS_FAULT;
					//push_flash_command(FLASH_WRITE_EVENT, EVENT_FAULT_ON, MBS.buffer);
				}
			} else
				if (DeviceData.Status & FD2930_DEVICE_STATUS_FAULT) {
					DeviceData.Status &= ~FD2930_DEVICE_STATUS_FAULT;
					//push_flash_command(FLASH_WRITE_EVENT, EVENT_FAULT_OFF, MBS.buffer);
				}
			if (DeviceData.Status & FD2930_DEVICE_STATUS_TESTING || DeviceData.Flags & FD2930_DEVICEFLAGS_IR_ERROR || DeviceData.Flags & FD2930_DEVICEFLAGS_UV_ERROR \
				|| !(DeviceData.Status & FD2930_DEVICE_STATUS_FLASH_OK) || !(DeviceData.Status & FD2930_DEVICE_STATUS_CRC_OK) \
				|| !(DeviceData.Status & FD2930_DEVICE_STATUS_IR_UV_SET) || !(DeviceData.Status & FD2930_DEVICE_STATUS_TEST_CALIBR) \
				|| !(DeviceData.Status & FD2930_DEVICE_STATUS_TEST_ZERO) || ((DeviceData.Config & FD2930_DEVICECONFIG_DUST_TO_RELAY) \
				&& (DeviceData.Flags & FD2930_DEVICEFLAGS_BREAK_DUST)))  {
				if (!(DeviceData.Status & FD2930_DEVICE_STATUS_BREAK)) {
					DeviceData.Status |= FD2930_DEVICE_STATUS_BREAK;
					//push_flash_command(FLASH_WRITE_EVENT, EVENT_BREAK_ON, MBS.buffer);
				}
			} else {
				if (DeviceData.Status & FD2930_DEVICE_STATUS_BREAK) {
					DeviceData.Status &= ~FD2930_DEVICE_STATUS_BREAK;
					//push_flash_command(FLASH_WRITE_EVENT, EVENT_BREAK_OFF, MBS.buffer);
				}
			}
			if (DeviceData.Status & FD2930_DEVICE_STATUS_BREAK) {
				DeviceData.Current420 = 3200;
			} else {
				if (DeviceData.Status & FD2930_DEVICE_STATUS_FAULT) DeviceData.Current420 = 3200;
				else DeviceData.Current420 = 4000;
			}
		break;
	}
}

/**
  * @brief
  * @param
  * @retval
  */
__STATIC_INLINE void SetLEDs()
{
    if (IRDA_Command_Rcvd) {
        return; // IRDA indication dominates over the others types. Indication state is in SYSTickHandler
    }
    if ((DeviceData.Status & FD2930_DEVICE_STATUS_BREAK) || (DeviceData.Status & FD2930_DEVICE_STATUS_TESTING)) LEDState = FD2930_LED_YELLOW;
    else if (DeviceData.Status & FD2930_DEVICE_STATUS_FIRE) LEDState = FD2930_LED_RED;
    else if (DeviceData.Status & FD2930_DEVICE_STATUS_PREFIRE) LEDState = FD2930_LED_RED_BLINKING;
    else if (DeviceData.Status & FD2930_DEVICE_STATUS_FAULT) LEDState = FD2930_LED_YELLOW_BLINKING;
    else if (DeviceData.Status & FD2930_DEVICE_STATUS_MAGNET) LEDState = FD2930_LED_BLUE;
    else LEDState = FD2930_LED_GREEN;

    switch(LEDState) {
    	case FD2930_LED_RED_BLINKING:
    	case FD2930_LED_YELLOW_BLINKING:
    		DeviceData.Flags |= FD2930_DEVICEFLAGS_BLINK_LED;
    	break;
    	default:
    		DeviceData.Flags &= ~FD2930_DEVICEFLAGS_BLINK_LED;
    }
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
	}
}

/**
  * @brief
  * @param
  * @retval
  */
__STATIC_INLINE void HandleRelayWork()
{
	static uint16_t cnt = 0;

	if (cnt < TIME_mS_TO_TICK(DELAY_RELAY_WORK)) {
		cnt++;
	} else {
		cnt = 0;
		if (DeviceData.Flags & FD2930_DEVICEFLAGS_WORK_RELAY_ON) {
			GPIO_TOGGLE_PIN(LPC_GPIO2, R_WORK);
		}
	}
}

/**
  * @brief
  * @param
  * @retval
  */
__STATIC_INLINE void HandleIRTestChannel()
{
	static uint16_t cnt = 0;

	if (DeviceState == FD2930_STATE_CHANNEL_CALIBR) {
		if (cnt < TIME_mS_TO_TICK(IR_CHANNEL_TIMING1)) {
			cnt++;
		} else {
			cnt = 0;
			GPIO_TOGGLE_PIN(LPC_GPIO2, IR_TEST);
		}
	} else {
		if ((DeviceState == FD2930_STATE_TEST_CALIBR) || (DeviceState == FD2930_STATE_SELFTEST) || (DeviceState == FD2930_STATE_TEST_ZERO)) {
			if (cnt < TIME_mS_TO_TICK(IR_CHANNEL_TIMING2)) {
				cnt++;
			} else {
				cnt = 0;
				GPIO_TOGGLE_PIN(LPC_GPIO2, IR_TEST);
			}
		}
	}
}

/**
  * @brief
  * @param
  * @retval
  */
__STATIC_INLINE void SetRelays()
{
	static uint16_t counter_relay_fire_delay = 0;
	static uint16_t counter_relay_work_delay = 0;

	switch (DeviceState) {
		case FD2930_STATE_TEST:
			if (DeviceData.Flags & FD2930_DEVICEFLAGS_DUST_RELAY_ON) {
				GPIO_RESET_PIN(LPC_GPIO2, R_DUST);
			} else {
				GPIO_SET_PIN(LPC_GPIO2, R_DUST);
			}
			if (DeviceData.Flags & FD2930_DEVICEFLAGS_FIRE_RELAY_ON) {
				GPIO_RESET_PIN(LPC_GPIO2, R_FIRE);
			} else {
				GPIO_SET_PIN(LPC_GPIO2, R_FIRE);
			}
		break;
		default:
			if ((DeviceData.Status & FD2930_DEVICE_STATUS_BREAK) || (DeviceData.Status & FD2930_DEVICE_STATUS_MAGNET)) {
				GPIO_SET_PIN(LPC_GPIO2, R_FIRE);
				GPIO_SET_PIN(LPC_GPIO2, R_FIRE_MVES);
			} else {
				if (DeviceData.Status & FD2930_DEVICE_STATUS_FIRE) {
					counter_relay_fire_delay++;
					if (counter_relay_fire_delay >= DeviceData.FireDelay) {
						if (DeviceData.Config & FD2930_DEVICECONFIG_RELAY_FIRE_ALLOWED) {GPIO_RESET_PIN(LPC_GPIO2, R_FIRE);};
						DeviceData.Flags |= FD2930_DEVICEFLAGS_FIRE_RELAY_ON;
						DeviceData.Current420 = 19990;
						counter_relay_fire_delay = DeviceData.FireDelay;
					}
				} else {
					counter_relay_fire_delay = 0;
					GPIO_SET_PIN(LPC_GPIO2, R_FIRE);
					GPIO_SET_PIN(LPC_GPIO2, R_FIRE_MVES);
					DeviceData.Flags &= ~FD2930_DEVICEFLAGS_FIRE_RELAY_ON;
				}
			}
			if ((DeviceData.Status & FD2930_DEVICE_STATUS_MAGNET) || (DeviceData.Status & FD2930_DEVICE_STATUS_TESTING) || (DeviceData.Flags & FD2930_DEVICEFLAGS_IR_ERROR) || (DeviceData.Flags & FD2930_DEVICEFLAGS_UV_ERROR) \
				|| !(DeviceData.Status & FD2930_DEVICE_STATUS_FLASH_OK) || !(DeviceData.Status & FD2930_DEVICE_STATUS_CRC_OK) || !(DeviceData.Status & FD2930_DEVICE_STATUS_IR_UV_SET) \
				|| !(DeviceData.Status & FD2930_DEVICE_STATUS_TEST_CALIBR)|| !(DeviceData.Status & FD2930_DEVICE_STATUS_TEST_ZERO) || (DeviceData.Config & DeviceData.Flags & 0xf000)) {
				counter_relay_work_delay++;
				if (counter_relay_work_delay >= DeviceData.FaultDelay) {
					DeviceData.Flags &= ~FD2930_DEVICEFLAGS_WORK_RELAY_ON;
					counter_relay_work_delay = DeviceData.FaultDelay;
				}
			} else {
				if (DeviceData.Config & FD2930_DEVICECONFIG_RELAY_FAULT_ALLOWED) {
					DeviceData.Flags |= FD2930_DEVICEFLAGS_WORK_RELAY_ON;
				} else {
					DeviceData.Flags &= ~FD2930_DEVICEFLAGS_WORK_RELAY_ON;
				}
				counter_relay_work_delay = 0;
			}
		break;
	}
}


/**
  * @brief
  * @param
  * @retval
  */
__STATIC_INLINE void SetHeater()
{
	static int flag_increase = 0;

	if ((DeviceData.Temperature < DeviceData.HeaterThres) && !flag_increase) {
		flag_increase = 1;
	} else {
		if (DeviceData.Temperature >= DeviceData.HeaterThres) {
		    flag_increase = 0;
		}
	}
	if (flag_increase) {  //нагрев
		if (HeatPowerInst < DeviceData.HeatPower) HeatPowerInst += 0.1;
		if (HeatPowerInst > 99 ) HeatPowerInst = 100;
	} else { 				//охлаждение
		if (HeatPowerInst) HeatPowerInst -= 0.1;
		if (HeatPowerInst < 1) HeatPowerInst = 1;
    }
	if (DeviceData.Config & FD2930_DEVICECONFIG_HEAT_ALLOWED) PWM1MatchUpdate(HeatPowerInst);
	else PWM1MatchUpdate(0);
}

static uint8_t MBRegModified;
/**
  * @brief
  * @param
  * @retval
  */
uint8_t MBPassCallBack(uint16_t addr, uint16_t valQty)
{
	uint16_t storVal, setVal;
	if (Modbus.Uart->RxBuf[1] == MODBUS_WRITE_MULT_REG)	{
		if ((addr >= Modbus.HoldRegSize) || (valQty > 1)) return 0;
		setVal = (Modbus.Uart->RxBuf[7] << 8) | Modbus.Uart->RxBuf[8];
	} else {
		setVal = valQty;
	}
	storVal = *(((uint16_t *)&DeviceData) + addr);

	if (setVal != storVal) MBRegModified = 1;
	else MBRegModified = 0;
	return 1;
}

/**
  * @brief
  * @param
  * @retval
  */
uint8_t MBCallBack(uint16_t addr, uint16_t qty)
{
	if ((addr == MB_REG_ADDR(DeviceData, MBId)) && (qty == 1)) {
		if (Protocol == PROTOCOL_IPES) {
			uint8_t mbId = (DeviceData.MBId >> 8) & 0xFF;
			uint8_t baud = DeviceData.MBId & 0xFF;
			if ((mbId > 0) && (mbId < 248) && ((baud == 1) || (baud == 2) || (baud == 4) || (baud == 8) || (baud == 16))) {
				ChangeConnectionSettings = 1;
				EEPROMWrite((uint8_t *)&DeviceData, EEPROM_PAGE_SIZE);
			}
		} else {
			if ((DeviceData.MBId > 0) && (DeviceData.MBId < 248)) {
				ChangeConnectionSettings = 1;
				EEPROMWrite((uint8_t *)&DeviceData, EEPROM_PAGE_SIZE);
			}
		}
		return 0;
	}
	if ((addr == MB_REG_ADDR(DeviceData, Baudrate)) && (qty == 1)) {
		if (Protocol == PROTOCOL_IPES) {
			uint8_t config = (DeviceData.Baudrate >> 8) & 0xFF;
			if (config & (1 << 2)) DeviceData.Config |= FD2930_DEVICECONFIG_FIRE_FIXATION;
			else DeviceData.Config &= ~FD2930_DEVICECONFIG_FIRE_FIXATION;
			if (config & (1 << 1)) DeviceData.Config &= ~FD2930_DEVICECONFIG_LOW_SENS;
			else DeviceData.Config |= FD2930_DEVICECONFIG_LOW_SENS;
			EEPROMWrite((uint8_t *)&DeviceData, EEPROM_PAGE_SIZE);
		} else {
			if (DeviceData.Baudrate == 1 || DeviceData.Baudrate == 2 || DeviceData.Baudrate == 4 || DeviceData.Baudrate == 12 || DeviceData.Baudrate == 24) {
				ChangeConnectionSettings = 1;
				EEPROMWrite((uint8_t *)&DeviceData, EEPROM_PAGE_SIZE);
			}
		}
		return 0;
	}
	if ((addr == MB_REG_ADDR(DeviceData, SerialNumber)) && (qty == 1)) {
		EEPROMWrite((uint8_t *)&DeviceData, EEPROM_PAGE_SIZE);
		return 0;
	}
	if ((addr == MB_REG_ADDR(DeviceData, Config)) && (qty == 1)) {
		if (((DeviceData.Config & 0x0F) <= 8) && ((DeviceData.Config & 0x0F) > 0)) {
			if ((DeviceData.Config & FD2930_DEVICECONFIG_IPES_MB_HEADER) != Protocol) {
				if (DeviceData.Config & FD2930_DEVICECONFIG_IPES_MB_HEADER) {
					DeviceData.Baudrate *= 4;
					DeviceData.MBId <<= 8;
					DeviceData.MBId |= DeviceData.Baudrate;
					uint8_t config = 0;
					if (DeviceData.Config & FD2930_DEVICECONFIG_FIRE_FIXATION) config |= (1 << 2);
					if ((DeviceData.Config & FD2930_DEVICECONFIG_LOW_SENS) == 0) config |= (1 << 1);
					DeviceData.Baudrate = config << 8;
				} else {
					uint8_t mbId = (DeviceData.MBId >> 8) & 0xFF;
					uint8_t baud = DeviceData.MBId & 0xFF;
					DeviceData.MBId = mbId;
					DeviceData.Baudrate = baud;
				}
			}
			EEPROMWrite((uint8_t *)&DeviceData, EEPROM_PAGE_SIZE);
		}
		return 0;
	}
	if ((addr == MB_REG_ADDR(DeviceData, UVThres)) && (qty == 1)) {
		if ((DeviceData.UVThres >= FD2930_MIN_THRES_UV) && (DeviceData.UVThres <= FD2930_MAX_THRES_UV)) {
			EEPROMWrite((uint8_t *)&DeviceData, EEPROM_PAGE_SIZE);
		}
		return 0;
	}
	if ((addr == MB_REG_ADDR(DeviceData, IRThres)) && (qty == 1)) {
		if ((DeviceData.IRThres >= FD2930_MIN_THRES_IR) && (DeviceData.IRThres <= FD2930_MAX_THRES_IR)) {
			EEPROMWrite((uint8_t *)&DeviceData, EEPROM_PAGE_SIZE);
		}
		return 0;
	}
	if ((addr == MB_REG_ADDR(DeviceData, UVCoeff)) && (qty == 1)) {
		if ((DeviceData.UVCoeff >= FD2930_MIN_K_UV) && (DeviceData.UVCoeff <= FD2930_MAX_K_UV)) {
			EEPROMWrite((uint8_t *)&DeviceData, EEPROM_PAGE_SIZE);
		}
		return 0;
	}
	if ((addr == MB_REG_ADDR(DeviceData, IRCoeff)) && (qty == 1)) {
		if ((DeviceData.IRCoeff >= FD2930_MIN_K_IR) && (DeviceData.IRCoeff <= FD2930_MAX_K_IR)) {
			EEPROMWrite((uint8_t *)&DeviceData, EEPROM_PAGE_SIZE);
		}
		return 0;
	}
	if ((addr == MB_REG_ADDR(DeviceData, FireDelay)) && (qty == 1)) {
		if ((DeviceData.FireDelay >= FD2930_MIN_WAIT_FIRE) && (DeviceData.FireDelay <= FD2930_MAX_WAIT_FIRE)) {
			EEPROMWrite((uint8_t *)&DeviceData, EEPROM_PAGE_SIZE);
		}
		return 0;
	}
	if ((addr == MB_REG_ADDR(DeviceData, FaultDelay)) && (qty == 1)) {
		if ((DeviceData.FaultDelay >= FD2930_MIN_WAIT_FAULT) && (DeviceData.FaultDelay <= FD2930_MAX_WAIT_FAULT)) {
			EEPROMWrite((uint8_t *)&DeviceData, EEPROM_PAGE_SIZE);
		}
		return 0;
	}
	if ((addr == MB_REG_ADDR(DeviceData, Command)) && (qty == 1)) {
		switch (DeviceData.Command) {
			case 0:
				ResetFireStatus();
				///push_flash_command(FLASH_WRITE_EVENT, EVENT_COMMAND + Cnt.w, MBS.buffer);
			break;
			case 1:
				DeviceState = FD2930_STATE_SELFTEST;
				DeviceData.Status |= FD2930_DEVICE_STATUS_SELF_TEST;
				GPIO_RESET_PIN(LPC_GPIO2, (UV_TEST)); GPIO_RESET_PIN(LPC_GPIO0, (UV_TEST2));
				//push_flash_command(FLASH_WRITE_EVENT, EVENT_COMMAND + Cnt.w, MBS.buffer);
			break;
			case 2:
				DeviceData.Config &= ~FD2930_DEVICECONFIG_FIRE_FIXATION;
				//push_flash_command(FLASH_WRITE_EVENT, EVENT_COMMAND + Cnt.w, MBS.buffer);
			break;
			case 3:
				SetDefaultParameters();
				//push_flash_command(FLASH_WRITE_EVENT, EVENT_COMMAND + Cnt.w, MBS.buffer);
			break;
			case 4:
				EEPROMErase();
				DeviceData.Status &= ~FD2930_DEVICE_STATUS_FLASH_OK;
				//push_flash_command(FLASH_WRITE_EVENT, EVENT_COMMAND + Cnt.w, MBS.buffer);
			    for(;;);
			break;
			case 5:
				DeviceData.StateFlags |= FD2930_STATE_FLAG_ERASE_ARCHIVE;
				//push_flash_command(FLASH_WRITE_EVENT, EVENT_COMMAND + Cnt.w, MBS.buffer);
			break;
			case 6:
				if (DeviceState == FD2930_STATE_TEST) {
					DeviceData.Flags |= FD2930_DEVICEFLAGS_FIRE_RELAY_ON;
					//push_flash_command(FLASH_WRITE_EVENT, EVENT_COMMAND + Cnt.w, MBS.buffer);
				}
			break;
			case 7:
				if (DeviceState == FD2930_STATE_TEST) {
					DeviceData.Flags &= ~FD2930_DEVICEFLAGS_FIRE_RELAY_ON;
					//push_flash_command(FLASH_WRITE_EVENT, EVENT_COMMAND + Cnt.w, MBS.buffer);
				}
			break;
			case 8:
				if (DeviceState == FD2930_STATE_TEST) {
					DeviceData.Flags |= FD2930_DEVICEFLAGS_WORK_RELAY_ON;
					//push_flash_command(FLASH_WRITE_EVENT, EVENT_COMMAND + Cnt.w, MBS.buffer);
			    }
			break;
			case 9:
				if (DeviceState == FD2930_STATE_TEST) {
					DeviceData.Flags &= ~FD2930_DEVICEFLAGS_WORK_RELAY_ON;
					//push_flash_command(FLASH_WRITE_EVENT, EVENT_COMMAND + Cnt.w, MBS.buffer);
				}
			break;
			case 10:
				if (DeviceState == FD2930_STATE_TEST) {
					DeviceData.Flags |= FD2930_DEVICEFLAGS_DUST_RELAY_ON;
					//push_flash_command(FLASH_WRITE_EVENT, EVENT_COMMAND + Cnt.w, MBS.buffer);
				}
			break;
			case 11:
				if (DeviceState == FD2930_STATE_TEST) {
					DeviceData.Flags &= ~FD2930_DEVICEFLAGS_DUST_RELAY_ON;
					//push_flash_command(FLASH_WRITE_EVENT, EVENT_COMMAND + Cnt.w, MBS.buffer);
				}
			break;
			case 12:
				if (DeviceState == FD2930_STATE_TEST) {
					DeviceData.Current420 = 19990;
					//push_flash_command(FLASH_WRITE_EVENT, EVENT_COMMAND + Cnt.w, MBS.buffer);
				}
			break;
			case 13:
				DeviceState = FD2930_STATE_CHANNEL_CALIBR;
			break;
			case 14:
				if (!(DeviceData.Status & FD2930_DEVICE_STATUS_TEST_ZERO)) {
					DeviceState = FD2930_STATE_TEST_CALIBR;
					DeviceData.Status &= ~FD2930_DEVICE_STATUS_TEST_CALIBR;
					GPIO_RESET_PIN(LPC_GPIO2, (UV_TEST));
					GPIO_RESET_PIN(LPC_GPIO0, (UV_TEST2));
					EEPROMWrite((uint8_t *)&DeviceData, EEPROM_PAGE_SIZE);
					//push_flash_command(FLASH_WRITE_EVENT, EVENT_COMMAND + Cnt.w, MBS.buffer);
				}
			break;
			case 15:
				//restore_parameters_from_SD();
				//push_flash_command(FLASH_WRITE_EVENT, EVENT_COMMAND + Cnt.w, MBS.buffer);
			break;
			case 16:
				//MBS.err = MBS_ERROR_ILLEGAL_DATA_VALUE;
			break;
			case 17:
				if (DeviceData.Status & FD2930_DEVICE_STATUS_SD_CARD) {
					if ((DeviceData.ArchPageIdx) >= DeviceData.ArchLastPage) DeviceData.ArchPageIdx = 0;
			        else DeviceData.ArchPageIdx++;
					//push_flash_command(FLASH_READ_ARCHIVE, fd2930.chosen_archive_page, MBS.buffer);
			    }
			break;
			case 18:
				if (DeviceData.Status & FD2930_DEVICE_STATUS_SD_CARD) {
					if (DeviceData.ArchPageIdx == 0) DeviceData.ArchPageIdx = DeviceData.ArchLastPage - 1;
					else DeviceData.ArchPageIdx--;
					//push_flash_command(FLASH_READ_ARCHIVE, fd2930.chosen_archive_page, MBS.buffer);
				}
			break;
			case 19:
				//MBS.err = MBS_ERROR_ILLEGAL_DATA_VALUE;
			break;
			case 20:
				DeviceState = FD2930_STATE_TEST;
				DeviceData.Status |= FD2930_DEVICE_STATUS_TESTING;
				//push_flash_command(FLASH_WRITE_EVENT, EVENT_COMMAND + Cnt.w, MBS.buffer);
			break;
			case 21:
				DeviceState = FD2930_STATE_WORKING;
				//push_flash_command(FLASH_WRITE_EVENT, EVENT_NORMAL_EVENT, MBS.buffer);
				DeviceData.Status &= ~FD2930_DEVICE_STATUS_TESTING;
				//push_flash_command(FLASH_WRITE_EVENT, EVENT_COMMAND + Cnt.w, MBS.buffer);
				break;
			case 22:
				//fd2930.K_IR = fd2930_K_IR;
				//fd2930.K_UV = fd2930_K_UV;
				if (!(DeviceData.Status & FD2930_DEVICE_STATUS_IR_UV_SET)) {
					//push_flash_command(FLASH_WRITE_EVENT, EVENT_CALIBR2, MBS.buffer);
					DeviceData.Status |= FD2930_DEVICE_STATUS_IR_UV_SET;
					//push_flash_command(FLASH_WRITE_EVENT, EVENT_COMMAND + Cnt.w, MBS.buffer);
				}
				EEPROMWrite((uint8_t *)&DeviceData, EEPROM_PAGE_SIZE);
			break;
			case 23:        //set test zero
				if (DeviceData.Status & FD2930_DEVICE_STATUS_IR_UV_SET) {
					DeviceState = FD2930_STATE_TEST_ZERO;
					DeviceData.Status &= ~FD2930_DEVICE_STATUS_TEST_ZERO;
					DeviceData.Status &= ~FD2930_DEVICE_STATUS_TEST_CALIBR;
					GPIO_RESET_PIN(LPC_GPIO2, (UV_TEST)); GPIO_RESET_PIN(LPC_GPIO0, (UV_TEST2));
					EEPROMWrite((uint8_t *)&DeviceData, EEPROM_PAGE_SIZE);
					//push_flash_command(FLASH_WRITE_EVENT, EVENT_COMMAND + Cnt.w, MBS.buffer);
				}
			break;
			default:
				//MBS.err = MBS_ERROR_ILLEGAL_DATA_VALUE;*/
			break;
		}
		return 0;
	}
	return 0;
}

/**
  * @brief
  * @param
  * @retval
  */
#define EWMA_FILTER(_PREV_, _CURR_, _COEFF_)				(_COEFF_ * _CURR_ + (1 - _COEFF_) * _PREV_)

/**
  * @brief
  * @param
  * @retval
  */
void SDADCTask(double avCoeffIR, double avCoeffUV)
{
	Max11040ChannelData_t *res = Max11040GetData(2);
	IRRaw = res->ch[0];
	UVRaw = res->ch[1];

	if (UVRaw < UVNoise / 10) {
		if (UVPickCnt < UV_PICK_LIMIT) UVPickCnt++;
		else UVPickCnt = 0;
	}

	if ((UVPickCnt > UV_PICK_WORK_AREA) || (UVPickCnt == 0) || (DeviceData.IRGain > IR_GAIN_UV_PICK)) {
		UV = EWMA_FILTER(UV, UVRaw, avCoeffUV);
	}
	IRAv = EWMA_FILTER(IRAv, IRRaw, avCoeffIR);

	if (IRRaw > IRAv) IRRect = IRRaw - IRAv;
	else IRRect = IRAv - IRRaw;
	IR = EWMA_FILTER(IR, IRRect, avCoeffIR);

	DeviceData.UVGain = UV * 4 * DeviceData.UVCoeff / 10;
	DeviceData.IRGain = IR * 4 * DeviceData.IRCoeff / 10;

	DeviceData.IRRaw = IRRaw;
	DeviceData.UVRaw = UVRaw;
	DeviceData.IRAv = IRAv;
	DeviceData.IRRect = IRRect;

	if (!(DeviceData.StateFlags & FD2930_STATE_FLAG_FFT_ACTIVE)) {
		if (FFTCnt < FFT_POINTS * 2) {
			FFTInputData[FFTCnt++] = DeviceData.IRRect;
		} else {
			FFTCnt = 0;
			DeviceData.StateFlags |= FD2930_STATE_FLAG_FFT_START;
		}
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
		    	if (!(DeviceData.Flags & FD2930_DEVICEFLAGS_ERROR_TEMPERATURE)) {;}////push_flash_command(FLASH_WRITE_EVENT, EVENT_T_FAULT, MBS.buffer);
		    	DeviceData.Flags |= FD2930_DEVICEFLAGS_ERROR_TEMPERATURE;
		    }
		} else {
		    if ((DeviceData.Flags & FD2930_DEVICEFLAGS_ERROR_TEMPERATURE)) {;}////push_flash_command(FLASH_WRITE_EVENT, EVENT_T_NORMAL, MBS.buffer);
		    DeviceData.Flags &= ~FD2930_DEVICEFLAGS_ERROR_TEMPERATURE;
		    CntTemperatureFault = 0;
		}
		if (DeviceData.UVVoltage > VOLTAGE_UV_WORKING_MAXIMUM || DeviceData.UVVoltage < VOLTAGE_UV_WORKING_MINIMUM) {
		    if (!(DeviceData.Flags & FD2930_DEVICEFLAGS_ERROR_UV_VOLTAGE)) {;}////push_flash_command(FLASH_WRITE_EVENT, EVENT_HV_FAULT, MBS.buffer);
		    DeviceData.Flags |= FD2930_DEVICEFLAGS_ERROR_UV_VOLTAGE;
		} else {
		    if ((DeviceData.Flags & FD2930_DEVICEFLAGS_ERROR_UV_VOLTAGE)) {;}////push_flash_command(FLASH_WRITE_EVENT, EVENT_HV_NORMAL, MBS.buffer);
		    DeviceData.Flags &= ~FD2930_DEVICEFLAGS_ERROR_UV_VOLTAGE;
		}
	}
}

/**
  * @brief
  * @param
  * @retval
  */
void SetDefaultParameters()
{
	DeviceData.Config = FD2930_DEFAULT_DEVICE_CONFIG;
	DeviceData.BlockService = 5813;
	DeviceData.SerialNumber = 0;
	DeviceData.Status = FD2930_DEVICE_STATUS_BREAK;
	DeviceData.MBId = FD2930_DEF_MBS_ADDR;
	DeviceData.Baudrate = FD2930_DEF_MBS_BAUD;
	DeviceData.WorkedTime = 0;
	DeviceData.HeatPower = FD2930_DEFAULT_HEATPOWER;
	DeviceData.HeaterThres = FD2930_DEFAULT_THRES_HEATER;
	DeviceData.FFTThres = FD2930_DEFAULT_GAIN_FFT;
	DeviceData.IRThres = FD2930_DEFAULT_THRES_IR;
	DeviceData.UVThres = FD2930_DEFAULT_THRES_UV;
	DeviceData.IRCoeff = FD2930_DEFAULT_K_IR;
	DeviceData.UVCoeff = FD2930_DEFAULT_K_UV;
	DeviceData.FaultDelay = FD2930_DEFAULT_WAIT_FAULT;
	DeviceData.FireDelay = FD2930_DEFAULT_WAIT_FIRE;
}

/**
  * @brief
  * @param
  * @retval
  */
void SetDefaultMBParameters()
{
	if (Protocol == PROTOCOL_IPES) {
		DeviceData.MBId = (FD2930_DEF_MBS_ADDR << 8) | IPES_DEF_MBS_BAUD;
	} else {
		DeviceData.MBId = FD2930_DEF_MBS_ADDR;
		DeviceData.Baudrate = FD2930_DEF_MBS_BAUD;
	}
	EEPROMWrite((uint8_t *)&DeviceData, EEPROM_PAGE_SIZE);
}
    
//------------------------------------------------------------------------------
// end
//------------------------------------------------------------------------------



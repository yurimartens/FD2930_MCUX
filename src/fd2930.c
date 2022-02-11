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

uint16_t		UVNoise, UVNoiseTest, IRNoise, IRNoiseTest;
uint16_t		UVTestLevel, IRTestLevel, UVTestFaultCnt, IRTestFaultCnt;

uint16_t		IRThresF, UVThresF;

uint16_t		IRTroubleCnt, IRTroubleCntPiece, UVTroubleCnt, UVTroubleCntPiece;

uint16_t 		UVPickCnt;
uint16_t 		SelfTestCnt, SelfTest, BadSelfTest;

uint16_t 		CheckFireStatusCnt;

uint16_t 		FFTCnt;

float 			IR, UV;

int16_t *FFTInputData  = (int16_t *)0x2007C000; /* AHB SRAM0 */
int16_t *FFTOutputData = (int16_t *)0x20080000; /* AHB SRAM1 */

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
	static uint32_t cnt = 0, cnt24 = 0, postFireCnt = 0;
	static uint64_t selfTestCnt = 0;

	SDADCTask(0.0014, 0.001);
	switch (DeviceState) {
		case FD2930_STATE_START1:
			if (cnt < START_DELAY) {
				cnt++;
				if ((!GPIO_READ_PIN(LPC_GPIO4, HALL1)) || (!GPIO_READ_PIN(LPC_GPIO4, HALL2))) {
					cnt = 0;
					// SetDefault
					// Init device
					DeviceState++;
				}
			} else {
				cnt = 0;
				// Init device
				DeviceState++;
			}
		break;
		case FD2930_STATE_START2:
			if (cnt < AFTER_START_DELAY) {
				cnt++;
			} else {
				cnt = 0;
				// DAC4-20 Init
				DeviceState++;
			}
		break;
		case FD2930_STATE_START3:
			if (cnt < (DELAY_10S + (DELAY_1S * (DeviceData.MBId % 16)))) {
				cnt++;
			} else {
				cnt = 0;
				if ((DeviceData.Status & FD2930_DEVICE_STATUS_IR_UV_SET) && (DeviceData.Status & FD2930_DEVICE_STATUS_TEST_CALIBR) && (DeviceData.Status & FD2930_DEVICE_STATUS_TEST_ZERO)) {
					if (DeviceData.Config & FD2930_DEVICECONFIG_SELFTEST_ALLOWED) {
						DeviceState = FD2930_STATE_SELFTEST;
						SelfTest++;
						GPIO_RESET_PIN(LPC_GPIO2, (UV_TEST)); GPIO_RESET_PIN(LPC_GPIO0, (UV_TEST2));
					} else {
						DeviceState = FD2930_STATE_WORKING;
						//push_flash_command(FLASH_WRITE_EVENT, EVENT_NORMAL_EVENT, MBS.buffer);
					}
				} else {
					DeviceState = FD2930_STATE_BREAK;
				}
				//AppFD2930Task();  //обновим статус, может прибор не работает
			}
		break;
		case FD2930_STATE_SELFTEST:
			DeviceData.Status |= FD2930_DEVICE_STATUS_SELF_TEST;
			if (SelfTestCnt < DELAY_5S) {
				SelfTestCnt++; // время свмотестирования //на 5-ой секунде выносим вердикт, выходим в деж режим
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
			        		if ((DeviceData.Temperature > 0) && (DeviceData.Temperature < 70) && (DeviceData.HeatPower < 20)) {// если температура не выходит за рамки и подогрев не в разогреве то  ставим флаг // просто если идет резкий разогрев, то ИК сенсор тупит и теряет чувствительность из за чего тест не проходит
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
			        		if ((DeviceData.Temperature > 0) && (DeviceData.Temperature < 70) && (DeviceData.HeatPower < 20)) { // если температура не выходит за рамки и подогрев не в разогреве то  ставим флаг // просто если идет резкий разогрев, то ИК сенсор тупит и теряет чувствительность из за чего тест не проходит
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
			    		   if ((DeviceData.Temperature > 0) && (DeviceData.Temperature < 50) && (DeviceData.HeatPower < 20)) { // если температура не выходит за рамки и подогрев не в разогреве то  ставим флаг // просто если идет резкий разогрев, то ИК сенсор тупит и теряет чувствительность из за чего тест не проходит
			    			   DeviceData.Flags |= FD2930_DEVICEFLAGS_BREAK_DUST;
			    			   DeviceData.Flags |= FD2930_DEVICEFLAGS_DUST;// этот флаг тоже поставим , потому что если уж не прошли очень плохие тесты, то плохие тоже не прошли. что бы выключалось реле неисправность
			    		   }
			    	   }
			    	   if (((DeviceData.IRGain - IRNoiseTest) < 0.7 * IRTestLevel)) { // если чувствительность ИК меньше 70% и  && (DeviceData.temperature < 60) то ставим флаг запыленности
			    		   if ((DeviceData.Temperature > 0) && (DeviceData.Temperature < 50) && (DeviceData.HeatPower < 20)) { //дублируем запыленность в регистре состояния// если температура не выходит за рамки и подогрев не в разогреве то  ставим флаг
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
			    		   if ((DeviceData.Temperature > 0) && (DeviceData.Temperature < 50) && (DeviceData.HeatPower < 20)) { // если температура не выходит за рамки и подогрев не в разогреве то  ставим флаг // просто если идет резкий разогрев, то ИК сенсор тупит и теряет чувствительность из за чего тест не проходит
			    			   DeviceData.Flags |= FD2930_DEVICEFLAGS_BREAK_DUST;
			    			   DeviceData.Flags |= FD2930_DEVICEFLAGS_DUST;// этот флаг тоже поставим , потому что если уж не прошли очень плохие тесты, то плохие тоже не прошли. что бы выключалось реле неисправность
			    		   }
			    	   }
			    	   if ((DeviceData.UVGain - UVNoiseTest) < 0.7 * UVTestLevel) { // если чувствительность УФ меньше 70% и  && (DeviceData.temperature < 60) то ставим флаг запыленности
			    		   if ((DeviceData.Temperature > 0) && (DeviceData.Temperature < 50) && (DeviceData.HeatPower < 20)) {
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
		        //push_flash_command(FLASH_WRITE_EVENT, EVENT_NORMAL_EVENT, MBS.buffer);
		        CheckFireStatusCnt = 0; //счетчик для разрешения анализа пожара, после процессов калибровки и самотестирования
			}
		break;
		case FD2930_STATE_WORKING:
		    //if (CheckFireStatusCnt >= DELAY_CHECK_FIRE_STATUS) CheckFireStatus();          //анализ данных каналов, спустя задержку после процессов самотестирования и калибровки
		    //SetFireStatus();	// else????????????

		    if (cnt < DELAY_1S) {
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
		    		if(DeviceData.IRGain < 150 && postFireCnt == 0) { // если на ИК канале все спокойно то вычисляем уровень шума в УФ и прибавляем часть его к установленному порогу,
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

		        //AppRTCTaskGetTime(); //get current time
		        //AppFD2930Task();  //заполнение буфера для контроллера высокого уровня, управление светодиодами и реле, установка тока

		        if ((!GPIO_READ_PIN(LPC_GPIO4, HALL1)) || (!GPIO_READ_PIN(LPC_GPIO4, HALL2))) {
		        	if (!(DeviceData.Status & FD2930_DEVICE_STATUS_MAGNET)) {
		        		DeviceData.Status |= FD2930_DEVICE_STATUS_MAGNET;
		        		//push_flash_command(FLASH_WRITE_EVENT, EVENT_HALL_ON, MBS.buffer);
		        	}
		        	//ResetFireStatus(); //сброс зафиксированного состояния пожар
		        } else {
		        	if (DeviceData.Status & FD2930_DEVICE_STATUS_MAGNET) {
		        		DeviceData.Status &= ~FD2930_DEVICE_STATUS_MAGNET;
		        		//push_flash_command(FLASH_WRITE_EVENT, EVENT_HALL_OFF, MBS.buffer);
		        	}
		    	}
		        GPIO_SET_PIN(LPC_GPIO2, (UV_TEST));
		        GPIO_SET_PIN(LPC_GPIO0, (UV_TEST2)); //turn off UV test source
		        GPIO_SET_PIN(LPC_GPIO2, IR_TEST);
		    }

		    if (DeviceData.Flags & FD2930_DEVICEFLAGS_DUST) { // если запыленность неаварийная тестируемся  чаще
		    	if (selfTestCnt < (3 * 60 * DELAY_1S)) {
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
		    	if (selfTestCnt < (10 * 60 * DELAY_1S)) {
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
		UV = EWMA_FILTER(UV, DeviceData.UVRaw, avCoeffUV);
	}
	DeviceData.IRAv = EWMA_FILTER(DeviceData.IRAv, DeviceData.IRRaw, avCoeffIR);

	if (DeviceData.IRRaw > DeviceData.IRAv) DeviceData.IRRect = DeviceData.IRRaw - DeviceData.IRAv;
	else DeviceData.IRRect = DeviceData.IRAv - DeviceData.IRRaw;
	IR = EWMA_FILTER(IR, DeviceData.IRRect, avCoeffIR);

	DeviceData.UVGain = UV * 4 * DeviceData.UVCoeff / 10;
	DeviceData.IRGain = IR * 4 * DeviceData.IRCoeff / 10;

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

    
//------------------------------------------------------------------------------
// end
//------------------------------------------------------------------------------



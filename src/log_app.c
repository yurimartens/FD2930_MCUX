/**
  ******************************************************************************
  * @file    log_app.c
  * @author  Yuri Martenstev <yurimartens@gmail.com>
  * @brief   Usage of logs
  ******************************************************************************
  */
#include <log_app.h>
#include <fd2930.h>
#include <fifo.h>
#include <string.h>
#include <stdio.h>


static FIFO_t 	FIFO;
static uint8_t	FIFOMem[LOG_ENTRY_SIZE * LOG_FIFO_ITEMS];
static uint8_t	FIFOTemp[LOG_ENTRY_SIZE];

static FIFO_t 	FIFOLive;
static uint8_t	FIFOMemLive[LOG_ENTRY_SIZE * LOG_FIFO_LIVE_ITEMS];
static uint8_t	FIFOTempLive[LOG_ENTRY_SIZE];

static uint8_t	FileLine[LOG_FILE_LINE_LEN];



/**
  * @brief
  * @param
  * @retval
  */
void LogAppInit()
{
	FIFOInit(&FIFO, FIFOMem, LOG_ENTRY_SIZE, LOG_FIFO_ITEMS);
	FIFOInit(&FIFOLive, FIFOMemLive, LOG_ENTRY_SIZE, LOG_FIFO_LIVE_ITEMS);

	if (LOG_ERROR_NONE == LogInit(&DeviceTime, DeviceData.SerialNumber)) {
		DeviceData.Status |= FD2930_DEVICE_STATUS_SD_CARD;
		ArchLastPage = LogGetEntriesNum();
		DeviceData.ArchLastPageHi = ArchLastPage >> 16;
		DeviceData.ArchLastPageLo = ArchLastPage;
	} else {
		DeviceData.Status &= ~FD2930_DEVICE_STATUS_SD_CARD;
		ArchLastPage = DeviceData.ArchLastPageHi = DeviceData.ArchLastPageLo = 0;
	}
}


/**
  * @brief
  * @param
  * @retval
  */
void LogAppPushData()
{
	if (DeviceData.Status & FD2930_DEVICE_STATUS_SD_CARD) {
		FIFOPush(&FIFO, (uint8_t *)&DeviceData);
	}
}

/**
  * @brief
  * @param
  * @retval
  */
void LogAppPushLiveData()
{
	if (DeviceData.Status & FD2930_DEVICE_STATUS_SD_CARD) {
		FIFOPush(&FIFOLive, (uint8_t *)&DeviceData);
	}
}

/**
  * @brief
  * @param
  * @retval
  */
void LogAppPopAndStoreAllData()
{
	uint16_t status;
	uint32_t regIdx;
	RTC_TIME_Type ct;
	char reason[LOG_EVENT_REASON_LEN + 1];

	while (FIFO_ERROR_NONE == FIFOPop(&FIFO, FIFOTemp)) {	// if SD isn't available FifoPop won't get anything
		regIdx = MB_REG_ADDR(DeviceData, Status);
		status = (FIFOTemp[regIdx * 2 + 1] << 8) | FIFOTemp[regIdx * 2];

		if (status & FD2930_DEVICE_STATUS_FIRE) {
			strcpy(reason, LOG_REASON_FIRE);
			while (1) {
				if (FIFO_ERROR_NONE != FIFOPop(&FIFO, FIFOTempLive)) break;
				regIdx = MB_REG_ADDR(DeviceData, Seconds);		// following regs must be in a strict order
				ct.SEC = FIFOTempLive[regIdx * 2];
				ct.MIN = FIFOTempLive[(++regIdx) * 2];
				ct.HOUR = FIFOTempLive[(++regIdx) * 2];
				ct.DOM = FIFOTempLive[(++regIdx) * 2];
				ct.MONTH = FIFOTempLive[(++regIdx) * 2];
				regIdx++;
				ct.YEAR = (FIFOTempLive[regIdx * 2 + 1] << 8) | FIFOTempLive[regIdx * 2];
				LogWriteEvent(FIFOTempLive, LOG_ENTRY_SIZE, &ct, "");
			}
		} else if (status & FD2930_DEVICE_STATUS_PREFIRE)
	        strcpy(reason, LOG_REASON_PREFIRE);
	    else if ((status & FD2930_DEVICE_STATUS_ALARM_IR) || (status & FD2930_DEVICE_STATUS_ALARM_UV))
	        strcpy(reason, LOG_REASON_ALARM_IRUV);
	    else if (status & FD2930_DEVICE_STATUS_BREAK)
	        strcpy(reason, LOG_REASON_BREAK);
	    else if (status & FD2930_DEVICE_STATUS_FAULT)
	        strcpy(reason, LOG_REASON_FAULT);
	    else if (status & FD2930_DEVICE_STATUS_MAGNET)
	        strcpy(reason, LOG_REASON_MAGNET);
	    else if (status & FD2930_DEVICE_STATUS_SELF_TEST)
	        strcpy(reason, LOG_REASON_SELF_TEST);
	    else strcpy(reason, " ");

		regIdx = MB_REG_ADDR(DeviceData, Seconds);		// following regs must be in a strict order
		ct.SEC = FIFOTemp[regIdx * 2];
		ct.MIN = FIFOTemp[(++regIdx) * 2];
		ct.HOUR = FIFOTemp[(++regIdx) * 2];
		ct.DOM = FIFOTemp[(++regIdx) * 2];
		ct.MONTH = FIFOTemp[(++regIdx) * 2];
		regIdx++;
		ct.YEAR = (FIFOTemp[regIdx * 2 + 1] << 8) | FIFOTemp[regIdx * 2];
		LogWriteEvent(FIFOTemp, LOG_ENTRY_SIZE, &ct, "");

		ArchLastPage = LogGetEntriesNum() - 1;
		DeviceData.ArchLastPageHi = ArchLastPage >> 16;
		DeviceData.ArchLastPageLo = ArchLastPage;
	}
}

/**
  * @brief
  * @param
  * @retval
  */
void LogAppRestoreData()
{
	if (DeviceData.Status & FD2930_DEVICE_STATUS_SD_CARD) {
		ArchLastPage = LogGetEntriesNum() - 1;
		DeviceData.ArchLastPageHi = ArchLastPage >> 16;
		DeviceData.ArchLastPageLo = ArchLastPage;
	    LogReadEvent(ArchPageIdx, FileLine, LOG_FILE_LINE_LEN);
	    LogParseEvent((uint8_t *)DeviceData.Archive.Page, FileLine, LOG_FILE_LINE_LEN);
	    int YY, MM, DD, hh, mm, ss;
	    sscanf((const char *)FileLine, "%02d:%02d:%02d/%02d:%02d:%02d", &DD, &MM, &YY, &hh, &mm, &ss);
	    uint32_t regIdx = MB_REG_ADDR(DeviceData, Seconds);		// following regs must be in a strict order
	    DeviceData.Archive.Page[regIdx] = ss;
	    DeviceData.Archive.Page[regIdx++] = mm;
	    DeviceData.Archive.Page[regIdx++] = hh;
	    DeviceData.Archive.Page[regIdx++] = DD;
	    DeviceData.Archive.Page[regIdx++] = MM;
	    DeviceData.Archive.Page[regIdx++] = YY;

	    regIdx = MB_REG_ADDR(DeviceData, ArchLastPageHi);
	    DeviceData.Archive.Page[regIdx] = DeviceData.ArchLastPageHi;
	    regIdx = MB_REG_ADDR(DeviceData, ArchLastPageLo);
	    DeviceData.Archive.Page[regIdx] = DeviceData.ArchLastPageLo;

	    regIdx = MB_REG_ADDR(DeviceData, ArchPageIdxHi);
	    DeviceData.Archive.Page[regIdx] = DeviceData.ArchPageIdxHi;
	    regIdx = MB_REG_ADDR(DeviceData, ArchPageIdxLo);
	    DeviceData.Archive.Page[regIdx] = DeviceData.ArchPageIdxLo;
	}
}

/**
  * @brief
  * @param
  * @retval
  */
void LogAppErase()
{
	LogErase();
}


//------------------------------------------------------------------------------
// END
//------------------------------------------------------------------------------


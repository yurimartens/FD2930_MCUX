/**
  ******************************************************************************
  * @file    log_app.h
  * @author  Yuri Martenstev <yurimartens@gmail.com>
  * @brief   Usage of logs
  ******************************************************************************
  */
#ifndef __LOG_APP_H
#define __LOG_APP_H

#ifdef __cplusplus
extern "C" {
#endif

#include <diskio.h>
#include <log.h>
#include <fifo.h>



#define LOG_ENTRY_SIZE					100
#define LOG_FIFO_ITEMS					8
#define LOG_FIFO_LIVE_ITEMS				30
#define LOG_FILE_LINE_LEN				170		// size of line, see HeaderLen in LogInit()


void LogAppInit();
void LogAppPushData();
void LogAppPushLiveData();
void LogAppPopAndStoreAllData();
void LogAppRestoreData(uint8_t bin);
void LogAppErase();

#ifdef __cplusplus
}

#endif

#endif //__LOG_APP_H

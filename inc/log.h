/**
  ******************************************************************************
  * @file    log.h
  * @author  Yuri Martenstev <yurimartens@gmail.com>
  * @brief   
  ******************************************************************************
  */ 
#ifndef __LOG_H
#define __LOG_H

#ifdef __cplusplus
extern "C" {
#endif


#include <stdint.h>
#include "lpc17xx_rtc.h"


//#define PPCAT_NX(A, B) A ## B

/*
 * Concatenate preprocessor tokens A and B after macro-expanding them.
 */
//#define PPCAT(A, B) PPCAT_NX(A, B)

//#define STRINGIZE_NX(A) #A
/*
 * Turn A into a string literal after macro-expanding it.
 */
//#define STRINGIZE(A) STRINGIZE_NX(A)

//#define FORMAT_STRING(WIDTH, TYPE)  % ## PPCAT_NX(WIDTH, TYPE)

#define FILEPATH_MAX_LEN        30
#define LOG_LOGICAL_DRIVE       "0:"
#define LOG_EVENT_DIR_NAME      "fd2960_logs"
#define LOG_EVENT_FILE_NAME     "events.log"

//#define LOG_EVENT_FILE_PATH     PPCAT(LOG_LOGICAL_DRIVE, LOG_EVENT_FILE_NAME)
//#define LOG_PARAMETER_FILE_PATH PPCAT(LOG_LOGICAL_DRIVE, LOG_PARAMETER_FILE_NAME)
     
#define LOG_EVENT_TIME_LEN      17     // see the actual size of time format in write proc
#define LOG_EVENT_TIME_LEN_Q    "%17s"
#define LOG_EVENT_REASON_LEN    8     
#define LOG_EVENT_REASON_LEN_Q  "%8s"
#define LOG_EVENT_ITEM_LEN      8
#define LOG_EVENT_ITEM_LEN_Q    "%8d"
#define LOG_EVENT_ITEM_LEN_H_Q  "%8s"
#define LOG_EVENT_EMPTY_LEN_Q   "%8s"

#define LOG_EVENT_TIME_HEADER   "Date/Time"
#define LOG_EVENT_REASON_HEADER "Reason"
     
#define LOG_FILE_STRING_LEN     300
     
#define LOG_COL_DELIMITER       " "

#define LOG_REASON_MAGNET       "Magnet"
#define LOG_REASON_SELF_TEST    "Test"
#define LOG_REASON_ALARM_IRUV   "Al_IRUV"
#define LOG_REASON_BREAK        "Break"
#define LOG_REASON_FAULT        "Fault"
#define LOG_REASON_PREFIRE      "Prefire"
#define LOG_REASON_FIRE         "Fire"
     
typedef enum {
    LOG_COL_DT_UINT16 = 0,
    LOG_COL_DT_INT16,
    LOG_COL_DT_UINT32,
    LOG_COL_DT_FLOAT,
} LogColumnDataType_t;
     
typedef struct {
    uint16_t            MBAddr;
    LogColumnDataType_t Type;
    char                Header[LOG_EVENT_ITEM_LEN];
} LogEventColumn_t;

typedef enum {
    LOG_ERROR_NONE = 0,
    LOG_ERROR_HW,
    LOG_ERROR_FS,
    LOG_ERROR_NO_SPACE,
    LOG_ERROR_BUF,
    LOG_ERROR_ENTRY_NUM,
} LogError_t;

LogError_t LogInit(RTC_TIME_Type *time, uint16_t sn);
void LogChangeHeaderSN(uint16_t sn);    
LogError_t LogWriteEvent(uint8_t *buf, uint16_t bufSize, RTC_TIME_Type *time, char *reason);
LogError_t LogWriteParameter(uint8_t *buf, uint16_t size);
LogError_t LogReadEvent(uint32_t, uint8_t *, uint16_t);
LogError_t LogParseEvent(uint8_t *, uint8_t *, uint16_t);
LogError_t LogErase(void);

uint32_t LogGetEntriesNum(void);

#ifdef __cplusplus
}
#endif

#endif //LOG_H

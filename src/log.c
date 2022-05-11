/**
  ******************************************************************************
  * @file    log.c
  * @author  Yuri Martenstev <yurimartens@gmail.com>
  * @brief   
  ******************************************************************************
  */ 
#include <log.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include <ff.h>

static FATFS fs;
static BYTE work[FF_MAX_SS];         // FS Working buffer

static uint16_t    LogEventColumns;
static uint32_t    HeaderLen, EntryLen;
static uint32_t    Entries;

static uint16_t    SN;
static uint8_t     SNChanged = 0;


// see MBS.h
const LogEventColumn_t    LogEventColumn[] = {
    {.MBAddr = 6,
     .Type = LOG_COL_DT_UINT16,
     .Header = "Stat"
    },
    {.MBAddr = 7,
     .Type = LOG_COL_DT_UINT16,
     .Header = "Flags"
    },
    {.MBAddr = 8,
     .Type = LOG_COL_DT_UINT16,
     .Header = "Config"
    },
    {.MBAddr = 9,
     .Type = LOG_COL_DT_UINT16,
     .Header = "UV"
    },
    {.MBAddr = 10,
     .Type = LOG_COL_DT_UINT16,
     .Header = "IR"
    },
    {.MBAddr = 12,
     .Type = LOG_COL_DT_UINT16,
     .Header = "UV_Thr"
    },
    {.MBAddr = 13,
     .Type = LOG_COL_DT_UINT16,
     .Header = "IR_Thr"
    },
    {.MBAddr = 14,
     .Type = LOG_COL_DT_UINT16,
     .Header = "IR_Sens"
    },
    {.MBAddr = 15,
     .Type = LOG_COL_DT_UINT16,
     .Header = "UV_Sens"
    },    
    {.MBAddr = 21,
     .Type = LOG_COL_DT_INT16,
     .Header = "Temp"
    },
    {.MBAddr = 22,
     .Type = LOG_COL_DT_UINT16,
     .Header = "UVPwr"
    },
};


static LogError_t CreateFileEventAndHeader(uint16_t sn);

/**
  * @brief
  * @param  
  * @param  
  * @param  
  * @retval  
  */
LogError_t LogInit(RTC_TIME_Type *time, uint16_t sn)
{
    uint8_t YY, MM, DD, hh, mm, ss; 
    
    SN = sn;
    
    LogEventColumns = sizeof(LogEventColumn) / sizeof(LogEventColumn_t);
    HeaderLen = LOG_EVENT_TIME_LEN + strlen(LOG_COL_DELIMITER) + LOG_EVENT_REASON_LEN + strlen(LOG_COL_DELIMITER) + LogEventColumns * (LOG_EVENT_ITEM_LEN + strlen(LOG_COL_DELIMITER)) + 2; // +2 crlf
    
    EntryLen = HeaderLen;
    YY = time->YEAR % 100;
    MM = time->MONTH;
    DD = time->DOM;
    hh = time->HOUR;
    mm = time->MIN;
    ss = time->SEC;
    FILINFO fno;
    fno.fdate = (WORD)(((YY - 1980) * 512U) | MM * 32U | DD);
    fno.ftime = (WORD)(hh * 2048U | mm * 32U | ss / 2U);
    
    FRESULT fr = f_mount(&fs, LOG_LOGICAL_DRIVE, 1);  
  
    if (FR_OK == fr)
    {        
        FATFS *pfs = &fs;
        DWORD size = 0;
        if (FR_OK != f_getfree(LOG_LOGICAL_DRIVE, &size, &pfs)) return LOG_ERROR_FS;
        if (0 == size) return LOG_ERROR_NO_SPACE;
        
        if (FR_OK != f_chdir(LOG_EVENT_DIR_NAME))
        {
            if (FR_OK != f_mkdir(LOG_EVENT_DIR_NAME)) return LOG_ERROR_FS;
            if (FR_OK != f_chdir(LOG_EVENT_DIR_NAME)) return LOG_ERROR_FS;                        
            f_utime(LOG_EVENT_DIR_NAME, &fno);
        }        
        if (LOG_ERROR_NONE != CreateFileEventAndHeader(sn)) return LOG_ERROR_FS;
    }
    else
    {
        if (FR_NO_FILESYSTEM == fr)
        {
            if (FR_OK != f_mkfs(LOG_LOGICAL_DRIVE, 0, work, sizeof(work))) return LOG_ERROR_FS; // Create FAT volume on the logical drive
            fr = f_mount(&fs, LOG_LOGICAL_DRIVE, 1);
            if (FR_OK == fr)
            {
                if (FR_OK != f_chdir(LOG_EVENT_DIR_NAME))
                {
                    if (FR_OK != f_mkdir(LOG_EVENT_DIR_NAME)) return LOG_ERROR_FS;
                    if (FR_OK != f_chdir(LOG_EVENT_DIR_NAME)) return LOG_ERROR_FS;                             
                    f_utime(LOG_EVENT_DIR_NAME, &fno);
                }
                if (LOG_ERROR_NONE != CreateFileEventAndHeader(sn)) return LOG_ERROR_FS;
            }
            else
            {
                return LOG_ERROR_FS; // TODO: parse res to detail an error
            }
        }
        else
        {
            return LOG_ERROR_HW; // TODO: parse res to detail an error
        }
    } 
    
    return LOG_ERROR_NONE;  
}
/**
  * @brief
  * @param  
  * @param  
  * @param  
  * @retval  
  */
void LogChangeHeaderSN(uint16_t sn)
{
    SN = sn;
    SNChanged = 1;
}

/**
  * @brief
  * @param  
  * @param  
  * @param  
  * @retval  
  */
LogError_t LogWriteEvent(uint16_t *buf, uint16_t bufSize, RTC_TIME_Type *time, char *reason)
{ 
    FIL f;
    uint16_t mbSize = bufSize / 2;
    int32_t value;
    char    str[LOG_EVENT_TIME_LEN + 1]; // +1 NULL // Attention! its len must be the biggest of all elements
    uint8_t YY, MM, DD, hh, mm, ss; 
        
    FRESULT fr = f_open(&f, LOG_EVENT_FILE_NAME, FA_READ | FA_WRITE | FA_OPEN_APPEND);
    if (FR_OK == fr)
    {
        YY = time->YEAR % 100;
        MM = time->MONTH;
        DD = time->DOM;
        hh = time->HOUR;
        mm = time->MIN;
        ss = time->SEC;
        FILINFO fno;
        fno.fdate = (WORD)(((YY - 1980) * 512U) | MM * 32U | DD);
        fno.ftime = (WORD)(hh * 2048U | mm * 32U | ss / 2U);
        
        sprintf(str, "%02d:%02d:%02d/%02d:%02d:%02d", DD, MM, YY, hh, mm, ss);   // format length tied to LOG_EVENT_TIME_LEN 
        f_puts(str, &f);	
        f_puts(LOG_COL_DELIMITER, &f);	
        
        sprintf(str, LOG_EVENT_REASON_LEN_Q, reason);
        f_puts(str, &f);	
        f_puts(LOG_COL_DELIMITER, &f);	        
        
        for (uint8_t i = 0; i < LogEventColumns; i++)
        {            
            if (LogEventColumn[i].MBAddr >= mbSize) return LOG_ERROR_BUF; 
            switch ((uint8_t)LogEventColumn[i].Type) 
            {
                case LOG_COL_DT_UINT16:
                    value = (uint16_t)buf[LogEventColumn[i].MBAddr];
                    sprintf(str, LOG_EVENT_ITEM_LEN_Q, (int)value);
                break;
                case LOG_COL_DT_INT16:
                    value = (int16_t)buf[LogEventColumn[i].MBAddr];
                    sprintf(str, LOG_EVENT_ITEM_LEN_Q, (int)value);
                break;
                case LOG_COL_DT_UINT32:
                    value = (buf[LogEventColumn[i].MBAddr] << 16) | buf[LogEventColumn[i].MBAddr + 1];
                    sprintf(str, LOG_EVENT_ITEM_LEN_Q, (int)value);
                break;
                default:
                    sprintf(str, LOG_EVENT_EMPTY_LEN_Q, "-");
            }
            f_puts(str, &f);	
            f_puts(LOG_COL_DELIMITER, &f);	
        }
        f_puts("\r\n", &f);	        
        if (FR_OK != f_utime(LOG_EVENT_FILE_NAME, &fno)) return LOG_ERROR_FS;        
        
        Entries++;
        
        if (SNChanged) {
            SNChanged = 0;
            FSIZE_t temp = f_tell(&f);
            f_lseek(&f, 0);
            // put an id information about device
            sprintf(str, LOG_ID_LINE_FORMAT, SN);
            f_puts(str, &f);    
            f_puts("\r\n", &f);	
            f_lseek(&f, temp);
        }        
        
        if (FR_OK != f_close(&f)) return LOG_ERROR_FS;
    }
    else
    {
        return LOG_ERROR_FS; // TODO: parse res to detail an error
    }     
    return LOG_ERROR_NONE;
}

/**
  * @brief
  * @param  
  * @param  
  * @param  
  * @retval  
  */
LogError_t LogWriteParameter(uint16_t *buf, uint16_t size)
{
    return LOG_ERROR_NONE;
}

/**
  * @brief
  * @param  
  * @param  
  * @param  
  * @retval  
  */
LogError_t LogReadHeader(uint8_t *buf)
{
    FIL f;

    FRESULT fr = f_open(&f, LOG_EVENT_FILE_NAME, FA_READ | FA_OPEN_EXISTING);
    if (FR_OK == fr)
    {
        f_gets((char *)buf, HeaderLen, &f);
        if (FR_OK != f_close(&f)) return LOG_ERROR_FS;
    }
    else
    {
        return LOG_ERROR_FS; // TODO: parse res to detail an error
    }
    return LOG_ERROR_NONE;
}

/**
  * @brief
  * @param
  * @param
  * @param
  * @retval
  */
LogError_t LogReadEvent(uint32_t entryNum, uint8_t *buf, uint16_t bufSize)
{ 
    FIL f;
    
    if (entryNum >= Entries) return LOG_ERROR_ENTRY_NUM;
    if (EntryLen > bufSize)  return LOG_ERROR_BUF;
        
    FRESULT fr = f_open(&f, LOG_EVENT_FILE_NAME, FA_READ | FA_OPEN_EXISTING);
    if (FR_OK == fr)
    {
        if (FR_OK != f_lseek(&f, LOG_ID_LINE_LEN + HeaderLen + (entryNum * EntryLen))) return LOG_ERROR_FS;
        f_gets((char *)buf, EntryLen, &f);
        
        if (FR_OK != f_close(&f)) return LOG_ERROR_FS;
    }
    else
    {
        return LOG_ERROR_FS; // TODO: parse res to detail an error
    }     
    return LOG_ERROR_NONE;
}

/**
  * @brief
  * @param  
  * @param  
  * @param  
  * @retval  
  */
LogError_t LogParseEvent(uint16_t *outBuf, uint8_t *inBuf, uint16_t bufSize)
{ 
    uint16_t mbSize = bufSize / 2;
    int value;
    uint32_t offset = LOG_EVENT_TIME_LEN + strlen(LOG_COL_DELIMITER) + LOG_EVENT_REASON_LEN + strlen(LOG_COL_DELIMITER);            
        
    for (uint8_t i = 0; i < LogEventColumns; i++)
    {            
        if (LogEventColumn[i].MBAddr >= mbSize) return LOG_ERROR_BUF; 
        switch ((uint8_t)LogEventColumn[i].Type) 
        {            
            case LOG_COL_DT_UINT16:
            case LOG_COL_DT_INT16:
                sscanf((const char *)&inBuf[offset], LOG_EVENT_ITEM_LEN_Q, &value);
                outBuf[LogEventColumn[i].MBAddr] = value;
            break;
            case LOG_COL_DT_UINT32:
                sscanf((const char *)&inBuf[offset], LOG_EVENT_ITEM_LEN_Q, &value);
                outBuf[LogEventColumn[i].MBAddr] = (value >> 16);
                outBuf[LogEventColumn[i].MBAddr + 1] = value;
            break;
            default:
                outBuf[LogEventColumn[i].MBAddr] = 0;
        }
        offset += LOG_EVENT_ITEM_LEN + strlen(LOG_COL_DELIMITER);        
    }        
    return LOG_ERROR_NONE;
}

/**
  * @brief
  * @param  
  * @param  
  * @param  
  * @retval  
  */
LogError_t LogErase(void)
{
    char filename[FILEPATH_MAX_LEN];
    DIR dir;
    
    f_chdir("/");
    if (FR_OK == f_opendir(&dir, LOG_EVENT_DIR_NAME))
    {
        FRESULT fr = FR_OK;
        FILINFO fno;
        while (1)
        {
            fr = f_readdir(&dir, &fno);
            if ((fr != FR_OK) || (fno.fname[0] == 0)) break;
            if ((fno.fattrib & AM_DIR) == 0) // not a dir
            {
                strcpy(filename, LOG_EVENT_DIR_NAME);
                sprintf(&filename[strlen(filename)], "/%s", fno.fname);
                f_unlink(filename);
            }
        }
        f_closedir(&dir);
        Entries = 0;
    }  
    if (FR_OK == f_chdir(LOG_EVENT_DIR_NAME)) CreateFileEventAndHeader(SN);
    else return LOG_ERROR_FS;
    
    return LOG_ERROR_NONE;
}
   
/**
  * @brief
  * @param  
  * @param  
  * @param  
  * @retval  
  */
uint32_t LogGetEntriesNum(void)
{
    return Entries;
}

/**
  * @brief
  * @param  
  * @param  
  * @param  
  * @retval  
  */
static LogError_t CreateFileEventAndHeader(uint16_t sn)
{
    FIL f;
    char    str[LOG_EVENT_TIME_LEN + 1];   // +1 NULL 
    
    if (FR_OK != f_open(&f, LOG_EVENT_FILE_NAME, FA_READ | FA_WRITE | FA_OPEN_ALWAYS)) return LOG_ERROR_FS; 
    if (0 == f_size(&f))
    {   
        // put an id information about device
        sprintf(str, LOG_ID_LINE_FORMAT, sn);
        f_puts(str, &f);    
        f_puts("\r\n", &f);	
        // put a log-file header of the table
        sprintf(str, LOG_EVENT_TIME_LEN_Q, LOG_EVENT_TIME_HEADER);
        f_puts(str, &f);	
        f_puts(LOG_COL_DELIMITER, &f);
        
        sprintf(str, LOG_EVENT_REASON_LEN_Q, LOG_EVENT_REASON_HEADER);
        f_puts(str, &f);	
        f_puts(LOG_COL_DELIMITER, &f);
        
        for (uint8_t i = 0; i < LogEventColumns; i++)
        {
            sprintf(str, LOG_EVENT_ITEM_LEN_H_Q, LogEventColumn[i].Header);
            f_puts(str, &f);	
            f_puts(LOG_COL_DELIMITER, &f);	
        }
        f_puts("\r\n", &f);	
    } 
    Entries = (f_size(&f) - LOG_ID_LINE_LEN - HeaderLen) / EntryLen;
        
    if (FR_OK != f_close(&f)) return LOG_ERROR_FS;
    
    return LOG_ERROR_NONE;  
}

//------------------------------------------------------------------------------
// END
//------------------------------------------------------------------------------

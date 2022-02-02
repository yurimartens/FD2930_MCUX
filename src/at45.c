
#include "at45.h"
#include "log.h"

#if 0
static INT16U address_home_page = PARAMETER_ADDRESS_1;

void (*flash_work[])(INT32U, unsigned char const*) =  //������� ������ � ����-�������
{
  write_parameters_SD,
  read_archive,
  write_event
};

flash_queue_tag_t flash_queue;
flash_queue_live_t flash_queue_live;

void AppAT45TaskInit(void)
{
  if (NVOL_Init()) 
  {
    read_and_parse_parameters();
    fd2930.device_status |= FD2930_DEVICE_STATUS_FLASH_OK;
  }
  else fd2930.device_status &= ~FD2930_DEVICE_STATUS_FLASH_OK; /* Failure */  
}

void push_flash_command(INT16U function, INT16U parameter, const INT8U *buffer)
{
  if(fd2930.device_status & FD2930_DEVICE_STATUS_SD_CARD)
  {
    flash_queue.function[flash_queue.sloc] = function;
    flash_queue.parameter[flash_queue.sloc] = parameter;
    memcpy(flash_queue.buffer[flash_queue.sloc], buffer, QUEUE_ELEMENT_LENGTH);
    flash_queue.sloc++;
    if(flash_queue.sloc == QUEUE_LENGHT) flash_queue.sloc = 0; 
    __no_operation();
  }
}

void push_live_data(void)
{
  if(fd2930.device_status & FD2930_DEVICE_STATUS_SD_CARD)
  {
    memcpy(flash_queue_live.buffer[flash_queue_live.sloc], MBS.buffer, QUEUE_ELEMENT_LENGTH);
    flash_queue_live.sloc++;
    if (flash_queue_live.newly < QUEUE_LIVE_LENGHT) flash_queue_live.newly++;
    if (flash_queue_live.sloc == QUEUE_LIVE_LENGHT) {
        flash_queue_live.sloc = 0; 
        flash_queue_live.rloc = 1; 
    } else {
        if (flash_queue_live.sloc == QUEUE_LIVE_LENGHT - 1) {
            flash_queue_live.rloc = 0; 
        } else {
            flash_queue_live.rloc = flash_queue_live.sloc + 1;
        }
    }
  }
}

INT8U *pop_live_data(void)
{
    INT8U *ret = 0;
  
    if(fd2930.device_status & FD2930_DEVICE_STATUS_SD_CARD) {
        if ((flash_queue_live.sloc != flash_queue_live.rloc) && (flash_queue_live.newly)){
            ret = (INT8U *)flash_queue_live.buffer[flash_queue_live.rloc];
            flash_queue_live.rloc++;
            flash_queue_live.newly--;
            if (flash_queue_live.rloc == QUEUE_LIVE_LENGHT) {
                flash_queue_live.rloc = 0;
            }
        } 
    }
    return ret;
}

void read_and_parse_parameters(void)
{
  FD2930 fd2930_test = {0};
  INT8U fd2930_buffer[sizeof(fd2930)];

  if(NVOL_GetVariable(NVRAMLOGBUF, fd2930_buffer, sizeof(fd2930_buffer)))
  {
    memcpy((void *) &fd2930_test, fd2930_buffer, sizeof(fd2930_buffer));
  }  
  fd2930_test.device_state = FD2930_STATE_STARTING1;  
  fd2930_test.firmvare_version = FD2930_FIRMWARE_VERSION;
  fd2930_test.device_type = FD2930_DEVICE_TYPE;
  fd2930_test.board_number = FD2930_BOARD_NUMBER;     
  
  if(fd2930_test.device_config == 0 || (fd2930_test.modbus_number < 1 || fd2930_test.modbus_number > 247) || \
      (fd2930_test.heat_power > FD2930_MAX_HEATPOWER || fd2930_test.heat_power < FD2930_MIN_HEATPOWER) || \
        (fd2930_test.baudrate != 1 && fd2930_test.baudrate != 2 && fd2930_test.baudrate != 4 && fd2930_test.baudrate != 8 && fd2930_test.baudrate != 12 && fd2930_test.baudrate != 16 && fd2930_test.baudrate != 24))
    
  {
    fd2930_test.device_config = FD2930_DEFAULT_DEVICE_CONFIG;
    AppRTCTaskInit();
    fd2930_test.block_service = 5813;
    fd2930_test.serial_number = 0;
    fd2930_test.device_status |= FD2930_DEVICE_STATUS_BREAK;
    fd2930_test.modbus_number = FD2930_DEF_MBS_ADR;
    fd2930_test.baudrate = FD2930_DEF_MBS_BAUD;
    fd2930_test.worked_time = 0;
    fd2930_test.heat_power = FD2930_DEFAULT_HEATPOWER;
    fd2930_test.thres_heater = FD2930_DEFAULT_TRES_HEATER;
    
    fd2930_test.cnt_autorecovery = 1;//����� �� �������� ������� ������������������
    
  }else
  {
    fd2930_test.cnt_autorecovery=0;
  } 
  
  if (fd2930_test.device_config & FD2930_DEVICECONFIG_IPES_MB_HEADER) {  
      if (fd2930_test.baudrate != 1 && fd2930_test.baudrate != 2 && fd2930_test.baudrate != 4 && fd2930_test.baudrate != 8 && fd2930_test.baudrate != 16) {
          fd2930_test.baudrate = IPES_DEF_MBS_BAUD;
      }
  } else {
      if (fd2930_test.baudrate != 1 && fd2930_test.baudrate != 2 && fd2930_test.baudrate != 4 && fd2930_test.baudrate != 12 && fd2930_test.baudrate != 24) {
          fd2930_test.baudrate = FD2930_DEF_MBS_BAUD;
      }
  }
  if(fd2930_test.fft_gain > FD2930_MAX_GAIN_FFT || fd2930_test.fft_gain < FD2930_MIN_GAIN_FFT) fd2930_test.fft_gain = FD2930_DEFAULT_GAIN_FFT;
  if(fd2930_test.thres_IR > FD2930_MAX_TRES_IR || fd2930_test.thres_IR < FD2930_MIN_TRES_IR) fd2930_test.thres_IR = FD2930_DEFAULT_TRES_IR;
  if(fd2930_test.thres_UV > FD2930_MAX_TRES_UV || fd2930_test.thres_UV < FD2930_MIN_TRES_UV) fd2930_test.thres_UV = FD2930_DEFAULT_TRES_UV;
  if(fd2930_test.K_IR > FD2930_MAX_K_IR || fd2930_test.K_IR < FD2930_MIN_K_IR) fd2930_test.K_IR = FD2930_DEFAULT_K_IR;
  fd2930_K_IR = fd2930_test.K_IR;
  if(fd2930_test.K_UV > FD2930_MAX_K_UV || fd2930_test.K_UV < FD2930_MIN_K_UV) fd2930_test.K_UV = FD2930_DEFAULT_K_UV;
  fd2930_K_UV = fd2930_test.K_UV;
  if(fd2930_test.wait_fault > FD2930_MAX_WAIT_FAULT || fd2930_test.wait_fault < FD2930_MIN_WAIT_FAULT) fd2930_test.wait_fault = FD2930_DEFAULT_WAIT_FAULT;
  if(fd2930_test.wait_fire > FD2930_MAX_WAIT_FIRE || fd2930_test.wait_fire < FD2930_MIN_WAIT_FIRE) fd2930_test.wait_fire = FD2930_DEFAULT_WAIT_FIRE;
  
  fd2930_test.archive_last_page = 1;
  fd2930_test.chosen_archive_page = fd2930_test.archive_last_page - 1;
  
  fd2930 = fd2930_test;
  fillMBSData();
  write_parameters();  
}

void write_parameters(void)
{
  INT8U fd2930_buffer[sizeof(fd2930)];
  
  memcpy(fd2930_buffer, (void *) &fd2930, sizeof(fd2930));
  if(!NVOL_SetVariable(NVRAMLOGBUF, fd2930_buffer, sizeof(fd2930)))
  {
    fd2930.device_status &= ~FD2930_DEVICE_STATUS_FLASH_OK; /* Failure */
  }
//  push_flash_command(FLASH_WRITE_PARAMETERS, 1, NULL);
}

//���� �������� ������ ���������� �� ����� �������� SD �����
//address - ����� �������� ������, ��������� � ����, FD2930_ADDR_ARCHIVEPAGENUMBER
//����� ��������� �������� ������ �������� �� � Modbus �����, ������� � ������ FD2930_ADDR_ARCHIVEPAGE
void read_archive(INT32U address, unsigned char const* b)
{
  INT8U buffer[LOG_FILE_STRING_LEN];
  fd2930.archive_last_page = LogGetEntriesNum();
  if(fd2930.device_status & FD2930_DEVICE_STATUS_SD_CARD) 
  {
    LogReadEvent(fd2930.chosen_archive_page, buffer, LOG_FILE_STRING_LEN);
    LogParseEvent(MBS.buffer + FD2930_ADDR_ARCHIVEPAGE * 2, buffer, LOG_FILE_STRING_LEN);
    uint32_t YY, MM, DD, hh, mm, ss;
    sscanf((const char *)buffer, "%02d:%02d:%02d/%02d:%02d:%02d", &DD, &MM, &YY, &hh, &mm, &ss);
    MBS.buffer[FD2930_ADDR_ARCHIVEPAGE * 2 + FD2930_ADDR_DATE * 2 + 1] = DD;
    MBS.buffer[FD2930_ADDR_ARCHIVEPAGE * 2 + FD2930_ADDR_MONTH * 2 + 1] = MM;
    MBS.buffer[FD2930_ADDR_ARCHIVEPAGE * 2 + FD2930_ADDR_YEAR * 2 + 1] = YY;
    MBS.buffer[FD2930_ADDR_ARCHIVEPAGE * 2 + FD2930_ADDR_HOUR * 2 + 1] = hh;
    MBS.buffer[FD2930_ADDR_ARCHIVEPAGE * 2 + FD2930_ADDR_MINUTE * 2 + 1] = mm;
    MBS.buffer[FD2930_ADDR_ARCHIVEPAGE * 2 + FD2930_ADDR_SECOND * 2 + 1] = ss;
    //memcpy(MBS.buffer + FD2930_ADDR_ARCHIVEPAGE * 2, buffer, 512);
  }
  MBS.buffer[FD2930_ADDR_PAGE_NUMBER_H * 2 + FD2930_ADDR_ARCHIVEPAGE * 2] = fd2930.archive_last_page >> 24;        
  MBS.buffer[FD2930_ADDR_PAGE_NUMBER_H * 2 + 1 + FD2930_ADDR_ARCHIVEPAGE * 2] = fd2930.archive_last_page >> 16;    
  MBS.buffer[FD2930_ADDR_PAGE_NUMBER_L * 2 + FD2930_ADDR_ARCHIVEPAGE * 2] = fd2930.archive_last_page >> 8;        
  MBS.buffer[FD2930_ADDR_PAGE_NUMBER_L * 2 + 1 + FD2930_ADDR_ARCHIVEPAGE * 2] = fd2930.archive_last_page;
  MBS.buffer[FD2930_ADDR_ARCH_PAGE_H * 2 + FD2930_ADDR_ARCHIVEPAGE * 2] = fd2930.chosen_archive_page >> 24;
  MBS.buffer[FD2930_ADDR_ARCH_PAGE_H * 2 + 1 + FD2930_ADDR_ARCHIVEPAGE * 2] = fd2930.chosen_archive_page >> 16; 
  MBS.buffer[FD2930_ADDR_ARCH_PAGE_L * 2 + FD2930_ADDR_ARCHIVEPAGE * 2] = fd2930.chosen_archive_page >> 8;
  MBS.buffer[FD2930_ADDR_ARCH_PAGE_L * 2 + 1 + FD2930_ADDR_ARCHIVEPAGE * 2] = fd2930.chosen_archive_page;   
}

//����� ������� � �����
//����� EVENT � ���������, ��������� ����� ������ �� ������������ ��������, 
//���� �� ������� �� ������, ����������� ����� ������ � ����� � ���������,
//���� �������� ���������� ����� ������ �� 0 � ����� � ���������
//����� �� ���� Modbus ����� � 0 �� QUEUE_ELEMENT_LENGTH �����
void write_event(INT32U event, const INT8U *buffer)
{
  INT8U ibuffer[QUEUE_ELEMENT_LENGTH];
  char reason[LOG_EVENT_REASON_LEN + 1];
  INT16U status = 0;
  RTC_TIME_Type ct;
  INT8U *pBuf;
  
  if(fd2930.device_status & FD2930_DEVICE_STATUS_SD_CARD) 
  {         
    memcpy(ibuffer, buffer, QUEUE_ELEMENT_LENGTH);
    status = (ibuffer[2 * FD2930_ADDR_DEVICESTATUS] << 8) | ibuffer[2 * FD2930_ADDR_DEVICESTATUS + 1];
    
    if (status & FD2930_DEVICE_STATUS_FIRE) {        
        strcpy(reason, LOG_REASON_FIRE);
        while (1) {
            pBuf = pop_live_data();
            if (pBuf == 0) break;
            ct.SEC = pBuf[FD2930_ADDR_SECOND * 2 + 1];
            ct.MIN = pBuf[FD2930_ADDR_MINUTE * 2 + 1];
            ct.HOUR = pBuf[FD2930_ADDR_HOUR * 2 + 1];
            ct.DOM = pBuf[FD2930_ADDR_DATE * 2 + 1];
            ct.MONTH = pBuf[FD2930_ADDR_MONTH * 2 + 1];
            ct.YEAR = (pBuf[FD2930_ADDR_YEAR * 2] << 8) | pBuf[FD2930_ADDR_YEAR * 2 + 1];
            LogWriteEvent(pBuf, QUEUE_ELEMENT_LENGTH, &ct, "");
        }         
    }
    else if (status & FD2930_DEVICE_STATUS_PREFIRE) 
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
    
    ct.SEC = ibuffer[FD2930_ADDR_SECOND * 2 + 1];
    ct.MIN = ibuffer[FD2930_ADDR_MINUTE * 2 + 1];
    ct.HOUR = ibuffer[FD2930_ADDR_HOUR * 2 + 1];
    ct.DOM = ibuffer[FD2930_ADDR_DATE * 2 + 1];
    ct.MONTH = ibuffer[FD2930_ADDR_MONTH * 2 + 1];
    ct.YEAR = (ibuffer[FD2930_ADDR_YEAR * 2] << 8) | ibuffer[FD2930_ADDR_YEAR * 2 + 1];
    
    LogWriteEvent(ibuffer, QUEUE_ELEMENT_LENGTH, &ct, reason);
    fd2930.archive_last_page = LogGetEntriesNum();
  }
  push_flash_command(FLASH_WRITE_PARAMETERS, 1, NULL);
}

void write_parameters_SD(INT32U param, unsigned char const* buffer)
{
  INT8U fd2930_buffer[524];

  if(fd2930.device_status & FD2930_DEVICE_STATUS_SD_CARD) 
  {
//    if(address_home_page == PARAMETER_ADDRESS_2) address_home_page = PARAMETER_ADDRESS_1;
//    else address_home_page = PARAMETER_ADDRESS_2;
    
    memcpy(fd2930_buffer, (void *) &fd2930, sizeof(fd2930));
    LogWriteParameter(fd2930_buffer, 524);
    //SD_WriteSector (address_home_page, fd2930_buffer, 1);
  }
}

void erase_SD(void)
{
  if(fd2930.device_status & FD2930_DEVICE_STATUS_SD_CARD) 
  {  
    //SD_WriteSector (PARAMETER_ADDRESS_1, ibuffer, 1);
    //SD_WriteSector (PARAMETER_ADDRESS_2, ibuffer, 1);
  }
  //read_parameters_SD();
}

void erase_archive(void)
{
  fd2930.device_state_flag |= FD2930_STATE_FLAG_ERASE_ARCHIVE;  
}

//void erase_parameters(void)
//{
//  eraseFlash();         //erase internal flash memory
//  //read_and_parse_parameters();  //set default parameters to start device
//  eraseFlash();         //erase internal flash memory
//  //read_and_parse_parameters();  //set default parameters to start devic
//  eraseFlash();         //erase internal flash memory
//  eraseFlash();         //erase internal flash memory
//  fd2930.device_status &= ~FD2930_DEVICE_STATUS_FLASH_OK;//������� ���� ����������� ����, �������� ����������� ��� ������, ��� ����� ����������� ����� �������
//  
//  //erase_SD();      //erase SD card
//  //read_and_parse_parameters();  //set default parameters to start device
//  //read_parameters_SD();
//  //Command = 8;
//}

void read_parameters_SD()
{
  FD2930 fd2930_test = {0};
  INT8U fd2930_buffer[524];
  
  if(fd2930.device_status & FD2930_DEVICE_STATUS_SD_CARD) 
  { 
    fd2930.archive_last_page = LogGetEntriesNum();
    //SD_ReadSector(address_home_page, fd2930_buffer, 1);
    memcpy((void *) &fd2930_test, fd2930_buffer, sizeof(fd2930_test));  
    
    if(fd2930_test.archive_last_page > ARCHIVE_LENGHT || !fd2930_test.archive_last_page) fd2930_test.archive_last_page = 1;
    fd2930_test.chosen_archive_page = fd2930_test.archive_last_page - 1;
    fd2930.archive_last_page = fd2930_test.archive_last_page;
    fd2930.chosen_archive_page = fd2930_test.chosen_archive_page;
    fd2930.worked_time = fd2930_test.worked_time;
    fillMBSData();
  }
}

//void restore_parameters_from_SD(void)
//{
//  FD2930 fd2930_test = {0};
//  INT8U fd2930_buffer[524];
//  
//  if(fd2930.device_status & FD2930_DEVICE_STATUS_SD_CARD) 
//  { 
//    SD_ReadSector(address_home_page, fd2930_buffer, 1);
//    memcpy((void *) &fd2930_test, fd2930_buffer, sizeof(fd2930_test));  
//    
//    if(fd2930_test.modbus_number < 1 || fd2930_test.modbus_number > 247) fd2930_test.modbus_number = FD2930_DEF_MBS_ADR;
//    if(fd2930_test.baudrate != 1 && fd2930_test.baudrate != 2 && fd2930_test.baudrate != 4 && fd2930_test.baudrate != 12 && \
//      fd2930_test.baudrate != 24) fd2930_test.baudrate = FD2930_DEF_MBS_BAUD;
//    fd2930_test.firmvare_version = FD2930_FIRMWARE_VERSION;
//    fd2930_test.device_type = FD2930_DEVICE_TYPE;
//    fd2930_test.board_number = FD2930_BOARD_NUMBER;
//    fd2930_test.current = 3200; 
//    fd2930_test.block_service = 0x0000;
//    if(fd2930_test.worked_time == 0xffffffff) fd2930_test.worked_time = 0;
//    fd2930_test.device_state_flag = 0x0000;   
//    if(fd2930_test.device_config == 0xffff) 
//    {
//      fd2930_test.device_config = FD2930_DEFAULT_DEVICE_CONFIG;
//      fd2930_test.serial_number = 0;
//      fd2930_test.device_status = 0x0000;
//    }
//    if(((fd2930_test.device_config & 0xf) < 1) || ((fd2930_test.device_config & 0xf) > 8)) fd2930_test.device_config |= 0x5; 
//    fd2930_test.device_status &= ~0x1ff;
//    fd2930_test.device_status |= FD2930_DEVICE_STATUS_BREAK;
//    fd2930_test.device_state = FD2930_STATE_STARTING1;
//    if(fd2930_test.thres_IR > FD2930_MAX_TRES_IR /*|| fd2930_test.thres_IR < FD2930_MIN_TRES_IR*/) fd2930_test.thres_IR = FD2930_DEFAULT_TRES_IR;
//    if(fd2930_test.thres_UV > FD2930_MAX_TRES_UV /*|| fd2930_test.thres_UV < FD2930_MIN_TRES_UV*/) fd2930_test.thres_UV = FD2930_DEFAULT_TRES_UV;
//    if(fd2930_test.K_IR > FD2930_MAX_K_IR || fd2930_test.K_IR < FD2930_MIN_K_IR) fd2930_test.K_IR = FD2930_DEFAULT_K_IR;
//    fd2930_K_IR = fd2930_test.K_IR;
//    if(fd2930_test.K_UV > FD2930_MAX_K_UV || fd2930_test.K_UV < FD2930_MIN_K_UV) fd2930_test.K_UV = FD2930_DEFAULT_K_UV;
//    fd2930_K_UV = fd2930_test.K_UV;
//    if(fd2930_test.wait_fault > FD2930_MAX_WAIT_FAULT /*|| fd2930_test.wait_fault < FD2930_MIN_WAIT_FAULT*/) fd2930_test.wait_fault = FD2930_DEFAULT_WAIT_FAULT;
//    if(fd2930_test.wait_fire > FD2930_MAX_WAIT_FIRE /*|| fd2930_test.wait_fire < FD2930_MIN_WAIT_FIRE*/) fd2930_test.wait_fire = FD2930_DEFAULT_WAIT_FIRE;
//    if(fd2930_test.heat_power > FD2930_MAX_HEATPOWER /*|| fd2930_test.heat_power < FD2930_MIN_HEATPOWER*/) fd2930_test.heat_power = FD2930_DEFAULT_HEATPOWER;
//    if(fd2930_test.thres_heater > FD2930_MAX_TRES_HEATER || fd2930_test.thres_heater < FD2930_MIN_TRES_HEATER) fd2930_test.thres_heater = FD2930_DEFAULT_TRES_HEATER;
//    //if(fd2930_test.archive_last_page > ARCHIVE_LENGHT) fd2930_test.archive_last_page = ARCHIVE_START_ADDRESS;
//    fd2930_test.archive_last_page = 0;
//    fd2930_test.chosen_archive_page = fd2930_test.archive_last_page - 1;
//    
//    fd2930 = fd2930_test;
//    fillMBSData();
//    write_parameters();    
//  }
//}

#endif


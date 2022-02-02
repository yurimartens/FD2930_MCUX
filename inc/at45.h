#ifndef __AT45_H
#define __AT45_H

#include <armcm3_type_def.h>

// a unique identifier for the non-volatile variable
#define NVRAMLOGBUF 2

#define ARCHIVE_LENGHT		3000000	//maximum page address
#define ARCHIVE_START_ADDRESS	10	//start address for archive (sector 2)
#define PARAMETER_ADDRESS_1     1  //address 1 for parameters storage (sector 0)
#define PARAMETER_ADDRESS_2     2  //address 2 for parameters storage (sector 1)
#define ARCHIVE_MAX_RECORD      (ARCHIVE_LENGHT - ARCHIVE_START_ADDRESS)

//������� ��� ������ � SD card
#define FLASH_WRITE_PARAMETERS          0
#define FLASH_READ_ARCHIVE              1
#define FLASH_WRITE_EVENT               2
//#define FLASH_READ_PARAMETERS_EVENT     3

//�������, ����������� � �������
#define EVENT_UNKNOWN                           0               //����������� �������
#define EVENT_POWER				1		//��������� �������
#define EVENT_IR_HIGH				2		//IR ���� �����
#define EVENT_IR_NORMAL				3		//IR �����
#define EVENT_UV_HIGH				4		//UV ���� �����
#define EVENT_UV_NORMAL				5		//UV �����
#define EVENT_FIRE_ON				6		//�����
#define EVENT_FIRE_OFF				7		//�����
#define EVENT_FAULT_ON				8		//������
#define EVENT_FAULT_OFF				9		//������
#define EVENT_BREAK_ON				10		//������
#define EVENT_BREAK_OFF				11		//������
#define EVENT_NORMAL_EVENT			12		//������� ���������� ������

#define EVENT_T_FAULT   			15		//����������� fault
#define EVENT_T_NORMAL  			16		//����������� ��������� � �����
#define EVENT_24V_FAULT				17		//24� fault
#define EVENT_24V_NORMAL			18		//24� ��������� ������� � �����
#define EVENT_HV_FAULT				19		//������� ������
#define EVENT_HV_NORMAL				20		//������� � �����

#define	EVENT_SET_ZERO				23		//��������� ����
#define EVENT_CALIBR1				24		//���������� �������� �������
#define EVENT_CALIBR2				25		//���������� �� ��

#define EVENT_HALL_ON				28		//������ ����� ��������
#define EVENT_HALL_OFF				29		//������ ����� ����������
#define EVENT_30MIN				30		//������ 30 �����

#define EVENT_PARAMETER				6000		//������� ��������
#define EVENT_COMMAND				5000		//������ �������, ��������� � ��������� ����������

#define LOG_REASON_MAGNET           "Magnet"
#define LOG_REASON_SELF_TEST        "Test"
#define LOG_REASON_ALARM_IRUV       "Al_IRUV"
#define LOG_REASON_BREAK            "Break"
#define LOG_REASON_FAULT            "Fault"
#define LOG_REASON_PREFIRE          "Prefire"
#define LOG_REASON_FIRE             "Fire"

#define QUEUE_LENGHT    8       //����� �������
#define QUEUE_ELEMENT_LENGTH    100       
typedef struct
{
  INT16U function[QUEUE_LENGHT];
  INT16U parameter[QUEUE_LENGHT];
  INT8U buffer[QUEUE_LENGHT][QUEUE_ELEMENT_LENGTH];
  INT16U sloc, rloc;  
} flash_queue_tag_t;

#define QUEUE_LIVE_LENGHT       30
typedef struct
{
  INT8U buffer[QUEUE_LIVE_LENGHT][QUEUE_ELEMENT_LENGTH];
  INT16U sloc, rloc, newly;  
} flash_queue_live_t;

extern void (*flash_work[])(INT32U, unsigned char const*);  //������� ������ � ����-�������

extern void read_archive(INT32U address, unsigned char const*);
extern void write_event(INT32U event, const INT8U *buffer);
extern void push_flash_command(INT16U function, INT16U parameter, const INT8U *buffer);
extern void push_live_data(void);
extern void read_parameters_SD();
INT8U WritePage(INT16U address, INT16U size);
INT8U ReadPage(INT16U address, INT16U size);
extern flash_queue_tag_t flash_queue;
extern flash_queue_live_t flash_queue_live;

void AppAT45TaskInit(void);
void read_and_parse_parameters(void);
void write_parameters(void);
void write_parameters_SD(INT32U param, unsigned char const* buffer);
void erase_SD(void);
void erase_parameters(void);
//void restore_parameters_from_SD(void);
void erase_archive(void);

#endif //__AT45_H

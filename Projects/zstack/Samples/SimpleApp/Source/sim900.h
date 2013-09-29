
#ifndef __SIM900__H__
#define __SIM900__H__

#include "OnBoard.h"
#include "ZComDef.h"

#define MAX_SMS_DATA_LENGHT     71
#define MAX_PHONE_NUMBER_LENGHT  20

//宏定义
//模块网络状态
#define  GSM_CONNECTED         0                //模块已经连接上网络,可以进行发送短信操作
#define  GPRS_CONNECTED        1                //模块连上gprs,可发送gprs数据.同时可发送短信
#define  NO_CONNECT            2                //模块未连接



extern byte Sim900_TaskID;


typedef struct
{
  osal_event_hdr_t hdr;
  uint8 *data; ////用于存储数据
  uint8 count;  //数据长度
} CbackEvent_t;


typedef struct
{
  bool isEmpty;
  uint8 *data; //用于存储数据
  uint8 count;  //数据长度
  void* next;
} GPRS_DATA_t;

typedef struct
{
  uint8 number[MAX_PHONE_NUMBER_LENGHT]; //用于存储电话号码
  uint8 lenght;   //数据长度
} PHONE_NUM_t;


typedef struct
{
  PHONE_NUM_t *Pnumber; //用于存储电话号码
  uint8 *data;
  uint8 count;  //数据长度
} SMS_DATA_t;





typedef struct
{
  osal_event_hdr_t  hdr;
  uint8             *msg;
} mtSerialR_Data_t;


//需要提供给外部使用的变量
extern uint8 SIM900_NetworkState;



//SIM300函数声明
void Sim900_Port_Init(void);    //端口初始化
void Sim900_Serial_CallBack(uint8 port, uint8 event);  //回调函数
void Send_At_Cmd(uint8 *p , uint8 cnt); //发送at指令

/**********************************************************************/
//任务初始化及任务时间处理
extern void Sim900_task_Init(byte task_id);

extern UINT16 Sim900_ProcessEvent( byte task_id, UINT16 events );

void Sim900StartUpConfirm(uint8 handle, uint8 status);

extern bool Sim900_Send_Sms(PHONE_NUM_t *number, uint8 *data, uint8 count); 

void  Sim900_SendSmsDataConfirm(uint8 handle, uint8 status);

extern bool Sim900_Setup_GprsConnet(void);

void GPRS_ReceiveDataIndication( uint8 DataLength, uint8* pData);

void SMS_ReceiveDataIndication( uint8 DataLength, SMS_DATA_t* pData);

void  Sim900_Setup_GprsConnetConfirm(uint8 handle , uint8 status);

extern bool Sim900_Send_GprsData( uint8 *data, uint8 count);

void  Sim900_SendGprsDataConfirm(uint8 handle, uint8 status);

void  Sim900_SendCback( uint16 event, uint8 status, uint8 count ,uint8 *data );

void  EnableSerial_CB(void);//使能串口回调函数

void  DisableSerial_CB(void);//关闭串口回调函数
 

#endif //__SIM900__H__
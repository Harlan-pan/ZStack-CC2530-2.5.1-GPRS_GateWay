
#ifndef __SIM900__H__
#define __SIM900__H__

#include "OnBoard.h"
#include "ZComDef.h"

#define MAX_SMS_DATA_LENGHT     71
#define MAX_PHONE_NUMBER_LENGHT  20

//�궨��
//ģ������״̬
#define  GSM_CONNECTED         0                //ģ���Ѿ�����������,���Խ��з��Ͷ��Ų���
#define  GPRS_CONNECTED        1                //ģ������gprs,�ɷ���gprs����.ͬʱ�ɷ��Ͷ���
#define  NO_CONNECT            2                //ģ��δ����



extern byte Sim900_TaskID;


typedef struct
{
  osal_event_hdr_t hdr;
  uint8 *data; ////���ڴ洢����
  uint8 count;  //���ݳ���
} CbackEvent_t;


typedef struct
{
  bool isEmpty;
  uint8 *data; //���ڴ洢����
  uint8 count;  //���ݳ���
  void* next;
} GPRS_DATA_t;

typedef struct
{
  uint8 number[MAX_PHONE_NUMBER_LENGHT]; //���ڴ洢�绰����
  uint8 lenght;   //���ݳ���
} PHONE_NUM_t;


typedef struct
{
  PHONE_NUM_t *Pnumber; //���ڴ洢�绰����
  uint8 *data;
  uint8 count;  //���ݳ���
} SMS_DATA_t;





typedef struct
{
  osal_event_hdr_t  hdr;
  uint8             *msg;
} mtSerialR_Data_t;


//��Ҫ�ṩ���ⲿʹ�õı���
extern uint8 SIM900_NetworkState;



//SIM300��������
void Sim900_Port_Init(void);    //�˿ڳ�ʼ��
void Sim900_Serial_CallBack(uint8 port, uint8 event);  //�ص�����
void Send_At_Cmd(uint8 *p , uint8 cnt); //����atָ��

/**********************************************************************/
//�����ʼ��������ʱ�䴦��
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

void  EnableSerial_CB(void);//ʹ�ܴ��ڻص�����

void  DisableSerial_CB(void);//�رմ��ڻص�����
 

#endif //__SIM900__H__
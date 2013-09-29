/******************** (C) COPYRIGHT 2012 ... Elec.,LTD ***********************
* File Name         : sim900.c
* Author            : huangshengbin
* Date First Issued : 11/16/2012
* Description       : gprsģ������
********************************************************************************
* History:
*   1. 11/16/2012, Ver:1.0.       by ��ʤ��, ����gprs�����������ർ��ϵͳ��������
*******************************************************************************/

#include "SIM900.h"
#include "osal.h"
#include "hal_led.h"
#include "configwithsms.h"

//�����¼�λ����
#define SIM900_INIT_EVENT                      0X0001       //��ʼ��
#define SIM900_CONNET_GPRS_EVENT               0x0002       //����gprs����
#define SIM900_GPRS_SEND_DATA_EVENT            0x0004       //����gprs����
#define SIM900_SMS_SEND_DATA_EVENT             0x0008       //���Ͷ���
#define SIM900_TIMEOUT_EVT_EVENT               0x0010       //sim900�·�������ʱ�¼�
#define SIM900_SERIAL_REV_TIMEOUT_EVENT        0x0020       //sim900����gprs/sms���ݰ���ʱ�¼�
#define SIM900_READSMS_CMD_EVENT               0x0040       //��ȡ������������


//ϵͳ�¼� Event
#define  SIM900_GPRS_SEND_DATA_CONFIRM_EVENT    0x01       //����gprs����ȷ��
#define  SIM900_SMS_SEND_DATA_CONFIRM_EVENT     0x02       //���Ͷ���ȷ��
#define  SIM900_SETUP_GPRSCONNET_CONFIRM_EVENT  0x04       //��������ȷ��
#define  SIM900_STARTUP_CONFIRM_EVENT           0x08       //������ʼ����ģ������ȷ���¼�
#define  GPRS_INCOMING_MSG_CMD                  0x10       //���յ�GPRS����
#define  SMS_INCOMING_MSG_CMD                   0x20       //���յ��������� 




//��Ӧ��ǰ�����ĳ�ʱʱ��  ����
#define  Init_TimeOutDelay             30000     //��ʼ�������г�ʱʱ��
#define  GPRS_Init_TimeOutDelay        10000     //gprs���ӳ�ʱʱ��
#define  SMS_Send_TimeOutDelay         10000     //���Ͷ��ų�ʱʱ��
#define  GPRS_Send_TimeOutDelay        7000     //����GPRS���ݳ�ʱʱ��
#define  GPRS_Rev_Data_TimeOutDelay    1000     //gprs���ݽ����ڼ� ��ʱʱ��
#define  SMS_Rev_Data_TimeOutDelay     1000      //���Ž��ճ�ʱ


#define SIM900_FALSE           0
#define SIM900_SUCCESS         1


//����
#define SIM900_UART_PORT     HAL_UART_PORT_0    //����0
#define SIM900_UART_BAUD     HAL_UART_BR_9600   //
//#define SIM900_UART_USE_DMA    //��ʹ��dma���д������ݷ��ͣ��򿪴˺�
uint8 Serial_CB = FALSE;                      //�Ƿ��ȡ��������
static uint8  tempDataPos;                    //���ڼ��㴮�ڻ���������


//ģ�鵱ǰ����״̬.
#define SIM900_STATE_IDLE             0xFE       //����
#define SIM900_STATE_INIT             0          //���ڼ���豸�Ƿ���� 
#define SIM900_STATE_GPRS_INIT        1    
#define SIM900_STATE_SMS_SEND_DATA    3
#define SIM900_STATE_READ_SMS         4
#define SIM900_STATE_GPRS_SEND_DATA   5
uint8 SIM900_RuningState  =  SIM900_STATE_IDLE; //���ڱ�ʾģ�鵱ǰ����״̬


uint8 SIM900_NetworkState =  NO_CONNECT;        //ģ�������״̬
  



#define GPRS_DATA_Q_MAX        6                //GPRS���ݷ��Ͷ������ֵ
#define SMS_DATA_Q_MAX         1               //SMS�������ݷ��Ͷ������ֵ

#define GPRS_SEND_DATA_RETRY_MAX  3             //����GPRS����ʧ�����Դ���


//��������ڲ�����
#define STEP1                  1
#define STEP2                  2
#define STEP3                  3
#define STEP4                  4
#define STEP5                  5
#define STEP6                  6
#define STEP7                  7
#define STEP8                  8



GPRS_DATA_t  gprs_data;  //�洢��Ҫ���͵�gprs���ݻ�����

SMS_DATA_t  SendSms_data;  //���ڷ��ͽ�Ҫ���͵�sms���ݻ�����

SMS_DATA_t  RevSms_data;  //���ڷ��ͽ�Ҫ���͵�sms���ݻ�����

uint8  *pSMsg = NULL;  //���ڽ��յ��Ļ�����ָ��,���ڴ��ݽ��յ��Ķ��Ż�GPRS����
uint8  TempDataLen;      //���յ���GPRS���ݳ���
uint8  SeqSmsNum;        //���ն���������,���ڶ�ȡ������

PHONE_NUM_t  phoneNum;  

uint8  tempStr[MAX_SMS_DATA_LENGHT];      //���ڴ��������ʱ��Ż��������

static uint8 SendDataing = FALSE;   //��ǰ�Ƿ��ڷ������ݲ�����0 ��. 1 ��
static uint8 tasksteps = STEP1;  //��������ʹ��
static uint8 state = STEP1;      //�ڴ��ڻص���ʹ��



//�ص����������ݽ��շ�ʽ
#define  GPRS_R_M   0x01   //gprs����
#define  SMS_R_M    0x02   //����
static uint8 Receive_M; //����ģʽ��־


void delay(void);

void reset_TasdkStatus(void);

void reset_CallbackStatus(void);

/************************************************************************
//////////////////////����void (uchar *p);ʵ�ֹ���:ͨ�����ڷ����ַ���//
************************************************************************/
void  Send_At_Cmd( uint8 *p , uint8 cnt)
{
#ifdef SIM900_UART_USE_DMA
  HalUARTWrite(SIM900_UART_PORT , p , cnt);
#else  
  U0CSR &= ~0x02;
  for (char i=0; i<cnt; i++)
  {
    U0DBUF = *p;
    while(! (U0CSR&0x02) );
    U0CSR &= ~0x02;
    p++;
  }
#endif  
}
/******************************************************************************
//����void SIM900_Port_Init(void);
ʵ�ֹ���:��ʼ��ģ��˿ڣ������ʣ��ص�������
******************************************************************************/
void Sim900_Port_Init(void)
{
  halUARTCfg_t uartConfig;
  uartConfig.configured           = TRUE;// 2x30 don't care - see uart driver.
  uartConfig.baudRate             = SIM900_UART_BAUD;
  uartConfig.flowControl          = HAL_UART_FLOW_OFF;
  uartConfig.callBackFunc         = Sim900_Serial_CallBack;
  
  HalUARTOpen (SIM900_UART_PORT, &uartConfig);
}


/******************************************************************************
//����void EnableSerial_CB(void);
ʵ�ֹ���:ʹ�ܴ��ڻص�����,����Ҫ�Ӵ��ڽ�������ʱ����
******************************************************************************/
void EnableSerial_CB(void)
{
  uint8 uart_buf_cnt;
  uint8 ch;
  uint8* temp = NULL;
  uart_buf_cnt = Hal_UART_RxBufLen(SIM900_UART_PORT); 
  if (uart_buf_cnt)
  {
    temp =(uint8 *) osal_mem_alloc(uart_buf_cnt); 
    if (temp)
    {
      HalUARTRead( SIM900_UART_PORT ,temp,  uart_buf_cnt);
      osal_mem_free(temp);
    }else
    {
      for (uint8 i=0;i<uart_buf_cnt;i++)
        HalUARTRead( SIM900_UART_PORT ,&ch, 1);
    }
  }
  Serial_CB = TRUE;
}


/******************************************************************************
//����void EnableSerial_CB(void);
ʵ�ֹ���:ʹ�ܴ��ڻص�����,����Ҫ�Ӵ��ڽ�������ʱ����
******************************************************************************/
void DisableSerial_CB(void)//�رմ��ڻص�����
{
   Serial_CB = FALSE;
}

/*****************************************************************************/

/*********************************************************************
 * @fn      SIM900_Task_Init
 *
 * @brief   Initialization function for the Simple API Task.
 *          This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notification ... ).
 *
 * @param   task_id - the ID assigned by OSAL.  This ID should be
 *                    used to send messages and set timers.
 *
 * @return  none
 */
byte Sim900_TaskID;
void Sim900_task_Init(byte task_id)
{
  Sim900_TaskID = task_id;
  Sim900_Port_Init(); 
  
  reset_TasdkStatus();
  osal_start_timerEx( Sim900_TaskID, SIM900_INIT_EVENT, 1500 ); 
}

/*********************************************************************
 * @fn      SAPI_ProcessZDOMsgs()
 *
 * @brief   Process response messages
 *
 * @param   none
 *
 * @return  none
 */
UINT16 Sim900_ProcessEvent( byte task_id, UINT16 events )
{
  
  osal_event_hdr_t *pMsg;
  if ( events & SYS_EVENT_MSG )
  {
    pMsg = (osal_event_hdr_t *) osal_msg_receive( task_id );
    while ( pMsg )
    {
      switch ( pMsg->event )
      {
        case SIM900_STARTUP_CONFIRM_EVENT:        //ģ���ʼ��
              Sim900StartUpConfirm( 0 , pMsg->status);
          break;
        case SIM900_GPRS_SEND_DATA_CONFIRM_EVENT:  //����gprs���ݻص�
             Sim900_SendGprsDataConfirm( 0 , pMsg->status);
          break;
        case SIM900_SETUP_GPRSCONNET_CONFIRM_EVENT:  //����gprs���ӻص�
             Sim900_Setup_GprsConnetConfirm( 0 , pMsg->status);
          break;
        case SIM900_SMS_SEND_DATA_CONFIRM_EVENT:  //���Ͷ��Żص�
            Sim900_SendSmsDataConfirm( 0 , pMsg->status);
          break;
        case GPRS_INCOMING_MSG_CMD:   //���յ�gprs����
          GPRS_ReceiveDataIndication( ((CbackEvent_t*)pMsg)->count, ((CbackEvent_t*)pMsg)->data);
          //���ݻ�����ʹ����Ϻ�����ͷ�!
          osal_mem_free(((CbackEvent_t*)pMsg)->data);
          break;
        case SMS_INCOMING_MSG_CMD:   //���յ���������
          SMS_ReceiveDataIndication( ((CbackEvent_t*)pMsg)->count, (SMS_DATA_t*)(((CbackEvent_t*)pMsg)->data));
          break;
         
        default:
          break;
      }
      // Release the memory
      osal_msg_deallocate( (uint8 *) pMsg );
      // Next
      pMsg = (osal_event_hdr_t *) osal_msg_receive( task_id );
    }
    // Return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }

  
   //ϵͳ��ʼ���¼�,��������ģ�鷢�ͳ�ʼ��ָ��
   if ( events & SIM900_INIT_EVENT )
   {
     switch ( tasksteps )
     {
        case STEP1 :
       {
         DisableSerial_CB();
         // P0_1 = 0; //�ѿ�������������ģ��
         tasksteps = STEP2;
         //��1��������ģ���ϵ��ȶ�
         osal_start_timerEx( Sim900_TaskID, SIM900_INIT_EVENT, 200 );//
         break;
       }
       case STEP2 :
       {
         Send_At_Cmd("AT\x0D\x0A", 4); //ͬ��������
        
         osal_start_timerEx( Sim900_TaskID, SIM900_INIT_EVENT, 200 );// 
         tasksteps = STEP3;
         break;
       }
       case STEP3 :
         {
           Send_At_Cmd("AT+IPR=9600\x0D\x0A", 13 );//����������
          
           osal_start_timerEx( Sim900_TaskID, SIM900_INIT_EVENT, 200 );// 
           tasksteps = STEP4;
           break;
         }
       case STEP4 :
         {
           Send_At_Cmd("ATE0V1\x0D\x0A", 8 );  //��������
           osal_start_timerEx( Sim900_TaskID, SIM900_INIT_EVENT, 20 );//  
           tasksteps = STEP5;
           break;
         }
       case STEP5 :
         {
           Send_At_Cmd("AT+CFUN=1\x0D\x0A", 11 );  //����ģ��ȫ������
           
           Send_At_Cmd("AT+CMGF=1\x0D\x0A", 11);// ���ö���ϵͳΪtextģʽ
           
           osal_start_timerEx( Sim900_TaskID, SIM900_INIT_EVENT, 200 );//  
           tasksteps = STEP6;
           break;
         }
       case STEP6 :
         {
           EnableSerial_CB();     //ʹ�ܴ��ڽ��ջص�.�ȴ�ģ����������"Call Ready"
           Send_At_Cmd("AT+CIURC=1\x0D\x0A", 12 );  //������ʼ����ɺ���ʾ ��CallReady��
           SIM900_RuningState = SIM900_STATE_INIT; //�޸��豸״̬Ϊ��ʼ��״̬
           osal_start_timerEx( Sim900_TaskID,
		   	       SIM900_TIMEOUT_EVT_EVENT,
                               Init_TimeOutDelay );//���ó�ʱ�¼�
           reset_CallbackStatus();
           tasksteps = STEP1;
          break;
         }
       default :
          break;
     }
      return (events ^ SIM900_INIT_EVENT);
   }
  if( events & SIM900_CONNET_GPRS_EVENT )      
  {
    switch ( tasksteps )
     {
       case STEP1 :
       {
         DisableSerial_CB();//�رմ��ڻص�
         //����GPRS����ǰ���ȹر�GPRS���� 
         //AT+CIPCLOSE�ǹرյ�ǰTCP��UDP����  ��ʱ���»�ȡģ���IP
         Send_At_Cmd("AT+CIPCLOSE=0\x0D\x0A" , 15 ); 
         tasksteps = STEP2;
         //��Ҫ�ȵ�ʱ��ȽϾ�
         osal_start_timerEx( Sim900_TaskID, SIM900_CONNET_GPRS_EVENT, 500 );
         break;
       }
       case STEP2 :
         {
         // AT+CIPSHUT �ǹر�GPRS��������� ��ʱ���»�ȡģ���IP��
           //�������µ�IP��ַ ��ȡģ���ַ���� AT+CIFSR
         Send_At_Cmd("AT+CIPSHUT\x0D\x0A" , 12 ); 
         tasksteps = STEP3;
         //��Ҫ�ȵ�ʱ��ȽϾ�
         osal_start_timerEx( Sim900_TaskID, SIM900_CONNET_GPRS_EVENT, 500 );
         break;
         } 
       case STEP3 :
         {
         Send_At_Cmd("AT+CIPMUX=0\x0D\x0A" , 13);  //��·����
         tasksteps = STEP4;
         osal_start_timerEx( Sim900_TaskID, SIM900_CONNET_GPRS_EVENT, 100 );//
         break;
         }
       case STEP4 :
         {
         Send_At_Cmd("AT+CSTT=\"uninet\"\x0D\x0A" , 18 );  //��ͨ
         //Send_At_Cmd("AT+CSTT=\"cmnet\"\x0D\x0A" , 17 ); //�ƶ�
         tasksteps = STEP5;
         osal_start_timerEx( Sim900_TaskID, SIM900_CONNET_GPRS_EVENT, 400 );//
          break;
         }
       case STEP5 :
         {
         //��������ʱ��ӱ�ʾ
         //���յ�gprs����ʱ,����ͷ Ϊ��"+IPD,data length:"
         Send_At_Cmd("AT+CIPHEAD=1\x0D\x0A" , 14 ); 
         tasksteps = STEP6;
         osal_start_timerEx( Sim900_TaskID, SIM900_CONNET_GPRS_EVENT, 400 );//
          break;
         }
       case STEP6 :
         {
          EnableSerial_CB();//�ȴ�����"CONNECT OK"
          SIM900_RuningState= SIM900_STATE_GPRS_INIT;
                    //���� ����TCP���� ���Ӳ����ں궨��������
          Send_At_Cmd("AT+CIPSTART=\"TCP\",\"123.4.567.891\",\"8888\"\x0D\x0A", 43); //Ŀ��ip ���˿�

          //���ó�ʱ�¼�
          osal_start_timerEx( Sim900_TaskID,
		              SIM900_TIMEOUT_EVT_EVENT,
                              GPRS_Init_TimeOutDelay );
          reset_CallbackStatus();
          tasksteps = STEP1;
          break;
         }
       default :
          break;
     }
    return (events ^ SIM900_CONNET_GPRS_EVENT);
  }
  
  if( events & SIM900_GPRS_SEND_DATA_EVENT )
  {
     switch ( tasksteps )
     {
       case STEP1 :
       {
         DisableSerial_CB();
         Send_At_Cmd("AT+CIPSEND\x0D\x0A" , 12 ); //�������ȵ����ݷ���
         tasksteps = STEP2;
         //�ȴ�����">"��b
         osal_start_timerEx( Sim900_TaskID, SIM900_GPRS_SEND_DATA_EVENT, 200 );
         break;
       }
       case STEP2 :
         {
         EnableSerial_CB();
         Send_At_Cmd( gprs_data.data , gprs_data.count );
         Send_At_Cmd("\x1A" , 1 );  //�������ݽ�����
         SIM900_RuningState = SIM900_STATE_GPRS_SEND_DATA;
        
          //���ó�ʱ�¼�
          osal_start_timerEx( Sim900_TaskID, SIM900_TIMEOUT_EVT_EVENT,\
          GPRS_Send_TimeOutDelay );
          reset_CallbackStatus();
          tasksteps = STEP1;
           break;
         }    
       default :
          break;
     }
   return (events ^ SIM900_GPRS_SEND_DATA_EVENT);
  }
 
  if( events & SIM900_READSMS_CMD_EVENT )
  {
       // DisableSerial_CB();
        Send_At_Cmd("AT+CMGR=", 8);
        
        //fortest
        //Send_At_Cmd(&tempStr[1], tempStr[0]);
        Send_At_Cmd("19", 2);
        
        Send_At_Cmd("\x0D\x0A" , 2);

        SIM900_RuningState = SIM900_STATE_READ_SMS;
        reset_CallbackStatus();
        
        //���ó�ʱʱ��
        osal_start_timerEx( Sim900_TaskID, SIM900_TIMEOUT_EVT_EVENT,
        SMS_Rev_Data_TimeOutDelay );
        
        EnableSerial_CB();

   return (events ^ SIM900_READSMS_CMD_EVENT);
  }
  
  //���Ͷ��ţ�
  if( events & SIM900_SMS_SEND_DATA_EVENT )
  {
     switch (tasksteps )
     {
     case STEP1:   
        DisableSerial_CB();
        Send_At_Cmd("AT+CMGF=1\x0D\x0A", 11);//����Ӣ�Ķ���
        tasksteps = STEP2;
        //�ȴ�����ok
        osal_start_timerEx( Sim900_TaskID, SIM900_SMS_SEND_DATA_EVENT, 50 );
        break;
     case STEP2:
        Send_At_Cmd("AT+CMGS=\"" , 9); //�� AT+CMGS="����"�س� ����������
        Send_At_Cmd(SendSms_data.Pnumber->number , SendSms_data.Pnumber->lenght);
        Send_At_Cmd("\"\x0D\x0A" , 3);
        tasksteps = STEP3;
        //�ȴ�>��
        osal_start_timerEx( Sim900_TaskID, SIM900_SMS_SEND_DATA_EVENT, 50 );
         break;
     case STEP3:
         EnableSerial_CB();
         //�����һ��at���ǰ�����ú�״̬
        SIM900_RuningState = SIM900_STATE_SMS_SEND_DATA;
        Send_At_Cmd(SendSms_data.data , SendSms_data.count);
        Send_At_Cmd("\x1A" , 1 ); //���Ͷ��Ž����� 
        osal_start_timerEx( Sim900_TaskID, SIM900_TIMEOUT_EVT_EVENT,
        SMS_Send_TimeOutDelay );
        break;
     default :
          break;
     }
     
   return (events ^ SIM900_SMS_SEND_DATA_EVENT);
  }
  
  
  if (events & SIM900_TIMEOUT_EVT_EVENT)
  {
     
    if ( SIM900_RuningState  ==   SIM900_STATE_INIT )
    {
      //�ڳ�ʼ��״̬ʱ,��ʱ��ģ����δ������GSM����,��Ϊ��ʼ��ʧ��.
      //���ͻص�,������Ϊ  SIM900StartUpConfirm();
      Sim900_SendCback( SIM900_STARTUP_CONFIRM_EVENT, SIM900_FALSE, 0, NULL);
    }
    
    if ( SIM900_RuningState  ==   SIM900_STATE_GPRS_INIT )
    {
      //gprs���ӳ�ʱʧ��!
      Sim900_SendCback( SIM900_SETUP_GPRSCONNET_CONFIRM_EVENT , SIM900_FALSE, 0, NULL);
    }
    
    if ( SIM900_RuningState  ==   SIM900_STATE_SMS_SEND_DATA )
    {
      //���ŷ��ͳ�ʱ.ʧ��!
      Sim900_SendCback( SIM900_SMS_SEND_DATA_CONFIRM_EVENT , SIM900_FALSE, 0, NULL);
    }
    
    if ( SIM900_RuningState  ==  SIM900_STATE_GPRS_SEND_DATA )
    {
      //gprs���ݷ��ͳ�ʱ.ʧ��!
      Sim900_SendCback( SIM900_GPRS_SEND_DATA_CONFIRM_EVENT , SIM900_FALSE, 0, NULL);
    }
    
    
    if ( SIM900_RuningState  ==  SIM900_STATE_READ_SMS )
    {
      //sms���ݶ�ȡ.ʧ��!
     
    }
    
      
     
      //��ģ��״̬��Ϊ����
     //������ʱ���ܶ��ż�gprs����
     SIM900_RuningState = SIM900_STATE_IDLE;
    
     return (events ^ SIM900_TIMEOUT_EVT_EVENT);
  }
  
   if (events & SIM900_SERIAL_REV_TIMEOUT_EVENT)
  {
    if (pSMsg)
    {
      osal_mem_free(pSMsg);
      pSMsg = NULL;
    }
    
     TempDataLen = 0;      //���յ���GPRS���ݳ���
     reset_CallbackStatus();      //�ڴ��ڻص���ʹ��
    
     return (events ^ SIM900_SERIAL_REV_TIMEOUT_EVENT);
  }
  
  // Discard unknown events
  return 0;
}




//SIM900���ڻص�����
void Sim900_Serial_CallBack(uint8 port, uint8 event)
{
  
  uint8 ch;


//��Щ�������ݲ����ȡ��������ֱ�ӷ���
   if (Serial_CB == FALSE)
   {
     return ;
   }
  
  
    //��ʼ��״̬ ���һ��ָ��Ϊ "AT+CFUN=1" 
    //�ȴ�"Call Ready"
    //�ַ���,˵��ģ���Ѿ�������������������
    if( SIM900_RuningState == SIM900_STATE_INIT ) 
    {
      while (Hal_UART_RxBufLen(port))
      {
        HalUARTRead (port, &ch, 1);
        switch (state)
        {
            case STEP1:        //�ȴ� "Call Ready"�ַ��� ˵��ģ���Ѿ�׼����
               if (ch == 'C')
                state = STEP2;
               else
                state = STEP1;
              break;            
            case STEP2:
               if (ch == 'a')
                 state = STEP3;
               else
                 state = STEP1;
              break;
            case STEP3:
               if (ch == 'l')
                  state = STEP4;
               else
                  state = STEP1;
              break;
            case STEP4:
               if (ch == 'l')
                  state = STEP5;
               else
                  state = STEP1;
              break;
            case STEP5:
               if (ch == ' ')
                  state = STEP6;
               else
                  state = STEP1;
                break;
            case STEP6:
               if (ch == 'R')
               //�رճ�ʱ��ʱ��
                 osal_stop_timerEx( Sim900_TaskID,  SIM900_TIMEOUT_EVT_EVENT );
               //�ص�ȷ�Ϻ���
                Sim900_SendCback(SIM900_STARTUP_CONFIRM_EVENT, SIM900_SUCCESS, 0, NULL);
                 state = STEP1;  
                 //��ģ��״̬��Ϊ����
                 SIM900_RuningState = SIM900_STATE_IDLE;            
                 //����ִ�е�����ͱ���ֱ�ӷ���,
                 return; 
        }
      }
    }

   if( SIM900_RuningState == SIM900_STATE_GPRS_INIT ) 
    {
      while (Hal_UART_RxBufLen(port))
      {
        HalUARTRead (port, &ch, 1);
        switch (state)
        {
            case STEP1:        //�ȴ� "CONNECT OK"�ַ��� ˵��ģ���Ѿ�����GPRS����
               if (ch == 'E')  //����������,�����ȡ��벿�ֽ����ж�
                state = STEP2;
               else
                state = STEP1;
              break;            
            case STEP2:
               if (ch == 'C')
                 state = STEP3;
               else
                 state = STEP1;
              break;
            case STEP3:
               if (ch == 'T')
                  state = STEP4;
               else
                  state = STEP1;
              break;
            case STEP4:
               if (ch == ' ')
                  state = STEP5;
               else
                  state = STEP1;
              break;
            case STEP5:
               if (ch == 'O')
                  state = STEP6;
               else
                  state = STEP1;
                break;
            case STEP6:
                if (ch == 'K')
                 //�رճ�ʱ��ʱ��
                 osal_stop_timerEx( Sim900_TaskID,  SIM900_TIMEOUT_EVT_EVENT );
                 Sim900_SendCback(SIM900_SETUP_GPRSCONNET_CONFIRM_EVENT, SIM900_SUCCESS, 0, NULL);
                 state = STEP1;
                //��ģ��״̬��Ϊ����
                 SIM900_RuningState = SIM900_STATE_IDLE;
                 //����ִ�е�����ͱ���ֱ�ӷ���
                 return; 
        }
      }
    }
    
      if (SIM900_RuningState == SIM900_STATE_SMS_SEND_DATA )
      {
        while (Hal_UART_RxBufLen(port))
        {
          HalUARTRead (port, &ch, 1);
          switch (state)
          {
              case STEP1:        //�ȴ� "+CMGS"�ַ��� ˵��ģ�鷢�Ͷ��ųɹ�
                 if (ch == '+')  
                  state = STEP2;
                 else
                  state = STEP1;
                break;            
              case STEP2:
                 if (ch == 'C')
                   state = STEP3;
                 else
                   state = STEP1;
                break;
              case STEP3:
                 if (ch == 'M')
                    state = STEP4;
                 else
                    state = STEP1;
                break;
              case STEP4:
                 if (ch == 'G')
                    state = STEP5;
                 else
                    state = STEP1;
                break;
              case STEP5:
                 if (ch == 'S')
                    state = STEP6;
                 else
                    state = STEP1;
                  break;
              case STEP6:
                 if (ch == 'K')
                   //�رճ�ʱ��ʱ��
                   osal_stop_timerEx( Sim900_TaskID,  SIM900_TIMEOUT_EVT_EVENT );
                   Sim900_SendCback(SIM900_SMS_SEND_DATA_CONFIRM_EVENT, SIM900_SUCCESS, 0,NULL);
                   state = STEP1;
                //��ģ��״̬��Ϊ����
                 SIM900_RuningState = SIM900_STATE_IDLE;                 
                 //����ִ�е�����ͱ���ֱ�ӷ���,
                 return; 
          }
        }
     }
    
   
    //����gprs����
     if ( SIM900_RuningState == SIM900_STATE_GPRS_SEND_DATA)
     {//
        while (Hal_UART_RxBufLen(port))
        {
          HalUARTRead (port, &ch, 1);
          switch (state)
          {
              case STEP1:        //�ȴ�"SEND OK"�ַ��� ˵��ģ�鷢�Ͷ��ųɹ�
                 if (ch == 'S')  
                  state = STEP2;
                 else
                  state = STEP1;
                break;            
              case STEP2:
                 if (ch == 'E')
                   state = STEP3;
                 else
                   state = STEP1;
                break;
              case STEP3:
                 if (ch == 'N')
                    state = STEP4;
                 else
                    state = STEP1;
                break;
              case STEP4:
                 if (ch == 'D')
                    state = STEP5;
                 else
                    state = STEP1;
                break;
              case STEP5:
                 if (ch == ' ')
                    state = STEP6;
                 else
                    state = STEP1;
                  break;
              case STEP6:
                 if (ch == 'O')
                    state = STEP7;
                 else
                    state = STEP1;
                  break;
              case STEP7:
                 if (ch == 'K')
                 {
                   //�رճ�ʱ��ʱ��
                   osal_stop_timerEx( Sim900_TaskID,  SIM900_TIMEOUT_EVT_EVENT );
                   Sim900_SendCback(SIM900_GPRS_SEND_DATA_CONFIRM_EVENT, SIM900_SUCCESS, 0,NULL);
                   state = STEP1;
                //��ģ��״̬��Ϊ����
                 SIM900_RuningState = SIM900_STATE_IDLE;
                 }else
                 {
                   state = STEP1;
                 }
                 //����ִ�е�����ͱ���ֱ�ӷ���,
                 return; 
          }
        }
     }
    
    
    //�������SMS  GPRS ���ݵĽ��ܴ���
    //ע��!һ��Ҫ���ڻص������������
    //gprs���ݽ���Ϊ �ж�  "+IPD,8:12345678"   �ַ���������8Ϊ���ݳ���
    //sms���ݽ���Ϊ  �ж�  "+CMTI: "SM",seq"      ����seq��ʾ�������.��ȡʱ��Ҫ�����ò���
    if ( SIM900_RuningState == SIM900_STATE_IDLE )
    {
        static   uint8 *pTemp = NULL;
      	static uint8 lenzu[3]; //���ڴ�����ݳ�����Ϣ
        static uint8 i;
        uint8  bytesInRxBuffer;  //��ʱ����
       while (Hal_UART_RxBufLen(port))
        {
          HalUARTRead (port, &ch, 1);
          switch (state)
          {
              case STEP1:
                 if (ch == '+')
                 {
                   //������ʱ�����ڳ�ʱ����
                   osal_start_timerEx( Sim900_TaskID, SIM900_SERIAL_REV_TIMEOUT_EVENT, GPRS_Rev_Data_TimeOutDelay ); 
                   
                  state = STEP2;
                 }
                 else
                 {
                  state = STEP1;
                 }
                break;            
              case STEP2:
                 if (ch == 'I')
                 {
                   state = STEP3;
                   Receive_M = GPRS_R_M; //����ģʽ�޸�ΪGPRSģʽ
                 }
                 else if (ch == 'C')
                 {
                   state = STEP3;
                   Receive_M = SMS_R_M; //����ģʽ�޸�ΪSMSģʽ
                 }else
                 {
                    state = STEP1;
                 }
                break;
              case STEP3:
                 if (Receive_M == GPRS_R_M)
                 {
                    if (ch == 'P')
                      state = STEP4;
                    else
                      state = STEP1;
                 }else if (Receive_M == SMS_R_M)
                 {
                    if (ch == 'M')
                      state = STEP4;
                    else
                      state = STEP1;
                 }
                break;
              case STEP4:
                 if (Receive_M == GPRS_R_M)
                 {
                    if (ch == 'D')
                      state = STEP5;
                    else
                      state = STEP1;
                 }else if (Receive_M == SMS_R_M)
                 {
                    if (ch == 'T')
                      state = STEP5;
                    else
                      state = STEP1;
                 }
                break;
              case STEP5:
                 if (Receive_M == GPRS_R_M)
                 {
                   //ȷ����gprs������                   
                   // ch = ",";  //��������

				   //��λ���㳤��ʱʹ�õ��ı���
                   osal_memset( lenzu, 0, 3 );
                   i = 0;
                     state = STEP6;
                   
                 }else if (Receive_M == SMS_R_M)
                 {
                     if (ch == 'I')
                     {
                     
                       pTemp = tempStr;
                       state = STEP6;
                     }
                    else
                      state = STEP1;
                 }
                  break;
              case STEP6:
                 if (Receive_M == GPRS_R_M)
                 {
                   //��ȡ���ݳ���
                    if ( ch != ':') 
                    {
                      lenzu[i] = ch ; //�ŵ���������
                      i++;
                      continue;
                    }
                    //�ж����ݳ���
                    if ( 1 == i)
                    {
                      TempDataLen = lenzu[0] - '0' ;  
                    }
                    if ( 2 == i)
                    {
                      TempDataLen = (lenzu[0] - '0') * 10 + ( lenzu[1] - '0') ; 
                    }
                    if ( 3 == i)
                    {
                      TempDataLen = (lenzu[0] - '0') * 100 + (lenzu[1] - '0') * 10 + (lenzu[2] - '0') ;
                    }
					
                  tempDataPos = 0; 
                  //���仺��
                  pSMsg = (uint8 *)osal_mem_alloc(TempDataLen);
                  if (pSMsg)
                  {
                    state = STEP7;
                  }
                  else
                  {
                   //��ȡ����ʧ��
                    state = STEP1;
                    return;
                  }
                    
                   
                 }else if (Receive_M == SMS_R_M)
                 {
                   //�Ѿ�ȷ���Ƕ��� 

                   //1\�������ȡ�������
                   //2\����ϵͳ�¼���֪�յ�������Ϣ
                   //3\ϵͳ�����ö�ȡ����ָ���ȡ���ž�������
                   
                   
                   //���������ʣ�µ�  ": "SM",seq\r\n" ������ݣ��Ի�ȡ�������seq
                   if (ch != '\r')
                   {//������ʣ�µĵ���β������
                    *(pTemp++) = ch;
                    continue;
                   }else
                   {//���н������

                   *(pTemp++) = ch; // '/r'
                    
                    uint8 l = 0;
                    while (tempStr[7+l] != '\r')
                    {
                      l++;
                    }
                    
                    
                    //��һ���ֽڴ�����кų���
                    tempStr[0] = l; 
                    
                    osal_memcpy(&tempStr[1],&tempStr[7],l);
                    
                    //֪ͨ�����ж���Ϣ�����¼�
                    osal_set_event( Sim900_TaskID , SIM900_READSMS_CMD_EVENT); 
                    
                    
                    state = STEP1;
                    
                    return;
                   }
                 }
             
                  break;
              case STEP7:
			  	      
                      //�ѵ�һ��ֱ�Ӷ�ȡ��������
                      pSMsg[tempDataPos++] = ch;    //
                      //��ȡ���ڻ��������ݳ���
                      bytesInRxBuffer = Hal_UART_RxBufLen(port);
              
                      // If the remain of the data is there, read them all, otherwise, just read enough 
                      if (bytesInRxBuffer <= TempDataLen  - tempDataPos )
                      {
                        HalUARTRead (port, &pSMsg[tempDataPos], bytesInRxBuffer);
                        tempDataPos += bytesInRxBuffer;
                      }
                      else
                      {
                        HalUARTRead (port, &pSMsg[tempDataPos], TempDataLen - tempDataPos);
                        tempDataPos += (TempDataLen - tempDataPos );
                      }

                      if ( tempDataPos >= TempDataLen)
                      {
                          osal_stop_timerEx( Sim900_TaskID, SIM900_SERIAL_REV_TIMEOUT_EVENT ); 
                          Sim900_SendCback(GPRS_INCOMING_MSG_CMD, SIM900_SUCCESS, TempDataLen , pSMsg);
                          state = STEP1;
                      }
                 break;
              default:
                  break;
          }
        }
    }
   
   
   
   //��ȡ�������� 
/*
+CMGR: "REC READ","+8613918186089","02/01/30,20:40:31+00"
This is a test
   
OK
*/
   if ( SIM900_RuningState == SIM900_STATE_READ_SMS)
   {
      static uint8 n;
      
      while (Hal_UART_RxBufLen(port))
        {
          HalUARTRead (port, &ch, 1);
          switch (state)
          {
              case STEP1:        //����"+CMGR" �ַ���
                 if (ch == '+')
                  state = STEP2;
                 else
                  state = STEP1;
                break;            
              case STEP2:
                 if (ch == 'C')
                   state = STEP3;
                 else
                   state = STEP1;
                break;
              case STEP3:
                 if (ch == 'M')
                    state = STEP4;
                 else
                    state = STEP1;
                break;
              case STEP4:
                 if (ch == 'G')
                    state = STEP5;
                 else
                    state = STEP1;
                break;
              case STEP5:
                 if (ch == 'R')
                 {
                    n = 0;
                    state = STEP6;
                 }
                 else
                    state = STEP1;
                  break;
              case STEP6:
                //��ȡ�������ݰ�ͷ��
                
                //read one line
                if (ch != '\n')
                {
                  tempStr[n++] = ch;
                  continue;
                }else
                {
                  n=0;
                  if (match_phoneNum(tempStr, &phoneNum) == TRUE)
                  {//�ҵ��ֻ����룬������phoneNum�����б���
                      state = STEP7; 
                  }else
                  {//�����ֻ������������ʧ��
                    SIM900_RuningState = SIM900_STATE_IDLE;  
                    state = STEP1; 
                    osal_stop_timerEx( Sim900_TaskID, SIM900_TIMEOUT_EVT_EVENT ); 
                    return;
                  }
                  
                }
                
              case STEP7:
                //��ȡ��������
                
                //read one line
                if (ch != '\r')
                {
                  tempStr[n++] = ch;
                  
                  if (n>MAX_SMS_DATA_LENGHT)
                  {//�ж϶��������Ƿ����
                    SIM900_RuningState = SIM900_STATE_IDLE;
                    osal_stop_timerEx( Sim900_TaskID, SIM900_TIMEOUT_EVT_EVENT ); 
                    state = STEP1;
                    return;
                  }
                  continue;
                }else
                {
                  SIM900_RuningState = SIM900_STATE_IDLE;  //���Ŷ�ȡ�ɹ�Ҫ��ģ������״̬���Ļؿ���
                  RevSms_data.data = &tempStr[1];
                  RevSms_data.count = n-1;
                  RevSms_data.Pnumber = &phoneNum;
                  Sim900_SendCback(SMS_INCOMING_MSG_CMD,SIM900_SUCCESS, n-1, (uint8*)&RevSms_data);
                  osal_stop_timerEx( Sim900_TaskID, SIM900_TIMEOUT_EVT_EVENT ); 
                  state = STEP1;
                }
                 return; 
          }
        }
   }
}



/*********************************************************************
 * @fn      SIM900_SendCback
 *
 * @brief   Sends a message to the SIM900 task ( itself ) so that a
 *           callback can be generated later.
 *
 * @return  none
 */

void Sim900_SendCback( uint16 event, uint8 status, uint8 count ,uint8 *data )
{
  CbackEvent_t *pMsg;

  pMsg = (CbackEvent_t *)osal_msg_allocate( sizeof(CbackEvent_t) );
  if( pMsg )
  {
    pMsg->hdr.event = event;
    pMsg->hdr.status = status;
    pMsg->count = count;
    pMsg->data = data;
    osal_msg_send( Sim900_TaskID, (uint8 *)pMsg );
  }

}


//���ⲿʹ��,����Ӣ�Ķ���,��δ���Բ������ݳ���.��������140���ַ�.
//numberΪ�ֻ�����     dataָ���跢�͵�����ָ��
//count Ϊ���͵����ݳ��� 
bool Sim900_Send_Sms(PHONE_NUM_t *number, uint8 *data, uint8 count)
{
  if( SIM900_NetworkState != NO_CONNECT && SendDataing == FALSE)
  {
    reset_TasdkStatus();
    SendDataing = TRUE;
    osal_memcpy(SendSms_data.Pnumber->number,number->number, number->lenght);
    SendSms_data.Pnumber->lenght = number->lenght;
    SendSms_data.data = data;
    SendSms_data.count = count;
    osal_set_event( Sim900_TaskID , SIM900_SMS_SEND_DATA_EVENT);  
     return TRUE;;  
  }else
  {
    //ģ�鵱ǰ��֧�ַ��Ͳ���
   return FALSE;
  }
}
//���Ͷ���ȷ�Ϻ���
void  Sim900_SendSmsDataConfirm(uint8 handle, uint8 status)
{
 //�������ݷ���״̬Ϊ����
  SendDataing = FALSE;
   if( status == SIM900_SUCCESS)
  {

  }else
  {
 
  }
}

//���ⲿʹ��,����gprs����
bool Sim900_Send_GprsData( uint8 *data, uint8 count)
{
  uint8 cnt=0;
  if( SIM900_NetworkState == GPRS_CONNECTED)//�����������ҵ�ǰ����������
  {
    if (gprs_data.isEmpty == 0) //��ǰ����ṹΪ�գ�û��Ҫ���͵�����
    {
      gprs_data.data = (uint8 *)osal_mem_alloc( count );
      if (gprs_data.data)
      {
        gprs_data.isEmpty = 1;  //��Ϊ������
        osal_memcpy( gprs_data.data, data, count );
        gprs_data.count = count ;//���ݳ���
        gprs_data.next = NULL;
      }else
      {
        return FALSE;
      }
    }
    else  //��ǰ�ṹ�������ݣ�������ݷŵ���һ���ṹ����
    {
    	GPRS_DATA_t* p = &gprs_data;
        
        //�ҵ������е����һ���ṹ��
    	for (;p->next!=NULL;p++)
	{
	  if (++cnt > GPRS_DATA_Q_MAX)
	   return FALSE;  //�����Ѿ���������������ݽ�����
	}
        
        p->next = NULL;
    	((GPRS_DATA_t*)(p->next))->data = NULL;
        
        p->next =(GPRS_DATA_t*)osal_mem_alloc(sizeof(GPRS_DATA_t));
        ((GPRS_DATA_t*)(p->next))->data= (uint8 *)osal_mem_alloc( count );
        if (p->next && ((GPRS_DATA_t*)(p->next))->data) //�����뵽�ռ�
        {
          ((GPRS_DATA_t*)(p->next))->isEmpty = 1; //����һ������Ϊ������״̬
          osal_memcpy( ((GPRS_DATA_t*)(p->next))->data, data, count );
          ((GPRS_DATA_t*)(p->next))->count = count ; //���ݳ���
          ((GPRS_DATA_t*)(p->next))->next = NULL;
        }else
         {
            if (p->next)
              osal_mem_free(gprs_data.next);
            if (((GPRS_DATA_t*)(p->next))->data)
              osal_mem_free(gprs_data.next);
            
            return FALSE;
         }
    }
      
    if (gprs_data.isEmpty == 1)
    {
       reset_TasdkStatus();
       //�������ݷ���״̬Ϊ���ڷ�������
       SendDataing = TRUE;
       osal_set_event( Sim900_TaskID , SIM900_GPRS_SEND_DATA_EVENT);  
       return TRUE;
     }else
     {
       return FALSE;
     }
  }else
  {
    return FALSE;
  }
}
//����gprs����ȷ�Ϻ���
void  Sim900_SendGprsDataConfirm(uint8 handle, uint8 status)
{
  static uint8 sendDataFailedCnt = 0;
   //�������ݷ���״̬Ϊ����
   SendDataing = FALSE;
   
   if( status == SIM900_SUCCESS)
  {
    sendDataFailedCnt = 0;
    osal_mem_free(gprs_data.data); //�ͷŵ�ǰ�ṹ���ڴ档��
    gprs_data.isEmpty = 0;
    
    if (((GPRS_DATA_t*)(gprs_data.next)) != NULL)//��������
    {
           
      //����һ�������е����ݸ��Ƶ���ǰ�ṹ����
      gprs_data.isEmpty =((GPRS_DATA_t*)(gprs_data.next))->isEmpty;          
      gprs_data.data = ((GPRS_DATA_t*)(gprs_data.next))->data;         
      gprs_data.count =((GPRS_DATA_t*)(gprs_data.next))->count;
      osal_mem_free((GPRS_DATA_t*)gprs_data.next); //�ͷŽṹ���ڴ�
      gprs_data.next = ((GPRS_DATA_t*)(gprs_data.next))->next; 
                         
      reset_TasdkStatus();         
      SendDataing = TRUE;
      osal_set_event( Sim900_TaskID , SIM900_GPRS_SEND_DATA_EVENT); //�ٴ����������¼�   
    }
    
  }else 
  {
    if (sendDataFailedCnt++ < GPRS_SEND_DATA_RETRY_MAX)
    {
      reset_TasdkStatus();               
      SendDataing = TRUE;
      osal_set_event( Sim900_TaskID , SIM900_GPRS_SEND_DATA_EVENT); //�������������¼�
    }else
    {
      sendDataFailedCnt = 0;
      
      //��ջ�����
        if (gprs_data.isEmpty == 1) //
        {
          osal_mem_free(gprs_data.data); //�ͷŵ�ǰ�ṹ�������ڴ档
          gprs_data.isEmpty = 0;
          
          while (((GPRS_DATA_t*)(gprs_data.next)) != NULL)
          {
            uint8 *temp;
            temp = ((GPRS_DATA_t*)(gprs_data.next))->next;
            osal_mem_free((GPRS_DATA_t*)gprs_data.next); //�ͷŽṹ���ڴ�
            osal_mem_free(((GPRS_DATA_t*)(gprs_data.next))->data); //�ͷŵ�ǰ�ṹ���ڴ档��
            gprs_data.next = temp;
          }
        }
      
      //���Ͳ��ɹ��������Ѿ���������������
      
      SIM900_NetworkState = GSM_CONNECTED;
      reset_TasdkStatus();
      osal_set_event( Sim900_TaskID, SIM900_CONNET_GPRS_EVENT );//
    }
  }
}



bool Sim900_Setup_GprsConnet(void)
{
  if(SIM900_NetworkState != NO_CONNECT )
  {
    reset_TasdkStatus();
    osal_set_event( Sim900_TaskID, SIM900_CONNET_GPRS_EVENT );//
    return TRUE;
  }else
  {
   //���ģ��û������GSM����,����ʧ��
   return FALSE;
  }
}

void  Sim900_Setup_GprsConnetConfirm(uint8 handle , uint8 status)
{
  if( status == SIM900_SUCCESS)
  {
      SIM900_NetworkState = GPRS_CONNECTED;
      
      
  }else
  {   
    
    
    reset_TasdkStatus();
    Sim900_Setup_GprsConnet();
    
    SIM900_RuningState = SIM900_STATE_IDLE;
  }
}

void Sim900StartUpConfirm( uint8 handle, uint8 status )
{//����ر�־λ��������
  if(status == SIM900_SUCCESS)
  {
     SIM900_NetworkState = GSM_CONNECTED;//���ñ�־λ,������GSM����
     Sim900_Setup_GprsConnet();
  }else
  {
      SIM900_NetworkState = NO_CONNECT;//���ñ�־λ,ģ��δ����
      
      SystemReset(); //ϵͳ��λ���Զ��������ӷ�����
  }
}



//gprs���ݽ���ȷ��
void GPRS_ReceiveDataIndication( uint8 DataLength, uint8* pData)
{
    //��Ҫ���ز���ʱ������ķ��ͺ���ȥע��
   // Sim900_Send_GprsData( pData, DataLength);
}

//SMS���ݽ���ȷ��
void SMS_ReceiveDataIndication( uint8 DataLength, SMS_DATA_t* pData)
{
  
}


void reset_TasdkStatus(void)
{
  tasksteps = STEP1; 
}

void reset_CallbackStatus(void)
{
  state = STEP1;
}

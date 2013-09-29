/******************** (C) COPYRIGHT 2012 ... Elec.,LTD ***********************
* File Name         : sim900.c
* Author            : huangshengbin
* Date First Issued : 11/16/2012
* Description       : gprs模块驱动
********************************************************************************
* History:
*   1. 11/16/2012, Ver:1.0.       by 黄胜斌, 修正gprs重连次数过多导致系统崩溃问题
*******************************************************************************/

#include "SIM900.h"
#include "osal.h"
#include "hal_led.h"
#include "configwithsms.h"

//任务事件位掩码
#define SIM900_INIT_EVENT                      0X0001       //初始化
#define SIM900_CONNET_GPRS_EVENT               0x0002       //连接gprs网络
#define SIM900_GPRS_SEND_DATA_EVENT            0x0004       //发送gprs数据
#define SIM900_SMS_SEND_DATA_EVENT             0x0008       //发送短信
#define SIM900_TIMEOUT_EVT_EVENT               0x0010       //sim900下发操作超时事件
#define SIM900_SERIAL_REV_TIMEOUT_EVENT        0x0020       //sim900接收gprs/sms数据包超时事件
#define SIM900_READSMS_CMD_EVENT               0x0040       //读取短信内容命令


//系统事件 Event
#define  SIM900_GPRS_SEND_DATA_CONFIRM_EVENT    0x01       //发送gprs数据确认
#define  SIM900_SMS_SEND_DATA_CONFIRM_EVENT     0x02       //发送短信确认
#define  SIM900_SETUP_GPRSCONNET_CONFIRM_EVENT  0x04       //数据连接确认
#define  SIM900_STARTUP_CONFIRM_EVENT           0x08       //经过初始化后模块启动确认事件
#define  GPRS_INCOMING_MSG_CMD                  0x10       //接收到GPRS数据
#define  SMS_INCOMING_MSG_CMD                   0x20       //接收到短信数据 




//对应当前操作的超时时间  毫秒
#define  Init_TimeOutDelay             30000     //初始化过程中超时时间
#define  GPRS_Init_TimeOutDelay        10000     //gprs连接超时时间
#define  SMS_Send_TimeOutDelay         10000     //发送短信超时时间
#define  GPRS_Send_TimeOutDelay        7000     //发送GPRS数据超时时间
#define  GPRS_Rev_Data_TimeOutDelay    1000     //gprs数据接收期间 超时时间
#define  SMS_Rev_Data_TimeOutDelay     1000      //短信接收超时


#define SIM900_FALSE           0
#define SIM900_SUCCESS         1


//串口
#define SIM900_UART_PORT     HAL_UART_PORT_0    //串口0
#define SIM900_UART_BAUD     HAL_UART_BR_9600   //
//#define SIM900_UART_USE_DMA    //若使用dma进行串口数据发送，打开此宏
uint8 Serial_CB = FALSE;                      //是否读取串口数据
static uint8  tempDataPos;                    //用于计算串口缓冲区长度


//模块当前运行状态.
#define SIM900_STATE_IDLE             0xFE       //空闲
#define SIM900_STATE_INIT             0          //正在检测设备是否存在 
#define SIM900_STATE_GPRS_INIT        1    
#define SIM900_STATE_SMS_SEND_DATA    3
#define SIM900_STATE_READ_SMS         4
#define SIM900_STATE_GPRS_SEND_DATA   5
uint8 SIM900_RuningState  =  SIM900_STATE_IDLE; //用于标示模块当前运行状态


uint8 SIM900_NetworkState =  NO_CONNECT;        //模块的网络状态
  



#define GPRS_DATA_Q_MAX        6                //GPRS数据发送队列最大值
#define SMS_DATA_Q_MAX         1               //SMS短信数据发送队列最大值

#define GPRS_SEND_DATA_RETRY_MAX  3             //发送GPRS数据失败重试次数


//任务操作内部步骤
#define STEP1                  1
#define STEP2                  2
#define STEP3                  3
#define STEP4                  4
#define STEP5                  5
#define STEP6                  6
#define STEP7                  7
#define STEP8                  8



GPRS_DATA_t  gprs_data;  //存储将要发送的gprs数据缓冲区

SMS_DATA_t  SendSms_data;  //用于发送将要发送的sms数据缓冲区

SMS_DATA_t  RevSms_data;  //用于发送将要发送的sms数据缓冲区

uint8  *pSMsg = NULL;  //串口接收到的缓冲区指针,用于传递接收到的短信或GPRS数据
uint8  TempDataLen;      //接收到的GPRS数据长度
uint8  SeqSmsNum;        //接收短信序列码,用于读取短信用

PHONE_NUM_t  phoneNum;  

uint8  tempStr[MAX_SMS_DATA_LENGHT];      //用于存放来短信时序号或短信内容

static uint8 SendDataing = FALSE;   //当前是否处于发送数据操作中0 否. 1 是
static uint8 tasksteps = STEP1;  //在任务中使用
static uint8 state = STEP1;      //在串口回调中使用



//回调函数中数据接收方式
#define  GPRS_R_M   0x01   //gprs数据
#define  SMS_R_M    0x02   //短信
static uint8 Receive_M; //接收模式标志


void delay(void);

void reset_TasdkStatus(void);

void reset_CallbackStatus(void);

/************************************************************************
//////////////////////函数void (uchar *p);实现功能:通过串口发送字符串//
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
//函数void SIM900_Port_Init(void);
实现功能:初始化模块端口，波特率，回调函数等
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
//函数void EnableSerial_CB(void);
实现功能:使能串口回调函数,当需要从串口接收数据时调用
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
//函数void EnableSerial_CB(void);
实现功能:使能串口回调函数,当需要从串口接收数据时调用
******************************************************************************/
void DisableSerial_CB(void)//关闭串口回调函数
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
        case SIM900_STARTUP_CONFIRM_EVENT:        //模块初始化
              Sim900StartUpConfirm( 0 , pMsg->status);
          break;
        case SIM900_GPRS_SEND_DATA_CONFIRM_EVENT:  //发送gprs数据回调
             Sim900_SendGprsDataConfirm( 0 , pMsg->status);
          break;
        case SIM900_SETUP_GPRSCONNET_CONFIRM_EVENT:  //建立gprs连接回调
             Sim900_Setup_GprsConnetConfirm( 0 , pMsg->status);
          break;
        case SIM900_SMS_SEND_DATA_CONFIRM_EVENT:  //发送短信回调
            Sim900_SendSmsDataConfirm( 0 , pMsg->status);
          break;
        case GPRS_INCOMING_MSG_CMD:   //接收到gprs数据
          GPRS_ReceiveDataIndication( ((CbackEvent_t*)pMsg)->count, ((CbackEvent_t*)pMsg)->data);
          //数据缓冲区使用完毕后必须释放!
          osal_mem_free(((CbackEvent_t*)pMsg)->data);
          break;
        case SMS_INCOMING_MSG_CMD:   //接收到短信数据
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

  
   //系统初始化事件,在这里向模块发送初始化指令
   if ( events & SIM900_INIT_EVENT )
   {
     switch ( tasksteps )
     {
        case STEP1 :
       {
         DisableSerial_CB();
         // P0_1 = 0; //把控制线拉低启动模块
         tasksteps = STEP2;
         //等1秒左右让模块上电稳定
         osal_start_timerEx( Sim900_TaskID, SIM900_INIT_EVENT, 200 );//
         break;
       }
       case STEP2 :
       {
         Send_At_Cmd("AT\x0D\x0A", 4); //同步波特率
        
         osal_start_timerEx( Sim900_TaskID, SIM900_INIT_EVENT, 200 );// 
         tasksteps = STEP3;
         break;
       }
       case STEP3 :
         {
           Send_At_Cmd("AT+IPR=9600\x0D\x0A", 13 );//波特率设置
          
           osal_start_timerEx( Sim900_TaskID, SIM900_INIT_EVENT, 200 );// 
           tasksteps = STEP4;
           break;
         }
       case STEP4 :
         {
           Send_At_Cmd("ATE0V1\x0D\x0A", 8 );  //开启回显
           osal_start_timerEx( Sim900_TaskID, SIM900_INIT_EVENT, 20 );//  
           tasksteps = STEP5;
           break;
         }
       case STEP5 :
         {
           Send_At_Cmd("AT+CFUN=1\x0D\x0A", 11 );  //开启模块全部功能
           
           Send_At_Cmd("AT+CMGF=1\x0D\x0A", 11);// 配置短信系统为text模式
           
           osal_start_timerEx( Sim900_TaskID, SIM900_INIT_EVENT, 200 );//  
           tasksteps = STEP6;
           break;
         }
       case STEP6 :
         {
           EnableSerial_CB();     //使能串口接收回调.等待模块联网返回"Call Ready"
           Send_At_Cmd("AT+CIURC=1\x0D\x0A", 12 );  //开机初始化完成后显示 “CallReady”
           SIM900_RuningState = SIM900_STATE_INIT; //修改设备状态为初始化状态
           osal_start_timerEx( Sim900_TaskID,
		   	       SIM900_TIMEOUT_EVT_EVENT,
                               Init_TimeOutDelay );//设置超时事件
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
         DisableSerial_CB();//关闭串口回调
         //进行GPRS连接前首先关闭GPRS连接 
         //AT+CIPCLOSE是关闭当前TCP或UDP连接  这时重新获取模块的IP
         Send_At_Cmd("AT+CIPCLOSE=0\x0D\x0A" , 15 ); 
         tasksteps = STEP2;
         //需要等的时间比较久
         osal_start_timerEx( Sim900_TaskID, SIM900_CONNET_GPRS_EVENT, 500 );
         break;
       }
       case STEP2 :
         {
         // AT+CIPSHUT 是关闭GPRS网络的连接 这时重新获取模块的IP，
           //将分配新的IP地址 获取模块地址命令 AT+CIFSR
         Send_At_Cmd("AT+CIPSHUT\x0D\x0A" , 12 ); 
         tasksteps = STEP3;
         //需要等的时间比较久
         osal_start_timerEx( Sim900_TaskID, SIM900_CONNET_GPRS_EVENT, 500 );
         break;
         } 
       case STEP3 :
         {
         Send_At_Cmd("AT+CIPMUX=0\x0D\x0A" , 13);  //单路连接
         tasksteps = STEP4;
         osal_start_timerEx( Sim900_TaskID, SIM900_CONNET_GPRS_EVENT, 100 );//
         break;
         }
       case STEP4 :
         {
         Send_At_Cmd("AT+CSTT=\"uninet\"\x0D\x0A" , 18 );  //联通
         //Send_At_Cmd("AT+CSTT=\"cmnet\"\x0D\x0A" , 17 ); //移动
         tasksteps = STEP5;
         osal_start_timerEx( Sim900_TaskID, SIM900_CONNET_GPRS_EVENT, 400 );//
          break;
         }
       case STEP5 :
         {
         //接收数据时添加标示
         //接收到gprs数据时,数据头 为："+IPD,data length:"
         Send_At_Cmd("AT+CIPHEAD=1\x0D\x0A" , 14 ); 
         tasksteps = STEP6;
         osal_start_timerEx( Sim900_TaskID, SIM900_CONNET_GPRS_EVENT, 400 );//
          break;
         }
       case STEP6 :
         {
          EnableSerial_CB();//等待返回"CONNECT OK"
          SIM900_RuningState= SIM900_STATE_GPRS_INIT;
                    //这里 进行TCP连接 连接参数在宏定义中设置
          Send_At_Cmd("AT+CIPSTART=\"TCP\",\"123.4.567.891\",\"8888\"\x0D\x0A", 43); //目的ip 及端口

          //设置超时事件
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
         Send_At_Cmd("AT+CIPSEND\x0D\x0A" , 12 ); //不定长度的数据发送
         tasksteps = STEP2;
         //等待返回">"号b
         osal_start_timerEx( Sim900_TaskID, SIM900_GPRS_SEND_DATA_EVENT, 200 );
         break;
       }
       case STEP2 :
         {
         EnableSerial_CB();
         Send_At_Cmd( gprs_data.data , gprs_data.count );
         Send_At_Cmd("\x1A" , 1 );  //发送内容结束符
         SIM900_RuningState = SIM900_STATE_GPRS_SEND_DATA;
        
          //设置超时事件
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
        
        //设置超时时间
        osal_start_timerEx( Sim900_TaskID, SIM900_TIMEOUT_EVT_EVENT,
        SMS_Rev_Data_TimeOutDelay );
        
        EnableSerial_CB();

   return (events ^ SIM900_READSMS_CMD_EVENT);
  }
  
  //发送短信，
  if( events & SIM900_SMS_SEND_DATA_EVENT )
  {
     switch (tasksteps )
     {
     case STEP1:   
        DisableSerial_CB();
        Send_At_Cmd("AT+CMGF=1\x0D\x0A", 11);//发送英文短信
        tasksteps = STEP2;
        //等待返回ok
        osal_start_timerEx( Sim900_TaskID, SIM900_SMS_SEND_DATA_EVENT, 50 );
        break;
     case STEP2:
        Send_At_Cmd("AT+CMGS=\"" , 9); //把 AT+CMGS="号码"回车 拆成三条语句
        Send_At_Cmd(SendSms_data.Pnumber->number , SendSms_data.Pnumber->lenght);
        Send_At_Cmd("\"\x0D\x0A" , 3);
        tasksteps = STEP3;
        //等待>号
        osal_start_timerEx( Sim900_TaskID, SIM900_SMS_SEND_DATA_EVENT, 50 );
         break;
     case STEP3:
         EnableSerial_CB();
         //在最后一条at语句前把设置好状态
        SIM900_RuningState = SIM900_STATE_SMS_SEND_DATA;
        Send_At_Cmd(SendSms_data.data , SendSms_data.count);
        Send_At_Cmd("\x1A" , 1 ); //发送短信结束符 
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
      //在初始化状态时,超时后模块仍未连接上GSM网络,视为初始化失败.
      //发送回调,处理函数为  SIM900StartUpConfirm();
      Sim900_SendCback( SIM900_STARTUP_CONFIRM_EVENT, SIM900_FALSE, 0, NULL);
    }
    
    if ( SIM900_RuningState  ==   SIM900_STATE_GPRS_INIT )
    {
      //gprs连接超时失败!
      Sim900_SendCback( SIM900_SETUP_GPRSCONNET_CONFIRM_EVENT , SIM900_FALSE, 0, NULL);
    }
    
    if ( SIM900_RuningState  ==   SIM900_STATE_SMS_SEND_DATA )
    {
      //短信发送超时.失败!
      Sim900_SendCback( SIM900_SMS_SEND_DATA_CONFIRM_EVENT , SIM900_FALSE, 0, NULL);
    }
    
    if ( SIM900_RuningState  ==  SIM900_STATE_GPRS_SEND_DATA )
    {
      //gprs数据发送超时.失败!
      Sim900_SendCback( SIM900_GPRS_SEND_DATA_CONFIRM_EVENT , SIM900_FALSE, 0, NULL);
    }
    
    
    if ( SIM900_RuningState  ==  SIM900_STATE_READ_SMS )
    {
      //sms数据读取.失败!
     
    }
    
      
     
      //把模块状态改为空闲
     //方便随时接受短信及gprs数据
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
    
     TempDataLen = 0;      //接收到的GPRS数据长度
     reset_CallbackStatus();      //在串口回调中使用
    
     return (events ^ SIM900_SERIAL_REV_TIMEOUT_EVENT);
  }
  
  // Discard unknown events
  return 0;
}




//SIM900串口回调函数
void Sim900_Serial_CallBack(uint8 port, uint8 event)
{
  
  uint8 ch;


//有些无用数据不想读取，在这里直接返回
   if (Serial_CB == FALSE)
   {
     return ;
   }
  
  
    //初始化状态 最后一条指令为 "AT+CFUN=1" 
    //等待"Call Ready"
    //字符串,说明模块已经入网并可以正常工作
    if( SIM900_RuningState == SIM900_STATE_INIT ) 
    {
      while (Hal_UART_RxBufLen(port))
      {
        HalUARTRead (port, &ch, 1);
        switch (state)
        {
            case STEP1:        //等待 "Call Ready"字符串 说明模块已经准备好
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
               //关闭超时定时器
                 osal_stop_timerEx( Sim900_TaskID,  SIM900_TIMEOUT_EVT_EVENT );
               //回调确认函数
                Sim900_SendCback(SIM900_STARTUP_CONFIRM_EVENT, SIM900_SUCCESS, 0, NULL);
                 state = STEP1;  
                 //把模块状态改为空闲
                 SIM900_RuningState = SIM900_STATE_IDLE;            
                 //程序执行到这里就必须直接返回,
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
            case STEP1:        //等待 "CONNECT OK"字符串 说明模块已经连上GPRS网络
               if (ch == 'E')  //避免程序过长,这里截取后半部分进行判断
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
                 //关闭超时定时器
                 osal_stop_timerEx( Sim900_TaskID,  SIM900_TIMEOUT_EVT_EVENT );
                 Sim900_SendCback(SIM900_SETUP_GPRSCONNET_CONFIRM_EVENT, SIM900_SUCCESS, 0, NULL);
                 state = STEP1;
                //把模块状态改为空闲
                 SIM900_RuningState = SIM900_STATE_IDLE;
                 //程序执行到这里就必须直接返回
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
              case STEP1:        //等待 "+CMGS"字符串 说明模块发送短信成功
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
                   //关闭超时定时器
                   osal_stop_timerEx( Sim900_TaskID,  SIM900_TIMEOUT_EVT_EVENT );
                   Sim900_SendCback(SIM900_SMS_SEND_DATA_CONFIRM_EVENT, SIM900_SUCCESS, 0,NULL);
                   state = STEP1;
                //把模块状态改为空闲
                 SIM900_RuningState = SIM900_STATE_IDLE;                 
                 //程序执行到这里就必须直接返回,
                 return; 
          }
        }
     }
    
   
    //发送gprs数据
     if ( SIM900_RuningState == SIM900_STATE_GPRS_SEND_DATA)
     {//
        while (Hal_UART_RxBufLen(port))
        {
          HalUARTRead (port, &ch, 1);
          switch (state)
          {
              case STEP1:        //等待"SEND OK"字符串 说明模块发送短信成功
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
                   //关闭超时定时器
                   osal_stop_timerEx( Sim900_TaskID,  SIM900_TIMEOUT_EVT_EVENT );
                   Sim900_SendCback(SIM900_GPRS_SEND_DATA_CONFIRM_EVENT, SIM900_SUCCESS, 0,NULL);
                   state = STEP1;
                //把模块状态改为空闲
                 SIM900_RuningState = SIM900_STATE_IDLE;
                 }else
                 {
                   state = STEP1;
                 }
                 //程序执行到这里就必须直接返回,
                 return; 
          }
        }
     }
    
    
    //这里进行SMS  GPRS 数据的接受处理
    //注意!一定要放在回调函数的最后面
    //gprs数据接收为 判断  "+IPD,8:12345678"   字符串，其中8为数据长度
    //sms数据接收为  判断  "+CMTI: "SM",seq"      其中seq表示短信序号.读取时需要操作该参数
    if ( SIM900_RuningState == SIM900_STATE_IDLE )
    {
        static   uint8 *pTemp = NULL;
      	static uint8 lenzu[3]; //用于存放数据长度信息
        static uint8 i;
        uint8  bytesInRxBuffer;  //临时变量
       while (Hal_UART_RxBufLen(port))
        {
          HalUARTRead (port, &ch, 1);
          switch (state)
          {
              case STEP1:
                 if (ch == '+')
                 {
                   //开启定时器用于超时计算
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
                   Receive_M = GPRS_R_M; //接收模式修改为GPRS模式
                 }
                 else if (ch == 'C')
                 {
                   state = STEP3;
                   Receive_M = SMS_R_M; //接收模式修改为SMS模式
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
                   //确定是gprs数据了                   
                   // ch = ",";  //跳过逗号

				   //复位计算长度时使用到的变量
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
                   //读取数据长度
                    if ( ch != ':') 
                    {
                      lenzu[i] = ch ; //放到数组里面
                      i++;
                      continue;
                    }
                    //判断数据长度
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
                  //分配缓存
                  pSMsg = (uint8 *)osal_mem_alloc(TempDataLen);
                  if (pSMsg)
                  {
                    state = STEP7;
                  }
                  else
                  {
                   //读取数据失败
                    state = STEP1;
                    return;
                  }
                    
                   
                 }else if (Receive_M == SMS_R_M)
                 {
                   //已经确定是短信 

                   //1\在这里读取短信序号
                   //2\发送系统事件告知收到短信消息
                   //3\系统再利用读取短信指令读取短信具体内容
                   
                   
                   //这里接收完剩下的  ": "SM",seq\r\n" 这段数据，以获取短信序号seq
                   if (ch != '\r')
                   {//接收完剩下的到行尾的数据
                    *(pTemp++) = ch;
                    continue;
                   }else
                   {//该行接收完毕

                   *(pTemp++) = ch; // '/r'
                    
                    uint8 l = 0;
                    while (tempStr[7+l] != '\r')
                    {
                      l++;
                    }
                    
                    
                    //第一个字节存放序列号长度
                    tempStr[0] = l; 
                    
                    osal_memcpy(&tempStr[1],&tempStr[7],l);
                    
                    //通知任务有短消息到来事件
                    osal_set_event( Sim900_TaskID , SIM900_READSMS_CMD_EVENT); 
                    
                    
                    state = STEP1;
                    
                    return;
                   }
                 }
             
                  break;
              case STEP7:
			  	      
                      //把第一个直接读取进缓冲区
                      pSMsg[tempDataPos++] = ch;    //
                      //读取串口缓冲区数据长度
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
   
   
   
   //读取短信内容 
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
              case STEP1:        //搜索"+CMGR" 字符串
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
                //读取短信数据包头。
                
                //read one line
                if (ch != '\n')
                {
                  tempStr[n++] = ch;
                  continue;
                }else
                {
                  n=0;
                  if (match_phoneNum(tempStr, &phoneNum) == TRUE)
                  {//找到手机号码，保存在phoneNum数组中备用
                      state = STEP7; 
                  }else
                  {//由于手机号码过长导致失败
                    SIM900_RuningState = SIM900_STATE_IDLE;  
                    state = STEP1; 
                    osal_stop_timerEx( Sim900_TaskID, SIM900_TIMEOUT_EVT_EVENT ); 
                    return;
                  }
                  
                }
                
              case STEP7:
                //读取短信数据
                
                //read one line
                if (ch != '\r')
                {
                  tempStr[n++] = ch;
                  
                  if (n>MAX_SMS_DATA_LENGHT)
                  {//判断短信内容是否过长
                    SIM900_RuningState = SIM900_STATE_IDLE;
                    osal_stop_timerEx( Sim900_TaskID, SIM900_TIMEOUT_EVT_EVENT ); 
                    state = STEP1;
                    return;
                  }
                  continue;
                }else
                {
                  SIM900_RuningState = SIM900_STATE_IDLE;  //短信读取成功要把模块运行状态更改回空闲
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


//供外部使用,发送英文短信,暂未测试测试数据长度.当至少有140个字符.
//number为手机号码     data指向需发送的数据指针
//count 为发送的数据长度 
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
    //模块当前不支持发送操作
   return FALSE;
  }
}
//发送短信确认函数
void  Sim900_SendSmsDataConfirm(uint8 handle, uint8 status)
{
 //更新数据发送状态为空闲
  SendDataing = FALSE;
   if( status == SIM900_SUCCESS)
  {

  }else
  {
 
  }
}

//供外部使用,发送gprs数据
bool Sim900_Send_GprsData( uint8 *data, uint8 count)
{
  uint8 cnt=0;
  if( SIM900_NetworkState == GPRS_CONNECTED)//网络已连接且当前无其他操作
  {
    if (gprs_data.isEmpty == 0) //当前链表结构为空，没有要发送的数据
    {
      gprs_data.data = (uint8 *)osal_mem_alloc( count );
      if (gprs_data.data)
      {
        gprs_data.isEmpty = 1;  //置为有数据
        osal_memcpy( gprs_data.data, data, count );
        gprs_data.count = count ;//数据长度
        gprs_data.next = NULL;
      }else
      {
        return FALSE;
      }
    }
    else  //当前结构中有数据，则把数据放到下一个结构体中
    {
    	GPRS_DATA_t* p = &gprs_data;
        
        //找到链表中的最后一个结构体
    	for (;p->next!=NULL;p++)
	{
	  if (++cnt > GPRS_DATA_Q_MAX)
	   return FALSE;  //链表已经不允许继续放数据进来了
	}
        
        p->next = NULL;
    	((GPRS_DATA_t*)(p->next))->data = NULL;
        
        p->next =(GPRS_DATA_t*)osal_mem_alloc(sizeof(GPRS_DATA_t));
        ((GPRS_DATA_t*)(p->next))->data= (uint8 *)osal_mem_alloc( count );
        if (p->next && ((GPRS_DATA_t*)(p->next))->data) //能申请到空间
        {
          ((GPRS_DATA_t*)(p->next))->isEmpty = 1; //置下一个链表为有数据状态
          osal_memcpy( ((GPRS_DATA_t*)(p->next))->data, data, count );
          ((GPRS_DATA_t*)(p->next))->count = count ; //数据长度
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
       //更新数据发送状态为正在发送数据
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
//发送gprs数据确认函数
void  Sim900_SendGprsDataConfirm(uint8 handle, uint8 status)
{
  static uint8 sendDataFailedCnt = 0;
   //更新数据发送状态为空闲
   SendDataing = FALSE;
   
   if( status == SIM900_SUCCESS)
  {
    sendDataFailedCnt = 0;
    osal_mem_free(gprs_data.data); //释放当前结构体内存。。
    gprs_data.isEmpty = 0;
    
    if (((GPRS_DATA_t*)(gprs_data.next)) != NULL)//还有数据
    {
           
      //把下一个链表中的数据复制到当前结构体中
      gprs_data.isEmpty =((GPRS_DATA_t*)(gprs_data.next))->isEmpty;          
      gprs_data.data = ((GPRS_DATA_t*)(gprs_data.next))->data;         
      gprs_data.count =((GPRS_DATA_t*)(gprs_data.next))->count;
      osal_mem_free((GPRS_DATA_t*)gprs_data.next); //释放结构体内存
      gprs_data.next = ((GPRS_DATA_t*)(gprs_data.next))->next; 
                         
      reset_TasdkStatus();         
      SendDataing = TRUE;
      osal_set_event( Sim900_TaskID , SIM900_GPRS_SEND_DATA_EVENT); //再次启动发送事件   
    }
    
  }else 
  {
    if (sendDataFailedCnt++ < GPRS_SEND_DATA_RETRY_MAX)
    {
      reset_TasdkStatus();               
      SendDataing = TRUE;
      osal_set_event( Sim900_TaskID , SIM900_GPRS_SEND_DATA_EVENT); //重新启动发送事件
    }else
    {
      sendDataFailedCnt = 0;
      
      //清空缓冲区
        if (gprs_data.isEmpty == 1) //
        {
          osal_mem_free(gprs_data.data); //释放当前结构体数据内存。
          gprs_data.isEmpty = 0;
          
          while (((GPRS_DATA_t*)(gprs_data.next)) != NULL)
          {
            uint8 *temp;
            temp = ((GPRS_DATA_t*)(gprs_data.next))->next;
            osal_mem_free((GPRS_DATA_t*)gprs_data.next); //释放结构体内存
            osal_mem_free(((GPRS_DATA_t*)(gprs_data.next))->data); //释放当前结构体内存。。
            gprs_data.next = temp;
          }
        }
      
      //发送不成功，可能已经断网，进行重连
      
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
   //如果模块没有连上GSM网络,返回失败
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
{//对相关标志位进行设置
  if(status == SIM900_SUCCESS)
  {
     SIM900_NetworkState = GSM_CONNECTED;//设置标志位,已连上GSM网络
     Sim900_Setup_GprsConnet();
  }else
  {
      SIM900_NetworkState = NO_CONNECT;//设置标志位,模块未联网
      
      SystemReset(); //系统复位后，自动重新连接服务器
  }
}



//gprs数据接收确认
void GPRS_ReceiveDataIndication( uint8 DataLength, uint8* pData)
{
    //需要环回测试时将下面的发送函数去注释
   // Sim900_Send_GprsData( pData, DataLength);
}

//SMS数据接收确认
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

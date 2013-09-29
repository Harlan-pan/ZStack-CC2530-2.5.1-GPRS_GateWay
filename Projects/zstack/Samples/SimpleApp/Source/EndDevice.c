/******************************************************************************
 * INCLUDES
 */

#include "ZComDef.h"
#include "OSAL.h"
#include "hal_key.h"
#include "hal_lcd.h"
#include "hal_led.h"
#include "hal_adc.h"
#include "hal_mcu.h"
#include "hal_uart.h"
#include "DemoApp.h"
#include "sapi.h"
#include "hal_timer.h"
#include "hal_MMA8451Q.h"
/******************************************************************************
 * CONSTANTS
 */

#define REPORT_FAILURE_LIMIT                4
#define ACK_REQ_INTERVAL                    5 // each 5th packet is sent with ACK request

// Application States
#define APP_INIT                            0    // Initial state
#define APP_START                           1    // Sensor has joined network
#define APP_BIND                            2    // Sensor is in process of binding
#define APP_REPORT                          4    // Sensor is in reporting state

// Application osal event identifiers
// Bit mask of events ( from 0x0000 to 0x00FF )
#define MY_START_EVT                        0x0001
#define MY_DETECT_ACTION_EVT                0x0002      //̽����ΪҪʵʱ����
#define MY_URGENT_ALARM_EVT                 0x0004      //���������¼�
#define MY_REPORT_ACC_EVT                   0x0008      //����������ٶ�ֵ�¼�
#define MY_TUMBLE_ALARM_EVT                 0x0010      //��������

// ADC definitions for CC2430/CC2530 from the hal_adc.c file
#if defined (HAL_MCU_CC2530)
#define HAL_ADC_REF_125V    0x00    /* Internal 1.25V Reference */
#define HAL_ADC_DEC_064     0x00    /* Decimate by 64 : 8-bit resolution */
#define HAL_ADC_DEC_128     0x10    /* Decimate by 128 : 10-bit resolution */
#define HAL_ADC_DEC_512     0x30    /* Decimate by 512 : 14-bit resolution */
#define HAL_ADC_CHN_VDD3    0x0f    /* Input channel: VDD/3 */
#define HAL_ADC_CHN_TEMP    0x0e    /* Temperature sensor */
#endif // HAL_MCU_CC2530

#define BUZZER  P1_1
/******************************************************************************
 * TYPEDEFS
 */

/******************************************************************************
 * LOCAL VARIABLES
 */
//static uint8 timer3State =        FALSE;
//static uint16  detectdelay=5;              //ÿ��Ƶ��֮��ļ��ʱ�����
static uint16 Xdata,Ydata,Zdata;
//static uint8 SendAcc[]="AT+GETACC=";                //�򴮿ڷ�����������
//static uint8 SendBuf[]="                  ";                //�򴮿ڷ�����������
static uint16 urgetnDealy = 1000;             //��������������1s
static uint32 ReportPeriod = 10000;             //��������������1s
static uint16 TumblePeriod = 500;
static uint8 Int_Status=0;
static uint8 Count=0;   //���ڼ�ʱ
static uint8 Step_One=FALSE,Step_Two=FALSE,Step_Three=FALSE,Step_Four=FALSE;
static uint8 Int_Flag=FALSE;
//static uint8 Time=0,PL_Count=0;
/******************************************************************************
 * GLOBAL VARIABLES
 */

// Inputs and Outputs for Sensor device
#define NUM_OUT_CMD_SENSOR                1
#define NUM_IN_CMD_SENSOR                 1

// List of output and input commands for Sensor device
const cId_t zb_OutCmdList[NUM_OUT_CMD_SENSOR] =
{
  SENSOR_REPORT_CMD_ID
};
const cId_t zb_InCmdList[NUM_IN_CMD_SENSOR]=
{
  SENSOR_ALARM_CMD_ID
};

// Define SimpleDescriptor for Sensor device
const SimpleDescriptionFormat_t zb_SimpleDesc =
{
  MY_ENDPOINT_ID,             //  Endpoint
  MY_PROFILE_ID,              //  Profile ID
  DEV_ID_SENSOR,              //  Device ID
  DEVICE_VERSION_SENSOR,      //  Device Version
  0,                          //  Reserved
  NUM_IN_CMD_SENSOR,          //  Number of Input Commands
  (cId_t *) zb_InCmdList,             //  Input Command List
  NUM_OUT_CMD_SENSOR,         //  Number of Output Commands
  (cId_t *) zb_OutCmdList     //  Output Command List
};
/*
union protocol{
  uint8  TxBuf[11];
  struct{
    uint8  head[2];
    uint16 SAddr;
    uint8  Cmd;
    uint8  NodeData[6];
  }NodeMessage;
}ZigBeeNode;
*/
/******************************************************************************
 * LOCAL FUNCTIONS
 */

void uartRxCB( uint8 port, uint8 event );
void Get_Acc_Value(void);
void HexToStr(void);
static uint8 strsearch(uint8 *ptr2,uint8 *ptr1_at);
static uint8 Compare(void);
void Ext_Int_Init(void);
__interrupt void P2_ISR(void);
 void Timer1_Init(void);
  __interrupt void T1_ISR(void);
static void Analy_Int(void);
/*****************************************************************************
 * @fn          zb_HandleOsalEvent
 *
 * @brief       The zb_HandleOsalEvent function is called by the operating
 *              system when a task event is set
 *
 * @param       event - Bitmask containing the events that have been set
 *
 * @return      none
 */
void zb_HandleOsalEvent( uint16 event )
{

  if(event & MY_DETECT_ACTION_EVT)
  {
     Analy_Int();
    // zb_SendDataRequest( 0x0000, SENSOR_REPORT_CMD_ID, sizeof(SendAcc), SendAcc, 0,0, 0 );
     osal_set_event(sapi_TaskID,MY_DETECT_ACTION_EVT);       //�ٴδ���̽���¼�
  }  

  //-------������������¼�---------------//
  if(event & MY_URGENT_ALARM_EVT)
  {
    uint8 urgentcall[]="AT+URGENTCALL";
    zb_SendDataRequest( 0x0000, SENSOR_REPORT_CMD_ID, sizeof(urgentcall), urgentcall, 0,0, 0 );
    osal_start_timerEx(sapi_TaskID,MY_URGENT_ALARM_EVT,urgetnDealy);    //ÿһ������Э��������һ�ν��������¼�
  }
  //-----����������ٶ�ֵ�¼�----------//
  if( event & MY_REPORT_ACC_EVT)
  {
    uint8 getacc[16]={'A','T','+','G','E','T','A','C','C','='};
      Get_Acc_Value();
      getacc[10]=Xdata&0xff00;
      getacc[11]=Xdata&0x00FF;
      getacc[12]=Ydata&0xff00;
      getacc[13]=Ydata&0x00FF;
      getacc[14]=Zdata&0xff00;
      getacc[15]=Zdata&0x00FF;
      /*
      _ltoa(Xdata,&getacc[10],16);
      _ltoa(Ydata,&getacc[14],16);
      _ltoa(Zdata,&getacc[18],16);
      */
      zb_SendDataRequest( 0x0000, SENSOR_REPORT_CMD_ID, sizeof(getacc), getacc, 0,0, 0 );
      if(ReportPeriod >0)
      {
       osal_start_timerEx(sapi_TaskID,MY_REPORT_ACC_EVT,ReportPeriod);      //�ٴδ���������ٶ�ֵ�¼� 
      }
       
  }
  
   //-----������������¼�----------//
  if( event & MY_TUMBLE_ALARM_EVT)
  {
    uint8 tumble[]="AT+TUMBLE";
    zb_SendDataRequest( 0x0000, SENSOR_REPORT_CMD_ID, sizeof(tumble), tumble, 0,0, 0 );
    osal_start_timerEx(sapi_TaskID,MY_TUMBLE_ALARM_EVT,TumblePeriod);
  }
}

/******************************************************************************
 * @fn      zb_HandleKeys
 *
 * @brief   Handles all key events for this device.
 *
 * @param   shift - true if in shift/alt.
 * @param   keys - bit field for key events. Valid entries:
 *                 EVAL_SW4
 *                 EVAL_SW3
 *                 EVAL_SW2
 *                 EVAL_SW1
 *
 * @return  none
 */
void zb_HandleKeys( uint8 shift, uint8 keys )
{
  // Shift is used to make each button/switch dual purpose.
  if ( shift )
  {
    if ( keys & HAL_KEY_SW_1 )
    {
      
    }
    if ( keys & HAL_KEY_SW_2 )
    {
    }
    if ( keys & HAL_KEY_SW_3 )
    {
    }
    if ( keys & HAL_KEY_SW_4 )
    {
    }
  }
  else
  {
    if ( keys & HAL_KEY_SW_6 )
    {
      osal_set_event(sapi_TaskID,MY_URGENT_ALARM_EVT);  //�������������¼�
    }
  }
}

/******************************************************************************
 * @fn          zb_StartConfirm
 *
 * @brief       The zb_StartConfirm callback is called by the ZigBee stack
 *              after a start request operation completes
 *
 * @param       status - The status of the start operation.  Status of
 *                       ZB_SUCCESS indicates the start operation completed
 *                       successfully.  Else the status is an error code.
 *
 * @return      none
 */
void zb_StartConfirm( uint8 status )
{
  // If the device sucessfully started, change state to running
  if ( status == ZB_SUCCESS ) 
  {
    HalLedBlink(HAL_LED_2,2,50,300);     //����������������������  
    //��������֮���ʼ��MMA
    MMA845x_Init();
    //��ʼ��2530�ⲿ�ж�P2.0
    Ext_Int_Init();
    
  /*FF�ж�ʹ��������*/  
   InterruptsActive(0,INT_EN_FF_MT_1_MASK,INT_CFG_FF_MT_1_MASK);     //Freefall/Motion�жϣ��ܽ�ӳ�䵽INT1
   Conf_FF();          //FreeFall  ���ٶ�ֵ����0.315g������30ms���������������ж�
  
  /*Transient�ж�ʹ��������*/
  InterruptsActive(0,INT_EN_TRANS_MASK,INT_CFG_TRANS_MASK);         //Transient�ж�  �ܽ�ӳ�䵽INT1
  Conf_Transient();            //����Transient�ж�
  Timer1_Init();               //���ö�ʱ��1
  osal_set_event(sapi_TaskID,MY_DETECT_ACTION_EVT);       //��ʼ�������������̽���¼�
  osal_start_timerEx(sapi_TaskID,MY_REPORT_ACC_EVT,ReportPeriod);
  }
}

/******************************************************************************
 * @fn          zb_SendDataConfirm
 *
 * @brief       The zb_SendDataConfirm callback function is called by the
 *              ZigBee after a send data operation completes
 *
 * @param       handle - The handle identifying the data transmission.
 *              status - The status of the operation.
 *
 * @return      none
 */
void zb_SendDataConfirm( uint8 handle, uint8 status )
{
  
}

/******************************************************************************
 * @fn          zb_BindConfirm
 *
 * @brief       The zb_BindConfirm callback is called by the ZigBee stack
 *              after a bind operation completes.
 *
 * @param       commandId - The command ID of the binding being confirmed.
 *              status - The status of the bind operation.
 *
 * @return      none
 */
void zb_BindConfirm( uint16 commandId, uint8 status )
{

}

/******************************************************************************
 * @fn          zb_AllowBindConfirm
 *
 * @brief       Indicates when another device attempted to bind to this device
 *
 * @param
 *
 * @return      none
 */
void zb_AllowBindConfirm( uint16 source )
{
}

/******************************************************************************
 * @fn          zb_FindDeviceConfirm
 *
 * @brief       The zb_FindDeviceConfirm callback function is called by the
 *              ZigBee stack when a find device operation completes.
 *
 * @param       searchType - The type of search that was performed.
 *              searchKey - Value that the search was executed on.
 *              result - The result of the search.
 *
 * @return      none
 */
void zb_FindDeviceConfirm( uint8 searchType, uint8 *searchKey, uint8 *result )
{
}

/******************************************************************************
 * @fn          zb_ReceiveDataIndication
 *
 * @brief       The zb_ReceiveDataIndication callback function is called
 *              asynchronously by the ZigBee stack to notify the application
 *              when data is received from a peer device.
 *
 * @param       source - The short address of the peer device that sent the data
 *              command - The commandId associated with the data
 *              len - The number of bytes in the pData parameter
 *              pData - The data sent by the peer device
 *
 * @return      none
 */
void zb_ReceiveDataIndication( uint16 source, uint16 command, uint16 len, uint8 *pData  )
{
 uint8 position;
 uint16 tempbuf[3];
  if( strsearch("AT+ACK" ,pData) )       //����յ�Ӧ���ź���ôֹͣ���������¼�
 {
   osal_stop_timerEx(sapi_TaskID,MY_URGENT_ALARM_EVT);   //ֹͣ���������¼�
   osal_stop_timerEx(sapi_TaskID,MY_TUMBLE_ALARM_EVT);
 }
 else if( (position=strsearch("AT+PERIOD" ,pData)) )  //���ڷ�ΧΪ0--600
 {
  // osal_stop_timerEx(sapi_TaskID,MY_REPORT_ACC_EVT);
   tempbuf[0]=*(pData+position)-'0';        //��ȡ��λ
   tempbuf[1]=*(pData+position+1)-'0';      //��ȡʮλ
   tempbuf[2]=*(pData+position+2)-'0';      //��ȡ��λ
   ReportPeriod=(tempbuf[0]*100+tempbuf[1]*10+tempbuf[2]);    //Ĭ��Ϊ�����1000��Ϊs
   ReportPeriod *= 1000;
   if(ReportPeriod > 0)       //�����������Ϊ0�򲻴������ͼ��ٶ��¼�
   {
    osal_start_timerEx(sapi_TaskID,MY_REPORT_ACC_EVT,ReportPeriod);
   }

 }
 else
 {
   
 }
}

/******************************************************************************
 * @fn          uartRxCB
 *
 * @brief       Callback function for UART 
 *
 * @param       port - UART port
 *              event - UART event that caused callback 
 *
 * @return      none
 */
void uartRxCB( uint8 port, uint8 event )
{
}
/*
*******************************************************************
**�������ƣ�Get_Acc_Value
**�������ܣ��ɼ�������ٶ�ֵ
**��ڲ�������
**���ڲ�������
******************************************************************
*/
void Get_Acc_Value(void)
{
  Xdata=IIC_Read_Word(MMA845x_IIC_ADDRESS,OUT_X_MSB_REG);       //��ȡX��ֵ
  Ydata=IIC_Read_Word(MMA845x_IIC_ADDRESS,OUT_Y_MSB_REG);       //��ȡY��ֵ
  Zdata=IIC_Read_Word(MMA845x_IIC_ADDRESS,OUT_Z_MSB_REG);       //��ȡZ��ֵ
}
/*********************************************************************
 ** ��������: strsearch ()
 ** ��������: ��ָ���������������ҵ���ͬ������
 ** ��ڲ���: ptr2Ҫ���ҵ�����, ptr1��ǰ����
 ** ���ڲ���: ��Ϊ0���ҵ���Ϊ0��û���ҵ�  ���ص�ֵΪ=����ֵ��λ����Ϣ
 *********************************************************************/
static uint8 strsearch(uint8 *ptr2,uint8 *ptr1_at)//���ַ���*ptr2��*ptr1�е�λ��
//����������������ַ���*ptr2�Ƿ���ȫ������*ptr1��
//����:  0  û���ҵ�
//1-255 �ӵ�N���ַ���ʼ��ͬ
{  
	uint8 i,j,k;
	//uint8 flag;
	if(ptr2[0]==0) return(0);
	//flag=0;
	for(i=0,j=0;i<MAX_sousuo-2;i++)
	{
        	if(ptr1_at[i]==ptr2[j])
       		{	//��һ���ַ���ͬ
        		for(k=i;k<MAX_sousuo-2;k++,j++)
        		{
        			if(ptr2[j]==0)//�Ƚ���ȷ
        				return(j+1);               //����ֵ������������0
        			if(ptr1_at[k]!=ptr2[j]) break;
        		}
        		j=0;
        	}
	}
	return(0);
}
/*
*******************************************************************
**�������ƣ�Ext_Int_Init()
**�������ܣ��ⲿ�жϳ�ʼ��
**��ڲ���:��
**���ڲ�������
************************************************************************
*/
void Ext_Int_Init(void)
{
  P2INP&=~0X01;     //P2_0������
  P2IEN |= 0X01;    //ʹ��P2_0�ⲿ�ж�
  PICTL|=0X08;      //P2_0�½��ز����ж�
  EA=1;
  IEN2 |= 0X02;         //�˿�2�ж�ʹ��
  P2IFG&=~0X0F;       //����жϱ�־
}

/*********�жϷ������*********/
#pragma vector=P2INT_VECTOR
__interrupt void P2_ISR(void)
{
 P2IFG=0;       //���жϱ�־  
 Int_Flag=TRUE;   
 //Int_Status=IIC_Reg_Read(MMA845x_IIC_ADDRESS,INT_SOURCE_REG);  //��ȡ�ж�״̬
 P2IF = 0;
}

/*
**************************************************
**�������ƣ�Timer1_Init()
**�������ܣ���ʱ��1��ʼ��
**��ڲ�������
**���ڲ�������
**˵����T1CCOH=0X2F  T1CC0L=0XFF��ʱֵ����Ϊ100ms
        T1CCOH=0X45  T1CC0L=0X00��ʱֵ����Ϊ200ms       
***********************************************************
*/
void Timer1_Init(void)
{
  T1CTL |= 0x0E;    //128��Ƶ��ģģʽ
  //T1STAT |= 0X21;    //��������жϣ�ͨ��0��Ч
  T1CNTH=0;            //�����������λ
  T1CNTL=0;            //�����������λ
  T1CCTL0 |= 0X40;     //��ͨ��0�ж�
  T1CCTL0 |= 0X04;     //�Ƚ�ģʽ 
  T1CC0H=0X2F;
  T1CC0L=0XFF;
  //T1IE=1;     //ʹ�ܶ�ʱ��1�ж�
  EA=1;       //�����ж�
}
#pragma vector = T1_VECTOR
 __interrupt void T1_ISR(void)
 {
  IRCON = 0x00;			  //���жϱ�־,Ҳ����Ӳ���Զ����
  T1STAT=0;                       //���ͨ���жϱ�־
  Count++;
  if(Count==20)         //2�������ȡ������ٶ�ֵ��֮ǰ�Ƚ�
  {
      if(Step_One&&Step_Two)
      Step_Three=TRUE;  //��λ���������Ƚϼ��ٶ�ֵ��־��
      Count=0;        //��������
      T1IE=0;         //�رռ�ʱ��
  }

 }
/*
********************************************************************
**�������ƣ�HexToDec
**�������ܣ�ʮ������ת��Ϊʮ����
**��ڲ�������
**���ڲ�������
**********************************************************************
*/
static uint8 Compare(void)
{
 
  if(Xdata>0X7FFF)
  {
    Xdata = ~Xdata+1;       //����
  }
  if(Ydata>0X7FFF)
  {
    Ydata = ~Ydata+1;       //����
  }
  if(Zdata>0X7FFF)
  {
    Zdata = ~Zdata+1;       //����
  }
  //��4gģʽ��,14λ�ֱ���
  Xdata >>= 2;
  Ydata >>= 2;
  Zdata >>= 2;
  
  //������ٶ���ʵֵ  4Gģʽ��λmg
  Xdata /= 2.048;
  Ydata /= 2.048;
  if((Xdata>500)||(1000-Ydata>500)||(Zdata>500))
    return 1;
  else
    return 0;
}
/*
**********************************************************************
**�������ƣ�Analy_Int
**�������ܣ������ж�״̬
**��ڲ�������
**���ڲ�������
***************************************************************************
*/
static void Analy_Int(void)
{
 if(Int_Flag)
  {
    Int_Status=IIC_Reg_Read(MMA845x_IIC_ADDRESS,INT_SOURCE_REG);    //��ȡ�ж�Դ״̬���ж��ĸ��жϲ�����жϱ�־λ
    // -------------------------------------------// 
    /*
    if(Int_Status&0x10)               //PL�ж�
    {
     IIC_Reg_Read(MMA845x_IIC_ADDRESS,PL_STATUS_REG);     //��ȡ״̬λ���״̬��ʶ
     PL_Count++;          //����PL�жϺ�����һ���ж�
     if(PL_Count==2)      //�����Tranitent�жϺ��⵽����PL�ж��򱾴ε��������Ч
     {
     }
    }*/
   //-------------------------------------//   
   /*
    if(Int_Status&0x08)     //Tap�ж�
    {
    }*/
   //----------------------------------------//   
    if(Int_Status&0x04)     //FreeFall�ж�
    {
     Step_One=TRUE;       //��һ��̽��ɹ�
    }
    //------------------------------------------//    
    if(Int_Status&0x20)     //Transient�ж�
    {
     // BUZZER=1;
      if(Step_One)
      {
       Step_Two=TRUE;       //��λ�ڶ�����־
      // BUZZER=1;
      // Time=2;             //3.5s��ʱ  
       T1CNTH=0;            //�����������λ
       T1CNTL=0;            //�����������λ
       T1IE=1;              //����ʱ��
    //   InterruptsActive(0,INT_EN_LNDPRT_MASK,INT_CFG_LNDPRT_MASK);//��ʱ��PL�ж�L/P�жϣ��ܽ�ӳ�䵽INT1
      } 
    }
     //-------------------------------------------------//
    Int_Flag=FALSE;       //����жϱ�־
  }
  if(Step_One&&Step_Two&&Step_Three) 
  {
   Xdata=IIC_Read_Word(MMA845x_IIC_ADDRESS,OUT_X_MSB_REG);       //��ȡX��ֵ
   Ydata=IIC_Read_Word(MMA845x_IIC_ADDRESS,OUT_Y_MSB_REG);       //��ȡY��ֵ
   Zdata=IIC_Read_Word(MMA845x_IIC_ADDRESS,OUT_Z_MSB_REG);       //��ȡZ��ֵ
   if(Compare())          //����ʱ�ĳ�ʼֵ�Ƚ��������һ�����ֵ���ʼֵ�Ƚϱ仯0.7g         
   {
    Step_Four=TRUE;       //��λ���Ĳ���־
   }
            
   else                   //α����
   {
     Step_One=FALSE;      //�����һ����־
     Step_Two=FALSE;      //�����һ����־
     Step_Three=FALSE;    //�����һ����־
   }
  }
  if(Step_One&&Step_Two&&Step_Three&&Step_Four)       //�Ĳ�ȫ������
  {
    osal_set_event(sapi_TaskID,MY_TUMBLE_ALARM_EVT);    //�����������������¼�
    Step_One=FALSE;
    Step_Two=FALSE;
    Step_Three=FALSE;
    Step_Four=FALSE;
  }       
}
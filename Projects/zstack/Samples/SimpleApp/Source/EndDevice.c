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
#define MY_DETECT_ACTION_EVT                0x0002      //探测行为要实时运行
#define MY_URGENT_ALARM_EVT                 0x0004      //紧急报警事件
#define MY_REPORT_ACC_EVT                   0x0008      //报告三轴加速度值事件
#define MY_TUMBLE_ALARM_EVT                 0x0010      //跌倒报警

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
//static uint16  detectdelay=5;              //每个频率之间的间隔时间控制
static uint16 Xdata,Ydata,Zdata;
//static uint8 SendAcc[]="AT+GETACC=";                //向串口发送三轴数据
//static uint8 SendBuf[]="                  ";                //向串口发送三轴数据
static uint16 urgetnDealy = 1000;             //紧急报警的周期1s
static uint32 ReportPeriod = 10000;             //紧急报警的周期1s
static uint16 TumblePeriod = 500;
static uint8 Int_Status=0;
static uint8 Count=0;   //用于计时
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
     osal_set_event(sapi_TaskID,MY_DETECT_ACTION_EVT);       //再次触发探测事件
  }  

  //-------处理紧急报警事件---------------//
  if(event & MY_URGENT_ALARM_EVT)
  {
    uint8 urgentcall[]="AT+URGENTCALL";
    zb_SendDataRequest( 0x0000, SENSOR_REPORT_CMD_ID, sizeof(urgentcall), urgentcall, 0,0, 0 );
    osal_start_timerEx(sapi_TaskID,MY_URGENT_ALARM_EVT,urgetnDealy);    //每一秒钟向协调器发送一次紧急报警事件
  }
  //-----处理三轴加速度值事件----------//
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
       osal_start_timerEx(sapi_TaskID,MY_REPORT_ACC_EVT,ReportPeriod);      //再次触发报告加速度值事件 
      }
       
  }
  
   //-----处理跌倒报警事件----------//
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
      osal_set_event(sapi_TaskID,MY_URGENT_ALARM_EVT);  //触发紧急报警事件
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
    HalLedBlink(HAL_LED_2,2,50,300);     //调试用作蜂鸣器鸣叫两声  
    //加入网络之后初始化MMA
    MMA845x_Init();
    //初始化2530外部中断P2.0
    Ext_Int_Init();
    
  /*FF中断使能与配置*/  
   InterruptsActive(0,INT_EN_FF_MT_1_MASK,INT_CFG_FF_MT_1_MASK);     //Freefall/Motion中断，管脚映射到INT1
   Conf_FF();          //FreeFall  加速度值低于0.315g并持续30ms将产生自由落体中断
  
  /*Transient中断使能与配置*/
  InterruptsActive(0,INT_EN_TRANS_MASK,INT_CFG_TRANS_MASK);         //Transient中断  管脚映射到INT1
  Conf_Transient();            //配置Transient中断
  Timer1_Init();               //配置定时器1
  osal_set_event(sapi_TaskID,MY_DETECT_ACTION_EVT);       //初始化完后立即触发探测事件
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
  if( strsearch("AT+ACK" ,pData) )       //如果收到应答信号那么停止紧急呼叫事件
 {
   osal_stop_timerEx(sapi_TaskID,MY_URGENT_ALARM_EVT);   //停止紧急报警事件
   osal_stop_timerEx(sapi_TaskID,MY_TUMBLE_ALARM_EVT);
 }
 else if( (position=strsearch("AT+PERIOD" ,pData)) )  //周期范围为0--600
 {
  // osal_stop_timerEx(sapi_TaskID,MY_REPORT_ACC_EVT);
   tempbuf[0]=*(pData+position)-'0';        //提取百位
   tempbuf[1]=*(pData+position+1)-'0';      //提取十位
   tempbuf[2]=*(pData+position+2)-'0';      //提取个位
   ReportPeriod=(tempbuf[0]*100+tempbuf[1]*10+tempbuf[2]);    //默认为毫秒乘1000变为s
   ReportPeriod *= 1000;
   if(ReportPeriod > 0)       //如果发送来的为0则不触发发送加速度事件
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
**函数名称：Get_Acc_Value
**函数功能：采集三轴加速度值
**入口参数：无
**出口参数：无
******************************************************************
*/
void Get_Acc_Value(void)
{
  Xdata=IIC_Read_Word(MMA845x_IIC_ADDRESS,OUT_X_MSB_REG);       //读取X轴值
  Ydata=IIC_Read_Word(MMA845x_IIC_ADDRESS,OUT_Y_MSB_REG);       //读取Y轴值
  Zdata=IIC_Read_Word(MMA845x_IIC_ADDRESS,OUT_Z_MSB_REG);       //读取Z轴值
}
/*********************************************************************
 ** 函数名称: strsearch ()
 ** 函数功能: 在指定的数组里连续找到相同的内容
 ** 入口参数: ptr2要查找的内容, ptr1当前数组
 ** 出口参数: 不为0则找到，为0则没有找到  返回的值为=后数值的位置信息
 *********************************************************************/
static uint8 strsearch(uint8 *ptr2,uint8 *ptr1_at)//查字符串*ptr2在*ptr1中的位置
//本函数是用来检查字符串*ptr2是否完全包含在*ptr1中
//返回:  0  没有找到
//1-255 从第N个字符开始相同
{  
	uint8 i,j,k;
	//uint8 flag;
	if(ptr2[0]==0) return(0);
	//flag=0;
	for(i=0,j=0;i<MAX_sousuo-2;i++)
	{
        	if(ptr1_at[i]==ptr2[j])
       		{	//第一个字符相同
        		for(k=i;k<MAX_sousuo-2;k++,j++)
        		{
        			if(ptr2[j]==0)//比较正确
        				return(j+1);               //返回值是整数，不含0
        			if(ptr1_at[k]!=ptr2[j]) break;
        		}
        		j=0;
        	}
	}
	return(0);
}
/*
*******************************************************************
**函数名称：Ext_Int_Init()
**函数功能：外部中断初始化
**入口参数:无
**出口参数：无
************************************************************************
*/
void Ext_Int_Init(void)
{
  P2INP&=~0X01;     //P2_0上下拉
  P2IEN |= 0X01;    //使能P2_0外部中断
  PICTL|=0X08;      //P2_0下降沿产生中断
  EA=1;
  IEN2 |= 0X02;         //端口2中断使能
  P2IFG&=~0X0F;       //清除中断标志
}

/*********中断服务程序*********/
#pragma vector=P2INT_VECTOR
__interrupt void P2_ISR(void)
{
 P2IFG=0;       //清中断标志  
 Int_Flag=TRUE;   
 //Int_Status=IIC_Reg_Read(MMA845x_IIC_ADDRESS,INT_SOURCE_REG);  //读取中断状态
 P2IF = 0;
}

/*
**************************************************
**函数名称：Timer1_Init()
**函数功能：定时器1初始化
**入口参数：无
**出口参数：无
**说明：T1CCOH=0X2F  T1CC0L=0XFF定时值正好为100ms
        T1CCOH=0X45  T1CC0L=0X00定时值正好为200ms       
***********************************************************
*/
void Timer1_Init(void)
{
  T1CTL |= 0x0E;    //128分频，模模式
  //T1STAT |= 0X21;    //开启溢出中断，通道0有效
  T1CNTH=0;            //清除计数器高位
  T1CNTL=0;            //清除计数器低位
  T1CCTL0 |= 0X40;     //开通道0中断
  T1CCTL0 |= 0X04;     //比较模式 
  T1CC0H=0X2F;
  T1CC0L=0XFF;
  //T1IE=1;     //使能定时器1中断
  EA=1;       //开总中断
}
#pragma vector = T1_VECTOR
 __interrupt void T1_ISR(void)
 {
  IRCON = 0x00;			  //清中断标志,也可由硬件自动完成
  T1STAT=0;                       //清除通道中断标志
  Count++;
  if(Count==20)         //2秒后再提取三轴加速度值与之前比较
  {
      if(Step_One&&Step_Two)
      Step_Three=TRUE;  //置位第三步（比较加速度值标志）
      Count=0;        //计数清零
      T1IE=0;         //关闭计时器
  }

 }
/*
********************************************************************
**函数名称：HexToDec
**函数功能：十六进制转化为十进制
**入口参数：无
**出口参数：无
**********************************************************************
*/
static uint8 Compare(void)
{
 
  if(Xdata>0X7FFF)
  {
    Xdata = ~Xdata+1;       //补码
  }
  if(Ydata>0X7FFF)
  {
    Ydata = ~Ydata+1;       //补码
  }
  if(Zdata>0X7FFF)
  {
    Zdata = ~Zdata+1;       //补码
  }
  //在4g模式中,14位分辨率
  Xdata >>= 2;
  Ydata >>= 2;
  Zdata >>= 2;
  
  //算出加速度真实值  4G模式单位mg
  Xdata /= 2.048;
  Ydata /= 2.048;
  if((Xdata>500)||(1000-Ydata>500)||(Zdata>500))
    return 1;
  else
    return 0;
}
/*
**********************************************************************
**函数名称：Analy_Int
**函数功能：分析中断状态
**入口参数：无
**出口参数：无
***************************************************************************
*/
static void Analy_Int(void)
{
 if(Int_Flag)
  {
    Int_Status=IIC_Reg_Read(MMA845x_IIC_ADDRESS,INT_SOURCE_REG);    //读取中断源状态，判断哪个中断并清除中断标志位
    // -------------------------------------------// 
    /*
    if(Int_Status&0x10)               //PL中断
    {
     IIC_Reg_Read(MMA845x_IIC_ADDRESS,PL_STATUS_REG);     //读取状态位清除状态标识
     PL_Count++;          //启动PL中断后会产生一次中断
     if(PL_Count==2)      //如果在Tranitent中断后检测到两次PL中断则本次跌倒检测无效
     {
     }
    }*/
   //-------------------------------------//   
   /*
    if(Int_Status&0x08)     //Tap中断
    {
    }*/
   //----------------------------------------//   
    if(Int_Status&0x04)     //FreeFall中断
    {
     Step_One=TRUE;       //第一步探测成功
    }
    //------------------------------------------//    
    if(Int_Status&0x20)     //Transient中断
    {
     // BUZZER=1;
      if(Step_One)
      {
       Step_Two=TRUE;       //置位第二步标志
      // BUZZER=1;
      // Time=2;             //3.5s计时  
       T1CNTH=0;            //清除计数器高位
       T1CNTL=0;            //清除计数器低位
       T1IE=1;              //开计时器
    //   InterruptsActive(0,INT_EN_LNDPRT_MASK,INT_CFG_LNDPRT_MASK);//此时打开PL中断L/P中断，管脚映射到INT1
      } 
    }
     //-------------------------------------------------//
    Int_Flag=FALSE;       //清除中断标志
  }
  if(Step_One&&Step_Two&&Step_Three) 
  {
   Xdata=IIC_Read_Word(MMA845x_IIC_ADDRESS,OUT_X_MSB_REG);       //读取X轴值
   Ydata=IIC_Read_Word(MMA845x_IIC_ADDRESS,OUT_Y_MSB_REG);       //读取Y轴值
   Zdata=IIC_Read_Word(MMA845x_IIC_ADDRESS,OUT_Z_MSB_REG);       //读取Z轴值
   if(Compare())          //将此时的初始值比较如果任意一个轴的值与初始值比较变化0.7g         
   {
    Step_Four=TRUE;       //置位第四步标志
   }
            
   else                   //伪跌倒
   {
     Step_One=FALSE;      //清除第一步标志
     Step_Two=FALSE;      //清除第一步标志
     Step_Three=FALSE;    //清除第一步标志
   }
  }
  if(Step_One&&Step_Two&&Step_Three&&Step_Four)       //四步全部满足
  {
    osal_set_event(sapi_TaskID,MY_TUMBLE_ALARM_EVT);    //立即触发跌倒报警事件
    Step_One=FALSE;
    Step_Two=FALSE;
    Step_Three=FALSE;
    Step_Four=FALSE;
  }       
}
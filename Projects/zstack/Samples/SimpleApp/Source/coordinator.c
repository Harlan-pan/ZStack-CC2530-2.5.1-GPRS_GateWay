/******************************************************************************
 * INCLUDES
 */

#include "ZComDef.h"
#include "OSAL.h"
#include "OSAL_Nv.h"
#include "sapi.h"
#include "hal_key.h"
#include "hal_led.h"
#include "hal_lcd.h"
#include "hal_uart.h"
#include "DemoApp.h"
#include "sim900.h"
/******************************************************************************
 * CONSTANTS
 */

#define REPORT_FAILURE_LIMIT                4
#define ACK_REQ_INTERVAL                    5 // each 5th packet is sent with ACK request

// General UART frame offsets
#define FRAME_SOF_OFFSET                    0
#define FRAME_LENGTH_OFFSET                 1 
#define FRAME_CMD0_OFFSET                   2
#define FRAME_CMD1_OFFSET                   3
#define FRAME_DATA_OFFSET                   4

// ZB_RECEIVE_DATA_INDICATION offsets
#define ZB_RECV_SRC_OFFSET                  0
#define ZB_RECV_CMD_OFFSET                  2
#define ZB_RECV_LEN_OFFSET                  4
#define ZB_RECV_DATA_OFFSET                 6
#define ZB_RECV_FCS_OFFSET                  8

// ZB_RECEIVE_DATA_INDICATION frame length
#define ZB_RECV_LENGTH                      15

// PING response frame length and offset
#define SYS_PING_RSP_LENGTH                 7 
#define SYS_PING_CMD_OFFSET                 1

// Stack Profile
#define ZIGBEE_2007                         0x0040
#define ZIGBEE_PRO_2007                     0x0041

#ifdef ZIGBEEPRO
#define STACK_PROFILE                       ZIGBEE_PRO_2007             
#else 
#define STACK_PROFILE                       ZIGBEE_2007
#endif

#define CPT_SOP                             0xFE
#define SYS_PING_REQUEST                    0x0021
#define SYS_PING_RESPONSE                   0x0161
#define ZB_RECEIVE_DATA_INDICATION          0x8746

// Application States
#define APP_INIT                            0
#define APP_START                           2
#define APP_BINDED                          3    

// Application osal event identifiers
#define MY_START_EVT                        0x0001
#define MY_REPORT_EVT                       0x0002
#define MY_FIND_COLLECTOR_EVT               0x0004
/*控制LED2闪烁就是控制蜂鸣器鸣叫*/

/******************************************************************************
 * TYPEDEFS
 */
typedef struct
{
  uint16              source;
  uint16              parent;
  uint8               temp;
  uint8               voltage;
} gtwData_t;

/******************************************************************************
 * LOCAL VARIABLES
 */

//static uint8 appState =             APP_INIT;
static uint8 reportState =          FALSE;
static uint8 myStartRetryDelay =    10;          // milliseconds
static uint8 isGateWay =            FALSE;
static uint16 myBindRetryDelay =    2000;        // milliseconds
//static uint16 parentShortAddr;


/******************************************************************************
 * LOCAL FUNCTIONS
 */



/******************************************************************************
 * GLOBAL VARIABLES
 */

// Inputs and Outputs for Collector device
#define NUM_OUT_CMD_COLLECTOR                2
#define NUM_IN_CMD_COLLECTOR                 2

// List of output and input commands for Collector device
const cId_t zb_InCmdList[NUM_IN_CMD_COLLECTOR] =
{
  SENSOR_REPORT_CMD_ID,
  DUMMY_REPORT_CMD_ID
};

const cId_t zb_OutCmdList[NUM_IN_CMD_COLLECTOR] =
{
  SENSOR_REPORT_CMD_ID,
  DUMMY_REPORT_CMD_ID
};

// Define SimpleDescriptor for Collector device
const SimpleDescriptionFormat_t zb_SimpleDesc =
{
  MY_ENDPOINT_ID,             //  Endpoint
  MY_PROFILE_ID,              //  Profile ID
  DEV_ID_COLLECTOR,           //  Device ID
  DEVICE_VERSION_COLLECTOR,   //  Device Version
  0,                          //  Reserved
  NUM_IN_CMD_COLLECTOR,       //  Number of Input Commands
  (cId_t *) zb_InCmdList,     //  Input Command List
  NUM_OUT_CMD_COLLECTOR,      //  Number of Output Commands
  (cId_t *) zb_OutCmdList     //  Output Command List
};


static uint8 strsearch(uint8 *ptr2,uint8 *ptr1_at);
/******************************************************************************
 * FUNCTIONS
 */

/******************************************************************************
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

  
  if(event & SYS_EVENT_MSG)
  {
    
  }
  
  if ( event & MY_START_EVT )
  {
   
  }
  
  if ( event & MY_REPORT_EVT )
  {

  }
  if ( event & MY_FIND_COLLECTOR_EVT )
  { 
    
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
  //uint8 buf[]="yanjun";
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

    }
    if ( keys & HAL_KEY_SW_2 )
    {
     
    }
    if ( keys & HAL_KEY_SW_1 )
    {

    }
    if ( keys & HAL_KEY_SW_4 )
    {
      
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
    
    
  }
  else
  {
    // Try again later with a delay
    osal_start_timerEx( sapi_TaskID, MY_START_EVT, myStartRetryDelay );
  }
}

/******************************************************************************
 * @fn          zb_SendDataConfirm
 *
 * @brief       The zb_SendDataConfirm callback function is called by the
 *              ZigBee stack after a send data operation completes
 *
 * @param       handle - The handle identifying the data transmission.
 *              status - The status of the operation.
 *
 * @return      none
 */
void zb_SendDataConfirm( uint8 handle, uint8 status )
{
  if ( status != ZB_SUCCESS && !isGateWay ) 
  {
  }
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
  if( status == ZB_SUCCESS )
  {
    //appState = APP_BINDED;
    // Set LED2 to indicate binding successful
    HalLedSet ( HAL_LED_2, HAL_LED_MODE_ON );
    
    // After failure reporting start automatically when the device
    // is binded to a new gateway
    if ( reportState ) 
    {
      // Start reporting
      osal_set_event( sapi_TaskID, MY_REPORT_EVT );
    }
  }
  else
  {
    osal_start_timerEx( sapi_TaskID, MY_FIND_COLLECTOR_EVT, myBindRetryDelay );
  }
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
  // uint8 offset = 0;//用于偏移量计算
 // HalUARTWrite(0,pData, len);
  //透传，发送到服务器
  uint8 *pAddr;
  pAddr = NLME_GetExtAddr();
  for (uint8 i=0, j=7; i<8; i++,j--)
  {
    pData[i+2] = pAddr[j];
  }
  Sim900_Send_GprsData( pData, len );

  /*
  //紧急报警
  if ( offset = strsearch( "AT+URGENTCALL", pData ))
  {
   zb_SendDataRequest( source, NULL, 6 ,
                      "AT+ACK", 0, 0, 0 );
   return ;
  }
  
  //跌倒报警
  if (offset = strsearch( "AT+TUMBLE", pData ))
  {
       zb_SendDataRequest( source, NULL, 6 ,
                      "AT+ACK", 0, 0, 0 );
         return ;
  }
  
  //传感器数据
  if (offset = strsearch( "AT+GETACC=", pData ))
  {
       zb_SendDataRequest( source, NULL, 6 ,
                      "AT+ACK", 0, 0, 0 );
         return ;
  }
  */
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
  //uint8 leng;
  //uint8 revbuf[15];
  //if ( (event ==HAL_UART_RX_TIMEOUT )||(event ==HAL_UART_RX_FULL )||(event ==HAL_UART_RX_ABOUT_FULL ))
 // {
   // leng = HalUARTRead( HAL_UART_PORT_0, revbuf, 15 );
   // zb_SendDataRequest( 0xFFFF, SENSOR_REPORT_CMD_ID, leng, revbuf, 0,0, 0 );
    // HalUARTWrite(0,revbuf,leng);      //将接收到的信息直接用串口发送出去
 // }
      
  //zb_SendDataRequest( 0xFFFF, SENSOR_REPORT_CMD_ID, leng, revbuf, 0,0, 0 );
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



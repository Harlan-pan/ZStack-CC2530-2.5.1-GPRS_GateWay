/******************************************************************************
 * INCLUDES
 */

#include "ZComDef.h"
#include "hal_drivers.h"
#include "OSAL.h"
#include "OSAL_Tasks.h"
//#include "OSAL_Custom.h"
#include "sim900.h"
#if defined ( MT_TASK )
  #include "MT.h"
  #include "MT_TASK.h"
#endif

#include "nwk.h"
#include "APS.h"
#include "ZDApp.h"

#include "osal_nv.h"
#include "NLMEDE.h"
#include "AF.h"
#include "OnBoard.h"
#include "nwk_util.h"
#include "ZDProfile.h"
#include "ZDObject.h"
#include "hal_led.h"
#include "hal_key.h"
#include "sapi.h"
#include "MT_SAPI.h"
#include "DemoApp.h"
#include "hal_uart.h"

extern uint8 zgStartDelay;
extern uint8 zgSapiEndpoint;
/*********************************************************************
 * CONSTANTS
 */

#if !defined OSAL_SAPI
#define OSAL_SAPI  TRUE
#endif

#if !defined SAPI_CB_FUNC
#define SAPI_CB_FUNC  TRUE
#endif

// Message ID's for application user messages must be in 0xE0-0xEF range
#define ZB_USER_MSG                       0xE0
#define SAPICB_DATA_CNF   0xE0
#define SAPICB_BIND_CNF   0xE1
#define SAPICB_START_CNF  0xE2
/*串口有关宏定义*/

#if !defined( SERIAL_APP_PORT )
#define SERIAL_APP_PORT  0
#endif

#if !defined( SERIAL_APP_BAUD )
#define SERIAL_APP_BAUD  HAL_UART_BR_38400
#endif

// When the Rx buf space is less than this threshold, invoke the Rx callback.
#if !defined( SERIAL_APP_THRESH )
#define SERIAL_APP_THRESH  48
#endif

#if !defined( SERIAL_APP_RX_SZ )
#define SERIAL_APP_RX_SZ  128
#endif

#if !defined( SERIAL_APP_TX_SZ )
#define SERIAL_APP_TX_SZ  128
#endif

// Millisecs of idle time after a byte is received before invoking Rx callback.
#if !defined( SERIAL_APP_IDLE )
#define SERIAL_APP_IDLE  6
#endif

// Loopback Rx bytes to Tx for throughput testing.
#if !defined( SERIAL_APP_LOOPBACK )
#define SERIAL_APP_LOOPBACK  FALSE
#endif

// This is the max byte count per OTA message.
#if !defined( SERIAL_APP_TX_MAX )
#define SERIAL_APP_TX_MAX  80
#endif

#define SERIAL_APP_RSP_CNT  4

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

#if OSAL_SAPI
// The order in this table must be identical to the task initialization calls below in osalInitTask.
const pTaskEventHandlerFn tasksArr[] = {
  macEventLoop,
  nwk_event_loop,
  Hal_ProcessEvent,
#if defined( MT_TASK )
  MT_ProcessEvent,
#endif
  APS_event_loop,
  ZDApp_event_loop,

  SAPI_ProcessEvent,
  
#if  defined ( CONTROLLER_GPRS )
   Sim900_ProcessEvent
#endif
};

const uint8 tasksCnt = sizeof( tasksArr ) / sizeof( tasksArr[0] );
uint16 *tasksEvents;
#endif

endPointDesc_t sapi_epDesc;
uint8 sapi_TaskID;
static uint16 sapi_bindInProgress;

/*********************************************************************
 * LOCAL FUNCTIONS
 */

void SAPI_ProcessZDOMsgs( zdoIncomingMsg_t *inMsg );
static void SAPI_SendCback( uint8 event, uint8 status, uint16 data );

static void SAPI_StartConfirm( uint8 status );
static void SAPI_SendDataConfirm( uint8 handle, uint8 status );
static void SAPI_BindConfirm( uint16 commandId, uint8 status );

static void SAPI_ReceiveDataIndication( uint16 source,
                              uint16 command, uint16 len, uint8 *pData  );
static void SAPI_AllowBindConfirm( uint16 source );

/******************************************************************************
 * @fn          zb_SystemReset
 *
 * @brief       The zb_SystemReset function reboots the ZigBee device.  The
 *              zb_SystemReset function can be called after a call to
 *              zb_WriteConfiguration to restart Z-Stack with the updated
 *              configuration.
 *
 * @param       none
 *
 * @return      none
 */
void zb_SystemReset ( void )
{
  SystemReset();
}

/******************************************************************************
 * @fn          zb_StartRequest
 *
 * @brief       The zb_StartRequest function starts the ZigBee stack.  When the
 *              ZigBee stack starts, the device reads configuration parameters
 *              from Nonvolatile memory and the device joins its network.  The
 *              ZigBee stack calls the zb_StartConrifm callback function when
 *              the startup process completes.
 *
 * @param       none
 *
 * @return      none
 */
void zb_StartRequest()
{
  uint8 logicalType;

  zb_ReadConfiguration( ZCD_NV_LOGICAL_TYPE, sizeof(uint8), &logicalType );

  // Check for bad combinations of compile flag definitions and device type setting.
  if ((logicalType > ZG_DEVICETYPE_ENDDEVICE)      ||
#if !ZG_BUILD_ENDDEVICE_TYPE   // Only RTR or Coord possible.
      (logicalType == ZG_DEVICETYPE_ENDDEVICE)     ||
#endif
#if !ZG_BUILD_RTR_TYPE         // Only End Device possible.
      (logicalType == ZG_DEVICETYPE_ROUTER)        ||
      (logicalType == ZG_DEVICETYPE_COORDINATOR)   ||
#elif ZG_BUILD_RTRONLY_TYPE    // Only RTR possible.
      (logicalType == ZG_DEVICETYPE_COORDINATOR)   ||
#elif !ZG_BUILD_JOINING_TYPE   // Only Coord possible.
      (logicalType == ZG_DEVICETYPE_ROUTER)        ||
#endif
      (0))
  {
    logicalType = ZB_INVALID_PARAMETER;
    SAPI_SendCback(SAPICB_START_CNF, logicalType, 0);
  }
  else
  {
    logicalType = ZB_SUCCESS;
    ZDOInitDevice(zgStartDelay);
  }

  
  return;
}

/******************************************************************************
 * @fn          zb_BindDevice
 *
 * @brief       The zb_BindDevice function establishes or removes a 慴inding? *              between two devices.  Once bound, an application can send
 *              messages to a device by referencing the commandId for the
 *              binding.
 *
 * @param       create - TRUE to create a binding, FALSE to remove a binding
 *              commandId - The identifier of the binding
 *              pDestination - The 64-bit IEEE address of the device to bind to
 *
 * @return      The status of the bind operation is returned in the
 *              zb_BindConfirm callback.
 */
void zb_BindDevice ( uint8 create, uint16 commandId, uint8 *pDestination )
{
  zAddrType_t destination;
  uint8 ret = ZB_ALREADY_IN_PROGRESS;

  if ( create )
  {
    if (sapi_bindInProgress == 0xffff)
    {
      if ( pDestination )
      {
        destination.addrMode = Addr64Bit;
        osal_cpyExtAddr( destination.addr.extAddr, pDestination );

        ret = APSME_BindRequest( sapi_epDesc.endPoint, commandId,
                                            &destination, sapi_epDesc.endPoint );

        if ( ret == ZSuccess )
        {
          // Find nwk addr
          ZDP_NwkAddrReq(pDestination, ZDP_ADDR_REQTYPE_SINGLE, 0, 0 );
          osal_start_timerEx( ZDAppTaskID, ZDO_NWK_UPDATE_NV, 250 );
        }
      }
      else
      {
        ret = ZB_INVALID_PARAMETER;
        destination.addrMode = Addr16Bit;
        destination.addr.shortAddr = NWK_BROADCAST_SHORTADDR;
        if ( ZDO_AnyClusterMatches( 1, &commandId, sapi_epDesc.simpleDesc->AppNumOutClusters,
                                                sapi_epDesc.simpleDesc->pAppOutClusterList ) )
        {
          // Try to match with a device in the allow bind mode
          ret = ZDP_MatchDescReq( &destination, NWK_BROADCAST_SHORTADDR,
              sapi_epDesc.simpleDesc->AppProfId, 1, &commandId, 0, (cId_t *)NULL, 0 );
        }
        else if ( ZDO_AnyClusterMatches( 1, &commandId, sapi_epDesc.simpleDesc->AppNumInClusters,
                                                sapi_epDesc.simpleDesc->pAppInClusterList ) )
        {
          ret = ZDP_MatchDescReq( &destination, NWK_BROADCAST_SHORTADDR,
              sapi_epDesc.simpleDesc->AppProfId, 0, (cId_t *)NULL, 1, &commandId, 0 );
        }

        if ( ret == ZB_SUCCESS )
        {
          // Set a timer to make sure bind completes
#if ( ZG_BUILD_RTR_TYPE )
          osal_start_timerEx(sapi_TaskID, ZB_BIND_TIMER, AIB_MaxBindingTime);
#else
          // AIB_MaxBindingTime is not defined for an End Device
          osal_start_timerEx(sapi_TaskID, ZB_BIND_TIMER, zgApsDefaultMaxBindingTime);
#endif
          sapi_bindInProgress = commandId;
          return; // dont send cback event
        }
      }
    }

    SAPI_SendCback( SAPICB_BIND_CNF, ret, commandId );
  }
  else
  {
    // Remove local bindings for the commandId
    BindingEntry_t *pBind;

    // Loop through bindings an remove any that match the cluster
    while ( pBind = bindFind( sapi_epDesc.simpleDesc->EndPoint, commandId, 0 ) )
    {
      bindRemoveEntry(pBind);
    }
    osal_start_timerEx( ZDAppTaskID, ZDO_NWK_UPDATE_NV, 250 );
  }
  return;
}
/******************************************************************************
 * @fn          zb_PermitJoiningRequest
 *
 * @brief       The zb_PermitJoiningRequest function is used to control the
 *              joining permissions and thus allow or disallow new devices from
 *              joining the network.
 *
 * @param       destination - The destination parameter indicates the address
 *                            of the device for which the joining permissions
 *                            should be set. This is usually the local device
 *                            address or the special broadcast address that denotes
 *                            all routers and coordinator ( 0xFFFC ). This way
 *                            the joining permissions of a single device or the
 *                            whole network can be controlled.
 *              timeout -  Indicates the amount of time in seconds for which
 *                         the joining permissions should be turned on.
 *                         If timeout is set to 0x00, the device will turn off the
 *                         joining permissions indefinitely. If it is set to 0xFF,
 *                         the joining permissions will be turned on indefinitely.
 *
 *
 * @return      ZB_SUCCESS or a failure code
 *
 */

uint8 zb_PermitJoiningRequest ( uint16 destination, uint8 timeout )
{
#if defined( ZDO_MGMT_PERMIT_JOIN_REQUEST )
  zAddrType_t dstAddr;

  dstAddr.addrMode = Addr16Bit;
  dstAddr.addr.shortAddr = destination;

  return( (uint8) ZDP_MgmtPermitJoinReq( &dstAddr, timeout, 0, 0 ) );
#else
  (void)destination;
  (void)timeout;
  return ZUnsupportedMode;
#endif
}
/******************************************************************************
 * @fn          zb_SendDataRequest
 *
 * @brief       The zb_SendDataRequest function initiates transmission of data
 *              to a peer device
 *
 * @param       destination - The destination of the data.  The destination can
 *                            be one of the following:
 *                            - 16-Bit short address of device [0-0xfffD]
 *                            - ZB_BROADCAST_ADDR sends the data to all devices
 *                              in the network.
 *                            - ZB_BINDING_ADDR sends the data to a previously
 *                              bound device.
 *
 *              commandId - The command ID to send with the message.  If the
 *                          ZB_BINDING_ADDR destination is used, this parameter
 *                          also indicates the binding to use.
 *
 *              len - The size of the pData buffer in bytes
 *              handle - A handle used to identify the send data request.
 *              txOptions - TRUE if requesting acknowledgement from the destination.
 *              radius - The max number of hops the packet can travel through
 *                       before it is dropped.
 *
 * @return      none
 */
void zb_SendDataRequest ( uint16 destination, uint16 commandId, uint8 len,
                          uint8 *pData, uint8 handle, uint8 txOptions, uint8 radius )
{
  afStatus_t status;
  afAddrType_t dstAddr;

  txOptions |= AF_DISCV_ROUTE;

  // Set the destination address
  if (destination == ZB_BINDING_ADDR)
  {
    // Binding
    dstAddr.addrMode = afAddrNotPresent;
  }
  else
  {
    // Use short address
    dstAddr.addr.shortAddr = destination;
    dstAddr.addrMode = afAddr16Bit;

    if ( ADDR_NOT_BCAST != NLME_IsAddressBroadcast( destination ) )
    {
      txOptions &= ~AF_ACK_REQUEST;
    }
  }

  dstAddr.panId = 0;                                    // Not an inter-pan message.
  dstAddr.endPoint = sapi_epDesc.simpleDesc->EndPoint;  // Set the endpoint.

  // Send the message
  status = AF_DataRequest(&dstAddr, &sapi_epDesc, commandId, len,
                          pData, &handle, txOptions, radius);

  if (status != afStatus_SUCCESS)
  {
    SAPI_SendCback( SAPICB_DATA_CNF, status, handle );
  }
}

/******************************************************************************
 * @fn          zb_ReadConfiguration
 *
 * @brief       The zb_ReadConfiguration function is used to get a
 *              Configuration Protperty from Nonvolatile memory.
 *
 * @param       configId - The identifier for the configuration property
 *              len - The size of the pValue buffer in bytes
 *              pValue - A buffer to hold the configuration property
 *
 * @return      none
 */
uint8 zb_ReadConfiguration( uint8 configId, uint8 len, void *pValue )
{
  uint8 size;

  size = (uint8)osal_nv_item_len( configId );
  if ( size > len )
  {
    return ZFailure;
  }
  else
  {
    return( osal_nv_read(configId, 0, size, pValue) );
  }
}
/******************************************************************************
 * @fn          zb_WriteConfiguration
 *
 * @brief       The zb_WriteConfiguration function is used to write a
 *              Configuration Property to nonvolatile memory.
 *
 * @param       configId - The identifier for the configuration property
 *              len - The size of the pValue buffer in bytes
 *              pValue - A buffer containing the new value of the
 *                       configuration property
 *
 * @return      none
 */
uint8 zb_WriteConfiguration( uint8 configId, uint8 len, void *pValue )
{
  return( osal_nv_write(configId, 0, len, pValue) );
}
/******************************************************************************
 * @fn          zb_GetDeviceInfo
 *
 * @brief       The zb_GetDeviceInfo function retrieves a Device Information
 *              Property.
 *
 * @param       param - The identifier for the device information
 *              pValue - A buffer to hold the device information
 *
 * @return      none
 */
void zb_GetDeviceInfo ( uint8 param, void *pValue )
{
  switch(param)
  {
    case ZB_INFO_DEV_STATE:
      osal_memcpy(pValue, &devState, sizeof(uint8));
      break;
    case ZB_INFO_IEEE_ADDR:
      osal_memcpy(pValue, &aExtendedAddress, Z_EXTADDR_LEN);
      break;
    case ZB_INFO_SHORT_ADDR:
      osal_memcpy(pValue, &_NIB.nwkDevAddress, sizeof(uint16));
      break;
    case ZB_INFO_PARENT_SHORT_ADDR:
      osal_memcpy(pValue, &_NIB.nwkCoordAddress, sizeof(uint16));
      break;
    case ZB_INFO_PARENT_IEEE_ADDR:
      osal_memcpy(pValue, &_NIB.nwkCoordExtAddress, Z_EXTADDR_LEN);
      break;
    case ZB_INFO_CHANNEL:
      osal_memcpy(pValue, &_NIB.nwkLogicalChannel, sizeof(uint8));
      break;
    case ZB_INFO_PAN_ID:
      osal_memcpy(pValue, &_NIB.nwkPanId, sizeof(uint16));
      break;
    case ZB_INFO_EXT_PAN_ID:
      osal_memcpy(pValue, &_NIB.extendedPANID, Z_EXTADDR_LEN);
      break;
  }
}

/******************************************************************************
 * @fn          zb_FindDeviceRequest
 *
 * @brief       The zb_FindDeviceRequest function is used to determine the
 *              short address for a device in the network.  The device initiating
 *              a call to zb_FindDeviceRequest and the device being discovered
 *              must both be a member of the same network.  When the search is
 *              complete, the zv_FindDeviceConfirm callback function is called.
 *
 * @param       searchType - The type of search to perform. Can be one of following:
 *                           ZB_IEEE_SEARCH - Search for 16-bit addr given IEEE addr.
 *              searchKey - Value to search on.
 *
 * @return      none
 */
void zb_FindDeviceRequest( uint8 searchType, void *searchKey )
{
  if (searchType == ZB_IEEE_SEARCH)
  {
    ZDP_NwkAddrReq((uint8*) searchKey, ZDP_ADDR_REQTYPE_SINGLE, 0, 0 );
  }
}
/******************************************************************************
 * @fn          SAPI_StartConfirm
 *
 * @brief       The SAPI_StartConfirm callback is called by the ZigBee stack
 *              after a start request operation completes
 *
 * @param       status - The status of the start operation.  Status of
 *                       ZB_SUCCESS indicates the start operation completed
 *                       successfully.  Else the status is an error code.
 *
 * @return      none
 */
void SAPI_StartConfirm( uint8 status )
{
#if defined ( MT_SAPI_CB_FUNC )
  /* First check if MT has subscribed for this callback. If so , pass it as
  a event to MonitorTest and return control to calling function after that */
  if ( SAPICB_CHECK( SPI_CB_SAPI_START_CNF ) )
  {
    zb_MTCallbackStartConfirm( status );
  }
  else
#endif  //MT_SAPI_CB_FUNC
  {
#if ( SAPI_CB_FUNC )
    zb_StartConfirm( status );
#endif
  }
}

/******************************************************************************
 * @fn          SAPI_SendDataConfirm
 *
 * @brief       The SAPI_SendDataConfirm callback function is called by the
 *              ZigBee after a send data operation completes
 *
 * @param       handle - The handle identifying the data transmission.
 *              status - The status of the operation.
 *
 * @return      none
 */
void SAPI_SendDataConfirm( uint8 handle, uint8 status )
{
#if defined ( MT_SAPI_CB_FUNC )
  /* First check if MT has subscribed for this callback. If so , pass it as
  a event to MonitorTest and return control to calling function after that */
  if ( SAPICB_CHECK( SPI_CB_SAPI_SEND_DATA_CNF ) )
  {
    zb_MTCallbackSendDataConfirm( handle, status );
  }
  else
#endif  //MT_SAPI_CB_FUNC
  {
#if ( SAPI_CB_FUNC )
    zb_SendDataConfirm( handle, status );
#endif
  }
}

/******************************************************************************
 * @fn          SAPI_BindConfirm
 *
 * @brief       The SAPI_BindConfirm callback is called by the ZigBee stack
 *              after a bind operation completes.
 *
 * @param       commandId - The command ID of the binding being confirmed.
 *              status - The status of the bind operation.
 *              allowBind - TRUE if the bind operation was initiated by a call
 *                          to zb_AllowBindRespones.  FALSE if the operation
 *                          was initiated by a call to ZB_BindDevice
 *
 * @return      none
 */
void SAPI_BindConfirm( uint16 commandId, uint8 status )
{
#if defined ( MT_SAPI_CB_FUNC )
  /* First check if MT has subscribed for this callback. If so , pass it as
  a event to MonitorTest and return control to calling function after that */
  if ( SAPICB_CHECK( SPI_CB_SAPI_BIND_CNF ) )
  {
    zb_MTCallbackBindConfirm( commandId, status );
  }
  else
#endif  //MT_SAPI_CB_FUNC
  {
#if ( SAPI_CB_FUNC )
    zb_BindConfirm( commandId, status );
#endif
  }
}
/******************************************************************************
 * @fn          SAPI_AllowBindConfirm
 *
 * @brief       Indicates when another device attempted to bind to this device
 *
 * @param
 *
 * @return      none
 */
void SAPI_AllowBindConfirm( uint16 source )
{
  #if defined ( MT_SAPI_CB_FUNC )
  /* First check if MT has subscribed for this callback. If so , pass it as
  a event to MonitorTest and return control to calling function after that */
  if ( SAPICB_CHECK( SPI_CB_SAPI_ALLOW_BIND_CNF ) )
  {
    zb_MTCallbackAllowBindConfirm( source );
  }
  else
#endif  //MT_SAPI_CB_FUNC
  {
#if ( SAPI_CB_FUNC )
    zb_AllowBindConfirm( source );
#endif
  }
}

/******************************************************************************
 * @fn          SAPI_ReceiveDataIndication
 *
 * @brief       The SAPI_ReceiveDataIndication callback function is called
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
void SAPI_ReceiveDataIndication( uint16 source, uint16 command, uint16 len, uint8 *pData  )
{
#if defined ( MT_SAPI_CB_FUNC )
  /* First check if MT has subscribed for this callback. If so , pass it as
  a event to MonitorTest and return control to calling function after that */
  if ( SAPICB_CHECK( SPI_CB_SAPI_RCV_DATA_IND ) )
  {
    zb_MTCallbackReceiveDataIndication( source, command, len, pData  );
  }
  else
#endif  //MT_SAPI_CB_FUNC
  {
#if ( SAPI_CB_FUNC )
    zb_ReceiveDataIndication( source, command, len, pData  );
#endif
  }
}
/*********************************************************************
 * @fn      SAPI_ProcessEvent
 *
 * @brief   Simple API Task event processor.  This function
 *          is called to process all events for the task.  Events
 *          include timers, messages and any other user defined events.
 *
 * @param   task_id  - The OSAL assigned task ID.
 * @param   events - events to process.  This is a bit map and can
 *                   contain more than one event.
 *
 * @return  none
 */
UINT16 SAPI_ProcessEvent( byte task_id, UINT16 events )
{
  osal_event_hdr_t *pMsg;
  afIncomingMSGPacket_t *pMSGpkt;
  afDataConfirm_t *pDataConfirm;

  if ( events & SYS_EVENT_MSG )
  {
    pMsg = (osal_event_hdr_t *) osal_msg_receive( task_id );
    while ( pMsg )
    {
      switch ( pMsg->event )
      {
        case ZDO_CB_MSG:
          //SAPI_ProcessZDOMsgs( (zdoIncomingMsg_t *)pMsg );
          break;

        case AF_DATA_CONFIRM_CMD:
          // This message is received as a confirmation of a data packet sent.
          // The status is of ZStatus_t type [defined in ZComDef.h]
          // The message fields are defined in AF.h
          pDataConfirm = (afDataConfirm_t *) pMsg;
          SAPI_SendDataConfirm( pDataConfirm->transID, pDataConfirm->hdr.status );
          break;

        case AF_INCOMING_MSG_CMD:
          pMSGpkt = (afIncomingMSGPacket_t *) pMsg;
          SAPI_ReceiveDataIndication( pMSGpkt->srcAddr.addr.shortAddr, pMSGpkt->clusterId,
                                    pMSGpkt->cmd.DataLength, pMSGpkt->cmd.Data);
          break;

        case ZDO_STATE_CHANGE:
          // If the device has started up, notify the application
          if (pMsg->status == DEV_END_DEVICE ||
              pMsg->status == DEV_ROUTER ||
              pMsg->status == DEV_ZB_COORD )
          {
            SAPI_StartConfirm( ZB_SUCCESS );
          }
          if (pMsg->status == DEV_ZB_COORD )
	   {
               osal_start_timerEx( sapi_TaskID,
               SAMPLEAPP_SEND_PERIODIC_MSG_EVT,
               SAMPLEAPP_SEND_PERIODIC_MSG_TIMEOUT );
           }
	   
          else  if (pMsg->status == DEV_HOLD ||
                  pMsg->status == DEV_INIT)
          {
           SAPI_StartConfirm( ZB_INIT );
          }
          break;

        case ZDO_MATCH_DESC_RSP_SENT:
          SAPI_AllowBindConfirm( ((ZDO_MatchDescRspSent_t *)pMsg)->nwkAddr );
          break;

        case KEY_CHANGE:
#if ( SAPI_CB_FUNC )
          zb_HandleKeys( ((keyChange_t *)pMsg)->state, ((keyChange_t *)pMsg)->keys );
#endif
          break;

        case SAPICB_DATA_CNF:
         // SAPI_SendDataConfirm( (uint8)((sapi_CbackEvent_t *)pMsg)->data,
          //                          ((sapi_CbackEvent_t *)pMsg)->hdr.status );
          break;

        case SAPICB_BIND_CNF:
         // SAPI_BindConfirm( ((sapi_CbackEvent_t *)pMsg)->data,
         //                     ((sapi_CbackEvent_t *)pMsg)->hdr.status );
          break;

        case SAPICB_START_CNF:
          SAPI_StartConfirm( ((sapi_CbackEvent_t *)pMsg)->hdr.status );
          break;

        default:
          // User messages should be handled by user or passed to the application
          if ( pMsg->event >= ZB_USER_MSG )
          {

          }
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

  if ( events & ZB_ALLOW_BIND_TIMER )
  {
    afSetMatch(sapi_epDesc.simpleDesc->EndPoint, FALSE);
    return (events ^ ZB_ALLOW_BIND_TIMER);
  }

  if ( events & ZB_BIND_TIMER )
  {
    // Send bind confirm callback to application
    SAPI_BindConfirm( sapi_bindInProgress, ZB_TIMEOUT );
    sapi_bindInProgress = 0xffff;

    return (events ^ ZB_BIND_TIMER);
  }

  // This must be the last event to be processed
  if ( events & ( ZB_USER_EVENTS ) )
  {
    // User events are passed to the application
#if ( SAPI_CB_FUNC )
    zb_HandleOsalEvent( events );
#endif

    // Do not return here, return 0 later
  }
   if ( events & SAMPLEAPP_SEND_PERIODIC_MSG_EVT )
  {
    // Send the periodic message
    if (SIM900_NetworkState == GPRS_CONNECTED)
    {
      //send somthing
    }

    // Setup to send message again in normal period (+ a little jitter)
    osal_start_timerEx( sapi_TaskID, SAMPLEAPP_SEND_PERIODIC_MSG_EVT,
        (SAMPLEAPP_SEND_PERIODIC_MSG_TIMEOUT + (osal_rand() & 0x00FF)) );

    // return unprocessed events
    return (events ^ SAMPLEAPP_SEND_PERIODIC_MSG_EVT);
  }

  // Discard unknown events
  return 0;
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
/*
void SAPI_ProcessZDOMsgs( zdoIncomingMsg_t *inMsg )
{
  switch ( inMsg->clusterID )
  {
    case NWK_addr_rsp:
      {
        // Send find device callback to application
        ZDO_NwkIEEEAddrResp_t *pNwkAddrRsp = ZDO_ParseAddrRsp( inMsg );
        SAPI_FindDeviceConfirm( ZB_IEEE_SEARCH, (uint8*)&pNwkAddrRsp->nwkAddr, pNwkAddrRsp->extAddr );
      }
      break;

    case Match_Desc_rsp:
      {
        zAddrType_t dstAddr;
        ZDO_ActiveEndpointRsp_t *pRsp = ZDO_ParseEPListRsp( inMsg );

        if ( sapi_bindInProgress != 0xffff )
        {
          // Create a binding table entry
          dstAddr.addrMode = Addr16Bit;
          dstAddr.addr.shortAddr = pRsp->nwkAddr;

          if ( APSME_BindRequest( sapi_epDesc.simpleDesc->EndPoint,
                     sapi_bindInProgress, &dstAddr, pRsp->epList[0] ) == ZSuccess )
          {
            osal_stop_timerEx(sapi_TaskID,  ZB_BIND_TIMER);
            osal_start_timerEx( ZDAppTaskID, ZDO_NWK_UPDATE_NV, 250 );

            // Find IEEE addr
            ZDP_IEEEAddrReq( pRsp->nwkAddr, ZDP_ADDR_REQTYPE_SINGLE, 0, 0 );
#if defined ( MT_SAPI_CB_FUNC )
            zb_MTCallbackBindConfirm( sapi_bindInProgress, ZB_SUCCESS );
#endif              
            // Send bind confirm callback to application
#if ( SAPI_CB_FUNC )
            zb_BindConfirm( sapi_bindInProgress, ZB_SUCCESS );
#endif
            sapi_bindInProgress = 0xffff;
          }
        }
      }
      break;
  }
}
*/

/*********************************************************************
 * @fn      SAPI_Init
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
void SAPI_Init( byte task_id )
{
//  halUARTCfg_t uartConfig;
  sapi_TaskID = task_id;
  sapi_bindInProgress = 0xffff;

  sapi_epDesc.task_id = &sapi_TaskID;
  sapi_epDesc.endPoint = 0;
  
#if ( SAPI_CB_FUNC )
  sapi_epDesc.endPoint = zb_SimpleDesc.EndPoint;
  sapi_epDesc.task_id = &sapi_TaskID;
  sapi_epDesc.simpleDesc = (SimpleDescriptionFormat_t *)&zb_SimpleDesc;
  sapi_epDesc.latencyReq = noLatencyReqs;

  // Register the endpoint/interface description with the AF
  afRegister( &sapi_epDesc );
#endif

  // Turn off match descriptor response by default
  afSetMatch(sapi_epDesc.simpleDesc->EndPoint, TRUE);

  // Register callback evetns from the ZDApp
  ///ZDO_RegisterForZDOMsg( sapi_TaskID, NWK_addr_rsp );
  //ZDO_RegisterForZDOMsg( sapi_TaskID, Match_Desc_rsp );

  // Register for HAL events
  RegisterForKeys( sapi_TaskID );
  /*
  uartConfig.configured           = TRUE;              // 2x30 don't care - see uart driver.
  uartConfig.baudRate             = SERIAL_APP_BAUD;
  uartConfig.flowControl          = FALSE;
  uartConfig.flowControlThreshold = SERIAL_APP_THRESH; // 2x30 don't care - see uart driver.
  uartConfig.rx.maxBufSize        = SERIAL_APP_RX_SZ;  // 2x30 don't care - see uart driver.
  uartConfig.tx.maxBufSize        = SERIAL_APP_TX_SZ;  // 2x30 don't care - see uart driver.
  uartConfig.idleTimeout          = SERIAL_APP_IDLE;   // 2x30 don't care - see uart driver.
  uartConfig.intEnable            = TRUE;              // 2x30 don't care - see uart driver.
  uartConfig.callBackFunc         = uartRxCB;
  HalUARTOpen (HAL_UART_PORT_1, &uartConfig);
  */


  // Set an event to start the application
 // osal_set_event(task_id, ZB_ENTRY_EVENT);

}
/*********************************************************************
 * @fn      SAPI_SendCback
 *
 * @brief   Sends a message to the sapi task ( itself ) so that a
 *           callback can be generated later.
 *
 * @return  none
 */
void SAPI_SendCback( uint8 event, uint8 status, uint16 data )
{
  sapi_CbackEvent_t *pMsg;

  pMsg = (sapi_CbackEvent_t *)osal_msg_allocate( sizeof(sapi_CbackEvent_t) );
  if( pMsg )
  {
    pMsg->hdr.event = event;
    pMsg->hdr.status = status;
    pMsg->data = data;

    osal_msg_send( sapi_TaskID, (uint8 *)pMsg );
  }

}

#if OSAL_SAPI
/*********************************************************************
 * @fn      osalInitTasks
 *
 * @brief   This function invokes the initialization function for each task.
 *
 * @param   void
 *
 * @return  none
 */
void osalInitTasks( void )
{
  uint8 taskID = 0;

  tasksEvents = (uint16 *)osal_mem_alloc( sizeof( uint16 ) * tasksCnt);
  osal_memset( tasksEvents, 0, (sizeof( uint16 ) * tasksCnt));

  macTaskInit( taskID++ );
  nwk_init( taskID++ );
  Hal_Init( taskID++ );
#if defined( MT_TASK )
  MT_TaskInit( taskID++ );
#endif
  APS_Init( taskID++ );
  ZDApp_Init( taskID++ );
  SAPI_Init( taskID++ );
  
#if defined ( CONTROLLER_GPRS )   
  Sim900_task_Init(taskID);
#endif
}
#endif

/*********************************************************************
*********************************************************************/


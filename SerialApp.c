/*********************************************************************
* INCLUDES
*/
#include <stdio.h>
#include <string.h>

#include "AF.h"
#include "OnBoard.h"
#include "OSAL_Tasks.h"
#include "SerialApp.h"
#include "ZDApp.h"
#include "ZDObject.h"
#include "ZDProfile.h"
#include "nwk_globals.h"

#include "hal_drivers.h"
#include "hal_key.h"
#include "hal_lcd.h"
#include "hal_led.h"
#include "hal_uart.h"
#include "hal_adc.h"
#include "hal_timer.h"
#include "DHT11.h"

#include "mfrc522.h"
#include "RFID.h"
#include "pwm.h"
#include "TempSens.h"
#include "Sensors.h"
#include "StepMotor.h"
#include "gps.h"
#include "CattleOrientation.h"
#include "gprs.h"

/*********************************************************************
* MACROS
*/
#define COORD_ADDR   0x00
#define ED_ADDR      0x01
#define UART0        0x00
#define MAX_NODE     0x04
#define UART_DEBUG   0x00        //调试宏,通过串口输出协调器和终端的IEEE、短地址


//----------------------------------------------------------------------------

bool virb_in_flag=0,sens_in_flag=0;
DEV_TYPE dev_type = DEV_TYPE_NULL;
SERIAL_TYPE serial_type = SERIAL_TYPE_GPRS_INIT;



uint8 system_time=0;
uint8 is_time_to_report=0;
uint8 period_time=0;
uint8 shut_down_time=SHUT_DOWN_TIME;
uint8 bt_press_down_time=0;
uint8 bt_press_up_time=0;
uint8 system_SOC=0;
EP_INFO ep_addr;


/*********************************************************************
* CONSTANTS
*/

#if !defined( SERIAL_APP_PORT )
#define SERIAL_APP_PORT  0
#endif

#if !defined( SERIAL_APP_BAUD )
//#define SERIAL_APP_BAUD  HAL_UART_BR_3840000000
#define SERIAL_APP_BAUD  HAL_UART_BR_9600
#endif

// When the Rx buf space is less than this threshold, invoke the Rx callback.
#if !defined( SERIAL_APP_THRESH )
#define SERIAL_APP_THRESH  64
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

#define SERIAL_APP_RSP_CNT  6

// This list should be filled with Application specific Cluster IDs.
const cId_t SerialApp_ClusterList[SERIALAPP_MAX_CLUSTERS] =
{
  SERIALAPP_CLUSTERID,
  SERIALAPP_CLUSTERID2,
  SERIALAPP_JSON_CLUSTERID,
  SERIALAPP_GPS_PKG,
  SERIALAPP_END_DEV_INFO,
  SERIALAPP_PACKAGED           //send the message to getway directly
};

const SimpleDescriptionFormat_t SerialApp_SimpleDesc =
{
  SERIALAPP_ENDPOINT,              //  int   Endpoint;
  SERIALAPP_PROFID,                //  uint16 AppProfId[2];
  SERIALAPP_DEVICEID,              //  uint16 AppDeviceId[2];
  SERIALAPP_DEVICE_VERSION,        //  int   AppDevVer:4;
  SERIALAPP_FLAGS,                 //  int   AppFlags:4;
  SERIALAPP_MAX_CLUSTERS,          //  byte  AppNumInClusters;
  (cId_t *)SerialApp_ClusterList,  //  byte *pAppInClusterList;
  SERIALAPP_MAX_CLUSTERS,          //  byte  AppNumOutClusters;
  (cId_t *)SerialApp_ClusterList   //  byte *pAppOutClusterList;
};

const endPointDesc_t SerialApp_epDesc =
{
  SERIALAPP_ENDPOINT,
  &SerialApp_TaskID,
  (SimpleDescriptionFormat_t *)&SerialApp_SimpleDesc,
  noLatencyReqs
};

/*********************************************************************
* TYPEDEFS
*/

/*********************************************************************
* GLOBAL VARIABLES
*/

uint8 SerialApp_TaskID;    // Task ID for internal task/event processing.

/*********************************************************************
* EXTERNAL VARIABLES
*/

/*********************************************************************
* EXTERNAL FUNCTIONS
*/

/*********************************************************************
* LOCAL VARIABLES
*/
static bool SendFlag = 0;

static uint8 SerialApp_MsgID;

static afAddrType_t SerialApp_TxAddr;
static afAddrType_t Broadcast_DstAddr;

static uint8 SerialApp_TxSeq;
static uint8 SerialApp_TxBuf[SERIAL_APP_TX_MAX+1];
static uint8 SerialApp_TxLen=0;

static afAddrType_t SerialApp_RxAddr;
static uint8 SerialApp_RspBuf[SERIAL_APP_RSP_CNT];

static devStates_t SerialApp_NwkState;
static uint8 SerialApp_MsgID;

//暂定5路终端
//如果是协调器，用于保存所有终端的数据
//如果是终端,endDevInfo[0]保留自己的数据
//EndDeviceDataInfo  endDevInfo[MAX_DEVICE];



/*********************************************************************
* LOCAL FUNCTIONS
*/
static void SerialApp_HandleKeys( uint8 shift, uint8 keys );
static void SerialApp_ProcessMSGCmd( afIncomingMSGPacket_t *pkt );
static void SerialApp_Send(void);
static void SerialApp_Resp(void);
static void SerialApp_CallBack(uint8 port, uint8 event);
static void PrintAddrInfo(uint16 shortAddr, uint8 *pIeeeAddr);
static void AfSendAddrInfo(void);
static void GetIeeeAddr(uint8 * pIeeeAddr, uint8 *pStr);
static void SerialApp_SendPeriodicMessage( void );
static uint8 GetDataLen(uint8 fc);
static void parseUartRxData(uint8* data, int len);
static void parseRfData(uint8* data, int len);
uint8 XorCheckSum(uint8 * pBuf, uint8 len);
void IO_init();
uint8 systemSoc(uint8 channel);
void Delay1ms(unsigned int msec);
char NumberToLetter(unsigned char number);
static void SerialApp_SendDataToGetway(uint8* data, int len,uint16 shortAddr);
#if defined(ZDO_COORDINATOR)

static void JsonCreatRfidSN(uint8 *data, uint16 addr);
static void JsonCreatTempHumi(uint8 *data, uint16 addr);
static void JsonCreatTemp(uint8 *data, uint16 addr);
static void JsonCreatIoDetect(uint8 *data, uint16 addr);
static void JsonCreatAiDetect(uint8 *data, uint16 addr);
static void JsonCreatDiDetect(uint8 *data, uint16 addr);
static void JsonCreatDoEdCtrlReturn(uint8 *data, uint16 addr);
static void JsonCreatSensDetected(bool stat);
static void JsonCreatVirbDetected(bool stat);
static void JsonCreatVirbSensDetected(bool virb_stat,bool sens_stat);
static void JsonCreatDoCoordCtrlReturn(uint8 cmd,bool stat);
static void JsonCreatSendEndDevInfo(uint8 *data, uint16 addr);
static void SerialApp_SendDataToEndDevice(uint8* data, int len);

#else

void SerialAppSetColorfulLight(uint8* data, int len);
static char NumberToLetter(unsigned char number);
bool SerialApp_SendDataToCoordinator(uint8* data, uint8 len,uint16 cluster_id);

#endif




/*********************************************************************
* @fn      SerialApp_Init
*
* @brief   This is called during OSAL tasks' initialization.
*
* @param   task_id - the Task ID assigned by OSAL.
*
* @return  none
*/
void SerialApp_Init( uint8 task_id )
{
  halUARTCfg_t uartConfig;
  SerialApp_TaskID = task_id;
#if defined(ZDO_ENDDEVICE)
  //macRadioUpdateTxPower();
#endif
  
  //SerialApp_RxSeq = 0xC3;

//  HalTimerConfig(HAL_TIMER_0,HAL_TIMER_MODE_CTC,HAL_TIMER_CHANNEL_SINGLE,HAL_TIMER_CH_MODE_OUTPUT_COMPARE,TRUE,timer1int);
//  HalTimerStart (HAL_TIMER_0, 1000000);
  afRegister( (endPointDesc_t *)&SerialApp_epDesc );
  
  RegisterForKeys( task_id );
  
  uartConfig.configured           = TRUE;              // 2x30 don't care - see uart driver.
  uartConfig.baudRate             = SERIAL_APP_BAUD;
  uartConfig.flowControl          = FALSE;
  uartConfig.flowControlThreshold = SERIAL_APP_THRESH; // 2x30 don't care - see uart driver.
  uartConfig.rx.maxBufSize        = SERIAL_APP_RX_SZ;  // 2x30 don't care - see uart driver.
  uartConfig.tx.maxBufSize        = SERIAL_APP_TX_SZ;  // 2x30 don't care - see uart driver.
  uartConfig.idleTimeout          = SERIAL_APP_IDLE;   // 2x30 don't care - see uart driver.
  uartConfig.intEnable            = TRUE;              // 2x30 don't care - see uart driver.
  uartConfig.callBackFunc         = SerialApp_CallBack;
  HalUARTOpen (UART0, &uartConfig);
#if defined(ROUTER)
#else  
  IO_init();
#endif
//  serial_type=SERIAL_TYPE_GPRS_SEND;
//  gprsSendMsgToServer(gprs_cmd_nums);
      osal_start_timerEx( SerialApp_TaskID, SERIALAPP_SEND_PERIODIC_EVT,
                       (SERIALAPP_SEND_PERIODIC_TIMEOUT + (osal_rand() & 0x00FF)) );
#if defined ( LCD_SUPPORTED )
  //HalLcdWriteString( "SerialApp", HAL_LCD_LINE_2 );
#endif
 
     
}

/*********************************************************************
* @fn      SerialApp_ProcessEvent
*
* @brief   Generic Application Task event processor.
*
* @param   task_id  - The OSAL assigned task ID.
* @param   events   - Bit map of events to process.
*
* @return  Event flags of all unprocessed events.
*/
UINT16 SerialApp_ProcessEvent( uint8 task_id, UINT16 events )
{
  (void)task_id;  // Intentionally unreferenced parameter
  
  if ( events & SYS_EVENT_MSG )
  {
    afIncomingMSGPacket_t *MSGpkt;
    
    while ( (MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( SerialApp_TaskID )) )
    {
      switch ( MSGpkt->hdr.event )
      {
      case ZDO_CB_MSG:
        //SerialApp_ProcessZDOMsgs( (zdoIncomingMsg_t *)MSGpkt );
        break;
        
      case KEY_CHANGE:
        SerialApp_HandleKeys( ((keyChange_t *)MSGpkt)->state, ((keyChange_t *)MSGpkt)->keys );
        break;
        
      case AF_INCOMING_MSG_CMD:
        SerialApp_ProcessMSGCmd( MSGpkt );
        break;
        
      case ZDO_STATE_CHANGE:
        SerialApp_NwkState = (devStates_t)(MSGpkt->hdr.status);
        if ( (SerialApp_NwkState == DEV_ZB_COORD)
            || (SerialApp_NwkState == DEV_ROUTER)
              || (SerialApp_NwkState == DEV_END_DEVICE) )
        {


#if defined(ZDO_COORDINATOR) //协调器通过串口输出自身短地址、IEEE  
          Broadcast_DstAddr.addrMode = (afAddrMode_t)AddrBroadcast;
          Broadcast_DstAddr.endPoint = SERIALAPP_ENDPOINT;
          Broadcast_DstAddr.addr.shortAddr = 0xFFFF;

#if UART_DEBUG           
          PrintAddrInfo( NLME_GetShortAddr(), aExtendedAddress + Z_EXTADDR_LEN - 1);
#endif 
          
#else                        //终端无线发送短地址、IEEE   
          AfSendAddrInfo();
          //JsonCreatSendEndDevInfo();   
#endif
          
        }
        break;				
      default:
        break;
      }
      
      osal_msg_deallocate( (uint8 *)MSGpkt );
    }
    
    return ( events ^ SYS_EVENT_MSG );
  }
  
  //在此事件中可以定时向协调器发送节点传感器参数信息
  if ( events & SERIALAPP_SEND_PERIODIC_EVT )
  {
  
      SerialApp_SendPeriodicMessage();

    
    osal_start_timerEx( SerialApp_TaskID, SERIALAPP_SEND_PERIODIC_EVT,
                       (SERIALAPP_SEND_PERIODIC_TIMEOUT + (osal_rand() & 0x00FF)) );
    
    return (events ^ SERIALAPP_SEND_PERIODIC_EVT);
  }
  
  if ( events & SERIALAPP_SEND_EVT )
  {
    SerialApp_Send();
    return ( events ^ SERIALAPP_SEND_EVT );
  }
  
  if ( events & SERIALAPP_RESP_EVT )
  {
    SerialApp_Resp();
    return ( events ^ SERIALAPP_RESP_EVT );
  }
  return ( 0 ); 
}

/*********************************************************************
* @fn      SerialApp_HandleKeys
*
* @brief   Handles all key events for this device.
*
* @param   shift - true if in shift/alt.
* @param   keys  - bit field for key events.
*
* @return  none
*/
void SerialApp_HandleKeys( uint8 shift, uint8 keys )
{
  zAddrType_t txAddr;

  if ( keys & HAL_KEY_SW_6 ) //按S1键启动或停止终端定时上报数据 
  {
#if defined(ZDO_COORDINATOR)  //协调器收到指令后的处理函数     
    if(!virb_in_flag)
    {
      virb_in_flag=1;
      _24V_ON=1;               //switch the 24V power on 
      //JsonCreatVirbDetected(1); 
      JsonCreatVirbSensDetected(virb_in_flag,sens_in_flag);
      
    }
#endif  
#ifdef ZIGBEE_SENS_TEMP  //协调器收到指令后的处理函数  
//    temp_sens_model=CHECK_MAC_ADDR_MODE;
//    HalLcdWriteString("     9999    ", HAL_LCD_LINE_1 );
#endif  
    HalUARTWrite(UART0, "AT+GETGPS=\"ALL\"\r", sizeof("AT+GETGPS=\"ALL\r\""));
  }
  
  if ( keys & HAL_KEY_SW_1 ) //按S2
  {
#if defined(ZDO_COORDINATOR)  //协调器收到指令后的处理函数   
    if(!sens_in_flag)
    {
      sens_in_flag=1;
      _24V_ON=1;               //switch the 24V power on 
      //JsonCreatSensDetected(1);   
      JsonCreatVirbSensDetected(virb_in_flag,sens_in_flag);
    }
#endif 
  }

  if ( keys & HAL_KEY_SW_2 )
  {
    HalLedSet ( HAL_LED_4, HAL_LED_MODE_OFF );
    
    // Initiate an End Device Bind Request for the mandatory endpoint
    txAddr.addrMode = Addr16Bit;
    txAddr.addr.shortAddr = 0x0000; // Coordinator
    ZDP_EndDeviceBindReq( &txAddr, NLME_GetShortAddr(), 
                         SerialApp_epDesc.endPoint,
                         SERIALAPP_PROFID,
                         SERIALAPP_MAX_CLUSTERS, (cId_t *)SerialApp_ClusterList,
                         SERIALAPP_MAX_CLUSTERS, (cId_t *)SerialApp_ClusterList,
                         FALSE );
  }
  
  if ( keys & HAL_KEY_SW_3 )
  {
  }
  
  if ( keys & HAL_KEY_SW_4 )
  {
    HalLedSet ( HAL_LED_4, HAL_LED_MODE_OFF );
    
    // Initiate a Match Description Request (Service Discovery)
    txAddr.addrMode = AddrBroadcast;
    txAddr.addr.shortAddr = NWK_BROADCAST_SHORTADDR;
    ZDP_MatchDescReq( &txAddr, NWK_BROADCAST_SHORTADDR,
                     SERIALAPP_PROFID,
                     SERIALAPP_MAX_CLUSTERS, (cId_t *)SerialApp_ClusterList,
                     SERIALAPP_MAX_CLUSTERS, (cId_t *)SerialApp_ClusterList,
                     FALSE );
  }
  
}
//------------------------------------------------------------
void SerialApp_SendDataToGetway(uint8* data, int len,uint16 shortAddr)
{
  uint8 tmp[100]={0};
  if(data==NULL)
  {
    return;
  }
  tmp[0]=0x24;
  tmp[1]=0x40;
  tmp[2]=HI_UINT16(shortAddr);
  tmp[3]=LO_UINT16(shortAddr);
  tmp[4] = ZIGBEE_FUN_CODE_GPS;
  tmp[5]=len;
  osal_memcpy(&tmp[6], data, len);
  tmp[5+len+1]=XorCheckSum(&tmp[2], len+4);
  tmp[5+len+2]='\r';
  tmp[5+len+3]='\n';
  HalUARTWrite(UART0, tmp, len+9);
}
//------------------------------------------------------------
static void SerialApp_SendDataToEndDevice(uint8* data, int len)
{
  if(data==NULL)
  {
    return;
  }
  
  Broadcast_DstAddr.addrMode = (afAddrMode_t)AddrBroadcast;
  Broadcast_DstAddr.endPoint = SERIALAPP_ENDPOINT;
  Broadcast_DstAddr.addr.shortAddr = 0xFFFF;
  
  if (afStatus_SUCCESS == AF_DataRequest(&Broadcast_DstAddr,
                                         (endPointDesc_t *)&SerialApp_epDesc,
                                         SERIALAPP_CLUSTERID,
                                         len, data,
                                         &SerialApp_MsgID, 0, AF_DEFAULT_RADIUS))
  {
    //	    if(FC == 0x0A) //如果开启自动刷新则不需要这步操作
    //	        NodeData[addr-1][3] = SerialApp_TxBuf[len-1];  //更新缓冲区灯的状态
    //	      
    //	    HalUARTWrite(UART0, SerialApp_TxBuf, len+2); //无线发送成功后原样返回给上位机	
    //osal_set_event(SerialApp_TaskID, SERIALAPP_SEND_EVT);
  }
  else  //暂时没发现错误，关闭终端发送也正常。无线发送失败后将数据位和校验位置0返给上位机	
  {
    //	    SerialApp_TxBuf[len-1] = 0x00;
    //	    SerialApp_TxBuf[len] = 0x00;
    //	    HalUARTWrite(UART0, SerialApp_TxBuf, len+2);
  }
}

bool SerialApp_SendDataToCoordinator(uint8* data, uint8 len,uint16 cluster_id)
{
  if(data==NULL)
  {
    return 1;
  }
  SerialApp_TxAddr.addrMode = (afAddrMode_t)Addr16Bit;
  SerialApp_TxAddr.endPoint = SERIALAPP_ENDPOINT;
  SerialApp_TxAddr.addr.shortAddr = 0x00;   
  if ( AF_DataRequest( &SerialApp_TxAddr, (endPointDesc_t *)&SerialApp_epDesc,
                      cluster_id,
                      len,
                      data,
                      &SerialApp_MsgID, 
                      0, 
                      AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
  {
    return 0;    //SUCCESS
  }
  else
  {
    // Error occurred in request to send.
    return 1;
  } 
}


static void parseUartRxData(uint8* data, int len)
{

  if(data==NULL)
  {
    return ;
  }
  
//  int data_len=data[5]; //数据长度  
//  unsigned short addr=((unsigned short)data[2]<<8) |data[3];//终端地址
  uint8 fc=data[4];		//功能码


  switch(fc)
  {
#if defined(ZDO_COORDINATOR)  //协调器收到指令后的处理函数
  case ZIGBEE_FUN_CODE_CUR_OR_VOLT_CTRL:
    CUR_ON = data[6];     //current on control; if data[6] is 0, work in volt model, otherwise in current model
    JsonCreatDoCoordCtrlReturn(ZIGBEE_FUN_CODE_CUR_OR_VOLT_CTRL_RETURN,(bool)data[6]);
    break;
  case ZIGBEE_FUN_CODE_CHECK_CONNECTION: 
    // evade the sensor is already in before turn on the power
    if(!IS_VIRB)    //  if detect the virb is in
    {
      virb_in_flag=1;
    } 
    if(!IS_SENS)    //  if detect the sens  in
    {
      sens_in_flag=1;
    }
    JsonCreatVirbSensDetected(virb_in_flag,sens_in_flag);
    break;
  default:
    SerialApp_SendDataToEndDevice(data,len);
    break;
#endif 
    
  }
}

static void parseRfData(uint8* data, int len)
{
  if(data==NULL)
  {
    return;
  }
//  int data_len=data[5]; //数据长度  
  uint16 addr=((uint16)data[2]<<8) |data[3];//终端地址
  uint8 fc=data[4];		//功能码

  
  //协调器才从串输出
#if defined(ZDO_COORDINATOR)  //协调器收到指令后的处理函数
  int Success;
  switch(fc)
  {
  case ZIGBEE_FUN_CODE_RFID_FIND_READ_CARD:
    JsonCreatRfidSN(data, addr);
    break;  
  case ZIGBEE_FUN_CODE_CHECK_TEMP_HUM:
    JsonCreatTempHumi(data, addr);
    break; 
  case ZIGBEE_FUN_CODE_CHECK_TEMP:
    JsonCreatTemp(data, addr);
    break; 
  case ZIGBEE_FUN_CODE_CHECK_HUMAN:
    JsonCreatIoDetect(data,addr);
    break;
  case ZIGBEE_FUN_CODE_CHECK_LAMP:
    JsonCreatIoDetect(data,addr);
    break;
  case ZIGBEE_FUN_CODE_AI_DETECT:
    JsonCreatAiDetect(data,addr);
    break;  
  case ZIGBEE_FUN_CODE_DI_DETECT:
    JsonCreatDiDetect(data,addr);
    break;   
  case ZIGBEE_FUN_CODE_DO_CTRL_RETURN:
    JsonCreatDoEdCtrlReturn(data,addr);
    break;   
  case ZIGBEE_FUN_CODE_END_DEV_INFO:
    JsonCreatSendEndDevInfo(data,addr);
    break;  
  
  default:
    break;
  }        
  
#else     //终端收到指令后的处理函数
  switch(fc)
  {
    

    
#ifdef ZIGBEE_STEP_MOTOR   //不是传感器节点，而是步进电机节点
  case ZIGBEE_FUN_CODE_STEP:
    
    //终端收到查询命令
    if(addr==ep_addr.short_addr ||addr==0xffff)
    {
      float count=(float)data[7]+(float)((float)data[8]/100);
      Contrl_Step_motor(data[6], count);
    }
    
    if ( AF_DataRequest( &SerialApp_TxAddr, (endPointDesc_t *)&SerialApp_epDesc,
                        SERIALAPP_CLUSTERID,
                        12,
                        data,
                        &SerialApp_MsgID, 
                        0, 
                        AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
    {
      // Successfully requested to be sent.
    }
    else
    {
      // Error occurred in request to send.
    }			
    break;
#endif    
    
    
#ifdef ZIGBEE_RFID      //非传感器节点和步进电机节点
  case ZIGBEE_FUN_CODE_RFID_FINDCARD:
    
    RfidFindCard();
    break;		
  case ZIGBEE_FUN_CODE_RFID_Conflict:
    
    RfidConflict();
    
    break;	
  case ZIGBEE_FUN_CODE_RFID_FIND_READ_CARD:
    
    RfidFindReadCard();
    break;
#endif        //终端类型定义结束   传感器，步进电机，RFID
    
    
#ifdef ZIGBEE_DO_CTRL      //DO节点
  case ZIGBEE_FUN_CODE_DO_CTRL:
    SerialApp_Do_Ctrl(data, len);
    break;		
#endif        //终端类型定义结束   传感器，步进电机，RFID
    
#ifdef ZIGBEE_COLORFUL_LIGHT    //七彩灯
  case ZIGBEE_FUN_CODE_SET_RGB_DATA:
    SerialAppSetColorfulLight(data, len);
    break;	
#endif        //终端类型定义结束   七彩灯 
    
  default:
    break;
  }
  
#endif      //协调器和终端定义结束
  
  
}

int mLed1=0;
void SerialApp_ProcessMSGCmd( afIncomingMSGPacket_t *pkt )
{
  uint16 shortAddr;
  uint8 *pIeeeAddr; 
  //uint8 strIeeeAddr[17] = {0};
  
  uint8 delay;
  uint8 afRxData[100]={0};
  uint8 afDataLen=0; 	
  
    // A message with a serial data block to be transmitted on the serial port.
  osal_memcpy(afRxData, pkt->cmd.Data, pkt->cmd.DataLength);
  afDataLen=pkt->cmd.DataLength;  
  
  //HalUARTWrite(UART0, afRxData, afDataLen);	
  
  shortAddr=pkt->srcAddr.addr.shortAddr;
  //APSME_LookupExtAddr(shortAddr,strIeeeAddr );
  //GetIeeeAddr(strIeeeAddr, strIeeeAddr);
  //查询单个终端上所有传感器的数据 3A 00 01 02 39 23  响应：3A 00 01 02 00 00 00 00 xor 23
  switch ( pkt->clusterId )
  {
   
  case SERIALAPP_CLUSTERID:

     
    
        
    //最短的数据包至少有9个字节
    //sd sd addr addr fc len xor ed ed
    //判断数据头正确,校验也正确
    if(afDataLen>=9 && afRxData[0]=='$' && afRxData[1]=='@' && afRxData[afDataLen-2]=='\r' && 
       afRxData[afDataLen-1]=='\n')
    {
      uint8 xor=0;
      
      //计算校验，不算头、尾和校验位，共5个字节
      xor=XorCheckSum(&afRxData[2], afDataLen-5);
      
      //校验码正确
      if(afRxData[afDataLen-3]==xor)
      {
        //针对数据进行解析
        parseRfData(afRxData, afDataLen);
      }
    }
    
    else
             //闪烁LED1
        if(mLed1==0)
        {
          //亮
           mLed1=1;
           P1_0=0;
        }
        else
        {
          //灭
          mLed1=0;
          P1_0=1;
        }
        
        char buf[16]; 
        memset(buf,0,16);
        //LCD显示信号
        sprintf(buf, "rssi:%d", pkt->rssi);
        HalLcdWriteString(buf, HAL_LCD_LINE_3 );
        
        //串口输出信号
        HalUARTWrite(0, buf, osal_strlen(buf));
        HalUARTWrite(0, "\r\n", 2);
    break;
    
    // A response to a received serial data block.
  case 2:
    if ((pkt->cmd.Data[1] == SerialApp_TxSeq) &&
        ((pkt->cmd.Data[0] == OTA_SUCCESS) || (pkt->cmd.Data[0] == OTA_DUP_MSG)))
    {
      SerialApp_TxLen = 0;
      osal_stop_timerEx(SerialApp_TaskID, SERIALAPP_SEND_EVT);
    }
    else
    {
      // Re-start timeout according to delay sent from other device.
      delay = BUILD_UINT16( pkt->cmd.Data[2], pkt->cmd.Data[3] );
      osal_start_timerEx( SerialApp_TaskID, SERIALAPP_SEND_EVT, delay );
    }
    break;
  case SERIALAPP_JSON_CLUSTERID:
    if(afRxData[0]=='{' && afRxData[afDataLen-1]=='}')
    {
      afRxData[afDataLen]=0;
      HalUARTWrite(UART0, afRxData, afDataLen+1);	  
    }
    break;
  case SERIALAPP_GPS_PKG: 
    //LCD显示
    gpsDisplay(afRxData+6); 
    HalUARTWrite(UART0, afRxData, afDataLen+1);
    SerialApp_SendDataToGetway(afRxData, afDataLen,shortAddr);
    break;
    
  case SERIALAPP_END_DEV_INFO:
    
    break;
  case SERIALAPP_PACKAGED:
    HalUARTWrite(UART0, afRxData, afDataLen);
        //LCD显示
    gpsDisplay(afRxData+6); 
    break;
  default:
    break;
  }
}

#if 0
uint8 TxBuffer[128];

uint8 SendData(uint8 addr, uint8 FC)
{
  uint8 ret, i, index=4;
  
  TxBuffer[0] = 0x3A;
  TxBuffer[1] = 0x00;
  TxBuffer[2] = addr;
  TxBuffer[3] = FC;
  
  switch(FC)
  {
  case 0x01: //查询所有终端传感器的数据
    for (i=0; i<MAX_NODE; i++)
    {
      osal_memcpy(&TxBuffer[index], NodeData[i], 4);
      index += 4;
    }
    TxBuffer[index] = XorCheckSum(TxBuffer, index);
    TxBuffer[index+1] = 0x23; 
    
    HalUARTWrite(UART0, TxBuffer, index+2);
    ret = 1;
    break;
  case 0x02: //查询单个终端上所有传感器的数据
    osal_memcpy(&TxBuffer[index], NodeData[addr-1], 4);
    index += 4;
    TxBuffer[index] = XorCheckSum(TxBuffer, index);
    TxBuffer[index+1] = 0x23; 
    
    HalUARTWrite(UART0, TxBuffer, index+2);		
    ret = 1;
    break;   
  default:
    ret = 0;
    break;
  }
  
  return ret;
}
#endif


/*********************************************************************
* @fn      SerialApp_Send
*
* @brief   Send data OTA.
*
* @param   none
*
* @return  none
*/
static void SerialApp_Send(void)
{
#if SERIAL_APP_LOOPBACK
  if (SerialApp_TxLen < SERIAL_APP_TX_MAX)
  {
    SerialApp_TxLen += HalUARTRead(SERIAL_APP_PORT, SerialApp_TxBuf+SerialApp_TxLen+1,
                                   SERIAL_APP_TX_MAX-SerialApp_TxLen);
  }
  
  if (SerialApp_TxLen)
  {
    (void)SerialApp_TxAddr;
    if (HalUARTWrite(SERIAL_APP_PORT, SerialApp_TxBuf+1, SerialApp_TxLen))
    {
      SerialApp_TxLen = 0;
    }
    else
    {
      osal_set_event(SerialApp_TaskID, SERIALAPP_SEND_EVT);
    }
  }
#else
  if (!SerialApp_TxLen && 
      (SerialApp_TxLen = HalUARTRead(UART0, SerialApp_TxBuf, SERIAL_APP_TX_MAX)))
  {
    if (SerialApp_TxLen>0)
    {
      uint8 UartDataLen=SerialApp_TxLen;
      uint8 UartRxData[SERIAL_APP_TX_MAX]={0};
      

      osal_memcpy(UartRxData, SerialApp_TxBuf, UartDataLen);

      
#if defined(ZDO_COORDINATOR)
#else
      
#ifdef ZIGBEE_GPS     
      
      //SerialApp_SendDataToCoordinator(UartRxData, UartDataLen,SERIALAPP_PACKAGED);
      GpsDataParse((char *)SerialApp_TxBuf, (uint8)UartDataLen);
#endif   
      
     
      
#endif     

      SerialApp_TxLen = 0;
    }
  }
#endif
}

/*********************************************************************
* @fn      SerialApp_Resp
*
* @brief   Send data OTA.
*
* @param   none
*
* @return  none
*/
static void SerialApp_Resp(void)
{
  if (afStatus_SUCCESS != AF_DataRequest(&SerialApp_RxAddr,
                                         (endPointDesc_t *)&SerialApp_epDesc,
                                         SERIALAPP_CLUSTERID2,
                                         SERIAL_APP_RSP_CNT, SerialApp_RspBuf,
                                         &SerialApp_MsgID, 0, AF_DEFAULT_RADIUS))
  {
    osal_set_event(SerialApp_TaskID, SERIALAPP_RESP_EVT);
  }
}

/*********************************************************************
* @fn      SerialApp_CallBack
*
* @brief   Send data OTA.
*
* @param   port - UART port.
* @param   event - the UART port event flag.
*
* @return  none
*/
static void SerialApp_CallBack(uint8 port, uint8 event)
{
  (void)port;
  
  if ((event & (HAL_UART_RX_FULL | HAL_UART_RX_ABOUT_FULL | HAL_UART_RX_TIMEOUT)) &&
#if SERIAL_APP_LOOPBACK
      (SerialApp_TxLen < SERIAL_APP_TX_MAX))
#else
    !SerialApp_TxLen)
#endif
{
  SerialApp_Send();
  
}
}

void SerialApp_SendPeriodicMessage( void )
{
#if defined(ZDO_COORDINATOR)
  SerialApp_SendDataToEndDevice("D1", 2);
  if(virb_in_flag)
  {
      if(IS_VIRB)    //  if detect the sens is not in
      {
        //JsonCreatVirbDetected(0);
        virb_in_flag=0;
        JsonCreatVirbSensDetected(virb_in_flag,sens_in_flag);
        
      } 
  }
  if(sens_in_flag)
  {
      if(IS_SENS)    //  if detect the sens is not in
      {
        //JsonCreatSensDetected(0);
        sens_in_flag=0;
        JsonCreatVirbSensDetected(virb_in_flag,sens_in_flag);
        
      } 
  }
  
  if(virb_in_flag==0&&sens_in_flag==0)    //if detect none:both sens and virb are not inserted
  {
    _24V_ON=0; 
  }
   
   
  if (SerialApp_NwkState != DEV_ZB_COORD)
  {
    return;
  }

#else
  period_time++;
  if(period_time==2)
  {
    period_time = 0;
    is_time_to_report = 1;
    GPS_POWER = 1;
  }


#endif

}



//通过串口输出短地址 IEEE
void PrintAddrInfo(uint16 shortAddr, uint8 *pIeeeAddr)
{
  uint8 strIeeeAddr[17] = {0};
  char  buff[30] = {0};    
  
  //获得短地址   
  sprintf(buff, "shortAddr:%04X   IEEE:", shortAddr);  
  
  //获得IEEE地址
  GetIeeeAddr(pIeeeAddr, strIeeeAddr);
  
  HalUARTWrite (UART0, (uint8 *)buff, strlen(buff));
  HalUARTWrite (UART0, strIeeeAddr, 16); 
  HalUARTWrite (UART0, "\n", 1);
}

void AfSendAddrInfo(void)
{
  uint16 shortAddr;
  uint8 strBuf[31]={0};  
   
  shortAddr=NLME_GetShortAddr();
  ep_addr.short_addr=shortAddr;
  osal_memcpy(&strBuf[6], NLME_GetExtAddr(), 8);
  osal_memcpy(ep_addr.ieee_addr, &strBuf[6], 8);
  
  
  strBuf[0] = 0x24;   
  strBuf[1] = 0x40;   
  strBuf[2] = HI_UINT16( shortAddr );        //存放短地址高8位
  strBuf[3] = LO_UINT16( shortAddr );        //存放短地址低8位
  strBuf[4] = ZIGBEE_FUN_CODE_END_DEV_INFO;
  strBuf[5] = 21;
  strBuf[14]=dev_type;                       // HEAD + short_add + mac_addr + dev_type
  
  osal_memcpy(&strBuf[15], ep_addr.hard_vers, 6);
  osal_memcpy(&strBuf[21], ep_addr.soft_vers, 6);
  strBuf[27]=XorCheckSum(&strBuf[2], 25); 
  strBuf[28]='\r';  
  strBuf[29]='\n'; 
  SerialApp_SendDataToCoordinator(strBuf, 30,SERIALAPP_CLUSTERID);
  
//  strBuf[15]=XorCheckSum(&strBuf[2], 25); 
//  strBuf[16]='\r';  
//  strBuf[17]='\n';  
//  SerialApp_SendDataToCoordinator(strBuf, 18,SERIALAPP_CLUSTERID);

}



#if defined(ZDO_COORDINATOR)                    //if work in cooardinator model
//------------------------------------------------------------创建RFID json串
void JsonCreatRfidSN(uint8 *data, uint16 addr)
{
  uint8 Success,cmd;
  uint8 system_soc=0;
  char rfid_json[100]={0};
  Success=data[6];//响应的第一个字节代表是否成功	
  cmd=data[4];
  system_soc=data[13];
  if(Success==1)
  {
    //BCD转ASC码
    char card_buff[10]={0};
    for(int i=0; i<4; i++)
    {
      unsigned char temp= data[9+i];
      card_buff[i*2]=NumberToLetter((temp>>4)&0x0f);
      card_buff[i*2+1]=NumberToLetter(temp&0x0f);
    }
    
    //LCD显示
    //HalLcdWriteString( card_buff, HAL_LCD_LINE_4 );
    sprintf(rfid_json,"{\"jsonType\":%d,\"cmd\":%d,\"short_addr\":\"%04X\",\"rfidSN\":\"%s\",\"SOC\":%d}",JSON_TYPE_ZIGBEE_TO_GETWAY,cmd,addr,card_buff,system_soc);
  }
  else
  {
     sprintf(rfid_json,"{\"jsonType\":%d,\"cmd\":%d,\"short_addr\":\"%04X\",\"errType\":%d,\"SOC\":%d}",JSON_TYPE_ZIGBEE_TO_GETWAY,ZIGBEE_FUN_CODE_ERR,addr,ERR_CAN_NOT_FIND_RFID_CARD,system_soc);
  }
  HalUARTWrite (UART0, (uint8*)rfid_json, strlen(rfid_json)+1); 
}


//------------------------------------------------------------创建检测到振动传感器插入 json串
void JsonCreatSensDetected(bool stat)
{
  char detect_json[100]={0};
  sprintf(detect_json,"{\"jsonType\":%d,\"cmd\":%d,\"stat\":%d}",JSON_TYPE_ZIGBEE_TO_GETWAY,ZIGBEE_FUN_CODE_SENS_DETECTED,stat);
  HalUARTWrite (UART0, (uint8*)detect_json, strlen(detect_json)+1); 
}

//------------------------------------------------------------创建检测到振动传感器插入 json串
void JsonCreatVirbDetected(bool stat)
{
  char detect_json[100]={0};
  sprintf(detect_json,"{\"jsonType\":%d,\"cmd\":%d,\"stat\":%d}",JSON_TYPE_ZIGBEE_TO_GETWAY,ZIGBEE_FUN_CODE_VIRB_DETECTED,stat);
  HalUARTWrite (UART0, (uint8*)detect_json, strlen(detect_json)+1); 
}
//------------------------------------------------------------创建检测到振动传感器或者温度传感器插入 的json串
void JsonCreatVirbSensDetected(bool virb_stat,bool sens_stat)
{
  char detect_json[100]={0};
  uint8 virb_sens_stat = 0;
  virb_sens_stat = (virb_stat<<1) + sens_stat;
  sprintf(detect_json,"{\"jsonType\":%d,\"cmd\":%d,\"stat\":%d}",JSON_TYPE_ZIGBEE_TO_GETWAY,ZIGBEE_FUN_CODE_VIRB_SENS_DETECT,virb_sens_stat);
  HalUARTWrite (UART0, (uint8*)detect_json, strlen(detect_json)+1); 
}




//------------------------------------------------------------创建温湿度传感器json串
void JsonCreatTempHumi(uint8 *data, uint16 addr)
{
    uint16 Temperature,Humidity;
    uint8 Success,cmd;
    char rfid_json[100]={0};
    Success=data[6];//响应的第一个字节代表是否成功	
    cmd=data[4];
    if(Success==1)
    {
      //成功
      //endDevInfo[addr].Flame=data[7];
      Temperature=((unsigned short)data[7]<<8) |data[8];
      Humidity   =((unsigned short)data[9]<<8) |data[10];
      sprintf(rfid_json,"{\"jsonType\":%d,\"cmd\":%d,\"short_addr\":\"%04X\",\"Temperature\":%d,\"Humidity\":%d}",JSON_TYPE_ZIGBEE_TO_GETWAY,cmd,addr,Temperature,Humidity);
      HalUARTWrite (UART0, (uint8*)rfid_json, strlen(rfid_json)+1);      
    }

}

//------------------------------------------------------------创建人体感应传感器json串
/*void JsonCreatHumanDetect(uint8 *data, uint16 addr)
{
    bool human_stat=0;
    uint8 Success;
    char rfid_json[100]={0};
    Success=data[6];//响应的第一个字节代表是否成功	
    if(Success==1)
    {
      //成功
      human_stat=data[7];
    }
    sprintf(rfid_json,"{\"jsonType\":%d,\"cmd\":%d,\"short_addr\":\"%04X\",\"stat\":%d}",JSON_TYPE_ZIGBEE_TO_GETWAY,ZIGBEE_FUN_CODE_CHECK_HUMAN,addr,human_stat);
    HalUARTWrite (UART0, rfid_json, strlen(rfid_json)+1);
}*/

//------------------------------------------------------------创建继电器状态json串
void JsonCreatIoDetect(uint8 *data, uint16 addr)
{
    bool stat=0;
    uint8 Success,cmd;
    char rfid_json[100]={0};
    Success=data[6];//响应的第一个字节代表是否成功	
    cmd=data[4];
    if(Success==1)
    {
      //成功
      stat=data[7];
      sprintf(rfid_json,"{\"jsonType\":%d,\"cmd\":%d,\"short_addr\":\"%04X\",\"stat\":%d}",JSON_TYPE_ZIGBEE_TO_GETWAY,cmd,addr,stat);
      HalUARTWrite (UART0, (uint8*)rfid_json, strlen(rfid_json)+1);      
    }

}

//------------------------------------------------------------创建8AI json串
void JsonCreatAiDetect(uint8 *data, uint16 addr)
{
    uint16 adc[8]={0};   // store the 8 AI value
    uint8 Success,cmd;
    char rfid_json[200]={0};
    Success=data[6];//响应的第一个字节代表是否成功	
    cmd=data[4];
    if(Success==1)
    {
      int i;
      for(i=0;i<8;i++)
      {
        adc[i]=(data[2*i+7]<<8) |data[2*i+8];
      }
      sprintf(rfid_json,"{\"jsonType\":%d,\"cmd\":%d,\"short_addr\":\"%04X\",\"ch0\":%d,\"ch1\":%d,\"ch2\":%d,\"ch3\":%d,\"ch4\":%d,\"ch5\":%d,\"ch6\":%d,\"ch7\":%d}",JSON_TYPE_ZIGBEE_TO_GETWAY,cmd,addr,adc[0],adc[1],adc[2],adc[3],adc[4],adc[5],adc[6],adc[7]);
      HalUARTWrite (UART0, (uint8*)rfid_json, strlen(rfid_json)+1);      
    }

}

//------------------------------------------------------------创建8DI json串
void JsonCreatDiDetect(uint8 *data, uint16 addr)
{
    uint8 DI=0;   // store the 8 AI value
    uint8 di[9]={0};
    uint8 Success,cmd;
    char rfid_json[200]={0};
    Success=data[6];//响应的第一个字节代表是否成功	
    cmd=data[4];
    if(Success==1)
    {
      DI=data[7];
      /*int i;
      for(i=0;i<8;i++)
      {
        di[i]+=(DI&0x01)+'0';
        DI>>=1; 
      }
      sprintf(rfid_json,"{\"jsonType\":%d,\"cmd\":%d,\"short_addr\":\"%04X\",\"DI\":\"%s\"}",JSON_TYPE_ZIGBEE_TO_GETWAY,cmd,addr,di);
      */
      sprintf(rfid_json,"{\"jsonType\":%d,\"cmd\":%d,\"short_addr\":\"%04X\",\"DI\":%d}",JSON_TYPE_ZIGBEE_TO_GETWAY,cmd,addr,DI);
      HalUARTWrite (UART0, (uint8*)rfid_json, strlen(rfid_json)+1);      
    }

}

//------------------------------------------------------------创建红外测温json串
void JsonCreatTemp(uint8 *data, uint16 addr)
{
    uint16 Temperature;
    uint8 Success,cmd;
    char rfid_json[100]={0};
    Success=data[6];//响应的第一个字节代表是否成功	
    cmd=data[4];
    if(Success==1)
    {
      //成功
      //endDevInfo[addr].Flame=data[7];
      Temperature=((unsigned short)data[7]<<8) |data[8];
      sprintf(rfid_json,"{\"jsonType\":%d,\"cmd\":%d,\"short_addr\":\"%04X\",\"Temperature\":%d}",JSON_TYPE_ZIGBEE_TO_GETWAY,cmd,addr,Temperature);
      HalUARTWrite (UART0, (uint8*)rfid_json, strlen(rfid_json)+1);  
    }

}

//------------------------------------------------------------控制终端反馈json串
void JsonCreatDoEdCtrlReturn(uint8 *data, uint16 addr)
{
    uint8 Success,cmd;
    char rfid_json[100]={0};
    Success=data[6];//响应的第一个字节代表是否成功	
    cmd=data[4];
    sprintf(rfid_json,"{\"jsonType\":%d,\"cmd\":%d,\"short_addr\":\"%04X\",\"result\":%d}",JSON_TYPE_ZIGBEE_TO_GETWAY,cmd,addr,Success);
    HalUARTWrite (UART0, (uint8*)rfid_json, strlen(rfid_json)+1);
}
//------------------------------------------------------------控制协调器反馈json串
void JsonCreatDoCoordCtrlReturn(uint8 cmd,bool stat)
{
    char rfid_json[100]={0};
    sprintf(rfid_json,"{\"jsonType\":%d,\"cmd\":%d,\"short_addr\":\"0000\",\"result\":%d}",JSON_TYPE_ZIGBEE_TO_GETWAY,cmd,stat); 
    HalUARTWrite (UART0, (uint8*)rfid_json, strlen(rfid_json)+1);
}
//------------------------------------------------------------send the end device info
void JsonCreatSendEndDevInfo(uint8 *data, uint16 addr)
{
  char dev_info_json[200]={0};  
  uint8 strIeeeAddr[17] = {0};
  uint16 shortAddr;
  char sw_version[7]={0};
  char hw_version[7]={0};
  shortAddr=data[2]<<8|data[3];
  dev_type=(DEV_TYPE)data[14];
  //获得IEEE地址
  GetIeeeAddr(&data[6]+7, strIeeeAddr);   //从末端往前段转换   data[6] is the start of IEEE addr
  osal_memcpy(hw_version, &data[15], 6);
  osal_memcpy(sw_version, &data[21], 6);
  sprintf((char*)dev_info_json,"{\"jsonType\":%d,\"cmd\":%d,\"IEEE_addr\":\"%s\",\"short_addr\":\"%04X\",\"dev_type\":%d,\"hard_vers\":\"%s\",\"soft_vers\":\"%s\"}",JSON_TYPE_ZIGBEE_TO_GETWAY,ZIGBEE_FUN_CODE_ZIGBEE_ADDR_REGIST,strIeeeAddr,shortAddr,dev_type,hw_version,sw_version);
 
//  sprintf(dev_info_json,"{\"jsonType\":%d,\"cmd\":%d,\"IEEE_addr\":\"%s\",\"short_addr\":\"%04X\",\"dev_type\":%d}",JSON_TYPE_ZIGBEE_TO_GETWAY,ZIGBEE_FUN_CODE_ZIGBEE_ADDR_REGIST,strIeeeAddr,shortAddr,dev_type);
  HalUARTWrite (UART0, (uint8*)dev_info_json, strlen(dev_info_json)+1);
  
}

#endif    //end of coordinater function define


#if defined(ZDO_ENDDEVICE)


//-------------------------------------------------------start of AI dection block
#ifdef ZIGBEE_AI_DETECT
void SerialApp_Send_ADC_Message( void )
{
  uint8 SendBuf[30]={0};  
  float vol=0.0; //adc采样电压  
  byte len=19;
  uint16 adc[8]={0};
  uint8 i;
  for(i=0;i<8;i++)
  {
    adc[i]=HalAdcRead(i, HAL_ADC_RESOLUTION_14);
    //最大采样值8192(因为最高位是符号位)
    {       
      /* //终端输出电压
      vol=(float)((float)(adc[i]*3.3))/8192.0;
      
      //取小数点的3位，放大1000倍
      adc[i]=vol*1000;*/
      adc[i]=((long)adc[i]*3300)>>13;
      
      SendBuf[2*i+7]=adc[i]>>8;            //data
      SendBuf[2*i+8] =adc[i];  //data
    }
  }
  SendBuf[0]='$';
  SendBuf[1]='@';
  SendBuf[2] = HI_UINT16( ep_addr.short_addr );
  SendBuf[3] = LO_UINT16( ep_addr.short_addr );
  SendBuf[4] = ZIGBEE_FUN_CODE_AI_DETECT; //fc  温湿度的响应
  SendBuf[5]=17;   //data len
  SendBuf[6]=1;   //成功
  
  SendBuf[23] = XorCheckSum(&SendBuf[2], 21);
  SendBuf[24] = '\r';
  SendBuf[25] = '\n'; 
  
  if ( AF_DataRequest( &SerialApp_TxAddr, (endPointDesc_t *)&SerialApp_epDesc,
                      SERIALAPP_CLUSTERID,
                      26,
                      SendBuf,
                      &SerialApp_MsgID, 
                      0, 
                      AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
  {
    // Successfully requested to be sent.
  }
  else
  {
    // Error occurred in request to send.
  }
}
#endif              //endif of AD detect block
//----------------------------------------------------


//-------------------------------------------------------start of DI dection block
#ifdef ZIGBEE_DI_DETECT
void SerialApp_Send_DI_Message( void )
{
  uint8 SendBuf[20]={0};  
  SendBuf[0]='$';
  SendBuf[1]='@';
  SendBuf[2] = HI_UINT16( ep_addr.short_addr );
  SendBuf[3] = LO_UINT16( ep_addr.short_addr );
  SendBuf[4] = ZIGBEE_FUN_CODE_DI_DETECT; //fc  温湿度的响应
  SendBuf[5]=2;   //data len
  SendBuf[6]=1;   //成功
  SendBuf[7]=P0;  //detect the 8DI
  SendBuf[8] = XorCheckSum(&SendBuf[2], 6);
  SendBuf[9] = '\r';
  SendBuf[10] = '\n';      

  if ( AF_DataRequest( &SerialApp_TxAddr, (endPointDesc_t *)&SerialApp_epDesc,
                      SERIALAPP_CLUSTERID,
                      11,
                      SendBuf,
                      &SerialApp_MsgID, 
                      0, 
                      AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
  {
    // Successfully requested to be sent.
    //HalUARTWrite(UART0, &SendBuf[7], 1); //无线发送成功后原样返回给上位机	
  }
  else
  {
    // Error occurred in request to send.
  }
}
#endif              //endif of AD detect block
//----------------------------------------------------


//-------------------------------------------------------start of DO CONTROL block
#ifdef ZIGBEE_DO_CTRL
void SerialApp_Do_Ctrl(uint8* data, int len)
{
  P0=data[6];
  uint8 SendBuf[20]={0};  
  SendBuf[0]='$';
  SendBuf[1]='@';
  SendBuf[2] = HI_UINT16( ep_addr.short_addr );
  SendBuf[3] = LO_UINT16( ep_addr.short_addr );
  SendBuf[4] = ZIGBEE_FUN_CODE_DO_CTRL_RETURN; //fc  温湿度的响应
  SendBuf[5]=1;   //data len
  SendBuf[6]=1;   //成功
  SendBuf[7] = XorCheckSum(&SendBuf[2], 5);
  SendBuf[8] = '\r';
  SendBuf[9] = '\n';      

  if ( AF_DataRequest( &SerialApp_TxAddr, (endPointDesc_t *)&SerialApp_epDesc,
                      SERIALAPP_CLUSTERID,
                      10,
                      SendBuf,
                      &SerialApp_MsgID, 
                      0, 
                      AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
  {
    // Successfully requested to be sent.
    //HalUARTWrite(UART0, &SendBuf[7], 1); //无线发送成功后原样返回给上位机	
  }
  else
  {
    // Error occurred in request to send.
  }
}
#endif              //endif of DO control block
//----------------------------------------------------

#ifdef ZIGBEE_COLORFUL_LIGHT    //七彩灯
void SerialAppSetColorfulLight(uint8 *data, int len)
{
  lightCtlModle=*(data+10);
  lightChangeStep=*(data+11);
  if(lightCtlModle==0)
    pwmConfig_RGB(data+6);
}

#endif        //终端类型定义结束   七彩灯 


#endif              //endif of EndDevice block



//-----------------------计算剩余电量-------------------------------

uint8 systemSoc(uint8 channel)
{
  uint16 system_SOC=0;
  system_SOC=HalAdcRead(1, HAL_ADC_RESOLUTION_10);
  system_SOC=(((long)system_SOC*3300)>>9);
  system_SOC=system_SOC*SAMPLE_RATE/LITHIUM_NUMS;   //the result is the SOC of one lithium
  system_SOC=(system_SOC-3200)/10;    //change 3.2~4.2 t0 0~100
  if(system_SOC>100)
    system_SOC=100;
  return system_SOC;
}
//-----------------------BCD转ASC码表-------------------------------
char NumberToLetter(unsigned char number)
{
  char buff[]="0123456789ABCDEF";
  
  if(number>15) return 0;
  
  return buff[number];
  
}
//-----------------------获取mac地址-------------------------------
void GetIeeeAddr(uint8 * pIeeeAddr, uint8 *pStr)
{
  uint8 i;
  uint8 *xad = pIeeeAddr;
  
  for (i = 0; i < Z_EXTADDR_LEN*2; xad--)
  {
    uint8 ch;
    ch = (*xad >> 4) & 0x0F;
    *pStr++ = ch + (( ch < 10 ) ? '0' : '7');
    i++;
    ch = *xad & 0x0F;
    *pStr++ = ch + (( ch < 10 ) ? '0' : '7');
    i++;
  }
}
//-----------------------奇偶校验-------------------------------
uint8 XorCheckSum(uint8 * pBuf, uint8 len)
{
  uint8 i;
  uint8 byRet=0;
  
  if(len == 0)
    return byRet;
  else
    byRet = pBuf[0];
  
  for(i = 1; i < len; i ++)
    byRet = byRet ^ pBuf[i];
  
  return byRet;
}
//-----------------------延时函数-------------------------------
void Delay1ms(unsigned int msec)
{ 
    unsigned int i,j;
    
    for (i=0; i<msec; i++)
        for (j=0; j<530; j++);
}

#if defined(ZDO_COORDINATOR) || defined(ZDO_ENDDEVICE)
/*********************************************************************
*********************************************************************/
/****************************
//初始化IO口程序
*****************************/
void IO_init()
{
#ifdef ZIGBEE_SENSOR
  sensors_init();
#endif 
  
#ifdef ZDO_COORDINATOR
  P0SEL &= ~0XF3;                  //设置P0_DIR普通IO
  P0DIR = P0_DIR;//P0_DIR输出
  P1SEL &= ~P1_DIR;                  //设置P0_DIR普通IO
  P1DIR |= P1_DIR;//P0_DIR输出
  KEEP=TRUE;
  LED=1;
  _24V_ON=0;
  CUR_ON=0;
  while(INT==false);   //if the button stay 0, then stop there
  for(int i=1;i<1;i++)
  {
    Delay1ms(1000);
    if(INT==0)   //the button is pressed
    {
      if(++bt_press_down_time > 10)bt_press_down_time = 10;
    }  
    else
    {
      if(bt_press_down_time>1)
        KEEP=0;
      else
        bt_press_down_time = 0;
    }  
    LED=0;
    Delay1ms(1000);
  }
//  
#endif	  
   
}
#endif	 
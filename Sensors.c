#include <ioCC2530.h>
#include "Sensors.h"
#include "SerialApp.h"
#include "hal_types.h"
#ifdef ZIGBEE_SENSOR
#include "DHT11.h"
void sensors_init(void)
{
  dev_type=DEV_TYPE_SENS;
  P0SEL &= 0xEf;                  //设置P0.4口为普通IO
  P0SEL &= 0xDf;                  //设置P0.5口为普通IO
  P0DIR |= 0x20;                  //设置P0.5为输出
  
  P0SEL &= ~0x40;                 //设置P0.6为普通IO口
  P0DIR &= ~0x40;                 //P0.6定义为输入口
  P0SEL &= 0x7f;                  //P0_7配置成通用io
  
  //p15
  P1SEL &= ~0x20;                 //设置P1.5为普通IO口
  P1DIR &= ~0x20;                 //P1.5定义为输入口
  
  LAMP_PIN = 0;                   //低电平继电器断开;高电平继电器吸合
}

//获得P0_5 继电器引脚的电平
//on  1:亮  0:灭
uint8 GetLamp( void )
{
  uint8 ret;
  
  if(LAMP_PIN == 0)
  {	
    ret = 0;
  }
  else
  {
    ret = 1;
  }
  
  return ret;
}

//on  true:亮  false:灭
void SetLamp(bool on)
{
  LAMP_PIN=(on)?1:0;
}

//获得P0_6 MQ-2气体传感器的数据
//高电平时，气体正常
uint8 GetGas( void )
{
  uint8 ret;
  
  if(GAS_PIN == 0)
    ret = 0;
  else
    ret = 1;
  
  return ret;
}

//获得P0_4 人体红外传感器的数据
//返回,1:有人0：无人
uint8 GetHuman( void )
{
  uint8 ret;
  
  if(HUMAN_PIN == 0)
    ret = 0;
  else
    ret = 1;
  
  return ret;
}

//发送dht11数据
void SerialApp_SendData_DHT11()
{
  uint8 SendBuf[20]={0};
  
  DHT11();                //获取温湿度
  SendBuf[0]='$';
  SendBuf[1]='@';
  SendBuf[2] = HI_UINT16( ep_addr.short_addr );
  SendBuf[3] = LO_UINT16( ep_addr.short_addr );
  SendBuf[4] = ZIGBEE_FUN_CODE_CHECK_TEMP_HUM; //fc  温湿度的响应
  SendBuf[5]=5;   //data len
  SendBuf[6]=1;   //成功
  SendBuf[7]=0;            //data
  SendBuf[8] = wendu;  //data
  SendBuf[9]=0;            //data
  SendBuf[10] = shidu;    //data
  SendBuf[11] = XorCheckSum(&SendBuf[2], 9);
  SendBuf[12] = '\r';
  SendBuf[13] = '\n';
  SerialApp_SendDataToCoordinator(SendBuf, 14,SERIALAPP_CLUSTERID);
}

//发送mq2数据
void SerialApp_SendData_MQ2()
{
  uint8 SendBuf[20]={0};
  
  SendBuf[0]='$';
  SendBuf[1]='@';
  SendBuf[2] = HI_UINT16( ep_addr.short_addr );
  SendBuf[3] = LO_UINT16( ep_addr.short_addr );
  SendBuf[4] = ZIGBEE_FUN_CODE_CHECK_Smoke; //fc  气体的响应
  SendBuf[5]=2;   //data len
  SendBuf[6]=1;   //成功
  SendBuf[7]=GetGas()>0?true:false;  //获取气体传感器的状态  
  SendBuf[8] = XorCheckSum(&SendBuf[2], 6);
  SendBuf[9] = '\r';
  SendBuf[10] = '\n';
  SerialApp_SendDataToCoordinator(SendBuf, 11,SERIALAPP_CLUSTERID);
}

//发送人体数据
void SerialApp_SendData_Human()
{
  uint8 SendBuf[20]={0};
  
  SendBuf[0]='$';
  SendBuf[1]='@';
  SendBuf[2] = HI_UINT16( ep_addr.short_addr );
  SendBuf[3] = LO_UINT16( ep_addr.short_addr );
  SendBuf[4] = ZIGBEE_FUN_CODE_CHECK_HUMAN; //fc  
  SendBuf[5]=2;   //data len
  SendBuf[6]=1;   //成功
  SendBuf[7]=GetHuman()>0?true:false;  //获取人体传感器的状态  
  SendBuf[8] = XorCheckSum(&SendBuf[2], 6);
  SendBuf[9] = '\r';
  SendBuf[10] = '\n';
  
  SerialApp_SendDataToCoordinator(SendBuf, 11,SERIALAPP_CLUSTERID);
}

//发送灯状态数据
void SerialApp_SendData_Lamp(uint8 fc)
{
  uint8 SendBuf[20]={0};
  
  SendBuf[0]='$';
  SendBuf[1]='@'; 
  SendBuf[2] = HI_UINT16( ep_addr.short_addr );
  SendBuf[3] = LO_UINT16( ep_addr.short_addr );
  SendBuf[4] = fc; //fc
  SendBuf[5]=2;   //data len
  SendBuf[6]=1;   //成功
  SendBuf[7]=GetLamp()>0?true:false;  //true:亮
  SendBuf[8] = XorCheckSum(&SendBuf[2], 6);
  SendBuf[9] = '\r';
  SendBuf[10] = '\n'; 
  SerialApp_SendDataToCoordinator(SendBuf, 11,SERIALAPP_CLUSTERID);
}
#endif
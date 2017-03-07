#include <ioCC2530.h>
#include <string.h>
#include "SerialApp.h"
#include "hal_types.h"
#include "RFID.h"
#include "mfrc522.h"
#ifdef ZIGBEE_RFID

bool FindReadRfidErr=1;

//----------------------------------RFID初始化---------
void rfidInit(void)
{
//  P0SEL = MF522_P0SEL;                  //设置P0口为普通IO
  P0DIR = MF522_P0DIR;                  //设置P0.6为输入
  
//  P1SEL = MF522_P1SEL;                  //设置P04口为普通IO
  P1DIR = MF522_P1DIR;                  //设置P0

  KEEP = 1;
  dev_type=DEV_TYPE_RFID;
  memset(ep_addr.hard_vers,0,MAX_BYTES_OF_VERS_INFO);
  memset(ep_addr.soft_vers,0,MAX_BYTES_OF_VERS_INFO);
  strcpy(ep_addr.hard_vers,HARDWARE_VERSION);
  strcpy(ep_addr.soft_vers,SOFTWARE_VERSION);
  
  CmdValid=0; 
  PcdReset();
  PcdAntennaOff(); 
  PcdAntennaOn();  
  M500PcdConfigISOType( 'A' );
}

void RfidFindCard(void)
{
  uint8 SendBuf[20]={0};
  
  RevBuffer[0]=0x02;
  RevBuffer[1]=0x26;
  iccardcode();
  
  //	HalUARTWrite(UART0, RevBuffer, RevBuffer[0]+1);
  
  SendBuf[0]='$';
  SendBuf[1]='@'; 
  SendBuf[2] = HI_UINT16( ep_addr.short_addr );
  SendBuf[3] = LO_UINT16( ep_addr.short_addr );
  SendBuf[4] = ZIGBEE_FUN_CODE_RFID_FINDCARD; //fc
  SendBuf[5]=3;   //data len
  SendBuf[6]=(RevBuffer[1]==0)?1:0;   //成功
  SendBuf[7]=RevBuffer[2];  //data
  SendBuf[8]=RevBuffer[3];  //data
  SendBuf[9] = XorCheckSum(&SendBuf[2], 7);
  SendBuf[10] = '\r';
  SendBuf[11] = '\n';
  SerialApp_SendDataToCoordinator(SendBuf, 12,SERIALAPP_CLUSTERID);
}

void RfidConflict(void)
{
  uint8 SendBuf[20]={0};
  
  RevBuffer[0]=0x03;
  iccardcode();
  
  //	HalUARTWrite(UART0, RevBuffer, RevBuffer[0]+1);
  
  SendBuf[0]='$';
  SendBuf[1]='@'; 
  SendBuf[2] = HI_UINT16( ep_addr.short_addr );
  SendBuf[3] = LO_UINT16( ep_addr.short_addr );
  SendBuf[4] = ZIGBEE_FUN_CODE_RFID_Conflict; //fc
  SendBuf[5]=5;   //data len
  SendBuf[6]=(RevBuffer[1]==0)?1:0;   //成功
  SendBuf[7]=RevBuffer[2];  //data
  SendBuf[8]=RevBuffer[3];  //data
  SendBuf[9]=RevBuffer[4];  //data
  SendBuf[10]=RevBuffer[5];  //data
  SendBuf[11] = XorCheckSum(&SendBuf[2], 9);
  SendBuf[12] = '\r';
  SendBuf[13] = '\n';
  SerialApp_SendDataToCoordinator(SendBuf, 14,SERIALAPP_CLUSTERID);
  
}

void RfidFindReadCard(void)
{
  uint8 SendBuf[20]={0};
  uint8 system_soc=0;
  system_soc=systemSoc(1);
  
  
  RevBuffer[0]=0x02;
  RevBuffer[1]=0x26;
  iccardcode();
  
  if(RevBuffer[1]==0)//寻卡成功
  {
    //读卡命令
    SendBuf[7]=RevBuffer[2];  //data
    SendBuf[8]=RevBuffer[3];  //data        
    
    //找到卡
    //读卡
    RevBuffer[0]=0x03;
    iccardcode();
    
    if(RevBuffer[1]==0)
    {
      //读卡成功
      //记录卡ID
      SendBuf[9]=RevBuffer[2];  //data
      SendBuf[10]=RevBuffer[3];  //data
      SendBuf[11]=RevBuffer[4];  //data
      SendBuf[12]=RevBuffer[5];  //data
      FindReadRfidErr=0;
      SendBuf[13]=system_soc;    //send the soc at the same time
    }
    else
      FindReadRfidErr=1;
  }
  else
    FindReadRfidErr=1;
  
  //	HalUARTWrite(UART0, RevBuffer, RevBuffer[0]+1);
  //if(FindReadRfidErr==0)  //successed
  {
    SendBuf[0]='$';
    SendBuf[1]='@'; 
    SendBuf[2] = HI_UINT16( ep_addr.short_addr );
    SendBuf[3] = LO_UINT16( ep_addr.short_addr );
    SendBuf[4] = ZIGBEE_FUN_CODE_RFID_FIND_READ_CARD; //fc
    SendBuf[5]=8;   //data len
    SendBuf[6]=(RevBuffer[1]==0)?1:0;   //成功
    //SendBuf[7]=RevBuffer[2];  //data
    //SendBuf[8]=RevBuffer[3];  //data
    SendBuf[14] = XorCheckSum(&SendBuf[2], 12);
    SendBuf[15] = '\r';
    SendBuf[16] = '\n';
    
    if(FindReadRfidErr==0)   //读卡成功，则发送
    {
      if (SerialApp_SendDataToCoordinator(SendBuf, 17,SERIALAPP_CLUSTERID)==0)
      {
        // Successfully requested to be sent.
        shut_down_time = system_time;
      }
      else
      {
        FindReadRfidErr=1;
      }    
    }

  }
}

#endif
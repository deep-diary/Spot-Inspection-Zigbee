#include "OnBoard.h"
#include <string.h>
#include <stdio.h>
#include "SerialApp.h"
#include "Hal_types.h"
#ifdef ZIGBEE_GPRS
#include "gprs.h"
#include "gps.h"
#include "CattleOrientation.h"
uint8 gprs_cmd_nums=0;
uint8 gprs_cmd_err=0;
    
const uint8 *AtCmdInit[GPRS_INIT_NUM_MAX]={
  "AT\r",
  "AT+IPR=115200\r",
  "AT+CPIN?\r",
  "AT+CREG?\r",
  "AT+CGATT?\r",
  "AT+CSTT=\"CMNET\"\r",
  "AT+CIICR\r",
};

const uint8 *AtCmdSend[GPRS_SEND_NUM_MAX]={
//  "AT+CIPMUX=0\r",
//  "AT+CSTT=\"CMNET\"\r",
//  "AT+CIICR\r",
//  "AT+CIFSR\r",
  "AT+CIPSTART=\"TCP\",\"120.27.134.154\",\"3333\"\r",
  "AT+CIPSEND=33\r",
  " ",            //this index is to send data to server
  "AT+CIPCLOSE\r",
  "AT+CIPSHUT\r"  
};


bool gprsInit(uint8 idex)
{
  Delay1ms(50);
  if(idex==GPRS_INIT_NUM_MAX)
  {
    serial_type = SERIAL_TYPE_GPS_INIT;
    gprs_cmd_nums=0;
    return 1;
  }

  HalUARTWrite(0, (uint8 *)AtCmdInit[idex], osal_strlen((char *)AtCmdInit[idex])); 
  return 0;
}

bool gprsSendMsgToServer(uint8 idex)
{
  Delay1ms(2000);

  if(idex==2)//ready to send message
  {
    cattleOrientationReport();
    return 0;
  }
  if(idex==GPRS_SEND_NUM_MAX)
  {
    serial_type = SERIAL_TYPE_FINISH_INIT;
    return 1;
  }
    
  HalUARTWrite(0, (uint8 *)AtCmdSend[idex], osal_strlen((char *)AtCmdSend[idex])); 
  return 0;
}
//-------------------------------if something is wrong, then close  all and restart
void gprsClose(void)
{
  serial_type = SERIAL_TYPE_FINISH_INIT;
  HalUARTWrite(0, (uint8 *)AtCmdSend[3], osal_strlen((char *)AtCmdSend[3]));  //  "AT+CIPCLOSE\r",
  Delay1ms(2000);
  HalUARTWrite(0, (uint8 *)AtCmdSend[4], osal_strlen((char *)AtCmdSend[4]));  //   "AT+CIPSHUT\r" 
  Delay1ms(2000);
  gprs_cmd_nums = 0;
  gprs_cmd_err=0;
}

bool gprsCheckAtRst(uint8 *data, char const *checked_bytes0,char const *checked_bytes1, AT_LOGIC logic)
{
  char *p0,*p1;
  bool rst=0;
  p0=strstr(data,checked_bytes0);
  p1=strstr(data,checked_bytes1);
  switch(logic)
  {
    case AT_LOGIC_OR:
      if((p0!=NULL)||(p1!=NULL))
        rst=1;
      break;
    case AT_LOGIC_AND:
      if((p0!=NULL)&&(p1!=NULL))
        rst=1;
      break;
    case AT_LOGIC_NOT:
      if((p0!=NULL)&&(p1==NULL))
        rst=1;
      break;
    default:
      break;
  }
  return rst;
}

bool gprsCheckAtRstSingle(uint8 *data, uint8 *checked_bytes)
{
  char *p;
  bool rst=0;
  p=strstr(data,checked_bytes);
  if((p!=NULL))
     rst=1;
  return rst;
}

#endif
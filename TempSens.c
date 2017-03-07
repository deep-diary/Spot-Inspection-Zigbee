#include "SerialApp.h"
#ifdef ZIGBEE_SENS_TEMP
#include <ioCC2530.h>
#include <string.h>
#include <stdio.h>
#include "TempSens.h"
#include "hal_lcd.h"
#include "hal_uart.h"
#include "hal_types.h"
#include "hal_adc.h"

uint8 temp_sens_model=TEMP_DETECT_MODE;

void temp_sens_init(void)
{
//  uint8 tmp_buff[16]={0};
  uint8 system_SOC= 0;
//  P1SEL &= ~0x03;                  //LED=P1.0,KEEP=P1.1,SOC=P1.2
  P1DIR |= TEMP_SENS_P1DIR;                    //LED=OUT,KEEP=OUT.SOC=IN
  P1DIR &= ~0x01;                    //LED=OUT,KEEP=OUT.SOC=IN
  KEEP=1;
  
  ep_addr.dev_type=DEV_TYPE_TEMP;
  memset(ep_addr.hard_vers,0,MAX_BYTES_OF_VERS_INFO);
  memset(ep_addr.soft_vers,0,MAX_BYTES_OF_VERS_INFO);
  strcpy(ep_addr.hard_vers,HARDWARE_VERSION);
  strcpy(ep_addr.soft_vers,SOFTWARE_VERSION);
  
  uint8 buff[3]={0XB9,0X01,0xB8};
  HalUARTWrite (HAL_UART_PORT_0, (uint8 *)buff, 3);  //open the lesa
  system_SOC=systemSoc(1);
  
#ifdef ENGLISH 
              HalLcdWriteString("      TEMP      ", HAL_LCD_LINE_1 );
#else 
              HalLcdWriteString("      温度      ", HAL_LCD_LINE_1 ); 
#endif   
//    system_SOC=10;
    HalLcdDispSOC(0,0,system_SOC);          
    HalLcdDispString(111, 0,"掉",CODE_16_16);
    Draw_BMP(16,2,112,8);
  
}

bool SerialApp_SendData_Temp(uint8 *temp)
{
  uint8 SendBuf[20]={0};
  int16 tmp;
  tmp=temp[0]<<8|temp[1];
  tmp=tmp-1000;    //得到放大10倍的温度值
  SendBuf[0]='$';
  SendBuf[1]='@';
  SendBuf[2] = HI_UINT16( ep_addr.short_addr );
  SendBuf[3] = LO_UINT16( ep_addr.short_addr );
  SendBuf[4] = ZIGBEE_FUN_CODE_CHECK_TEMP; //fc  温湿度的响应
  SendBuf[5]=5;   //data len
  SendBuf[6]=1;   //成功
  SendBuf[7]=tmp>>8;            //data
  SendBuf[8] = tmp;  //data
  SendBuf[9] = XorCheckSum(&SendBuf[2], 7);
  SendBuf[10] = '\r';
  SendBuf[11] = '\n';
  
  char disp_buff[8]={0};
  
  // --------------------------when reach the shut down time,then display nothing
  //if(system_time<SHUT_DOWN_TIME-1)    
  {
    float f_tmp; 
    LcdClearLine(2,6);
#ifdef FAHRENHEIT      //华氏度 
    f_tmp = (float)tmp*1.8/10+32;
    sprintf(disp_buff,"%3.1f℉",f_tmp);
#else
    f_tmp = (float)tmp*1.0/10;
    sprintf(disp_buff,"%3.1f℃",f_tmp);
#endif
    
    if(f_tmp>100||f_tmp<-10)   //---------------------------------change the disp position
    {
      HalLcdDispString(8,3,(uint8*)disp_buff,CODE_32_32);
    }
    else
    {
      HalLcdDispString(16,3,(uint8*)disp_buff,CODE_32_32); 
    }  
  }
  
  if(INT)  //the button is released
  {
    shut_down_time=system_time;
    if(SerialApp_SendDataToCoordinator(SendBuf, 12,SERIALAPP_CLUSTERID)==0)   //success
    {
      
#ifdef ENGLISH 
              HalLcdWriteString("UPLOARD SUCCESS ", HAL_LCD_LINE_1 );
#else 
              HalLcdWriteString("    上传成功    ", HAL_LCD_LINE_1 );
//              HalLcdDispString(16,0,"  上传成功",CODE_16_16);
#endif   
        return 0;
    }
    else
    {
      // Error occurred in request to send.
#ifdef ENGLISH 
              HalLcdWriteString(" UPLOARD FAILED ", HAL_LCD_LINE_1 );
#else 
              HalLcdWriteString("    上传失败    ", HAL_LCD_LINE_1 );
//              HalLcdDispString(16,0,"  上传成功",CODE_16_16);
#endif
        return 0;
    }  
  }
  else
  {
     bt_press_up_time = 0;
     return 1;
  }


}

#endif
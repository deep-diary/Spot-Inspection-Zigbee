/*********************************************************************
* INCLUDES
*/
#include <stdio.h>
#include <string.h>

#include "AF.h"
#include "OnBoard.h"
#include "OSAL_Tasks.h"
#include "ZDApp.h"
#include "ZDObject.h"
#include "ZDProfile.h"

#include "hal_drivers.h"
#include "hal_key.h"
#include "hal_led.h"
#include "hal_uart.h"
#include "nwk_globals.h"
#include "pwm.h"
#include "SerialApp.h"

#ifdef ZIGBEE_COLORFUL_LIGHT

//P1_1	红色  通道1
//P1_0	绿色  通道2
//P0_7  蓝色  通道3

//PWM最大计数周期
#define MAX_PWM_CYCLE         1500

//调整这个值，可以调整灯的功率
//这个值越大，功率越大
//不能大于上面的值MAX_PWM_CYCLE
//18欧限流的时候，1000达到了1W,1200，约1.3
#define MAX_PWM_UP_CYCLE 1200

//PWM最大级别，这里定义256级，即0~255，和RGB颜色是一致的。
#define MAX_PWM_LEVEL         255

static uint8 CurRed=150;
static uint8 CurGreen=120;
static uint8 CurBlue=200;
static uint8 CurBrightness=32;
static uint8 index=0;
uint8 lightCtlModle=1;
uint8 lightChangeStep=255;

extern uint16 HalUARTWrite(uint8 port, uint8 *buf, uint16 len);

/*使用P1_0口为输出、外设端口，来输出PWM波形*/
void init_port(void)
{
    P1DIR |= 0x03;    // p10,p11 输出output
    P1SEL |= 0x03;    // p10,p11 外设 peripheral

    P0DIR |= 0x81;    // p07 输出output  P00输出  KEEP
    P0SEL |= 0x80;    // p07 外设 peripheral
    
    
    P2DIR |= 0xC0;//定时器1通道2~3优先
    PERCFG|= 0x40; // set timer_1 I/O位置为2
    
    KEEP=1;
}

/*
    将基准值放入T1CC0 寄存器, 将被比较值放入T1CC2寄存器
    当T1CC2中的值与T1CC0中的值相等时，则T1CC2 设置or清除
*/
void init_timer(void)
{
    //初始化计数值
    T1CNTL=0;
    T1CNTH=0;

    //总的计数
    
    T1CC0L = LO_UINT16(MAX_PWM_CYCLE);
    T1CC0H = HI_UINT16(MAX_PWM_CYCLE);

    //P1_1	蓝色  通道1
    B_COUNT_L = 0x7f;  //  通道1   PWM signal period 占空比
    B_COUNT_H = 0x02;
    T1CCTL1 = 0x34;    // 设置向上比较

    //P1_0	绿色  通道2
    G_COUNT_L = 0x7f;  //  通道2   PWM signal period 占空比
    G_COUNT_H = 0x02;
    T1CCTL2 = 0x34;    // 设置向上比较


    //P0_7  红色  通道3
    R_COUNT_L = 0x7f;  //  通道3   PWM signal period 占空比
    R_COUNT_H = 0x02;
    T1CCTL3 = 0x34;    // 设置向上比较

    //128分频,模模式，上下计数
    T1CTL |= 0x0E; // divide with 128 and to do i up-down mode
}

void start_pwm(void) 
{
    init_port();
    init_timer();
    pwmConfig_Red(CurRed);
    pwmConfig_Green(CurGreen);
    pwmConfig_Blue(CurBlue);
}

uint8 getCurRed()
{
  return CurRed;
}

uint8 getCurGreen()
{
  return CurGreen;
}

uint8 getCurBlue()
{
  return CurBlue;
}

uint8 getCurBrightness()
{
  return CurBrightness;
}

//根据界面设置的亮度,
//计算当前亮度
uint16 CalcCurBrightness()
{
    return(uint16)((float) ((float)MAX_PWM_UP_CYCLE/(float)MAX_PWM_LEVEL))*CurBrightness;
}


//level 0~255，线性变化
//这里是控制P10，即绿色
void pwmConfig_Green(uint8 level)
{
    uint16 curLevel=MAX_PWM_CYCLE;//默认是灭的
 
    CurGreen=level;

    //确保这个值在0~MAX_PWM_LEVEL
    if(level>MAX_PWM_LEVEL)
    {
      level=level%MAX_PWM_LEVEL;
    }

    //计算出当前的占空比对应的值
    if(level>=0)
    {
      curLevel=((float)((float)CalcCurBrightness()/(float)MAX_PWM_LEVEL))*level;
    }

    if(curLevel>MAX_PWM_CYCLE)
    {
        curLevel=MAX_PWM_CYCLE;
    }
    if(curLevel==0)
    {
        curLevel=1;
    }

#ifdef PWM_TEST    
    {
        uint8 buff[20]={0};
        sprintf(buff, "green:%d.\r\n", curLevel);
        HalUARTWrite(0,buff, osal_strlen(buff));
    }
#endif
    
  //  T1CTL &= ~0x03;    
    G_COUNT_L = LO_UINT16(curLevel);  //  通道2   PWM signal period 占空比
    G_COUNT_H = HI_UINT16(curLevel);
 //   T1CTL |= 0x03;    
}

//level 0~255，线性变化
//这里是控制P11，即以红色
void pwmConfig_Red(uint8 level)
{
    uint16 curLevel=MAX_PWM_CYCLE;//默认是灭的
 
    CurRed=level;
    
    //确保这个值在0~MAX_PWM_LEVEL
    if(level>MAX_PWM_LEVEL)
    {
      level=level%MAX_PWM_LEVEL;
    }

    //计算出当前的占空比对应的值
    if(level>=0)
    {
      curLevel=((float)((float)CalcCurBrightness()/(float)MAX_PWM_LEVEL))*level;
    }

    if(curLevel>MAX_PWM_CYCLE)
    {
        curLevel=MAX_PWM_CYCLE;
    }
    if(curLevel==0)
    {
        curLevel=1;
    }

#ifdef PWM_TEST
    {
        uint8 buff[20]={0};
        sprintf(buff, "red:%d.\r\n", curLevel);
        HalUARTWrite(0,buff, osal_strlen(buff));
    }
#endif    
    
  //  T1CTL &= ~0x03;    
    B_COUNT_L = LO_UINT16(curLevel);  //  通道1   PWM signal period 占空比
    B_COUNT_H = HI_UINT16(curLevel);
//    T1CTL |= 0x03;    
}

//level 0~255，线性变化
//这里是控制P07，即蓝色
void pwmConfig_Blue(uint8 level)
{
    uint16 curLevel=MAX_PWM_CYCLE;//默认是灭的
 
    CurBlue=level;
    
    //确保这个值在0~MAX_PWM_LEVEL
    if(level>MAX_PWM_LEVEL)
    {
      level=level%MAX_PWM_LEVEL;
    }

    //计算出当前的占空比对应的值
    if(level>=0)
    {
      curLevel=((float)((float)CalcCurBrightness()/(float)MAX_PWM_LEVEL))*level;
    }

    if(curLevel>MAX_PWM_CYCLE)
    {
        curLevel=MAX_PWM_CYCLE;
    }
    if(curLevel==0)
    {
        curLevel=1;
    }

#ifdef PWM_TEST    
    {
        uint8 buff[20]={0};
        sprintf(buff, "blue:%d.\r\n", curLevel);
        HalUARTWrite(0,buff, osal_strlen(buff));
    }
#endif
    
  //  T1CTL &= ~0x03;    
    R_COUNT_L = LO_UINT16(curLevel);  //  通道3   PWM signal period 占空比
    R_COUNT_H = HI_UINT16(curLevel);
  //  T1CTL |= 0x03;    
}


//设置亮度
void pwmConfig_Brightness(uint8 level)
{
    CurBrightness=level;
}

void pwmConfig_RGB(uint8 *RGB)      //传入RGB，亮度数组，4个字节
{
  pwmConfig_Red(*RGB++);
  pwmConfig_Green(*RGB++);
  pwmConfig_Blue(*RGB++);
  pwmConfig_Brightness(*RGB);
}

void LightGradualChange(uint8 step)
{
//    uint8[4]=rgb;   //4bit:R,G,B,brightness 
    uint16 red=0,green=0,blue=0;
    red=getCurRed();
    green=getCurGreen();
    blue=getCurBlue();   
    if(index==0)
    {
        //0:红-->绿
      red>step? (red-=step) : (red=1);
      blue>step ? (blue-=step) : (blue=1);
      green<255-step ? (green+=step) : (green=255); 
        if(red==1 && blue==1 && green==255)
        {
            //转成下一步
            index=1;
        }
    }
    else if(index==1)
    {
        // 1:绿-->蓝
      green>step ? (green-=step) : (green=1);
      red>step ? (red-=step) : (red=1);
      blue<255-step ? (blue+=step) : (blue=255); 
        if(red==1 && green==1 && blue==255)
        {
            //转成下一步
            index=2;
        }
    }
    else if(index==2)
    {
        // 2:蓝-->白
      red<255-step ? (red+=step) : (red=255); 
      green<255-step ? (green+=step) : (green=255); 
      blue<255-step ? (blue+=step) : (blue=255); 
        if(red==255 && green==255 && blue==255)
        {
            //转成下一步
            index=3;
        }
    }
    else
    {
        // 3:白-->红
      green>step ? (green-=step) : (green=1);
      blue>step ? (blue-=step) : (blue=1);
      red<255-step ? (red+=step) : (red=255); 
        if(red==255 && green==1 && blue==1)
        {
            //转成下一步
            index=0;
        }		
    }
    pwmConfig_Red(red);
    pwmConfig_Green(green);
    pwmConfig_Blue(blue);
}

void LightSevenColorChange(uint8 color_num)   //color_num belongs to 0~7
{
  uint8 red=0,green=0,blue=0;

  if(color_num>7)
    color_num%=7;
  if(color_num<1)
    color_num=1;
  if(color_num&0x01)
    blue=255;
  if(color_num&0x02)
    green=255;
  if(color_num&0x04)  
    red=255;
  pwmConfig_Red(red);
  pwmConfig_Green(green);
  pwmConfig_Blue(blue);
}

void LightOnModle(uint8 modle_num,uint8 step,uint8 color_num)   //
{
  switch(modle_num)
  {
    case 1:
      LightGradualChange(step);
      break;
    case 2:
      LightSevenColorChange(color_num);
      break;
    default:
      break;
    
  }

}
#endif
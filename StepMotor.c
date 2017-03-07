#include <ioCC2530.h>
#include "SerialApp.h"
#include "hal_types.h"
#include "StepMotor.h"
#ifdef ZIGBEE_STEP_MOTOR


///////////////////步进电机定义/////////////////////////////
//**********************正向旋转相序表*****************************
uint8 FFW[8]={0x80,0xc0,0x40,0x60,0x20,0x30,0x10,0x90};
//**********************反向旋转相序表*****************************
uint8 REV[8]={0x90,0x10,0x30,0x20,0x60,0x40,0xc0,0x80};
DEV_TYPE dev_type=DEV_TYPE_STEP_MOTOR;

//**********************步进电机初始化******************************
void step_motor_init(void)
{
  P0SEL &= 0x0f;  //P0_4、P0_5、P0_6、P0_7配置成通用io
  P0DIR |= 0xF0; //P0_4、P0_5、P0_6、P0_7定义为输出
}

//**********************步进电机正转******************************
static void motor_up(float n)
{
  unsigned char i;
  unsigned int j;
  int movie_count=(int)(8*64*n);
  for (j=0; j<movie_count; j++)
  {
    for (i=0; i<8; i++)
    {
      P0 = FFW[i];
      Delay_ms(20);
    }
  }
}

//*********************步进电机反转********************************
static void motor_down(float n)
{
  unsigned char i;
  unsigned int j;
  int movie_count=(int)(8*64*n);
  for (j=0; j<movie_count; j++)
  {
    for (i=0; i<8; i++)
    {
      P0 = REV[i];
      Delay_ms(20);
    }
  }
}
//*********************步进电机控制********************************
void Contrl_Step_motor(int state, float count)
{
  if(state>0)
  {
    motor_down(count);
  }
  else
  {
    motor_up(count);
  }
}

static void Delay_ms(unsigned int msec)
{ 
    unsigned int i,j;
    
    for (i=0; i<msec; i++)
        for (j=0; j<530; j++);
}
#endif
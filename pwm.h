/*********************************************************************
* INCLUDES
*/
#ifndef _PWM_H_
#define _PWM_H_
#include "SerialApp.h"
#ifdef ZIGBEE_COLORFUL_LIGHT

#define LITHIUM_NUMS    1       //1节电池
#define SAMPLE_RATE     2       //1/2采样
#define SOFTWARE_VERSION     "20170224"       //软件版本
#define HARDWARE_VERSION     "20170224"       //硬件版本

//测试宏,串口输出相应的信息
#define PWM_TEST

#define KEEP	  P0_0        //direction:out，kill the power
#define INT       P0_1        //direction:in，signal of killing power
#define G_COUNT_L  T1CC1L   //P1_1	蓝色  通道1
#define G_COUNT_H  T1CC1H

#define B_COUNT_L T1CC2L   //P1_0	红色  通道2
#define B_COUNT_H T1CC2H

#define R_COUNT_L T1CC3L  //P0_7	绿色  通道3
#define R_COUNT_H T1CC3H


extern uint8 lightCtlModle;

extern uint8 lightChangeStep;
//启动PWM
extern void start_pwm(void);
//取得当前红色级别
uint8 getCurRed();
//取得当前绿色级别
uint8 getCurGreen();
//取得当前蓝色级别
uint8 getCurBlue();
//取得当前亮度级别
uint8 getCurBrightness();
//配置当前绿色级别
void pwmConfig_Green(uint8 level);
//配置当前蓝色级别
void pwmConfig_Blue(uint8 level);
//配置当前红色级别
void pwmConfig_Red(uint8 level);
//配置当前亮度级别
void pwmConfig_Brightness(uint8 level);
//配置当前RGB亮度级别
extern void pwmConfig_RGB(uint8 *RGB);
//渐变模式
void LightGradualChange(uint8 step);
//七色模式
void LightSevenColorChange(uint8 color_num);   //color_num belongs to 0~7
//模式选择函数
extern void LightOnModle(uint8 modle_num,uint8 step,uint8 color_num);
#endif
#endif
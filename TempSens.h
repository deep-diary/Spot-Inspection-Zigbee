#ifndef _TEMP_SENS_H_
#define _TEMP_SENS_H_

#include "SerialApp.h"
#ifdef ZIGBEE_SENS_TEMP

//#include <ioCC2530.h>
#include "hal_types.h"

#define KEEP      P1_1
#define INT       P1_0       //direction:in，signal of killing power
#define TEMP_SENS_P1DIR      0x02

#define LITHIUM_NUMS          2       //1节电池
#define SAMPLE_RATE           3       //1/2采样
#define SOFTWARE_VERSION     "20170224"       //软件版本
#define HARDWARE_VERSION     "20170224"       //硬件版本

#define TEMP_DETECT_MODE        0X01
#define CHECK_MAC_ADDR_MODE     0X02

//#define FAHRENHEIT      //华氏度
//#define ENGLISH         //如果定义了英文模式

extern uint8 temp_sens_model;
extern void temp_sens_init(void);
extern bool SerialApp_SendData_Temp(uint8 *temp);

#endif
#endif
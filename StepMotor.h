#ifndef __STEP_MOTOR_H__
#define __STEP_MOTOR_H__
#include "SerialApp.h"
#ifdef ZIGBEE_STEP_MOTOR
#include "hal_types.h"

//----------------------定义锂电池数量及总电压与采样电压的比例，，
#define LITHIUM_NUMS    1       //1节电池
#define SAMPLE_RATE     2       //1/2采样
#define SOFTWARE_VERSION     "20170224"       //软件版本
#define HARDWARE_VERSION     "20170224"       //硬件版本
static void motor_down(float n);
static void motor_up(float n);
static void Delay_ms(unsigned int msec);

extern void step_motor_init(void);
extern void Contrl_Step_motor(int state, float count);
#endif
#endif
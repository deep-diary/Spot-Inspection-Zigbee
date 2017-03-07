#ifndef __SENSORS_H__
#define __SENSORS_H__
#include "SerialApp.h"
#ifdef ZIGBEE_SENSOR
#include "hal_board.h"
#include "hal_types.h"

#define HUMAN_PIN     P0_4        //人体感应1:有人0：无人
#define LAMP_PIN      P0_5        //定义P0.5口为继电器输入端
#define GAS_PIN       P1_5        //定义P0.6口为烟雾传感器的输入端  
#define DHT11_DATA    P0_7        //dht11
#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof(arr)[0])

//----------------------定义锂电池数量及总电压与采样电压的比例，，
#define LITHIUM_NUMS    1       //1节电池
#define SAMPLE_RATE     2       //1/2采样
#define SOFTWARE_VERSION     "20170224"       //软件版本
#define HARDWARE_VERSION     "20170224"       //硬件版本
extern void sensors_init(void);
extern void SetLamp(bool on);
extern uint8 GetGas( void );
extern uint8 GetHuman( void );
extern void SerialApp_SendData_DHT11();
extern void SerialApp_SendData_MQ2();
extern void SerialApp_SendData_Human();
extern void SerialApp_SendData_Lamp(uint8 fc);
#endif

#endif
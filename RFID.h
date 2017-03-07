#ifndef __RFID_H__
#define __RFID_H__
#include "SerialApp.h"
#ifdef ZIGBEE_RFID
#include "hal_types.h"

#define LITHIUM_NUMS    1       //1节电池
#define SAMPLE_RATE     2       //1/2采样
#define SOFTWARE_VERSION     "170228"       //软件版本
#define HARDWARE_VERSION     "170224"       //硬件版本


extern bool FindReadRfidErr;

extern void rfidInit(void);
extern void RfidFindCard(void);
extern void RfidConflict(void);
extern void RfidFindReadCard(void);
#endif
#endif
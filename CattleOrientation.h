#ifndef _CATTER_ORIENTATION_H_
#define _CATTER_ORIENTATION_H_
#include "SerialApp.h"

#include "Hal_types.h"

#ifdef ZIGBEE_CATTER_ORIENTATION



extern uint8 report_buf[40];
extern uint16 report_buf_len;
extern void cattleOrientationReport(void);
extern void ioInitGpsPow(void);
#endif
#endif
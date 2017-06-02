#ifndef _GPRS_H_
#define _GPRS_H_
#include "SerialApp.h"
#include "Hal_types.h"

#ifdef ZIGBEE_GPRS

#define GPRS_INIT_NUM_MAX 7
#define GPRS_SEND_NUM_MAX 5



typedef enum
{
	AT_LOGIC_OR = 1,
        AT_LOGIC_AND,
        AT_LOGIC_NOT,
}AT_LOGIC; 


extern uint8 gprs_cmd_nums;
extern uint8 gprs_cmd_err;

extern bool gprsInit(uint8 idex);
extern bool gprsSendMsgToServer(uint8 idex);
extern void gprsClose(void);

bool gprsCheckAtRst(uint8 *data, char const *checked_bytes0,char const *checked_bytes1, AT_LOGIC logic);
bool gprsCheckAtRstSingle(uint8 *data, uint8 *checked_bytes);
#endif
#endif

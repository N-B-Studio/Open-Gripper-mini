
#ifndef _CAN_COMMUNICATE_H
#define _CAN_COMMUNICATE_H

#include "MyProject.h"


/******************************************************************************/
void can_closeloop(void);
void can_readVoltage(void);
void can_SetAbsAng(int32_t ang);
void can_SetRelAng(int32_t ang);
void can_PVTcommand(int32_t ang,uint16_t vel, uint8_t per);
void CANcommander_run(void);
/******************************************************************************/


#endif

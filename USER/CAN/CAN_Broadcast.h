#ifndef __CAN_BROADCAST_H
#define __CAN_BROADCAST_H

#include "stm32f0xx.h"
#include "Can.h"


extern void CAN_Send_By_Period(uint16_t CANBroadT_Value,uint16_t *TIM7_CANSendTime);

#endif

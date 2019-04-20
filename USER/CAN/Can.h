#ifndef __CAN_H
#define __CAN_H

#include <stdio.h>
#include "stm32f4xx.h"

extern void CanWriteData(uint32_t ID,uint8_t *databuf,uint8_t datalen);
extern void CAN_Configuration(uint8_t Bauderate);
extern uint8_t ReadCanAddr(void);
extern void CAN_Send_Digital_Value(void);

#endif

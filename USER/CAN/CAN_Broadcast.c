#include "CAN_Broadcast.h"
#include "RS232_Settings.h"

extern uint16_t DO_Digital_Data;

void CAN_Send_Digital_Value()
{
	uint8_t temp[3] = {0};
	temp[0] = DO_Digital_Data << 1;
	temp[1] = (DO_Digital_Data << 1) >> 8;
	temp[2] = 0;
	if(SensorSettings.CANAddrSetMethod == 0x02)
	{
		SensorSettings.CANAddr = ReadCanAddr();
	}
	CanWriteData(SensorSettings.CANAddr,temp,3);
}

void CAN_Send_By_Period(uint16_t CANBroadT_Value,uint16_t *TIM7_CANSendTime)
{
	switch (CANBroadT_Value)
	{
		case 1:
			if(*TIM7_CANSendTime == 1)
			{
				*TIM7_CANSendTime = 0;
				CAN_Send_Digital_Value();
			}
			break;
		case 2:
			if(*TIM7_CANSendTime == 2)
			{
				*TIM7_CANSendTime = 0;
				CAN_Send_Digital_Value();
			}
			break;
		case 3:
			if(*TIM7_CANSendTime == 5)
			{
				*TIM7_CANSendTime = 0;
				CAN_Send_Digital_Value();
			}
			break;
		case 4:
			if(*TIM7_CANSendTime == 10)
			{
				*TIM7_CANSendTime = 0;
				CAN_Send_Digital_Value();
			}
			break;
		case 5:
			if(*TIM7_CANSendTime == 20)
			{
				*TIM7_CANSendTime = 0;
				CAN_Send_Digital_Value();
			}
			break;
		default:break;
	}
}

#ifndef __RM3000_H
#define __RM3000_H

#include <stdbool.h>
#include <stdio.h>
#include "delay.h"
#include "stm32f4xx_spi.h"
#include "stm32f4xx.h"
#include "HwCfg.h"
typedef struct
{
	int16_t SensorValue[15];
	int16_t SensorValueTemp[15];
	int16_t SenState;//bit0:most left sensor ; bit14:most Right sensor
	int16_t compValue;//to desire the bet state of the 'senState Above'
	int16_t MagTapeWidth;//the width of the magTape,mostly is 30 50 or 5x5mm,also you can set out by youselft,or detect by itseft(not design yet)
	int8_t Sensity;//0-low sensity  1-high sensity
	bool cmdSendCanData;
	bool cmdSendRS232Data;
	bool cmdSendRS485Data;
	bool cmdSaveParameter;
	bool cmdIntialParameter;
	bool cmdPCConnect;
	int8_t handShakeInterface;//0-232 1-485 2-can
	bool ReqConnectAllow;
	bool ReqSetOKAllow;
	int16_t Xsection1;
	int16_t Xsection2;
	int16_t Xsection3;
	int16_t XsectionNum;
	int16_t TempCount[5];
	int16_t ResTimeStamp[5];
	////////////////
	int32_t RawData[10];	
	float ValWeigh_g[10];	
	bool	cmdSetZero[10];
	bool  cmdCalWeigh[10];
	float SetcalWeigh[10];
	float ValWeigh_g_flt[10];
	////////////////
	MoveStrutType MvgFlt[10];
} MagnetType; 

typedef struct
{
	int32_t CAN_ID;
	int8_t Reserve1;//
	int8_t Reserve2;//
	int8_t Reserve3;//
	int8_t Reserve4;//
	int16_t SenState;//bit0:most left sensor ; bit14:most Right sensor
} PassiveCanData; 

bool ComRequestCap(UrtBuf_type * pSrcBuf);
bool HandShakeCap(UrtBuf_type * pSrcBuf);
bool PCChangeParCap(UrtBuf_type * pSrcBuf);
bool PCCalSensor(UrtBuf_type * pSrcBuf);
uint16_t CRC_Verify(uint8_t *CRC_Buf,uint8_t Buf_Length);
extern uint8_t SensorErro;
extern uint8_t MalfunctionDeviceNum;
extern int32_t *mag_sample(uint8_t DeviceNum);
extern void Setup_ALL_MagIC(void);
extern void MagIC_Init(uint8_t DeviceNum);
extern void Init_ALL_MagIC(void);
extern void CLEAR_GPIO_Init(void);
extern void DRDY_GPIO_Init(void);
extern int16_t Normalization(int32_t values);
extern uint8_t SPI1_ReadWriteByte(uint8_t TxData);
extern void SPI_tom_CS_HIGH(uint8_t DeviceNum);
extern void SPI_tom_CS_LOW(uint8_t DeviceNum);
extern void usec_delay(unsigned int t);
extern int16_t SensorValue[15];
extern MagnetType MagnetSensors;
extern PassiveCanData CanSendBuf;
void MagIC_Measurement_All();
#endif


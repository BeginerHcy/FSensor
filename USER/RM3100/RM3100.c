/******************** (C) COPYRIGHT 2013 PNI Sensor Corp *****************
* File Name: ThreeD_magIC.c
* Sample Code for 3D MagIC ASIC
*************************************************************************/
#include "RM3100.h"
//#include "HwCfg.h"
#include "math.h"
#include <stdio.h> 
#include <stdlib.h> 
//#include "I2C_EEPROM.h"
#include "arm_math.h"
#include "24cxx.h"
#include "stmflash.h"
uint8_t Parameter1[100] = {0};
uint8_t ReadTemp1[100] = {0};
extern u8 dir;
extern u16 led0pwmval; 
#define	BasicFactor 429.4967f
bool flashLED[10];
uint8_t SensorErro = 0;
uint32_t TimeStamp,oldTimeStamp,deltaStamp,OldStamp[10],TimeStamp500ms,stableTime[10];
int16_t SensorValue[15] = {0};//
int16_t sendValve[30]=
{80,
100,
120,
150,
160,
170,
180,
190,
200,
220,
230,
250,
300};
MagnetType MagnetSensors;
PassiveCanData CanSendBuf;
//SysParameter_type gSystemPara;
int8_t polarSign[2]={1,-1};
float scaleRateWeight,weigfltTemp[15];
void polyfit(int n,float *x,float *y,int poly_n,float a[]);
int32_t Read_HX712(uint32_t GPIOOx,uint16_t GPIO_OPin, GPIOMode_TypeDef GPIOOMode, GPIOPuPd_TypeDef GPIOOPupd,GPIOOType_TypeDef OOType,
	                       uint32_t GPIOx, uint16_t GPIO_Pin,  GPIOMode_TypeDef GPIOMode,  GPIOPuPd_TypeDef GPIOPupd);
void usec_delay(unsigned int t)
{
	delay_us(10);
	//for(int i = 0;i<5;i++);
	
}
void subCyc10msTask(void);
/*************************************************************************
* Function Name : SPI_tom_CS_LOW
* Description : selects the 3D MagIC CS
* Input : None
* Output : None
* Return : None
*************************************************************************/
void SPI_tom_CS_LOW(uint8_t DeviceNum)
{	
	switch (DeviceNum)
	{
		case 1 : macSPI_PNI1_CS_ENABLE();break;
		case 2 : macSPI_PNI2_CS_ENABLE();break;
		case 3 : macSPI_PNI3_CS_ENABLE();break;
		case 4 : macSPI_PNI4_CS_ENABLE();break;
		case 5 : macSPI_PNI5_CS_ENABLE();break;
		default : break;
	}	
}
/*************************************************************************
* Function Name : SPI_tom_CS_HIGH
* Description : deselects the 3D MagIC CS
* Input : None
* Output : None
* Return : None
*************************************************************************/
void SPI_tom_CS_HIGH(uint8_t DeviceNum)
{
	// Set 3D MagIC Chip Select Pin High
	switch (DeviceNum)
	{
		case 1 : macSPI_PNI1_CS_DISABLE();break;
		case 2 : macSPI_PNI2_CS_DISABLE();break;
		case 3 : macSPI_PNI3_CS_DISABLE();break;
		case 4 : macSPI_PNI4_CS_DISABLE();break;
		case 5 : macSPI_PNI5_CS_DISABLE();break;
		default : break;
	}
}

/*************************************************************************
* Function Name : ThreeD_magic_setup
* Description : sets up the 3D MagIC to make a multi-axis measurement
* Input : None
* Output : None
* Return : None
*************************************************************************/
void ThreeD_magic_setup(uint8_t DeviceNum)
{
	usec_delay(1);//CMM
	SPI_tom_CS_LOW(DeviceNum);
	usec_delay(1);
	SPI1_ReadWriteByte(0x01);
	usec_delay(1);
	SPI1_ReadWriteByte(0x75);
	usec_delay(1);
	SPI_tom_CS_HIGH(DeviceNum);
}


/*************************************************************************
* Function Name : ThreeD_magic_init
* Description : initializes the 3D MagIC.
* Input : None
* Output : None
* Return : None
*************************************************************************/

void ThreeD_magic_init(uint8_t DeviceNum)
{
	unsigned int cycle_count_read_back[3];
	unsigned char i;
	unsigned int cycle_count = 100;//100;//200;
	uint8_t CMMRead[5]={0};
	//Set Cycle Count HERE
	usec_delay(1);
	SPI_tom_CS_LOW(DeviceNum);//WRITE CCXYZ
	usec_delay(1);
	//Write Cycle Count Reg, start at X-axis, auto increment to Y & Z
	SPI1_ReadWriteByte(0x04);
	//CC is programmed as 200 for now.
	for(i=0;i<3;i++)
	{
		usec_delay(1);
		SPI1_ReadWriteByte(0x00);	
		usec_delay(1);
		SPI1_ReadWriteByte(0x28);
		usec_delay(1);
	}
	SPI_tom_CS_HIGH(DeviceNum);
	
	
	usec_delay(1);
	SPI_tom_CS_LOW(DeviceNum);//READ CCXYZ
	usec_delay(1);
	SPI1_ReadWriteByte(0x84);
	for(i=0;i<3;i++)
	{
		usec_delay(1);
		cycle_count_read_back[i]=(SPI1_ReadWriteByte(0))<<8;
		usec_delay(1);
		cycle_count_read_back[i]|=SPI1_ReadWriteByte(0);
	}
	SPI_tom_CS_HIGH(DeviceNum);
	
	
	usec_delay(1);//CMM
	SPI_tom_CS_LOW(DeviceNum);
	usec_delay(1);
	SPI1_ReadWriteByte(0x01);
	usec_delay(1);
	SPI1_ReadWriteByte(0x75);
	usec_delay(1);
	SPI_tom_CS_HIGH(DeviceNum);
	
	usec_delay(1);//ReadCMM
	SPI_tom_CS_LOW(DeviceNum);
	usec_delay(1);
	CMMRead[DeviceNum-1] = SPI1_ReadWriteByte(0x81);
	usec_delay(1);
	CMMRead[DeviceNum-1] = SPI1_ReadWriteByte(0x00);
	usec_delay(1);
	SPI_tom_CS_HIGH(DeviceNum);
		
	usec_delay(1);//CMMRATE
	SPI_tom_CS_LOW(DeviceNum);
	SPI1_ReadWriteByte(0x0B);
	SPI1_ReadWriteByte(0x92);
	SPI_tom_CS_HIGH(DeviceNum);
}

/*************************************************************************
* Function Name : DataReady()
* Description : Check DRDY pin, return true if high, otherwise false.
* Input : None
* Output : None
* Return : true or false
*************************************************************************/
float PolyVal4(float A[],float X)
{
	return A[0]+A[1]*X+A[2]*X*X+A[3]*X*X*X+A[4]*X*X*X*X;
}
float PolyVal7(float A[],float X)
{
	return A[0]+A[1]*X+A[2]*X*X+A[3]*X*X*X+A[4]*X*X*X*X+A[5]*X*X*X*X*X+A[6]*X*X*X*X*X*X+A[7]*X*X*X*X*X*X*X;
}
bool DataReady(uint8_t DeviceNum)
{
	switch (DeviceNum)
	{
		case 1 : 
					return ReadIn(DRDY1);
			break;
		case 2 : 
					return ReadIn(DRDY2);
			break;
		case 3 : 
					return ReadIn(DRDY3);
			break;
		case 4 : 
					return ReadIn(DRDY4);
			break;
		case 5 : 
					return ReadIn(DRDY5);
			break;
		default : return false;
	}
}
void subCyc10msTask(void){

	for(u8 i=0;i<8;i++){
		//weigfltTemp[i] = MagnetSensors.ValWeigh_g[i]> gSystemPara.WeightRateKg*1000?weigfltTemp[i]:MagnetSensors.ValWeigh_g[i];
		MagnetSensors.MvgFlt[i].x = 0.4 * MagnetSensors.ValWeigh_g[i] + 0.6 * MagnetSensors.MvgFlt[i].x;
		MagnetSensors.MvgFlt[i].Ntime = 10;//max 200,
		MoveAvgFilter(&MagnetSensors.MvgFlt[i]);
		MagnetSensors.ValWeigh_g_flt[i] = MagnetSensors.MvgFlt[i].y;
		MagnetSensors.SensorValueTemp[i] = fabs(MagnetSensors.ValWeigh_g_flt[i])>32767?32767:MagnetSensors.ValWeigh_g_flt[i];
		MagnetSensors.SensorValue[i] = MagnetSensors.SensorValueTemp[i]>gSystemPara.WeightRateKg*1000?MagnetSensors.SensorValue[i]:MagnetSensors.SensorValueTemp[i];
		//MagnetSensors.SensorValue[i] = MagnetSensors.ValWeigh_g[i];
		//MagnetSensors.SensorValue[1] = MagnetSensors.RawData[0] & 0xFFFF;
		//MagnetSensors.SensorValue[2] = MagnetSensors.RawData[0]>>16 & 0xFFFF;
	}
}
/*************************************************************************
* Function Name : MagIC_Measurement_All()
* Description : Read All measurement
* Input : None
* Output : None
* Return : Temp
*************************************************************************/
//int32_t* MagIC_Measurement_All(void)
void MagIC_Measurement_All(void)
{
	
	uint8_t i;
	float TempFactor[8];
	int32_t WeighTemp[8],oldWeigh[8],TempRaw[8];
	/////AutoSendData/////
	if(TimeStamp>=oldTimeStamp)
		deltaStamp = TimeStamp - oldTimeStamp;
	oldTimeStamp = TimeStamp;
	///////////////////////////////////////////////////////////////////	
	TempRaw[0] = Read_HX712(SSN1,DRDY1);
	TempRaw[1] = Read_HX712(SSN2,DRDY2);
	TempRaw[2] = Read_HX712(SSN3,DRDY3);
	TempRaw[3] = Read_HX712(SSN4,DRDY4);
	TempRaw[4] = Read_HX712(SSN5,DRDY5);
	TempRaw[5] = Read_HX712(SSN6,DRDY6);
	TempRaw[6] = Read_HX712(SSN7,DRDY7);
	TempRaw[7] = Read_HX712(SSN8,DRDY8);
	
	
	MagnetSensors.RawData[0] = ((TempRaw[0] & 0x0000FFFF) == 0x0000FFFF)?MagnetSensors.RawData[0]:TempRaw[0];
	MagnetSensors.RawData[1] = ((TempRaw[1] & 0x0000FFFF) == 0x0000FFFF)?MagnetSensors.RawData[1]:TempRaw[1];
	MagnetSensors.RawData[2] = ((TempRaw[2] & 0x0000FFFF) == 0x0000FFFF)?MagnetSensors.RawData[2]:TempRaw[2];
	MagnetSensors.RawData[3] = ((TempRaw[3] & 0x0000FFFF) == 0x0000FFFF)?MagnetSensors.RawData[3]:TempRaw[3];
	MagnetSensors.RawData[4] = ((TempRaw[4] & 0x0000FFFF) == 0x0000FFFF)?MagnetSensors.RawData[4]:TempRaw[4];
	MagnetSensors.RawData[5] = ((TempRaw[5] & 0x0000FFFF) == 0x0000FFFF)?MagnetSensors.RawData[5]:TempRaw[5];
	MagnetSensors.RawData[6] = ((TempRaw[6] & 0x0000FFFF) == 0x0000FFFF)?MagnetSensors.RawData[6]:TempRaw[6];
	MagnetSensors.RawData[7] = ((TempRaw[7] & 0x0000FFFF) == 0x0000FFFF)?MagnetSensors.RawData[7]:TempRaw[7];
	
	
	//////////////////////////////////////////////////////////////////////////////
	for(i=0;i<8;i++){
		////////////////////////////////////////////////////////////////////////////
		if(MagnetSensors.cmdSetZero[i]){//Set Zero offset
			gSystemPara.Offset_Basic[i] = MagnetSensors.RawData[i];
			MagnetSensors.cmdSaveParameter = 1;
			MagnetSensors.cmdSetZero[i] = 0;
		}
		////////////////////////////////////////////////////////////////////////////
		WeighTemp[i] = fabs(MagnetSensors.ValWeigh_g[i] / 10.0);//because we want to kill the 1g accuracy
		if(WeighTemp[i]>=1 && WeighTemp[i]<gSystemPara.AutoSetZeroValue/10){//Auto set Zero,
			if(oldWeigh[i] == WeighTemp[i]){//if the result is stable 
				stableTime[i]+=deltaStamp;
			}
			else
				stableTime[i] = 0;
		}
		else
			stableTime[i] = 0;
		if(stableTime[i]>5000){
			MagnetSensors.cmdSetZero[i] = (gSystemPara.EnableAutoRest==1) ;
			stableTime[i] = 0;
		}
		////////////////////////////////////////////////////////////////////////////
		oldWeigh[i] = WeighTemp[i];
		////////////////////////////////////////////////////////////////////////////
		if(gSystemPara.WeightRateKg==0){
			gSystemPara.WeightRateKg = 20;//The default set for the Weiht Rate
		}
		scaleRateWeight = gSystemPara.WeightRateKg / 10.0;
		
		if(MagnetSensors.cmdCalWeigh[i]){//cal mount Factor or manuafactor Devi Factor
			if(MagnetSensors.ValWeigh_g[i]!=0 && MagnetSensors.SetcalWeigh[i]!=0){
				TempFactor[i] = 4000.0 * ((MagnetSensors.SetcalWeigh[i] / (( MagnetSensors.RawData[i] - gSystemPara.Offset_Basic[i] ) * scaleRateWeight / BasicFactor))-1);
				//because this value is very small,so we need to make it bigger.
				
				if(TempFactor[i]>-500 && TempFactor[i]<500){//Only within 14 degrees allow;
					gSystemPara.Dev_Factor[i] = TempFactor[i];
					MagnetSensors.cmdSaveParameter = 1;	
				}
			}
			MagnetSensors.cmdCalWeigh[i] = 0;
		}
		if(MagnetSensors.cmdChangeSign[i]){
			gSystemPara.sign_Weight[i] = gSystemPara.sign_Weight[i] * -1;
			MagnetSensors.cmdSaveParameter = 1;
			MagnetSensors.cmdChangeSign[i] = 0;
		}
		////////////////////////////////////////////////////////////////////////////
		MagnetSensors.ValWeigh_g[i] = gSystemPara.sign_Weight[i] * ( MagnetSensors.RawData[i] - gSystemPara.Offset_Basic[i] ) / ( 1 - ( gSystemPara.Dev_Factor[i] / 4000.0 )) * scaleRateWeight / BasicFactor;
	}
	///////////////////////////////////////////////////////////////////
	led0pwmval = 500;
	TIM_SetCompare4(TIM8,led0pwmval);	

	
	OldStamp[6]+=deltaStamp;
		if(OldStamp[6]>10){
		OldStamp[6] = 0;
		subCyc10msTask();
	}
		
	if(MagnetSensors.cmdPCConnect){
			OldStamp[5]+=deltaStamp;
		if(OldStamp[5]>200){
			OldStamp[5] = 0;
			if(MagnetSensors.handShakeInterface==0)
				MagnetSensors.cmdSendRS232Data = 1;
			else
				MagnetSensors.cmdSendRS485Data = 1;
		}
	}
	if(0==gSystemPara.DataAnsMethod)
	{
		switch(gSystemPara.RequestInterval){
			
			case 0://4ms
				OldStamp[0]+=deltaStamp;
				if(OldStamp[0]>4){
					OldStamp[0] = 0;
					if(0==gSystemPara.DataComInterface)//CAN
						MagnetSensors.cmdSendCanData = 1;
					else if(1==gSystemPara.DataComInterface && !MagnetSensors.cmdPCConnect)//RS485
						MagnetSensors.cmdSendRS485Data = 1;
					else if(2==gSystemPara.DataComInterface && !MagnetSensors.cmdPCConnect)//RS232
						MagnetSensors.cmdSendRS232Data = 1;
				}
				break;
			
			case 1://10ms
				OldStamp[1]+=deltaStamp;
				if(OldStamp[1]>10){
					OldStamp[1] = 0;
					if(0==gSystemPara.DataComInterface)//CAN
						MagnetSensors.cmdSendCanData = 1;
					else if(1==gSystemPara.DataComInterface && !MagnetSensors.cmdPCConnect)//RS485
						MagnetSensors.cmdSendRS485Data = 1;
					else if(2==gSystemPara.DataComInterface && !MagnetSensors.cmdPCConnect)//RS232
						MagnetSensors.cmdSendRS232Data = 1;				
				}
				break;
			
			case 2://20ms
				OldStamp[2]+=deltaStamp;
				if(OldStamp[2]>20){
					OldStamp[2] = 0;
					if(0==gSystemPara.DataComInterface)//CAN
						MagnetSensors.cmdSendCanData = 1;
					else if(1==gSystemPara.DataComInterface && !MagnetSensors.cmdPCConnect)//RS485
						MagnetSensors.cmdSendRS485Data = 1;
					else if(2==gSystemPara.DataComInterface && !MagnetSensors.cmdPCConnect)//RS232
						MagnetSensors.cmdSendRS232Data = 1;				
				}
				break;
			
			case 3://40ms
				OldStamp[3]+=deltaStamp;
				if(OldStamp[3]>40){
					OldStamp[3] = 0;
					if(0==gSystemPara.DataComInterface)//CAN
						MagnetSensors.cmdSendCanData = 1;
					else if(1==gSystemPara.DataComInterface && !MagnetSensors.cmdPCConnect)//RS485
						MagnetSensors.cmdSendRS485Data = 1;
					else if(2==gSystemPara.DataComInterface && !MagnetSensors.cmdPCConnect)//RS232
						MagnetSensors.cmdSendRS232Data = 1;				
				}			
				break;		
			
			case 4://100ms
				OldStamp[4]+=deltaStamp;
				if(OldStamp[4]>100){
					OldStamp[4] = 0;
					if(0==gSystemPara.DataComInterface)//CAN
						MagnetSensors.cmdSendCanData = 1;
					else if(1==gSystemPara.DataComInterface && !MagnetSensors.cmdPCConnect)//RS485
						MagnetSensors.cmdSendRS485Data = 1;
					else if(2==gSystemPara.DataComInterface && !MagnetSensors.cmdPCConnect)//RS232
						MagnetSensors.cmdSendRS232Data = 1;				
				}			
				break;
			
			default://40ms
				OldStamp[3]+=deltaStamp;
				if(OldStamp[3]>40){
					OldStamp[3] = 0;
					if(0==gSystemPara.DataComInterface)//CAN
						MagnetSensors.cmdSendCanData = 1;
					else if(1==gSystemPara.DataComInterface && !MagnetSensors.cmdPCConnect)//RS485
						MagnetSensors.cmdSendRS485Data = 1;
					else if(2==gSystemPara.DataComInterface && !MagnetSensors.cmdPCConnect)//RS232
						MagnetSensors.cmdSendRS232Data = 1;				
				}			
				break;	
		}
	}
	//////////////////////////////////////////////////
	////////COM Interface capture master request//////
	//////////////////////////////////////////////////
	if(1==gSystemPara.DataComInterface){//RS485
		if(ComRequestCap(&Uart6Data))
			MagnetSensors.cmdSendRS485Data = 1;
	}
	else if(2==gSystemPara.DataComInterface){//RS232
		if(ComRequestCap(&Uart3Data))
			MagnetSensors.cmdSendRS232Data = 1;
	}
	if(HandShakeCap(&Uart3Data)){//RS232
		MagnetSensors.cmdPCConnect = 1;
		MagnetSensors.ReqConnectAllow = 1;
		MagnetSensors.handShakeInterface = 0;
	}
	if(HandShakeCap(&Uart6Data)){//RS485
		MagnetSensors.cmdPCConnect = 1;
		MagnetSensors.ReqConnectAllow = 1;
		MagnetSensors.handShakeInterface = 1;
	}
	if(PCChangeParCap(&Uart3Data) || PCChangeParCap(&Uart6Data)){//RS232 or RS485
		MagnetSensors.cmdSaveParameter = 1;
	}
	if(PCCalSensor(&Uart3Data) || PCCalSensor(&Uart6Data)){//RS232 or RS485
		MagnetSensors.ReqSetOKAllow = 1;
	}
	//////////////////////////////////////////////////
	//////////////////////////////////////////////////
	///////////////Parameter save or initial//////////
	
	if(MagnetSensors.cmdSaveParameter)
	{
		MagnetSensors.cmdSaveParameter = 0;
		MagnetSensors.ReqSetOKAllow = 1;
		memcpy(&Parameter1,&gSystemPara,sizeof(gSystemPara));
		Parameter1[99] = 0xCD;
		STMFLASH_Write(FLASH_SAVE_ADDR,( uint32_t * )Parameter1,100);
	}
	if(MagnetSensors.cmdIntialParameter)
	{
		MagnetSensors.cmdIntialParameter = 0;
		////////////////////////
		gSystemPara.BufferHead = 0xAB;
		gSystemPara.CANBusBauderate = 1;//250k
		gSystemPara.CanNode = 0x100;
		gSystemPara.DataAnsMethod = 0;
		gSystemPara.DataComInterface = 0;
		gSystemPara.DetectPolar = 0;
		gSystemPara.WeightRateKg = 20;
		gSystemPara.RequestInterval = 2;
		gSystemPara.RS232Bauderate = 0;
		gSystemPara.RS485Bauderate = 0;
		gSystemPara.RS485Node = 10;
		gSystemPara.MagTapWide = 0;
		///default parameters
		memcpy(&Parameter1,&gSystemPara,sizeof(gSystemPara));
		Parameter1[99] = 0xCD;
		STMFLASH_Write(FLASH_SAVE_ADDR,( uint32_t * )Parameter1,100);
	}	
}
bool PCChangeParCap(UrtBuf_type * pSrcBuf){
	uint8_t filled = pSrcBuf->pRfil;
	uint8_t degred = pSrcBuf->pRder;
	uint8_t iFunCode=5,iLenCode=4,iDstSta=3,iSrcSta=2;
	uint8_t chnOKBfLen = 20;
	uint16_t cycResult;
	bool result=0;
	///
	result = 0;
	///
	if(filled >= degred+ chnOKBfLen)//1. Len enough.
	{
		for(;degred<=filled-1;degred++)
		{
			if(filled >= degred+ chnOKBfLen){
				if( pSrcBuf->rBuffer[degred]==0xAA && 
					pSrcBuf->rBuffer[degred+1]==0x54&&
					pSrcBuf->rBuffer[degred+2]==0x11&&
					pSrcBuf->rBuffer[degred+3]==0x00&&
					pSrcBuf->rBuffer[degred+4]==0x00&&
					pSrcBuf->rBuffer[degred+19]==0xAC)
				{
					pSrcBuf->pRder = degred+chnOKBfLen-1;
					gSystemPara.AutoSetZeroValue = pSrcBuf->rBuffer[degred+5];
					gSystemPara.DetectPolar = pSrcBuf->rBuffer[degred+6];
					gSystemPara.DataComInterface = pSrcBuf->rBuffer[degred+7];
					gSystemPara.RS485Node = pSrcBuf->rBuffer[degred+8];
					gSystemPara.CanNode = pSrcBuf->rBuffer[degred+9]*256 + pSrcBuf->rBuffer[degred+10];
					gSystemPara.RS485Bauderate = pSrcBuf->rBuffer[degred+11];
					gSystemPara.RS232Bauderate = pSrcBuf->rBuffer[degred+12];
					gSystemPara.CANBusBauderate = pSrcBuf->rBuffer[degred+13];
					gSystemPara.DataAnsMethod = pSrcBuf->rBuffer[degred+14];
					gSystemPara.RequestInterval = pSrcBuf->rBuffer[degred+15];
					gSystemPara.WeightRateKg = pSrcBuf->rBuffer[degred+16];
					gSystemPara.MagTapWide = pSrcBuf->rBuffer[degred+17];
					gSystemPara.EnableAutoRest = pSrcBuf->rBuffer[degred+18];
					if(gSystemPara.RS232Bauderate>3 || gSystemPara.RS232Bauderate <0)
						gSystemPara.RS232Bauderate = 0;
					if(gSystemPara.RS485Bauderate>3 || gSystemPara.RS485Bauderate <0)
						gSystemPara.RS485Bauderate = 0;
					if(gSystemPara.CANBusBauderate>3 || gSystemPara.CANBusBauderate <0)
						gSystemPara.CANBusBauderate = 1;
					result = 1;
				}
			}
		}
	}
	return result;
}
bool PCCalSensor(UrtBuf_type * pSrcBuf){
	uint8_t filled = pSrcBuf->pRfil,SensorIndex;
	uint8_t degred = pSrcBuf->pRder;
	uint8_t iFunCode=5,iLenCode=4,iDstSta=3,iSrcSta=2;
	uint8_t chnOKBfLen = 14;
	uint16_t cycResult;
	bool result=0;
	///
	result = 0;
	///

	if(filled >= degred+ chnOKBfLen)//1. Len enough.
	{
		for(;degred<=filled-1;degred++)
		{
			if(filled >= degred+ chnOKBfLen){
				if( pSrcBuf->rBuffer[degred]==0xAA && 
					pSrcBuf->rBuffer[degred+1]==0x51&&
					pSrcBuf->rBuffer[degred+2]==0x0B&&
					pSrcBuf->rBuffer[degred+3]==0x00&&
					pSrcBuf->rBuffer[degred+4]==0x00&&
					pSrcBuf->rBuffer[degred+13]==0xAC)
				{
					pSrcBuf->pRder = degred+chnOKBfLen-1;
					SensorIndex = pSrcBuf->rBuffer[degred+5];
					MagnetSensors.SetcalWeigh[SensorIndex-1] 	= pSrcBuf->rBuffer[degred+6] * 100.0;
					MagnetSensors.cmdSetZero[SensorIndex-1] 	= (pSrcBuf->rBuffer[degred+7]==1);
					MagnetSensors.cmdCalWeigh[SensorIndex-1] 	= (pSrcBuf->rBuffer[degred+8]==1);
					MagnetSensors.cmdChangeSign[SensorIndex-1] 	= (pSrcBuf->rBuffer[degred+9]==1);
					result = 1;
				}
			}
		}
	}
	return result;
}
bool HandShakeCap(UrtBuf_type * pSrcBuf)
{
	uint8_t filled = pSrcBuf->pRfil;
	uint8_t degred = pSrcBuf->pRder;
	uint8_t iFunCode=5,iLenCode=4,iDstSta=3,iSrcSta=2;
	uint8_t chnOKBfLen = 7;
	uint16_t cycResult;
	bool result=0;
	///
	result = 0;
	///
	if(filled >= degred+ chnOKBfLen)//1. Len enough.
	{
		for(;degred<=filled-1;degred++)
		{
			if(filled >= degred+ chnOKBfLen){
				if(pSrcBuf->rBuffer[degred]==0xAA && 
					pSrcBuf->rBuffer[degred+1]==0x55&&
					pSrcBuf->rBuffer[degred+2]==0x04&&
					pSrcBuf->rBuffer[degred+3]==0x00&&
					pSrcBuf->rBuffer[degred+4]==0x00&&
					pSrcBuf->rBuffer[degred+5]==0x01&&
					pSrcBuf->rBuffer[degred+6]==0xAB)
				{
					pSrcBuf->pRder = degred+chnOKBfLen-1;
					result = 1;
				}
			}
		}
	}
	return result;
}
bool ComRequestCap(UrtBuf_type * pSrcBuf)
{
	uint8_t filled = pSrcBuf->pRfil;
	uint8_t degred = pSrcBuf->pRder;
	uint8_t iFunCode=5,iLenCode=4,iDstSta=3,iSrcSta=2;
	uint8_t chnOKBfLen = 7;
	uint16_t cycResult;
	bool result=0;
	///
	result = 0;
	///
	if(filled >= chnOKBfLen-1)//1. Len enough.
	{
		for(;degred<=filled-1;degred++)
		{
			if(filled >= degred+ chnOKBfLen){
				if(pSrcBuf->rBuffer[degred]==0x52 && 
						pSrcBuf->rBuffer[degred+1]==0x4D &&
						pSrcBuf->rBuffer[degred+2]==0x67 &&
						pSrcBuf->rBuffer[degred+3]==0x73 &&
						pSrcBuf->rBuffer[degred+4]==0x77 &&
						pSrcBuf->rBuffer[degred+5]==0x5E &&
						pSrcBuf->rBuffer[degred+6]==0x69)
				{
					if(filled >= degred+ chnOKBfLen){
						cycResult = CRC_Verify(&(pSrcBuf->rBuffer[degred]),5);//
						pSrcBuf->pRder = degred+chnOKBfLen-1;
						result = 1;
					}
				}
			}
		}
	}
	return result;
};
int16_t Normalization(int32_t values)
{
	return (int16_t)(values/32);
}
uint8_t SPI1_ReadWriteByte(uint8_t TxData)
{
  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET){}//
	
	SPI_I2S_SendData(SPI1, TxData); //
		
  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET){} //
 
	return SPI_I2S_ReceiveData(SPI1); //
}
void polyfit(int n,float x[],float y[],int poly_n,float a[]) 
{ 
	int i,j; 
	float *tempx,*tempy,*sumxx,*sumxy,*ata;
	void gauss_solve(int n,float A[],float x[],float b[]);
	tempx=calloc(n,sizeof(float));
	sumxx=calloc(poly_n*2+1,sizeof(float));
	tempy=calloc(n,sizeof(float));
	sumxy=calloc(poly_n+1,sizeof(float));
	ata=calloc((poly_n+1)*(poly_n+1),sizeof(float));
	for (i=0;i<n;i++)
  {
		tempx[i]=1;
		tempy[i]=y[i];
  }
	for (i=0;i<2*poly_n+1;i++)
		for (sumxx[i]=0,j=0;j<n;j++)
		{
			sumxx[i]+=tempx[j];
			tempx[j]*=x[j];
		}
	for (i=0;i<poly_n+1;i++) 
		for (sumxy[i]=0,j=0;j<n;j++) 
		{ 
			sumxy[i]+=tempy[j]; 
			tempy[j]*=x[j]; 
		} 
	for (i=0;i<poly_n+1;i++) 
			 for (j=0;j<poly_n+1;j++) 
					ata[i*(poly_n+1)+j]=sumxx[i+j]; 
	gauss_solve(poly_n+1,ata,a,sumxy); 
	free(tempx); 
	free(sumxx); 
	free(tempy); 
	free(sumxy); 
	//free(ata); 
} 
void gauss_solve(int n,float A[],float x[],float b[])
{
	int i,j,k,r;
	float max;
	for (k=0;k<n-1;k++)
	{
		max=fabs(A[k*n+k]);/*find maxmum*/
		r=k;
		for (i=k+1;i<n-1;i++)
		if (max<fabs(A[i*n+i]))
		{
			max=fabs(A[i*n+i]);
			r=i;
		}
		if (r!=k)
			for (i=0;i<n;i++)/*change array:A[k] & A[r]*/
			{
				max=A[k*n+i];
				A[k*n+i]=A[r*n+i];
				A[r*n+i]=max;
			} 
		max=b[k];/*change array:b[k]&b[r]*/
		b[k]=b[r];
		b[r]=max;
		for (i=k+1;i<n;i++)
		{
			for (j=k+1;j<n;j++)
				A[i*n+j]-=A[i*n+k]*A[k*n+j]/A[k*n+k];
			b[i]-=A[i*n+k]*b[k]/A[k*n+k];
		}
	} 
	for (i=n-1;i>=0;x[i]/=A[i*n+i],i--)
			 for (j=i+1,x[i]=b[i];j<n;j++)
	x[i]-=A[i*n+j]*x[j]; 
}

uint16_t CRC_Verify(uint8_t *CRC_Buf,uint8_t Buf_Length)
{
	uint8_t i,j;
	uint16_t wCrc = 0xffff;
	uint16_t wPolyNum = 0xA001;
	for(i = 0;i < Buf_Length;i++)
	{
		wCrc ^= CRC_Buf[i];
		
		for(j=0;j<8;j++)
		{
			if(wCrc & 0x0001)
			{
				wCrc = (wCrc >> 1)^wPolyNum;
			}
			else
			{
				wCrc = wCrc>>1;
			}
		}
	}
	return wCrc;
}
int32_t Read_HX712(uint32_t GPIOOx,uint16_t GPIO_OPin, GPIOMode_TypeDef GPIOOMode, GPIOPuPd_TypeDef GPIOOPupd,GPIOOType_TypeDef OOType,
	                       uint32_t GPIOx, uint16_t GPIO_Pin,  GPIOMode_TypeDef GPIOMode,  GPIOPuPd_TypeDef GPIOPupd)  
{ 
	int32_t val = 0; 
	unsigned char i = 0; 

	GPIO_SetBits((GPIO_TypeDef *)GPIOx,GPIO_Pin);    	//DOUT=1 
	GPIO_ResetBits((GPIO_TypeDef *)GPIOOx,GPIO_OPin);    	//SCK=0
	////////////////////////////////////////////////////////////
	while(GPIO_ReadInputDataBit((GPIO_TypeDef *)GPIOx,GPIO_Pin));   //DOUT=0  
	delay_us(1); 
	for(i=0;i<24;i++) 
	{
		GPIO_SetBits((GPIO_TypeDef *)GPIOOx,GPIO_OPin);    //SCK=1 
		val=val<<1; 
		delay_us(1);  
		GPIO_ResetBits((GPIO_TypeDef *)GPIOOx,GPIO_OPin);    //SCK=0 
		if(GPIO_ReadInputDataBit((GPIO_TypeDef *)GPIOx,GPIO_Pin))   //DOUT=1 
		val++; 
		delay_us(1); 
	}
	GPIO_SetBits((GPIO_TypeDef *)GPIOOx,GPIO_OPin);
	delay_us(1);
	val = val^0x800000;
	GPIO_ResetBits((GPIO_TypeDef *)GPIOOx,GPIO_OPin); 
	delay_us(1);  
	return val;  
}
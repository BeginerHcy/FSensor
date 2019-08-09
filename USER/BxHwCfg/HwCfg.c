//#include "HwCfg.h"
#include "delay.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx.h"                  // Device header
#include "RM3100.h"
//#include "I2C_EEPROM.h"
#include "Can.h"
#include "24cxx.h"
#include "stmflash.h"
//////////Delare/////////
UrtBuf_type Uart1Data;
UrtBuf_type Uart3Data;
UrtBuf_type Uart6Data;
UrtBuf_type UartTempData;
SysParameter_type gSystemPara;
uint8_t cntFlash,cntFlash500;
extern bool flashLED[10];
extern uint32_t TimeStamp,TimeStamp500ms;
uint8_t Parameter[100] = {0};
uint8_t ReadTemp[100] = {0};
uint32_t ComBauderate[4] = {9600,19200,38400,115200};
float TapeWides[3] = {30,50,5};
u16 led0pwmval;
u8 dir;
//////////Delare/////////
#define Uart6RS485RE SetDO(Uart6DRE,0)
#define Uart6RS485SE SetDO(Uart6DRE,1)
void SystemConfig()
{
	////////////
	HwCfgInit();
	////////////
};

double MoveAvgFilter(MoveStrutType * pFunData)
{
	double SumBuffer=0;
	u32 indexCal=0;
	double temN=pFunData->Ntime;//Byte
	u32 BufSizeByte=sizeof(pFunData->buffer)/sizeof(double);
	u32 BufLenMin=sizeof(pFunData->buffer)-sizeof(double);
	///////
	ArryMLO(&pFunData->buffer[0],temN);
	///////
	pFunData->buffer[0] = pFunData->x;
	//////
	if(temN>=BufSizeByte) temN=BufSizeByte;
	
	if(temN==0) temN=1;
	//////
	pFunData->Nfilled++;
	//////
	if(pFunData->Nfilled>=temN)pFunData->Nfilled=temN;
	SumBuffer=0;
	/////
	for(indexCal=0;indexCal<pFunData->Nfilled;indexCal++)
	{
		 SumBuffer = SumBuffer + pFunData->buffer[indexCal];
	}
	/////
	pFunData->y =  SumBuffer/pFunData->Nfilled;
	return pFunData->y;
	
};
void ArryMLO(double* buf,u32 bufByte)
{
	//u32 bufByte = sizeof(buf)/sizeof(buf[0]);
	u32 iBuf=0;
	while(iBuf+1<bufByte)
	{
		iBuf++;
		buf[bufByte-iBuf] = buf[bufByte-iBuf-1];
	}
};
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
void HwCfgInit()
{
	/////////////
	CfgPINOut(Uart6DRE);//RS485 RDE Signal
	CfgPINOut(DOCans);//RS485 RDE Signal	////////////
	
	CfgPINOut(SSN1);
	CfgPINOut(SSN2);
	CfgPINOut(SSN3);
	CfgPINOut(SSN4);
	CfgPINOut(SSN5);
	CfgPINOut(SSN6);
	CfgPINOut(SSN7);
	CfgPINOut(SSN8);
	
	CfgPINIn(DRDY1);
	CfgPINIn(DRDY2);
	CfgPINIn(DRDY3);
	CfgPINIn(DRDY4);
	CfgPINIn(DRDY5);
	CfgPINIn(DRDY6);
	CfgPINIn(DRDY7);
	CfgPINIn(DRDY8);
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//设置系统中断优先级分组2
	////////////
	STMFLASH_Read(FLASH_SAVE_ADDR,( uint32_t * )ReadTemp,100);
	/////////////
	if(0xAB==ReadTemp[0]&&0xCD==ReadTemp[99]){
		memcpy(&gSystemPara,&ReadTemp,sizeof(gSystemPara));
	}
	else{
		////////////////////////
		gSystemPara.BufferHead = 0xAB;
		gSystemPara.CANBusBauderate = 1;//250k
		gSystemPara.CanNode = 0x100;
		gSystemPara.DataAnsMethod = 0;//auto
		gSystemPara.DataComInterface = 0;//can
		gSystemPara.DetectPolar = 0;//N
		gSystemPara.WeightRateKg = 20;//150knT
		gSystemPara.RequestInterval = 1;//10ms
		gSystemPara.RS232Bauderate = 0;//38400
		gSystemPara.RS485Bauderate = 0;//38400
		gSystemPara.RS485Node = 10;
		gSystemPara.MagTapWide = 0;//front
		///default parameters
		memcpy(&Parameter,&gSystemPara,sizeof(gSystemPara));
		Parameter[99] = 0xCD;
		STMFLASH_Write(FLASH_SAVE_ADDR,( uint32_t * )Parameter,100);
	}
	////////////
	if(gSystemPara.RS232Bauderate>3 || gSystemPara.RS232Bauderate <0)
		gSystemPara.RS232Bauderate = 0;
	if(gSystemPara.RS485Bauderate>3 || gSystemPara.RS485Bauderate <0)
		gSystemPara.RS485Bauderate = 0;
	if(gSystemPara.CANBusBauderate>3 || gSystemPara.CANBusBauderate <0)
		gSystemPara.CANBusBauderate = 1;
	//
	CfgUartx(Uartx3,gSystemPara.RS232Bauderate,Uart3TX,Uart3RX);
	CfgUartx(Uartx6,gSystemPara.RS485Bauderate,Uart6TX,Uart6RX);
	Uart6RS485RE;//
	////////////
	TimCfg(1000,TIMx3);
	TIM8_PWM_Init(500-1,168-1);
	////////////
	MagnetSensors.MagTapeWidth = TapeWides[gSystemPara.MagTapWide];
	SetDO(DOCans,0);
	////////////
	CAN_Configuration(gSystemPara.CANBusBauderate);
	/////////////
	CanSendBuf.CAN_ID = gSystemPara.CanNode;
	/////////////
};
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
void CfgPINIn(uint32_t GPIOx, uint16_t GPIO_Pin,GPIOMode_TypeDef GPIOMode,GPIOPuPd_TypeDef GPIOPupd)
{
	GPIO_InitTypeDef GPIO_InitStructure;
  APBCLKCfg(GPIOx, ENABLE); 						// 使能PC端口时钟
	//////////////设置成输入的，频率是50M/////////////
  GPIO_InitStructure.GPIO_Mode = GPIOMode;
	GPIO_InitStructure.GPIO_PuPd = GPIOPupd;	
	///////////////////////////////////////////////////
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin;	
  GPIO_Init((GPIO_TypeDef *)GPIOx,&GPIO_InitStructure);
};
void CfgPINOut(uint32_t  GPIOx,uint16_t GPIO_Pin,GPIOMode_TypeDef GPIOMode,GPIOPuPd_TypeDef GPIOPupd,GPIOOType_TypeDef OType)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	APBCLKCfg(GPIOx, ENABLE); 						// 使能PC端口时钟
	//////////////设置成输出的，频率是50M//////////////
  GPIO_InitStructure.GPIO_Mode = GPIOMode;       
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIOPupd;
	GPIO_InitStructure.GPIO_OType = OType;
	////////////////////////////////////////////////////
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin;	
  GPIO_Init((GPIO_TypeDef *)GPIOx,&GPIO_InitStructure);
  GPIO_ResetBits((GPIO_TypeDef *)GPIOx,GPIO_Pin);
};

void APBCLKCfg( uint32_t  GPIOx,FunctionalState NewState)
{
		if(GPIOx>=GPIOA_BASE && GPIOx<=DMA1_BASE)
			RCC_AHB1PeriphClockCmd(GPIO2APB2CLK(GPIOx), NewState);	
		else if(GPIOx>=TIM1_BASE && GPIOx<=SAI1_BASE)
			RCC_APB2PeriphClockCmd(GPIO2APB2CLK(GPIOx), NewState);	
		else if(GPIOx>=TIM2_BASE && GPIOx<=UART8_BASE)
			RCC_APB1PeriphClockCmd(GPIO2APB2CLK(GPIOx), NewState);	
	
}
uint32_t GPIO2APB2CLK(uint32_t  GPIOx)
{
		switch (GPIOx)
	{
		case PxA:
			return RCC_AHB1Periph_GPIOA;
			break;
		
		case PxB:
			return RCC_AHB1Periph_GPIOB;
			break;
		
		case PxC:
			return RCC_AHB1Periph_GPIOC;
			break;
		
		case PxD:
			return RCC_AHB1Periph_GPIOD;
			break;
		
		case Uartx1:
			return RCC_APB2Periph_USART1;
			break;
		
		case Uartx3:
			return RCC_APB1Periph_USART3;
			break;
		
		case Uartx6:
			return RCC_APB2Periph_USART6;
			break;
		case TIMx3:
			return RCC_APB1Periph_TIM3;
		  break;
	}
}

uint8_t UAartAFR(uint32_t UartX)
{
		switch (UartX)
	{
		case USART1_BASE: 
			return GPIO_AF_USART1;
			break;
		
		case USART3_BASE: 
			return GPIO_AF_USART3;
			break;
		
		case USART6_BASE:
			return GPIO_AF_USART6;
			break;
	}
}

uint8_t AFPinsource(uint16_t PinDefine)
{
		switch (PinDefine)
	{
		case GPIO_Pin_0: 
			return GPIO_PinSource0;
			break;
		case GPIO_Pin_1: 
			return GPIO_PinSource1;
			break;
		case GPIO_Pin_2: 
			return GPIO_PinSource2;
			break;
		case GPIO_Pin_3: 
			return GPIO_PinSource3;
			break;
		case GPIO_Pin_4: 
			return GPIO_PinSource4;
			break;
		case GPIO_Pin_5: 
			return GPIO_PinSource5;
			break;
		case GPIO_Pin_6: 
			return GPIO_PinSource6;
			break;
		case GPIO_Pin_7: 
			return GPIO_PinSource7;
			break;
		case GPIO_Pin_8: 
			return GPIO_PinSource8;
			break;
		case GPIO_Pin_9: 
			return GPIO_PinSource9;
			break;
		case GPIO_Pin_10: 
			return GPIO_PinSource10;
			break;
		case GPIO_Pin_11: 
			return GPIO_PinSource11;
			break;
		case GPIO_Pin_12: 
			return GPIO_PinSource12;
			break;
		case GPIO_Pin_13: 
			return GPIO_PinSource13;
			break;
		case GPIO_Pin_14: 
			return GPIO_PinSource14;
			break;
		case GPIO_Pin_15: 
			return GPIO_PinSource15;
			break;
	}
}
uint8_t ReadIn(uint32_t GPIOxADR, uint16_t GPIO_Pin,GPIOMode_TypeDef GPIOMode,GPIOPuPd_TypeDef GPIOPupd)
{
	GPIO_TypeDef* GPIOx;
	uint8_t bitstatus = 0x00;
	GPIOx=(GPIO_TypeDef*)GPIOxADR;
  
  /* Check the parameters */
  assert_param(IS_GPIO_ALL_PERIPH(GPIOx));
  assert_param(IS_GET_GPIO_PIN(GPIO_Pin)); 
  
  if ((GPIOx->IDR & GPIO_Pin) != (uint32_t)Bit_RESET)
  {
    bitstatus = (uint8_t)Bit_SET;
  }
  else
  {
    bitstatus = (uint8_t)Bit_RESET;
  }
  return bitstatus;
}
void SetDO(uint32_t GPIOxADR,uint16_t GPIO_Pin,GPIOMode_TypeDef GPIOMode,GPIOPuPd_TypeDef GPIOPupd,GPIOOType_TypeDef OType,bool state)
{
	
	GPIO_TypeDef* GPIOx;
	GPIOx=(GPIO_TypeDef*)GPIOxADR;
	
  assert_param(IS_GPIO_ALL_PERIPH(GPIOx));
  assert_param(IS_GPIO_PIN(GPIO_Pin));

	if(state) GPIOx->BSRRL = GPIO_Pin;
	else GPIOx->BSRRH = GPIO_Pin;
}

void CfgUartx(uint32_t UartX,uint8_t uartPar,uint32_t GPTx, uint16_t GPTX_Pin,uint32_t GPRx, uint16_t GPRX_Pin)
{
	
	//115200,8,N,1 "char uartPar[]"用来设置
	GPIO_InitTypeDef GPIO_InitStructureTx;
	GPIO_InitTypeDef GPIO_InitStructureRx;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStruct;
	
	APBCLKCfg(UartX, ENABLE); 						// 使能PC端口时钟
	APBCLKCfg(GPTx, ENABLE); 						  // 使能PC端口时钟
	APBCLKCfg(GPRx, ENABLE); 						  // 使能PC端口时钟
	
	GPIO_PinAFConfig((GPIO_TypeDef *)GPTx, AFPinsource(GPTX_Pin), UAartAFR(UartX));//Standard meathod
	GPIO_PinAFConfig((GPIO_TypeDef *)GPRx, AFPinsource(GPRX_Pin), UAartAFR(UartX));//Standard meathod
	
  GPIO_InitStructureTx.GPIO_Pin = GPTX_Pin;
  GPIO_InitStructureTx.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructureTx.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructureTx.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructureTx.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init((GPIO_TypeDef *)GPTx, &GPIO_InitStructureTx); 
		
  GPIO_InitStructureRx.GPIO_Pin = GPRX_Pin;
  GPIO_InitStructureRx.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructureRx.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructureRx.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructureRx.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init((GPIO_TypeDef *)GPRx, &GPIO_InitStructureRx); 
	
	USART_InitStructure.USART_BaudRate = ComBauderate[uartPar];																			//波特率设置：115200
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;											//数据位数设置：8位
	USART_InitStructure.USART_StopBits = USART_StopBits_1; 													//停止位设置：1位
	USART_InitStructure.USART_Parity = USART_Parity_No ;  													//是否奇偶校验：无
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;	//硬件流控制模式设置：没有使能
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;									//接收与发送都使能
	
	USART_Init((USART_TypeDef *)UartX, &USART_InitStructure);  
	USART_Cmd((USART_TypeDef *)UartX, ENABLE);		
	USART_ITConfig((USART_TypeDef *)UartX,USART_IT_RXNE,ENABLE);										//初始化USART1
																																									// USART1使能
	NVIC_InitStruct.NVIC_IRQChannel = MapIRQn(UartX);
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 2;     // 主优先级为1
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;            // 次优先级为0
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStruct);
	
}
enum IRQn MapIRQn(uint32_t BASEType)
{
	switch (BASEType)
	{
		
		case Uartx1:
			return USART1_IRQn;
			break;
		
		case Uartx6:
			return USART6_IRQn;
			break;
		
		case Uartx3:
			return USART3_IRQn;
			break;

		case TIMx3:
			return TIM3_IRQn;
			break;
		
		default:
			return 0;
			break;
	}
}
void USART6_IRQHandler(void)//RS485
{
	FillUrtBuf(&(Uart6Data),Uartx6);//////fill the Com Buffer//////
}
void USART3_IRQHandler(void)//RS232
{
	FillUrtBuf(&(Uart3Data),Uartx3);//////fill the Com Buffer//////
}
void FillUrtBuf(UrtBuf_type * pBoxIO,uint32_t USARTx)
{
	USART_TypeDef * Uarts;
	Uarts = (USART_TypeDef*)USARTx;
	if(USART_GetITStatus(Uarts, USART_IT_RXNE) != RESET)
	{
		uint8_t ResData;
		uint8_t iFill;
		ResData = Uarts->DR;		
		iFill=pBoxIO->pRfil;
		pBoxIO->rBuffer[iFill] = ResData;
		pBoxIO->pRfil++;		
		if(pBoxIO->pRfil >= UrtBfLen)
		{
			if(pBoxIO->pRder>0)pBoxIO->pRder--;
			pBoxIO->pRfil = UrtBfLen-1;
			MBLArry(pBoxIO->rBuffer,UrtBfLen);
		}
	}
}
void MBLArry(uint8_t *buffer,uint8_t bufLen)
{
	uint8_t iArry;
	if(bufLen<1) bufLen=1;
	for(iArry=0;iArry<bufLen-1;iArry++)
	{
		buffer[iArry] = buffer[iArry+1];
	}
}

void SendUrtBuf(UrtBuf_type * pBoxIO,uint32_t USARTx)
{
		unsigned char i;
		unsigned char bufLen = pBoxIO->sLen;
		USART_TypeDef * Uarts;
		Uarts = (USART_TypeDef*)USARTx;
		switch(USARTx)
		{
		  case Uartx6:
				Uart6RS485SE;
				break;
			default:
				;
				break;
		}
		for(i=0;i<bufLen;i++)
		{
			
			USART_SendData(Uarts, pBoxIO->sBuffer[i]);
			while (USART_GetFlagStatus(Uarts, USART_FLAG_TXE) == RESET);			
		}
		while(USART_GetFlagStatus(Uarts, USART_FLAG_TC)==RESET);
		switch(USARTx)
		{
		  case Uartx6:
				Uart6RS485RE;
				break;
			default:
				;
				break;
		}
};
void TimCfg(uint32_t timeUs ,uint32_t BASEType)
{
	uint16_t arr=timeUs/100-1;

	TIM_TypeDef * TIMxS;
	TIMxS = (TIM_TypeDef * )BASEType;
	APBCLKCfg(BASEType, ENABLE);
	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);  ///使能TIM3时钟
	
  TIM_TimeBaseInitStructure.TIM_Period = arr; 	//自动重装载值
	TIM_TimeBaseInitStructure.TIM_Prescaler=8400-1;  //定时器分频
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIMxS,&TIM_TimeBaseInitStructure);//初始化TIM3
	
	TIM_ITConfig(TIMxS,TIM_IT_Update,ENABLE); //允许定时器3更新中断
	TIM_Cmd(TIMxS,ENABLE); //使能定时器3
	
	NVIC_InitStructure.NVIC_IRQChannel=MapIRQn(BASEType); //定时器3中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x01; //抢占优先级1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x03; //子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

void TIM8_PWM_Init(u32 arr,u32 psc)
{		 					 
	//此部分需手动修改IO口设置	
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8,ENABLE);  	//TIM8时钟使能    
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE); 	//使能PORTC时钟		
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource9,GPIO_AF_TIM8); //GPIOC9复用为定时器8
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;           //GPIOF9
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        //复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//速度100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //上拉
	GPIO_Init(GPIOC,&GPIO_InitStructure);               //初始化PF9
	  
	TIM_TimeBaseStructure.TIM_Prescaler=psc;  //定时器分频
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseStructure.TIM_Period=arr;   //自动重装载值
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	TIM_TimeBaseInit(TIM8,&TIM_TimeBaseStructure);//初始化定时器14
	
	//初始化TIM8 Channel1 PWM模式	 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //选择定时器模式:TIM脉冲宽度调制模式2
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //输出极性:TIM输出比较极性低
	TIM_OC4Init(TIM8, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM1 4OC1
	TIM_OC4PreloadConfig(TIM8, TIM_OCPreload_Enable);  //使能TIM8在CCR1上的预装载寄存器
  TIM_ARRPreloadConfig(TIM8,ENABLE);//ARPE使能 
	TIM_Cmd(TIM8, ENABLE);  //使能TIM8
	TIM_CtrlPWMOutputs(TIM8,ENABLE);									  
}  
void TIM3_IRQHandler(void)
{
	static uint32_t oldTimeStamp[6];						///5个不同时间周期的中断处理
	static bool state100ms=0;
	if(TIM3->SR&0X0001)												//溢出中断
	{			
		TimeStamp++;														//生成一个TimeStamp的时钟		
		oldTimeStamp[task2ms]++;
		oldTimeStamp[task4ms]++;
		oldTimeStamp[task20ms]++;
		oldTimeStamp[task100ms]++;
		oldTimeStamp[task500ms]++;
		oldTimeStamp[taskLong]++;		
		/////////////////2MS执行一遍////////////////////
		if(oldTimeStamp[task2ms]>(time2ms))
		{
			if(led0pwmval<=0)dir=0;
			if(led0pwmval==500)dir=1;
			/////////////////
			if(dir==0&&led0pwmval>10&&led0pwmval<500)
				led0pwmval++;
			else if(dir==1&&led0pwmval>10&&led0pwmval<500)
				led0pwmval--;
			oldTimeStamp[task2ms] = 0;
		};
		/////////////////4MS执行一遍////////////////////
		if(oldTimeStamp[task4ms]>(time4ms))
		{		
			/////////////////
			//////////////////////////C A N////////////////////////////
			int8_t temp[10] = {0};
			if(MagnetSensors.cmdSendCanData){//Automatic data format			
				MagnetSensors.cmdSendCanData = 0;
				///////////////////////////////////	
				temp[0] = 1;
				temp[1] = MagnetSensors.SensorValue[0]>>8;
				temp[2] = MagnetSensors.SensorValue[0];				
				temp[3] = MagnetSensors.SensorValue[1]>>8;
				temp[4] = MagnetSensors.SensorValue[1];			
				temp[5] = MagnetSensors.SensorValue[2]>>8;
				temp[6] = MagnetSensors.SensorValue[2];	
				temp[7] = 0;
				CanWriteData(CanSendBuf.CAN_ID,(uint8_t *)temp,8);	
				
				temp[0] = 1;
				temp[1] = MagnetSensors.SensorValue[3]>>8;
				temp[2] = MagnetSensors.SensorValue[3];				
				temp[3] = MagnetSensors.SensorValue[4]>>8;
				temp[4] = MagnetSensors.SensorValue[4];			
				temp[5] = MagnetSensors.SensorValue[5]>>8;
				temp[6] = MagnetSensors.SensorValue[5];	
				temp[7] = 0;
				CanWriteData(CanSendBuf.CAN_ID,(uint8_t *)temp,8);	
				
				temp[0] = 1;
				temp[1] = MagnetSensors.SensorValue[6]>>8;
				temp[2] = MagnetSensors.SensorValue[6];				
				temp[3] = MagnetSensors.SensorValue[7]>>8;
				temp[4] = MagnetSensors.SensorValue[7];			
				temp[5] = 0;
				temp[6] = 0;	
				temp[7] = 0;
				CanWriteData(CanSendBuf.CAN_ID,(uint8_t *)temp,8);	
			}
			//////////////////////////4 8 5////////////////////////////
			if(MagnetSensors.cmdSendRS485Data){
				MagnetSensors.cmdSendRS485Data = 0;					
				UartTempData.sBuffer[0] = 0xAA;
				UartTempData.sBuffer[1] = 0x53;
				UartTempData.sBuffer[2] = 0x29;
				UartTempData.sBuffer[3] = 0x01;
				UartTempData.sBuffer[4] = 0x00;
				UartTempData.sBuffer[5] = MagnetSensors.SensorValue[0]>>8;
				UartTempData.sBuffer[6] = MagnetSensors.SensorValue[0];				
				UartTempData.sBuffer[7] = MagnetSensors.SensorValue[1]>>8;
				UartTempData.sBuffer[8] = MagnetSensors.SensorValue[1];			
				UartTempData.sBuffer[9] = MagnetSensors.SensorValue[2]>>8;
				UartTempData.sBuffer[10] = MagnetSensors.SensorValue[2];			
				UartTempData.sBuffer[11] = MagnetSensors.SensorValue[3]>>8;
				UartTempData.sBuffer[12] = MagnetSensors.SensorValue[3];		
				UartTempData.sBuffer[13] = MagnetSensors.SensorValue[4]>>8;
				UartTempData.sBuffer[14] = MagnetSensors.SensorValue[4];	
				UartTempData.sBuffer[15] = MagnetSensors.SensorValue[5]>>8;
				UartTempData.sBuffer[16] = MagnetSensors.SensorValue[5];
				UartTempData.sBuffer[17] = MagnetSensors.SensorValue[6]>>8;
				UartTempData.sBuffer[18] = MagnetSensors.SensorValue[6];
				UartTempData.sBuffer[19] = MagnetSensors.SensorValue[7]>>8;
				UartTempData.sBuffer[20] = MagnetSensors.SensorValue[7];															
				UartTempData.sBuffer[43] = 0xAE;				
				UartTempData.sLen = 44;
				SendUrtBuf(&UartTempData,Uartx6);				

			}			
			//////////////////////////2 3 2////////////////////////////
			if(MagnetSensors.ReqSetOKAllow){
				MagnetSensors.ReqSetOKAllow = 0;
				Uart3Data.sBuffer[0] = 0xAA;
				Uart3Data.sBuffer[1] = 0x52;
				Uart3Data.sBuffer[2] = 0x04;
				Uart3Data.sBuffer[3] = 0x01;
				Uart3Data.sBuffer[4] = 0x00;
				Uart3Data.sBuffer[5] = 0x01;
				Uart3Data.sBuffer[6] = 0xAC;
				Uart3Data.sLen = 7;
				MagnetSensors.cmdSendRS232Data = 0;
				if(MagnetSensors.handShakeInterface==0)
					SendUrtBuf(&Uart3Data,Uartx3);
				else
					SendUrtBuf(&Uart3Data,Uartx6);
			}
			else if(MagnetSensors.ReqConnectAllow){
				MagnetSensors.ReqConnectAllow = 0;
				Uart3Data.sBuffer[0] = 0xAA;
				Uart3Data.sBuffer[1] = 0x55;
				Uart3Data.sBuffer[2] = 0x11;
				Uart3Data.sBuffer[3] = 0x01;
				Uart3Data.sBuffer[4] = 0x00;
				Uart3Data.sBuffer[5] = gSystemPara.AutoSetZeroValue;
				Uart3Data.sBuffer[6] = gSystemPara.DetectPolar;
				Uart3Data.sBuffer[7] = gSystemPara.DataComInterface;
				Uart3Data.sBuffer[8] = gSystemPara.RS485Node;
				Uart3Data.sBuffer[9] = gSystemPara.CanNode>>8;
				Uart3Data.sBuffer[10] = gSystemPara.CanNode;
				Uart3Data.sBuffer[11] = gSystemPara.RS485Bauderate;
				Uart3Data.sBuffer[12] = gSystemPara.RS232Bauderate;
				Uart3Data.sBuffer[13] = gSystemPara.CANBusBauderate;
				Uart3Data.sBuffer[14] = gSystemPara.DataAnsMethod;
				Uart3Data.sBuffer[15] = gSystemPara.RequestInterval;
				Uart3Data.sBuffer[16] = gSystemPara.WeightRateKg;
				Uart3Data.sBuffer[17] = gSystemPara.MagTapWide;
				Uart3Data.sBuffer[18] = gSystemPara.EnableAutoRest;
				Uart3Data.sBuffer[19] = 0xAC;
				Uart3Data.sLen = 20;
				MagnetSensors.cmdSendRS232Data = 0;
				if(MagnetSensors.handShakeInterface==0)
					SendUrtBuf(&Uart3Data,Uartx3);
				else
					SendUrtBuf(&Uart3Data,Uartx6);
			}
			else if(MagnetSensors.cmdSendRS232Data){
				MagnetSensors.cmdSendRS232Data = 0;				
				UartTempData.sBuffer[0] = 0xAA;
				UartTempData.sBuffer[1] = 0x53;
				UartTempData.sBuffer[2] = 0x29;
				UartTempData.sBuffer[3] = 0x01;
				UartTempData.sBuffer[4] = 0x00;				
				UartTempData.sBuffer[5] = MagnetSensors.SensorValue[0]>>8;
				UartTempData.sBuffer[6] = MagnetSensors.SensorValue[0];				
				UartTempData.sBuffer[7] = MagnetSensors.SensorValue[1]>>8;
				UartTempData.sBuffer[8] = MagnetSensors.SensorValue[1];			
				UartTempData.sBuffer[9] = MagnetSensors.SensorValue[2]>>8;
				UartTempData.sBuffer[10] = MagnetSensors.SensorValue[2];			
				UartTempData.sBuffer[11] = MagnetSensors.SensorValue[3]>>8;
				UartTempData.sBuffer[12] = MagnetSensors.SensorValue[3];		
				UartTempData.sBuffer[13] = MagnetSensors.SensorValue[4]>>8;
				UartTempData.sBuffer[14] = MagnetSensors.SensorValue[4];	
				UartTempData.sBuffer[15] = MagnetSensors.SensorValue[5]>>8;
				UartTempData.sBuffer[16] = MagnetSensors.SensorValue[5];
				UartTempData.sBuffer[17] = MagnetSensors.SensorValue[6]>>8;
				UartTempData.sBuffer[18] = MagnetSensors.SensorValue[6];
				UartTempData.sBuffer[19] = MagnetSensors.SensorValue[7]>>8;
				UartTempData.sBuffer[20] = MagnetSensors.SensorValue[7];															
				UartTempData.sBuffer[43] = 0xAE;				
				UartTempData.sLen = 44;
				SendUrtBuf(&UartTempData,Uartx3);
			}
			oldTimeStamp[task4ms] = 0;
		};
		/////////////////20MS执行一遍////////////////////
		if(oldTimeStamp[task20ms]>(time20ms))
		{
			/////////////////
			if(dir==0&&(led0pwmval<=10 || led0pwmval>=500))
				led0pwmval++;
			else if(dir==1&&(led0pwmval<=10 || led0pwmval>=500))
				led0pwmval--;
			oldTimeStamp[task20ms] = 0;
		};
		/////////////////100ms执行一遍///////////////////
		if(oldTimeStamp[task100ms]>(time100ms))
		{
			cntFlash500++;
			flashLED[0] = cntFlash500==1||cntFlash500==3;//慢速闪烁
			if(cntFlash500>=10)cntFlash500=0;
			oldTimeStamp[task100ms] = 0;
		};
				/////////////////500ms执行一遍///////////////////
		if(oldTimeStamp[task500ms]>(time500ms))
		{
			TimeStamp500ms++;
			oldTimeStamp[task500ms] = 0;
		};
		////////////////长时间的时间，任意设定/////////
		if(oldTimeStamp[taskLong]>(timeLong))
		{	
			oldTimeStamp[taskLong] = 0;
		};
	}
	
	TIM3->SR&=~(1<<0);//清除中断标志位 	 
}

CanRxMsg RxMessage;

void CAN1_RX0_IRQHandler(void)
{
	CAN_ClearITPendingBit(CAN1,CAN_IT_FMP0);

	CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);
	
	if((RxMessage.StdId==1) && (RxMessage.RTR==CAN_RTR_REMOTE))
	{
			uint8_t temp[10] = {0};
			MagnetSensors.cmdSendCanData = 1;
	}
}

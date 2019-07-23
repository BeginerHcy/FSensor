#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "string.h"
#include "stm32f4xx_spi.h"

/* stdbool.h standard header */
#ifndef _STDBOOL
#define _STDBOOL



#define __bool_true_false_are_defined        1

#ifndef __cplusplus
                /* TYPES */

#if 199901L <= __STDC_VERSION__


#else /* 199901L <= __STDC_VERSION__ */
#if __TI_STRICT_ANSI_MODE__
typedef unsigned char _Bool;
#endif
#endif /* 199901L <= __STDC_VERSION__ */

                /* MACROS */
#define bool        _Bool
#define false        0
#define true        1
#endif /* __cplusplus */


#endif /* _STDBOOL */

/*
* Copyright (c) 1992-2004 by P.J. Plauger.  ALL RIGHTS RESERVED.
* Consult your license regarding permissions and restrictions.
V4.02:1476 */

/////define SPI Interface
#define      macSPIx                                     SPI1
#define      macSPI_APBxClock_FUN                        RCC_APB2PeriphClockCmd 
#define      macSPI_CLK                                  RCC_APB2Periph_SPI1

#define      macSPI_CS_APBxClock_FUN                     RCC_AHBPeriphClockCmd
#define      macSPI_CS_CLK                               RCC_AHBPeriph_GPIOB   
#define      macSPI_CS_PORT                              GPIOB
#define      macSPI_CS_PIN                               GPIO_Pin_11

#define      macSPI_CS1_APBxClock_FUN                    RCC_AHBPeriphClockCmd
#define      macSPI_CS1_CLK                              RCC_AHBPeriph_GPIOB   
#define      macSPI_CS1_PORT                             GPIOB
#define      macSPI_CS1_PIN                              GPIO_Pin_11

#define      macSPI_CS2_APBxClock_FUN                    RCC_AHBPeriphClockCmd
#define      macSPI_CS2_CLK                              RCC_AHBPeriph_GPIOB   
#define      macSPI_CS2_PORT                             GPIOB
#define      macSPI_CS2_PIN                              GPIO_Pin_12

#define      macSPI_CS3_APBxClock_FUN                    RCC_AHBPeriphClockCmd
#define      macSPI_CS3_CLK                              RCC_AHBPeriph_GPIOB   
#define      macSPI_CS3_PORT                             GPIOB
#define      macSPI_CS3_PIN                              GPIO_Pin_15

#define      macSPI_CS4_APBxClock_FUN                    RCC_AHBPeriphClockCmd
#define      macSPI_CS4_CLK                              RCC_AHBPeriph_GPIOB  
#define      macSPI_CS4_PORT                             GPIOB
#define      macSPI_CS4_PIN                              GPIO_Pin_14

#define      macSPI_CS5_APBxClock_FUN                    RCC_AHBPeriphClockCmd
#define      macSPI_CS5_CLK                              RCC_AHBPeriph_GPIOB   
#define      macSPI_CS5_PORT                             GPIOB
#define      macSPI_CS5_PIN                              GPIO_Pin_13

#define      macSPI_SCK_APBxClock_FUN                    RCC_AHBPeriphClockCmd
#define      macSPI_SCK_CLK                              RCC_AHBPeriph_GPIOA
#define      macSPI_SCK_PORT                             GPIOA   
#define      macSPI_SCK_PIN                              GPIO_Pin_5

#define      macSPI_MISO_APBxClock_FUN                   RCC_AHBPeriphClockCmd
#define      macSPI_MISO_CLK                             RCC_AHBPeriph_GPIOA
#define      macSPI_MISO_PORT                            GPIOA 
#define      macSPI_MISO_PIN                             GPIO_Pin_6

#define      macSPI_MOSI_APBxClock_FUN                   RCC_AHBPeriphClockCmd
#define      macSPI_MOSI_CLK                             RCC_AHBPeriph_GPIOA
#define      macSPI_MOSI_PORT                            GPIOA 
#define      macSPI_MOSI_PIN                             GPIO_Pin_7

////Redefine
#define PxA GPIOA_BASE
#define PxB GPIOB_BASE
#define PxC GPIOC_BASE
#define PxD GPIOD_BASE
#define Uartx1 USART1_BASE
#define Uartx2 USART2_BASE
#define Uartx3 USART3_BASE
#define Uartx6 USART6_BASE
#define TIMx3  TIM3_BASE///使用TIM3作为1ms中断源
///////////
#define PIN0 	GPIO_Pin_0
#define PIN1 	GPIO_Pin_1
#define PIN2 	GPIO_Pin_2
#define PIN3 	GPIO_Pin_3
#define PIN4 	GPIO_Pin_4
#define PIN5 	GPIO_Pin_5
#define PIN6 	GPIO_Pin_6
#define PIN7 	GPIO_Pin_7
#define PIN8 	GPIO_Pin_8
#define PIN9 	GPIO_Pin_9
#define PIN10 GPIO_Pin_10
#define PIN11 GPIO_Pin_11
#define PIN12 GPIO_Pin_12
#define PIN13 GPIO_Pin_13
#define PIN14 GPIO_Pin_14
#define PIN15 GPIO_Pin_15

///////////MAPING-OUTPUT
#define DOCans 	PxC,PIN8,GPIO_Mode_OUT,GPIO_PuPd_NOPULL,GPIO_OType_PP
#define Uart6DRE	PxC,PIN8,GPIO_Mode_OUT,GPIO_PuPd_NOPULL,GPIO_OType_PP
#define SSN1 		PxC,PIN3,GPIO_Mode_OUT,GPIO_PuPd_NOPULL,GPIO_OType_PP
#define SSN2 		PxA,PIN2,GPIO_Mode_OUT,GPIO_PuPd_NOPULL,GPIO_OType_PP
#define SSN3 		PxA,PIN4,GPIO_Mode_OUT,GPIO_PuPd_NOPULL,GPIO_OType_PP
#define SSN4 		PxA,PIN6,GPIO_Mode_OUT,GPIO_PuPd_NOPULL,GPIO_OType_PP
#define SSN5 		PxC,PIN4,GPIO_Mode_OUT,GPIO_PuPd_NOPULL,GPIO_OType_PP
#define SSN6 		PxB,PIN0,GPIO_Mode_OUT,GPIO_PuPd_NOPULL,GPIO_OType_PP
#define SSN7 		PxB,PIN2,GPIO_Mode_OUT,GPIO_PuPd_NOPULL,GPIO_OType_PP
#define SSN8 		PxB,PIN11,GPIO_Mode_OUT,GPIO_PuPd_NOPULL,GPIO_OType_PP
///////////MAPING-INPUT
#define DRDY1 	PxC,PIN2,GPIO_Mode_IN,GPIO_PuPd_NOPULL
#define DRDY2 	PxA,PIN1,GPIO_Mode_IN,GPIO_PuPd_NOPULL
#define DRDY3 	PxA,PIN3,GPIO_Mode_IN,GPIO_PuPd_NOPULL
#define DRDY4 	PxA,PIN5,GPIO_Mode_IN,GPIO_PuPd_NOPULL
#define DRDY5 	PxA,PIN7,GPIO_Mode_IN,GPIO_PuPd_NOPULL
#define DRDY6 	PxC,PIN5,GPIO_Mode_IN,GPIO_PuPd_NOPULL
#define DRDY7 	PxB,PIN1,GPIO_Mode_IN,GPIO_PuPd_NOPULL
#define DRDY8 	PxB,PIN10,GPIO_Mode_IN,GPIO_PuPd_NOPULL
///////////MAPING-URT4->CAN

///////////MAPING-URT3->232
#define Uart3TX PxC,PIN10
#define Uart3RX PxC,PIN11

///////////MAPING-URT1->485
#define Uart6TX PxC,PIN6
#define Uart6RX PxC,PIN7

///////////IIC1
#define IICSCL PxB,PIN6,GPIO_Mode_IN,GPIO_PuPd_NOPULL
#define IICSDA PxB,PIN7,GPIO_Mode_IN,GPIO_PuPd_NOPULL
#define IICSDAOut PxB,PIN7,GPIO_Mode_OUT,GPIO_PuPd_NOPULL,GPIO_OType_PP
//////SPI function
#define      macSPI_IMU_CS_ENABLE()                       GPIO_ResetBits( macSPI_CS_PORT, macSPI_CS_PIN )
#define      macSPI_IMU_CS_DISABLE()                      GPIO_SetBits( macSPI_CS_PORT, macSPI_CS_PIN )

#define      macSPI_PNI1_CS_ENABLE()                       GPIO_ResetBits( macSPI_CS1_PORT, macSPI_CS1_PIN )
#define      macSPI_PNI1_CS_DISABLE()                      GPIO_SetBits( macSPI_CS1_PORT, macSPI_CS1_PIN )

#define      macSPI_PNI2_CS_ENABLE()                       GPIO_ResetBits( macSPI_CS2_PORT, macSPI_CS2_PIN )
#define      macSPI_PNI2_CS_DISABLE()                      GPIO_SetBits( macSPI_CS2_PORT, macSPI_CS2_PIN )

#define      macSPI_PNI3_CS_ENABLE()                       GPIO_ResetBits( macSPI_CS3_PORT, macSPI_CS3_PIN )
#define      macSPI_PNI3_CS_DISABLE()                      GPIO_SetBits( macSPI_CS3_PORT, macSPI_CS3_PIN )

#define      macSPI_PNI4_CS_ENABLE()                       GPIO_ResetBits( macSPI_CS4_PORT, macSPI_CS4_PIN )
#define      macSPI_PNI4_CS_DISABLE()                      GPIO_SetBits( macSPI_CS4_PORT, macSPI_CS4_PIN )

#define      macSPI_PNI5_CS_ENABLE()                       GPIO_ResetBits( macSPI_CS5_PORT, macSPI_CS5_PIN )
#define      macSPI_PNI5_CS_DISABLE()                      GPIO_SetBits( macSPI_CS5_PORT, macSPI_CS5_PIN )
///////////////////////////////////////
#define TIMx3  TIM3_BASE///使用TIM3作为1ms中断源
#define	InputNum 16
#define OutputNum 16
#define UrtBfLen  100

#define time2ms   2
#define time4ms   4
#define time20ms  20
#define time100ms 100
#define time500ms 500
#define timeLong  1500
typedef	enum {task2ms=0,task4ms,task20ms,task100ms,task500ms,taskLong} cycTaskIndex;
typedef struct UrtBuf_type
{
	uint8_t	rBuffer[UrtBfLen];
	uint8_t  pRfil;
	uint8_t  pRder;
	
	int8_t	sBuffer[UrtBfLen];
	uint8_t  sLen;
	uint8_t  flagS;
}UrtBuf_type;

typedef struct UrtBufint_type
{
	int8_t	rBuffer[UrtBfLen];
	uint8_t  pRfil;
	uint8_t  pRder;
	
	int8_t	sBuffer[UrtBfLen];
	uint8_t  sLen;
	uint8_t  flagS;
}UrtBufint_type;
typedef struct MoveStrutType
{
	double x;
	double Ntime;
	double buffer[200];
	uint32_t  Nfilled;
	double y;
}MoveStrutType;
typedef struct SysParameter_type
{
	uint8_t	 BufferHead;
	uint8_t  DetectPolar;//0-N  1-S
	uint8_t  DataComInterface;//0-CAN,1-RS485,2-RS232
	uint8_t  RS485Node;//
	uint16_t CanNode;//
	uint8_t  RS485Bauderate;//0-9600 1-19200 2-38400 3-115200
	uint8_t	 RS232Bauderate;//0-9600 1-19200 2-38400 3-115200
	uint8_t	 CANBusBauderate;//0-125k 1-250k 2-500k 3-1000k 
	uint8_t  DataAnsMethod;//0-automate 1-passive
	uint8_t  RequestInterval;//0-4ms 1-10ms  2-20ms 3-40ms 4-100ms
	uint8_t  MagSensity;//0-80 1-150 2-200 3-250 4-300;
	uint8_t  MagTapWide;//0-30 1- 50;
	uint8_t  MountDir;//0-default 1- back;
	uint8_t  SensityValve;
	///////////////////////////
	int32_t  Offset_Basic[10];
	int16_t  Dev_Factor[10];
	///////////////////////////
}SysParameter_type;

extern UrtBuf_type Uart1Data;
extern UrtBuf_type Uart3Data;
extern SysParameter_type gSystemPara;
////////////////some Function to config the Hardware
void SystemConfig();
uint32_t GPIO2APB2CLK(uint32_t  GPIOx);
enum IRQn MapIRQn(uint32_t BASEType);
void CfgPINOut(uint32_t  GPIOx,uint16_t GPIO_Pin,GPIOMode_TypeDef GPIOMode,GPIOPuPd_TypeDef GPIOPupd,GPIOOType_TypeDef OType);
void CfgPINIn(uint32_t GPIOx, uint16_t GPIO_Pin,GPIOMode_TypeDef GPIOMode,GPIOPuPd_TypeDef GPIOPupd);
void CfgUartx(uint32_t UartX,uint8_t uartPar,uint32_t GPTx, uint16_t GPTX_Pin,uint32_t GPRx, uint16_t GPRX_Pin);
void HwCfgInit();
void TimCfg(uint32_t timeUs,uint32_t BASEType);
//SPI 相关
void SPI_PNI_Init(void);
uint16_t SPI_IMU_SendByte(uint16_t byte);
uint16_t WirteReg(uint8_t Addr, uint8_t data);
uint16_t ReadReg(uint8_t Addr);
void JTAGOFF();
uint16_t ReceiveByte(void);
void APBCLKCfg( uint32_t  GPIOx,FunctionalState NewState);
double TempScal(int16_t a);
void Temp2Raw(void);
double VelScal(int16_t a);
double AccScal(int16_t a);
double VelOFFScal(int16_t a);
double AccOFFScal(int16_t a);
extern void TIM8_PWM_Init(u32 arr,u32 psc);
//uint8_t SPI1_ReadWriteByte(uint8_t TxData);
extern uint8_t ReadIn(uint32_t GPIOxADR, uint16_t GPIO_Pin,GPIOMode_TypeDef GPIOMode,GPIOPuPd_TypeDef GPIOPupd);
extern void SetDO(uint32_t GPIOxADR,uint16_t GPIO_Pin,GPIOMode_TypeDef GPIOMode,GPIOPuPd_TypeDef GPIOPupd,GPIOOType_TypeDef OType,bool state);
void FillUrtBuf(UrtBuf_type * pBoxIO,uint32_t USARTx);
extern void SendUrtBuf(UrtBuf_type * pBoxIO,uint32_t USARTx);
void MBLArry(uint8_t *buffer,uint8_t bufLen);
void TimCfg(uint32_t timeUs ,uint32_t BASEType);
double MoveAvgFilter(MoveStrutType* pFunData);
void ArryMLO(double* buf,u32 bufByte);
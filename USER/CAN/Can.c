#include <math.h>
#include "Can.h"

static void CAN_GPIO_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_GPIOA_CLK_ENABLE();
	/* CAN1 Periph clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);//使能CAN1时钟	
	
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource11,GPIO_AF_CAN1);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource12,GPIO_AF_CAN1); 
	/* Configure CAN pin: RX */									               // PA11
	GPIO_InitStructure.GPIO_Pin    = GPIO_Pin_11|GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode   = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType  = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed  = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd   = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStructure);    
}

void CAN_Configuration(uint8_t Bauderate)
{
	CAN_InitTypeDef        CAN_InitStructure;
	CAN_FilterInitTypeDef  CAN_FilterInitStructure;
	CAN_GPIO_Configuration();
	
	/* CAN register init */
	CAN_DeInit(CAN1);
	CAN_StructInit(&CAN_InitStructure);
	
	NVIC_InitTypeDef NVIC_InitStructure; 
	/* Enable the Can Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;     // 主优先级为1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;            // 次优先级为0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* CAN cell init */
	CAN_InitStructure.CAN_TTCM = DISABLE; /* 时间触发禁止, 时间触发：CAN硬件的内部定时器被激活，并且被用于产生时间戳 */
	CAN_InitStructure.CAN_ABOM = DISABLE; /* 自动离线禁止，自动离线：一旦硬件监控到128次11个隐性位，就自动退出离线状态。在这里要软件设定后才能退出 */
	CAN_InitStructure.CAN_AWUM = DISABLE; /* 自动唤醒禁止，有报文来的时候自动退出休眠	*/
	CAN_InitStructure.CAN_NART = DISABLE; /* 报文重传, 如果错误一直传到成功止，否则只传一次 */
	CAN_InitStructure.CAN_RFLM = DISABLE; /* 接收FIFO锁定, 1--锁定后接收到新的报文摘不要，0--接收到新的报文则覆盖前一报文	*/
	CAN_InitStructure.CAN_TXFP = DISABLE; /* 发送优先级  0---由标识符决定  1---由发送请求顺序决定	*/
	CAN_InitStructure.CAN_Mode = CAN_Mode_Normal; /* 模式	*/
	CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;      /* 重新同步跳宽，只有can硬件处于初始化模式时才能访问这个寄存器 */
	CAN_InitStructure.CAN_BS1 = CAN_BS2_6tq;      /* 时间段1 */
	CAN_InitStructure.CAN_BS2 = CAN_BS1_7tq;      /* 时间段2 */

	switch(Bauderate){
		case 0://125k
			CAN_InitStructure.CAN_Prescaler = 24;
			break;
		case 1://250k
			CAN_InitStructure.CAN_Prescaler = 12;
			break;
		case 2://500k
			CAN_InitStructure.CAN_Prescaler = 6;
			break;
		case 3://1000k
			CAN_InitStructure.CAN_Prescaler = 3;
			break;
		default:
			CAN_InitStructure.CAN_Prescaler = 12;
			break;			
	}
	//CAN_InitStructure.CAN_Prescaler = 6;          /* 波特率预分频数 */  // 42/(1+6+7)/3 = 1000K
	CAN_Init(CAN1,&CAN_InitStructure);	

	CAN_FilterInitStructure.CAN_FilterNumber=0;     /* 过滤器0 */
	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask;  /* 屏敝模式 */
	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit; /* 32位 */
	CAN_FilterInitStructure.CAN_FilterIdHigh=0x0000;  /* 以下四个都为0, 表明不过滤任何id */
	CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000;
	CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=0;  /* 能够通过该过滤器的报文存到fifo0中 */
	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;
	
	//CAN_ITConfig(CAN,CAN_IT_FMP0, ENABLE);   /* 挂号中断, 进入中断后读fifo的报文函数释放报文清中断标志 */
	
	CAN_FilterInit(&CAN_FilterInitStructure);
	
	CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);//FIFO0消息挂号中断允许.	
	//RCC_APB1PeriphResetCmd(RCC_APB1Periph_CAN, ENABLE);
}

 /**
  * @file   CanWriteData
  * @brief  Can Write Date to CAN-BUS
  * @param  无
  * @retval 无
  */
void CanWriteData(uint32_t ID,uint8_t *databuf,uint8_t datalen)
{
	CanTxMsg TxMessage;
	TxMessage.StdId = ID;  /* 设置标准id  注意标准id的最高7位不能全是隐性(1)。共11位 ,This parameter can be a value between 0 to 0x7FF. */
	TxMessage.RTR 	= CAN_RTR_DATA; /* 设置为数据帧 */
	TxMessage.IDE 	= CAN_ID_STD;   /* 使用标准id	*/
	TxMessage.DLC 	= datalen;
	for(uint8_t i = 0;i < datalen;i++)
	{
		TxMessage.Data[i] = databuf[i]; 
	}
	CAN_Transmit(CAN1,&TxMessage);  /* 返回这个信息请求发送的邮箱号0,1,2或没有邮箱申请发送no_box */
}
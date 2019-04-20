#include <math.h>
#include "Can.h"

static void CAN_GPIO_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_GPIOA_CLK_ENABLE();
	/* CAN1 Periph clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);//ʹ��CAN1ʱ��	
	
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
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;     // �����ȼ�Ϊ1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;            // �����ȼ�Ϊ0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* CAN cell init */
	CAN_InitStructure.CAN_TTCM = DISABLE; /* ʱ�䴥����ֹ, ʱ�䴥����CANӲ�����ڲ���ʱ����������ұ����ڲ���ʱ��� */
	CAN_InitStructure.CAN_ABOM = DISABLE; /* �Զ����߽�ֹ���Զ����ߣ�һ��Ӳ����ص�128��11������λ�����Զ��˳�����״̬��������Ҫ����趨������˳� */
	CAN_InitStructure.CAN_AWUM = DISABLE; /* �Զ����ѽ�ֹ���б�������ʱ���Զ��˳�����	*/
	CAN_InitStructure.CAN_NART = DISABLE; /* �����ش�, �������һֱ�����ɹ�ֹ������ֻ��һ�� */
	CAN_InitStructure.CAN_RFLM = DISABLE; /* ����FIFO����, 1--��������յ��µı���ժ��Ҫ��0--���յ��µı����򸲸�ǰһ����	*/
	CAN_InitStructure.CAN_TXFP = DISABLE; /* �������ȼ�  0---�ɱ�ʶ������  1---�ɷ�������˳�����	*/
	CAN_InitStructure.CAN_Mode = CAN_Mode_Normal; /* ģʽ	*/
	CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;      /* ����ͬ������ֻ��canӲ�����ڳ�ʼ��ģʽʱ���ܷ�������Ĵ��� */
	CAN_InitStructure.CAN_BS1 = CAN_BS2_6tq;      /* ʱ���1 */
	CAN_InitStructure.CAN_BS2 = CAN_BS1_7tq;      /* ʱ���2 */

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
	//CAN_InitStructure.CAN_Prescaler = 6;          /* ������Ԥ��Ƶ�� */  // 42/(1+6+7)/3 = 1000K
	CAN_Init(CAN1,&CAN_InitStructure);	

	CAN_FilterInitStructure.CAN_FilterNumber=0;     /* ������0 */
	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask;  /* ����ģʽ */
	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit; /* 32λ */
	CAN_FilterInitStructure.CAN_FilterIdHigh=0x0000;  /* �����ĸ���Ϊ0, �����������κ�id */
	CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000;
	CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=0;  /* �ܹ�ͨ���ù������ı��Ĵ浽fifo0�� */
	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;
	
	//CAN_ITConfig(CAN,CAN_IT_FMP0, ENABLE);   /* �Һ��ж�, �����жϺ��fifo�ı��ĺ����ͷű������жϱ�־ */
	
	CAN_FilterInit(&CAN_FilterInitStructure);
	
	CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);//FIFO0��Ϣ�Һ��ж�����.	
	//RCC_APB1PeriphResetCmd(RCC_APB1Periph_CAN, ENABLE);
}

 /**
  * @file   CanWriteData
  * @brief  Can Write Date to CAN-BUS
  * @param  ��
  * @retval ��
  */
void CanWriteData(uint32_t ID,uint8_t *databuf,uint8_t datalen)
{
	CanTxMsg TxMessage;
	TxMessage.StdId = ID;  /* ���ñ�׼id  ע���׼id�����7λ����ȫ������(1)����11λ ,This parameter can be a value between 0 to 0x7FF. */
	TxMessage.RTR 	= CAN_RTR_DATA; /* ����Ϊ����֡ */
	TxMessage.IDE 	= CAN_ID_STD;   /* ʹ�ñ�׼id	*/
	TxMessage.DLC 	= datalen;
	for(uint8_t i = 0;i < datalen;i++)
	{
		TxMessage.Data[i] = databuf[i]; 
	}
	CAN_Transmit(CAN1,&TxMessage);  /* ���������Ϣ�����͵������0,1,2��û���������뷢��no_box */
}
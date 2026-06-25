
#include "can.h"
#include "CAN_com.h"


/******************************************************************************/
CanRxMsg can_rx;
/******************************************************************************/
void CAN1_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure; 
	CAN_InitTypeDef CAN_InitStructure;
	CAN_FilterInitTypeDef CAN_FilterInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);	                   											 
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);	//使能CAN1时钟	

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;		//TX 复用推挽
	GPIO_Init(GPIOA, &GPIO_InitStructure);				
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;		//RX 上拉输入
	GPIO_Init(GPIOA, &GPIO_InitStructure);				
	
	NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	//CAN单元设置
	CAN_DeInit(CAN1);
	CAN_InitStructure.CAN_TTCM = DISABLE;		//非时间触发通信模式  
	CAN_InitStructure.CAN_ABOM = DISABLE;		//软件自动离线管理	 
	CAN_InitStructure.CAN_AWUM = DISABLE;		//睡眠模式通过软件唤醒(清除CAN->MCR的SLEEP位)
	CAN_InitStructure.CAN_NART = ENABLE;		//禁止报文自动传送 
	CAN_InitStructure.CAN_RFLM = DISABLE;		//报文不锁定,新的覆盖旧的 
	CAN_InitStructure.CAN_TXFP = DISABLE;		//优先级由报文标识符决定 
	CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;	    //模式设置
	//设置波特率
	CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;	//重新同步跳跃宽度(Tsjw)为tsjw+1个时间单位  CAN_SJW_1tq	 CAN_SJW_2tq CAN_SJW_3tq CAN_SJW_4tq
	CAN_InitStructure.CAN_BS1 = CAN_BS1_9tq;  //Tbs1=tbs1+1个时间单位CAN_BS1_1tq ~CAN_BS1_16tq
	CAN_InitStructure.CAN_BS2 = CAN_BS2_8tq;	//Tbs2=tbs2+1个时间单位CAN_BS2_1tq ~CAN_BS2_8tq
	CAN_InitStructure.CAN_Prescaler = 2;    	//分频系数   波特率为 36/(4*(1+9+8))=500k  36/(2*(1+9+8))=1M
	CAN_Init(CAN1, &CAN_InitStructure);
	
	CAN_FilterInitStructure.CAN_FilterNumber = 0;	  					
	CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask; 
	CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit; 	
	CAN_FilterInitStructure.CAN_FilterIdHigh = 0x0000;	//32位ID
	CAN_FilterInitStructure.CAN_FilterIdLow = 0x0000;
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x0000;	//32位MASK
	CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0000;
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_Filter_FIFO0;	//过滤器0关联到FIFO0
	CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;	//激活过滤器0
	CAN_FilterInit(&CAN_FilterInitStructure);
	
	CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE); //FIFO0 消息挂号中断允许.
}
/******************************************************************************/
uint8_t Can1_SendMsg(uint8_t *p,uint8_t len,uint32_t id)
{
	uint32_t i;
	uint8_t mbox;
	CanTxMsg TxMessage;
	
	TxMessage.StdId=id;           //设备ID
	TxMessage.ExtId=0;            //扩展帧ID，此处没有使用
  TxMessage.IDE=CAN_ID_STD;     //设定消息标识符的类型
  TxMessage.RTR=CAN_RTR_DATA;   //设定待传输消息的帧类型
  TxMessage.DLC = len;          //设定待传输消息的帧长度
	
	for(i=0;i<len;i++)
	{TxMessage.Data[i]=p[i];}
	
  mbox = CAN_Transmit(CAN1,&TxMessage);  //开始一个消息的传输
  i = 0;
  while((CAN_TransmitStatus(CAN1, mbox) != CAN_TxStatus_Ok) && (i < 0x7FFF))i++;		//等待发送完成
  if(i >= 0x7FFF)return 1;
	return 0;
}
/******************************************************************************/
void USB_LP_CAN1_RX0_IRQHandler(void)
{
	CAN_Receive(CAN1,CAN_FIFO0, &can_rx);
	CANcommander_run();
}
/******************************************************************************/




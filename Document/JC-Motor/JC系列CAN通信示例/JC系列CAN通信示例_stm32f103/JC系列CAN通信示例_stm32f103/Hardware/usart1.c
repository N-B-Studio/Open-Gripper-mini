
#include "usart1.h"


/******************************************************************************/
uint8_t rcv1_cntr;
uint8_t rcv1_flag;
uint8_t rcv1_buff[USART1_BUFF_SIZE];
uint8_t snd1_buff[USART1_BUFF_SIZE];
/******************************************************************************/
//加入以下代码，支持printf函数，而不需要选择MicroLIB
#if 1
#pragma import(__use_no_semihosting)
//标准库需要的支持函数
struct __FILE 
{ 
	int handle; 

}; 

FILE __stdout;
//定义_sys_exit避免使用半主机模式
void _sys_exit(int x) 
{ 
	x = x; 
} 
//重定义fputc函数
int fputc(int ch, FILE *f)
{      
	while((USART1->SR&0X40)==0);
    USART1->DR = (u8) ch;      
	return ch;
}
#endif 
/******************************************************************************/
void USART1_Init(uint32_t baud)
{
  GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitSructure;
	USART_InitTypeDef USART_InitStructure;
	DMA_InitTypeDef DMA_Struct;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1|RCC_APB2Periph_GPIOA, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1,ENABLE);      //DMA1
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;         //PA9是串口1发送
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;   //复用推挽输出	
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; //速度
	GPIO_Init(GPIOA, &GPIO_InitStructure);            //对选中管脚初始化
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;        //PA10是串口1接收
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOA,&GPIO_InitStructure); 
	
	NVIC_InitSructure.NVIC_IRQChannel = USART1_IRQn;//开放串口1中断
	NVIC_InitSructure.NVIC_IRQChannelCmd = ENABLE;  //使能指定通道
	NVIC_Init(&NVIC_InitSructure);
	
	USART_DeInit(USART1);//复位串口1寄存器
  USART_InitStructure.USART_BaudRate = baud;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;	//数据位8位
  USART_InitStructure.USART_StopBits = USART_StopBits_1;			//停止位1位
  USART_InitStructure.USART_Parity = USART_Parity_No;				  //无校验位
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;   //无硬件流控
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
  USART_Init(USART1, &USART_InitStructure);                   //配置串口参数 
	USART_ClearFlag(USART1, USART_FLAG_TC );                    //清除发送完成标志位
  USART_Cmd(USART1, ENABLE);                                  //使能串口
	USART_ITConfig(USART1,USART_IT_IDLE,ENABLE);	              //使能空闲中断
	
	//DMA_Rec_Init
	DMA_DeInit(DMA1_Channel5);  
	DMA_Struct.DMA_PeripheralBaseAddr = (uint32_t)&USART1->DR;   //DMA外设基地址
	DMA_Struct.DMA_MemoryBaseAddr = (uint32_t)rcv1_buff;      //DMA内存基地址
	DMA_Struct.DMA_DIR = DMA_DIR_PeripheralSRC;  		        //数据传输方向，从外设到内存
	DMA_Struct.DMA_BufferSize = USART1_BUFF_SIZE;  	   	    //DMA通道的DMA缓存的大小
	DMA_Struct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;  //外设地址寄存器不变
	DMA_Struct.DMA_MemoryInc = DMA_MemoryInc_Enable;  		  //内存地址寄存器递增
	DMA_Struct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;  //数据宽度为8位
	DMA_Struct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;//数据宽度为8位
	DMA_Struct.DMA_Mode = DMA_Mode_Circular;  				      //工作在循环缓存模式
	DMA_Struct.DMA_Priority = DMA_Priority_High; 		  //DMA通道 x拥有高优先级 
	DMA_Struct.DMA_M2M = DMA_M2M_Disable;  					  //DMA通道x没有设置为内存到内存传输
	DMA_Init(DMA1_Channel5, &DMA_Struct);  
	DMA_Cmd(DMA1_Channel5,ENABLE);					//DMA使能
	USART_DMACmd(USART1,USART_DMAReq_Rx,ENABLE);	//串口接收DMA使能
}
/******************************************************************************/
void USART1_IRQHandler(void)//串口1
{
	rcv1_cntr=USART1->SR;
	rcv1_cntr=USART1->DR;     //清除中断标志
	rcv1_cntr=USART1_BUFF_SIZE-DMA_GetCurrDataCounter(DMA1_Channel5);  //读取接收字节个数
	DMA_Cmd(DMA1_Channel5,DISABLE);         //关闭DMA
	DMA1_Channel5->CNDTR=USART1_BUFF_SIZE;   //重新设置DMA
	DMA_Cmd(DMA1_Channel5,ENABLE);          //开启DMA
	rcv1_flag=1;
}
/******************************************************************************/
void USART1_send(uint8_t *p,uint8_t num)
{
	while(num--)
	{
		USART_SendData(USART1,*p++);
		while(USART_GetFlagStatus(USART1,USART_FLAG_TXE)==RESET);	//判断是否发送完成
	}
}
/******************************************************************************/
void USART1_one(uint8_t dat)
{
	USART_SendData(USART1,dat);
	while(USART_GetFlagStatus(USART1,USART_FLAG_TXE)==RESET);	//判断是否发送完成
}
/******************************************************************************/






#include "MyProject.h"


/*******************************************************************************
代码演示 CAN通信控制JC2804
创建日期：20251010
作    者：loop222 @郑州
公    司：郑州俱瓷电子科技有限公司
淘宝店铺：俱瓷科技
=================================================
PA9为单片机的TX1，接USB串口的RX,
PA10为单片机的RX1，接USB串口的TX,
PA11为单片机的CAN1RX，PA12为单片机的CAN1TX，接TTL转can模块后接电机,
PA6为单片机的TIM3_1，捕获接收机的PWM信号
PA7为单片机的TIM3_2，捕获接收机的PWM信号
=================================================
JC2804按照默认配置，位置梯形轨迹，校准并保存，上电为空闲状态
代码上电后先发送闭环指令，电机进入闭环状态，
串口发送“A60”设置JC2804的绝对位置60°
*******************************************************************************/
#define LED_blink    GPIOC->ODR^=(1<<13)
/******************************************************************************/
void usart1_order(void);
/******************************************************************************/
void GPIO_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);//使能GPIO
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;         //PC13是LED
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;   //推挽输出	
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;  //速度
	GPIO_Init(GPIOC, &GPIO_InitStructure);             //对选中管脚初始化
	GPIO_ResetBits(GPIOC,GPIO_Pin_13);                 //上电点亮LED
}
/******************************************************************************/
/******************************************************************************/
int main(void)
{
	GPIO_Config();
	USART1_Init(115200);    //串口1接电脑的串口调试助手，收发指令
	CAN1_Init();            //波特率1M
	TIM4_1ms_Init();        //interrupt per 1ms
	delay_ms(500);
	delay_ms(500);          //上电后等待1秒钟，等JC2804稳定
	printf("CAN demo.\r\n");
	can_closeloop();        //发送进入闭环指令。收到回复，打印位置、速度、电流
	
	while(1)
	{
		if(uwTick>=500)   //0.5s
		{
			uwTick=0;
			LED_blink;
		}
		
		usart1_order();
	}
}
/******************************************************************************/
/******************************************************************************/
//PA9为单片机的TX1接USB串口的RX; PA10为单片机的RX1接USB串口的TX,
void usart1_order(void)
{
	float target;
	
	if(rcv1_flag==0)return; //没有收到数据，退出
	rcv1_flag=0;
	
	switch(rcv1_buff[0])
	{
		case 'H': {
			printf("Hello World!\r\n");
		} break;
		case 'R': {  //读取电压
			can_readVoltage();  //等电机返回数据时打印电压值
		} break;
		case 'A': {  //A360  设置绝对位置
			target=atof((const char *)(rcv1_buff+1));
			can_SetAbsAng(target*100);  //位置放大100倍
			//printf("Abs=%.2f\r\n", target);
		} break;
		case 'B': {  //B90  设置相对位置
			target=atof((const char *)(rcv1_buff+1));
			can_SetRelAng(target*100);  //位置放大100倍
			//printf("Rel=%.2f\r\n", target);
		} break;
		case 'P': {  //P180 设置PVT指令
			target=atof((const char *)(rcv1_buff+1));
			can_PVTcommand(target*100, 120, 80);  //位置,速度rpm,力矩%
		} break;
	}
	memset(rcv1_buff,0,16);  //清空接收buff
}
/******************************************************************************/




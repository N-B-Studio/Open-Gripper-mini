
#include "CAN_com.h"


//CAN通信的详细说明请看《JC系列CAN通信协议》
/******************************************************************************/
uint8_t  can_sndbuff[8];
/******************************************************************************/
//进入闭环：2B 00 A2 00 00 01 00 00
void can_closeloop(void)
{
	can_sndbuff[0]=0x2B;  //写入2个字节
	can_sndbuff[1]=0x00;
	can_sndbuff[2]=0xA2;
	can_sndbuff[3]=0x00;
	can_sndbuff[4]=0x00;
	can_sndbuff[5]=0x01;
	can_sndbuff[6]=0;
	can_sndbuff[7]=0;
	Can1_SendMsg(can_sndbuff,8,0x600+1);  //1号设备
}
/******************************************************************************/
//读取电压：4B 00 04 00 00 00 00 00
void can_readVoltage(void)
{
	can_sndbuff[0]=0x4b;  //读取2个字节
	can_sndbuff[1]=0x00;  //寄存器H
	can_sndbuff[2]=0x04;  //寄存器L
	can_sndbuff[3]=0;
	can_sndbuff[4]=0;
	can_sndbuff[5]=0;
	can_sndbuff[6]=0;
	can_sndbuff[7]=0;
	Can1_SendMsg(can_sndbuff,8,0x600+1);
}
/******************************************************************************/
//绝对位置0°：23 00 23 00 00 00 00 00
void can_SetAbsAng(int32_t ang)  //ang已放大100倍
{
	can_sndbuff[0]=0x23;  //写入4个字节
	can_sndbuff[1]=0x00;
	can_sndbuff[2]=0x23;  //绝对位置
	can_sndbuff[3]=0x00;
	can_sndbuff[4]=ang>>24;  //位置H
	can_sndbuff[5]=ang>>16;  //位置M
	can_sndbuff[6]=ang>>8;   //位置M
	can_sndbuff[7]=ang;      //位置L
	Can1_SendMsg(can_sndbuff,8,0x600+1);
}
/******************************************************************************/
//相对位置360°：23 00 25 00 00 00 8C A0
void can_SetRelAng(int32_t ang)
{
	can_sndbuff[0]=0x23;  //写入4个字节
	can_sndbuff[1]=0x00;
	can_sndbuff[2]=0x25;  //相对位置
	can_sndbuff[3]=0x00;
	can_sndbuff[4]=ang>>24;  //位置H
	can_sndbuff[5]=ang>>16;  //位置M
	can_sndbuff[6]=ang>>8;   //位置M
	can_sndbuff[7]=ang;      //位置L
	Can1_SendMsg(can_sndbuff,8,0x600+1);
}
/******************************************************************************/
//PVT指令：25 00 00 8C A0 00 3C 50
void can_PVTcommand(int32_t ang,uint16_t vel, uint8_t per)
{
	can_sndbuff[0]=0x25;     //PVT指令
	can_sndbuff[1]=ang>>24;  //位置H
	can_sndbuff[2]=ang>>16;  //位置M
	can_sndbuff[3]=ang>>8;   //位置M
	can_sndbuff[4]=ang;      //位置L
	can_sndbuff[5]=vel>>8;   //速度H
	can_sndbuff[6]=vel;      //速度L
	can_sndbuff[7]=per;      //力矩%
	Can1_SendMsg(can_sndbuff,8,0x600+1);
}
/******************************************************************************/
int32_t  JCpos;   //位置
int16_t  JCvel;   //速度
int16_t  JCcur;   //电流

void CANcommander_run(void)  //CAN通信接收数据处理, 在can接收中断中处理
{
	uint16_t val;
	int16_t  m;
	
	if(can_rx.StdId==0x581)
	{
		switch(can_rx.Data[0])
		{
			case 0x4B: {  //读取2个字节
				if(can_rx.Data[2]==0x04) { //读取电压
					val = (can_rx.Data[4]<<8) + can_rx.Data[5];
					printf("voltage=%.1f\r\n", (float)val/10);
				}
			} break;
			case 0x2A: {  //回传数据
				m=(int8_t)can_rx.Data[1];   //用3个字节表示正负位置，高字节需要转换下
				JCpos = (m<<16) + (can_rx.Data[2]<<8) + can_rx.Data[3];  //位置
				JCvel = (can_rx.Data[4]<<8) + can_rx.Data[5];            //速度
				JCcur = (can_rx.Data[6]<<8) + can_rx.Data[7];            //电流
				printf("pos=%.2f,vel=%.2f,cur=%.2f\r\n", (float)JCpos/100,(float)JCvel/100,(float)JCcur/100);
			} break;
			case 0x80: {  //通信错误
			} break;
		}
	}
}
/*****************************************************************************/


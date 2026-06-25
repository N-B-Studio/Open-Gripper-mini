
#ifndef STM32_USART1_H
#define STM32_USART1_H
/******************************************************************************/
#include "stdio.h"
#include "stm32f10x.h"

/******************************************************************************/
#define USART1_BUFF_SIZE 64
/******************************************************************************/
extern  uint8_t rcv1_cntr;
extern  uint8_t rcv1_flag;
extern  uint8_t rcv1_buff[USART1_BUFF_SIZE];
extern  uint8_t snd1_buff[USART1_BUFF_SIZE];
/******************************************************************************/
void USART1_Init(uint32_t baud);
void USART1_send(uint8_t *p,uint8_t num);
/******************************************************************************/


#endif

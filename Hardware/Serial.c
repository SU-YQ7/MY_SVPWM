#include "stm32f4xx.h"
#include <stdio.h>


void Serial_Init(void)
{
	USART_InitTypeDef usart_structure;
	GPIO_InitTypeDef gpio_structure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);
	
	gpio_structure.GPIO_Mode=GPIO_Mode_AF;
	gpio_structure.GPIO_OType=GPIO_OType_PP;
	gpio_structure.GPIO_Pin=GPIO_Pin_9|GPIO_Pin_10;
	gpio_structure.GPIO_PuPd=GPIO_PuPd_UP;
	gpio_structure.GPIO_Speed=GPIO_Fast_Speed;
	
	GPIO_Init(GPIOA,&gpio_structure);
	
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1);
	
	usart_structure.USART_BaudRate=115200;
	usart_structure.USART_HardwareFlowControl=USART_HardwareFlowControl_None;
	usart_structure.USART_Mode=USART_Mode_Rx|USART_Mode_Tx;
	usart_structure.USART_Parity=USART_Parity_No;
	usart_structure.USART_StopBits=USART_StopBits_1;
	usart_structure.USART_WordLength=USART_WordLength_8b;
	
	USART_Init(USART1,&usart_structure);
	
	USART_Cmd(USART1,ENABLE);
}


int fputc(int ch, FILE *f)
{
    USART_SendData(USART1, (uint8_t)ch);
    while(RESET == USART_GetFlagStatus(USART1, USART_FLAG_TXE));

    return ch;
}




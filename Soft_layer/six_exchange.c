#include "stm32f4xx.h"
#include "delay.h"
#include "TIM.h"
extern uint32_t Val; 
void Shut_Down(void)
{
     	Delay_ms(50);
		GPIO_ResetBits(GPIOE,GPIO_Pin_8);
		GPIO_ResetBits(GPIOE,GPIO_Pin_10);
		GPIO_ResetBits(GPIOE,GPIO_Pin_12);
}
void  GPIO_SIX_Exchange(void)
{
	GPIO_SetBits(GPIOE,GPIO_Pin_8);//U_L
	GPIO_SetBits(GPIOE,GPIO_Pin_11);//V_H
    Shut_Down();
	GPIO_SetBits(GPIOE,GPIO_Pin_11);//V_H
    GPIO_SetBits(GPIOE,GPIO_Pin_12);//W_L
    Shut_Down();
	GPIO_SetBits(GPIOE,GPIO_Pin_9);//U_H
    GPIO_SetBits(GPIOE,GPIO_Pin_12);//W_L
    Shut_Down();
	GPIO_SetBits(GPIOE,GPIO_Pin_9);//U_H
    GPIO_SetBits(GPIOE,GPIO_Pin_10);//V_L
    Shut_Down();
	GPIO_SetBits(GPIOE,GPIO_Pin_10);//V_L
    GPIO_SetBits(GPIOE,GPIO_Pin_13);//W_H
    Shut_Down();
	GPIO_SetBits(GPIOE,GPIO_Pin_8);//U_L
    GPIO_SetBits(GPIOE,GPIO_Pin_13);//W_H
    Shut_Down();
}



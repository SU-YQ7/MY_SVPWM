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
void Houer_TIM(int i)
{
	  if(Val==0x4)
	  {

       TIM_SetCompare1(TIM1,0);
       TIM_SetCompare2(TIM1,0);
	   TIM_SetCompare3(TIM1,i);

	    GPIO_ResetBits(GPIOE,GPIO_Pin_8);//U_L
        GPIO_SetBits(GPIOE,GPIO_Pin_10);//V_L
		GPIO_ResetBits(GPIOE,GPIO_Pin_12);
	  }
	   if(Val==0x6)
	  {


	 	TIM_SetCompare1(TIM1,i);
        TIM_SetCompare2(TIM1,0);
	    TIM_SetCompare3(TIM1,0);
		  
		  
		GPIO_ResetBits(GPIOE,GPIO_Pin_8);
	    GPIO_SetBits(GPIOE,GPIO_Pin_10);//V_L
		GPIO_ResetBits(GPIOE,GPIO_Pin_12);
		  
	  }
	   if(Val==0x2) 
	  {


		  TIM_SetCompare1(TIM1,i);
          TIM_SetCompare2(TIM1,0);
	      TIM_SetCompare3(TIM1,0);
       
		  GPIO_ResetBits(GPIOE,GPIO_Pin_8);
		  GPIO_ResetBits(GPIOE,GPIO_Pin_10);
	      GPIO_SetBits(GPIOE,GPIO_Pin_12);
	  }
	  
	   if(Val==0x3)
	  {


	   TIM_SetCompare1(TIM1,0);
       TIM_SetCompare2(TIM1,i);
	   TIM_SetCompare3(TIM1,0);
		
		GPIO_ResetBits(GPIOE,GPIO_Pin_8);
		GPIO_ResetBits(GPIOE,GPIO_Pin_10);
	    GPIO_SetBits(GPIOE,GPIO_Pin_12);

		  
	  }
	   if(Val==0x1)
	  {


	   TIM_SetCompare1(TIM1,0);
       TIM_SetCompare2(TIM1,i);
	   TIM_SetCompare3(TIM1,0);
		  
	
       GPIO_SetBits(GPIOE,GPIO_Pin_8);
	   GPIO_ResetBits(GPIOE,GPIO_Pin_10);
	   GPIO_ResetBits(GPIOE,GPIO_Pin_12);
		  
	  }
	   if(Val==0x5) 
	  {

       TIM_SetCompare1(TIM1,0);
       TIM_SetCompare2(TIM1,0);
	   TIM_SetCompare3(TIM1,i);
		  
		GPIO_SetBits(GPIOE,GPIO_Pin_8);
		GPIO_ResetBits(GPIOE,GPIO_Pin_10);
		GPIO_ResetBits(GPIOE,GPIO_Pin_12);
		  
	  }
	
	
	
	
	
}



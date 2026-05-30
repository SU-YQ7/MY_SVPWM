#include "stm32f4xx.h"
#include "six_exchange.h"
#include "ADC.h"
uint32_t Val = 0;
extern uint16_t cnt;
void GPIO_Input(void)
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	GPIO_InitTypeDef  GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
	 RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);

	GPIO_InitTypeDef  GPIOF_InitStructure;
    GPIOF_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIOF_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIOF_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOF, &GPIOF_InitStructure);
	GPIOF_InitStructure.GPIO_Pin = GPIO_Pin_7;
    GPIOF_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_Init(GPIOF, &GPIOF_InitStructure);
	
}

void TIM_Config(void)
{

	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef  TIM_OCInitStructure;
   RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
   GPIO_InitTypeDef  GPIO_InitStructure;

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9|GPIO_Pin_11|GPIO_Pin_13;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOE, &GPIO_InitStructure);

	
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource9, GPIO_AF_TIM1);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource11, GPIO_AF_TIM1);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource13, GPIO_AF_TIM1);
      RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
    TIM_TimeBaseStructure.TIM_Period = 100; 
    TIM_TimeBaseStructure.TIM_Prescaler = 100; 
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);


    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
    TIM_OCInitStructure.TIM_Pulse = 0; // ռ�ձ�50%��Period/2��
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_Low;
    TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;
    TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;
    TIM_OC2Init(TIM1, &TIM_OCInitStructure);
    TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
	
	
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
    TIM_OCInitStructure.TIM_Pulse = 0; // ռ�ձ�50%��Period/2��
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_Low;
    TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;
    TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;
    TIM_OC1Init(TIM1, &TIM_OCInitStructure);
    TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
	
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
    TIM_OCInitStructure.TIM_Pulse = 0; // ռ�ձ�50%��Period/2��
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
    TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;
    TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;
    TIM_OC3Init(TIM1, &TIM_OCInitStructure);
    TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);
	
//	TIM_ClearFlag(TIM1, TIM_FLAG_Update);					
				
//    	TIM_BDTRInitTypeDef TIM_BDTRInitStructure;
//   TIM_BDTRInitStructure.TIM_OSSRState = TIM_OSSRState_Enable;
//   TIM_BDTRInitStructure.TIM_OSSIState = TIM_OSSIState_Enable;
//   TIM_BDTRInitStructure.TIM_LOCKLevel = TIM_LOCKLevel_OFF;
//   TIM_BDTRInitStructure.TIM_DeadTime = 5;
//   TIM_BDTRInitStructure.TIM_Break = TIM_Break_Disable;
//   TIM_BDTRInitStructure.TIM_BreakPolarity = TIM_BreakPolarity_High;
//   TIM_BDTRInitStructure.TIM_AutomaticOutput = TIM_AutomaticOutput_Disable;

//    TIM_BDTRConfig(TIM1, &TIM_BDTRInitStructure);
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);		
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_TIM10_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);	
    TIM_ARRPreloadConfig(TIM1, ENABLE);
    TIM_CtrlPWMOutputs(TIM1, ENABLE);

   TIM_Cmd(TIM1, ENABLE);
}
uint32_t hallsensor_state(void)
{
    __IO static uint32_t state ;
    state  = 0;

        if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_5) != RESET)  /* ����������״̬��ȡU */
        {
            state |= 0x01U;
        }
        if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_4) != RESET)  /* ����������״̬��ȡV */
        {
            state |= 0x02U;
        }
        if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_3) != RESET)  /* ����������״̬��ȡW */
        {
            state |= 0x04U;
        }
    
    return state;
}

void GPIO_Output(void)
{

   RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
   GPIO_InitTypeDef  GPIO_InitStructure;

   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_10|GPIO_Pin_12;
   // GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_10|GPIO_Pin_12|GPIO_Pin_9|GPIO_Pin_11|GPIO_Pin_13;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOE, &GPIO_InitStructure);
	
}
void TIM1_UP_TIM10_IRQHandler()
{
		if (TIM_GetITStatus(TIM1, TIM_IT_Update)!= RESET)	
	{

		Val=hallsensor_state();
        uint16_t j=((double)cnt/65535.0)*100.0;
        Houer_TIM(j);

		TIM_ClearITPendingBit(TIM1, TIM_IT_Update);			
																													
	}
	
}


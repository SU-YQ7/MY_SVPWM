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
   
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_10|GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOE, &GPIO_InitStructure);
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource8, GPIO_AF_TIM1);
  	GPIO_PinAFConfig(GPIOE, GPIO_PinSource10, GPIO_AF_TIM1);
  	GPIO_PinAFConfig(GPIOE, GPIO_PinSource12, GPIO_AF_TIM1);
      RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);              // 使能 TIM1 时钟（挂在 APB2 总线上）
    TIM_TimeBaseStructure.TIM_Period = 6000;                           // 自动重装值 ARR=5999+1，即计数顶点(占空比满量程基准)
    TIM_TimeBaseStructure.TIM_Prescaler = 10-1;                          // 预分频 PSC=9，计数时钟 = TIM1时钟/10
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;                         // 时钟分频(用于死区/输入滤波采样)，此处不分频
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_CenterAligned1; // 中央对齐模式(三角波计数 0→ARR→0)，SVPWM 标准载波
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 1;                     // 重复计数=1，吞掉一次更新事件，使每个三角波周期只触发一次 UP 中断
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);                      // 写入时基配置
    // 载波频率(设 TIM1时钟=168MHz)：f = 168e6 / (2*(PSC+1)*(ARR+1)) = 168e6/(2*10*6000) ≈ 1.4kHz


    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;                  // PWM模式：CNT>CCR 时为有效电平
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;      // 使能主通道 OCx 输出
    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;   // 互补通道 OCxN 输出
    TIM_OCInitStructure.TIM_Pulse = 10; //                             // 比较值 CCR(初始占空比 = CCR/ARR)
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;          // 主通道有效电平为高
    TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;         // 互补通道有效电平为高
    TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;       // MOE=0(刹车/空闲)时主通道输出低
    TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;     // MOE=0 时互补通道输出低
    TIM_OC2Init(TIM1, &TIM_OCInitStructure);                          // 应用到 CH2 (PE11)
    TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);                 // CCR2 预装载使能，新占空比在更新事件时生效


	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;                  // PWM模式：CNT>CCR 时为有效电平
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;      // 使能主通道 OCx 输出
    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;   // 互补通道 OCxN 输出
    TIM_OCInitStructure.TIM_Pulse = 50; //                            // 比较值 CCR(初始占空比 = CCR/ARR)
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;          // 主通道有效电平为高
    TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;         // 互补通道有效电平为高
    TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;       // MOE=0 时主通道输出低
    TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;     // MOE=0 时互补通道输出低
    TIM_OC1Init(TIM1, &TIM_OCInitStructure);                          // 应用到 CH1 (PE9)
    TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);                 // CCR1 预装载使能，新占空比在更新事件时生效

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;                  // PWM模式：CNT>CCR 时为有效电平
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;      // 使能主通道 OCx 输出
    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;   // 互补通道 OCxN 输出
    TIM_OCInitStructure.TIM_Pulse = 3000; //                          // 比较值 CCR(初始占空比 = CCR/ARR)
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;          // 主通道有效电平为高
    TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;         // 互补通道有效电平为高
    TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;       // MOE=0 时主通道输出低
    TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;     // MOE=0 时互补通道输出低
    TIM_OC3Init(TIM1, &TIM_OCInitStructure);                          // 应用到 CH3 (PE13)
    TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);                 // CCR3 预装载使能，新占空比在更新事件时生效

	TIM_ClearFlag(TIM1, TIM_FLAG_Update);                            // 清除更新标志，避免初始化后立即误触发中断
				
   TIM_BDTRInitTypeDef TIM_BDTRInitStructure;                          // 刹车与死区(BDTR)配置，高级定时器专用
   TIM_BDTRInitStructure.TIM_OSSRState = TIM_OSSRState_Enable;          // 运行模式下关闭输出时仍由定时器接管引脚电平
   TIM_BDTRInitStructure.TIM_OSSIState = TIM_OSSIState_Enable;          // 空闲模式下关闭输出时仍由定时器接管引脚电平
   TIM_BDTRInitStructure.TIM_LOCKLevel = TIM_LOCKLevel_OFF;             // 不锁定寄存器，允许后续修改
   TIM_BDTRInitStructure.TIM_DeadTime = 5;                             // 死区时间，防止上下桥臂同时导通直通
   TIM_BDTRInitStructure.TIM_Break = TIM_Break_Disable;                // 关闭刹车输入功能
   TIM_BDTRInitStructure.TIM_BreakPolarity = TIM_BreakPolarity_High;   // 刹车输入高电平有效(此处未用)
   TIM_BDTRInitStructure.TIM_AutomaticOutput = TIM_AutomaticOutput_Disable; // 关闭自动输出，需软件置 MOE 才使能输出

    TIM_BDTRConfig(TIM1, &TIM_BDTRInitStructure);                       // 写入 BDTR 配置
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);                     // 中断优先级分组2(2位抢占+2位子优先级)
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_TIM10_IRQn;            // TIM1 更新中断通道
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;          // 抢占优先级 0(最高)
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;                 // 子优先级 0
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;                    // 使能该中断通道
    NVIC_Init(&NVIC_InitStructure);                                   // 写入 NVIC 配置
    TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);                        // 使能更新中断(每三角波周期触发一次)
    TIM_ARRPreloadConfig(TIM1, ENABLE);                               // ARR 预装载使能，周期值在更新事件时生效
    TIM_CtrlPWMOutputs(TIM1, ENABLE);                                 // 总输出使能(置 MOE)，PWM 引脚开始输出

   TIM_Cmd(TIM1, ENABLE);                                            // 启动 TIM1 计数器
}
uint32_t hallsensor_state(void)
{
    __IO static uint32_t state ;
    state  = 0;

        if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_5) != RESET)  
        {
            state |= 0x01U;
        }
        if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_4) != RESET) 
        {
            state |= 0x02U;
        }
        if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_3) != RESET)  
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

//		    Val=hallsensor_state();
//        uint16_t j=((double)cnt/65535.0)*100.0;
//        Houer_TIM(j);

		TIM_ClearITPendingBit(TIM1, TIM_IT_Update);			
																													
	}
	
}


#include "tim2_adc1_ch11.h"
#include "my_svpwm.h"
#include "i2c.h"

uint16_t spd_time=10;
uint16_t ADC_value;
extern uint16_t q16_m_value;
extern uint16_t us_m;
extern uint16_t us_angle;

uint16_t mech_angle7;
uint16_t elec_angle7;
void m_tick(void)
{
	if(spd_time != 0) 		spd_time--;

}
void TIM2_ADC1_CH11_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    ADC_CommonInitTypeDef ADC_CommonInitStructure;
    ADC_InitTypeDef ADC_InitStructure;


    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    /* TIM2 时钟 = 84MHz；84 分频 -> 1MHz(1us/计数)，计满 50 -> 每 50us 触发一次更新事件 */
    TIM_TimeBaseInitStructure.TIM_Prescaler = 84 - 1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_Period = 50 - 1;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStructure);
    TIM_SelectOutputTrigger(TIM2, TIM_TRGOSource_Update);
    TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div4;
    ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
    ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
    ADC_CommonInit(&ADC_CommonInitStructure);

    ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
    ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_Rising;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T2_TRGO;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfConversion = 1;
    ADC_Init(ADC1, &ADC_InitStructure);

    ADC_RegularChannelConfig(ADC1, ADC_Channel_11, 1, ADC_SampleTime_15Cycles);

    ADC_Cmd(ADC1, ENABLE);
    TIM_Cmd(TIM2, ENABLE);
}

void TIM2_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
    {
         ADC_value = ADC_GetConversionValue(ADC1);
				   m_us_radius_calculate();
				   us_m=q16_m_value;
				   mech_angle7 = (uint16_t)(AS5600_ReadRawAngleTwo() * 0.08789f);  //机械角度 0~359
				   elec_angle7 = (mech_angle7 * 7) % 360;                      //电角度 = 机械角度 * 极对数
				   us_angle = (elec_angle7 + 270) % 360;
				   m_svpwm_generate(us_m, us_angle);
         m_tick();
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
    }
}

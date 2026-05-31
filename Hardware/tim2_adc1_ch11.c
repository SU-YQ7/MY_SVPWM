#include "tim2_adc1_ch11.h"
#include "my_svpwm.h"
uint16_t tim2_adc1_ch11_value;
uint16_t spd_time=10;
uint16_t flag=0;
 uint16_t ADC_value;
 uint16_t cnt=0;
extern uint16_t q16_m_value;
extern uint16_t us_m;
extern uint16_t us_angle;
uint16_t state;
extern uint16_t start_sign;

/*=================== 开环旋转测试 ===================
  调试开关：=1 时不使用霍尔反馈，角度匀速递增强制旋转磁场。
  用途：分离故障范围——
    开环能平滑转 → SVPWM算法/功率级/三相接线 OK，抖动是霍尔反馈问题；
    开环仍抖/不转 → 问题在 SVPWM 数学或三相相序。
  排查完后改回 0 即可恢复霍尔闭环。*/
uint16_t openloop_test   = 0;      //0=霍尔闭环(正常运行)，1=开环旋转测试(标定/排障)
uint16_t openloop_angle  = 0;      //开环电角度累加 0~359
uint16_t openloop_div    = 11;     //每多少个50us tick 角度+1°，越大转越慢
uint16_t openloop_um     = 30000;  //固定幅值 Q16(≈0.46)，太小转不动则调大，过流则调小
static uint16_t ol_cnt   = 0;

/*=================== 霍尔标定采集(开环时用) ===================
  开环旋转时记录"霍尔状态→电角度"映射与跳变顺序。
  转一圈后用调试器看下面两个数组，即可重建 ROTOR_ANGLE_TABLE_CW：
    hall_cal_angle[1..6] = 各霍尔状态进入时的开环电角度
    hall_cal_seq[]       = 霍尔状态的实际跳变顺序(判断正转方向)
  详见对话里的填表说明。*/
uint16_t hall_cal_angle[8] = {0};
uint16_t hall_cal_seq[16]  = {0};
uint16_t hall_cal_idx      = 0;
static uint16_t hall_cal_prev = 0xFF;
extern uint16_t hallsensor_get_state(void);
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

//uint16_t TIM2_ADC1_CH11_GetValue(void)
//{
//    while (ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET)
//     {
//     }
//     tim2_adc1_ch11_value = ADC_GetConversionValue(ADC1);
//     ADC_ClearFlag(ADC1, ADC_FLAG_EOC);
//		 flag=0;
//    return tim2_adc1_ch11_value;
//}

void TIM2_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
    {
//            ADC_value = ADC_GetConversionValue(ADC1);
//            m_us_radius_calculate();
//		        cnt=q16_m_value;
//	          state = m_rotor_angle_calculate();
//		  	    m_tick();
         ADC_value = ADC_GetConversionValue(ADC1);
			   m_us_radius_calculate();
			   if(openloop_test)
			   {
			       /*开环：角度匀速递增，不依赖霍尔，固定幅值强制旋转磁场*/
			       if(++ol_cnt >= openloop_div)
			       {
			           ol_cnt = 0;
			           if(++openloop_angle >= 360) openloop_angle = 0;
			       }
			       us_m     = openloop_um;
			       us_angle = openloop_angle;

			       /*霍尔标定采集：记录每个霍尔状态进入时的开环电角度与跳变顺序*/
			       {
			           uint16_t h = hallsensor_get_state();
			           if(h >= 1 && h <= 6 && h != hall_cal_prev)
			           {
			               hall_cal_angle[h] = openloop_angle;       //该霍尔状态对应的电角度
			               if(hall_cal_idx < 16)
			                   hall_cal_seq[hall_cal_idx++] = h;     //记录跳变顺序
			               hall_cal_prev = h;
			           }
			       }
			   }
			   else
			   {
			       /*闭环：霍尔反馈*/
			       us_m=q16_m_value;
			       us_angle= m_rotor_angle_calculate();          //转子电角度 0~359
			       us_angle = (us_angle + 90) % 360;             //电压矢量超前转子90°，产生正转转矩(若反转/抖则改 +270)
			   }
			   m_svpwm_generate(us_m, us_angle);
               m_tick();
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
    }
}

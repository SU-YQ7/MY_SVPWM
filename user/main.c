#include "stm32f4xx.h"
#include "TIM.h"
#include "delay.h"
#include "six_exchange.h"
#include "my_svpwm.h"
#include "Serial.h"
#include "tim2_adc1_ch11.h"
extern uint16_t state;
extern uint16_t flag;
extern  uint16_t ADC_value;
extern uint16_t q16_m_value;

int main(void)
{
	GPIO_Input();//霍尔值输入配置
	Delay_Init();
  GPIO_Output();//下管输出
  TIM_Config();//上管定时器配置
  Serial_Init();
	TIM2_ADC1_CH11_Init();
	while(1)
	{
      printf("%d,%d\n",mech_angle, rotor_angle);

	}
	
	
}
	

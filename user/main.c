#include "stm32f4xx.h"
#include "TIM.h"
#include "delay.h"
#include "six_exchange.h"
#include "my_svpwm.h"
#include "Serial.h"
#include "tim2_adc1_ch11.h"
uint16_t cnt=0;
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
	  cnt=TIM2_ADC1_CH11_GetValue();
      printf("ADC=%d\r\n",cnt);
	}
	
	
}
	

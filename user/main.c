#include "stm32f4xx.h"
#include "TIM.h"
#include "delay.h"
#include "six_exchange.h"
#include "my_svpwm.h"
#include "Serial.h"
#include "tim2_adc1_ch11.h"
#include "math.h"
#include "stdio.h"
#include "i2c.h"
extern uint16_t state;
extern uint16_t flag;
extern  uint16_t ADC_value;
extern uint16_t q16_m_value;

int main(void)
{
  GPIO_Output();//下管输出
  TIM_Config();//上管定时器配置
  Serial_Init();
  TIM2_ADC1_CH11_Init();
	GPIO_I2c();
	while(1)
	{
    //printf("Angle_I2C=%.4f\r\n",AS5600_ReadRawAngleTwo()*0.08789);
	}
}
	

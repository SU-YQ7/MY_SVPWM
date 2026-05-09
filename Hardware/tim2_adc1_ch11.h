#ifndef __TIM2_ADC1_CH11_H
#define __TIM2_ADC1_CH11_H

#include "stm32f4xx.h"

void TIM2_ADC1_CH11_Init(void);
uint16_t TIM2_ADC1_CH11_GetValue(void);
void TIM2_IRQHandler(void);

#endif


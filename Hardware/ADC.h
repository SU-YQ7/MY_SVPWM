#ifndef __ADC_H
#define __ADC_H
#include "stdio.h"
typedef struct __FILE FILE;

void ADC_Config(void);
void ADC1_DMA_Init(void);
void USART_Config(void);
void calc_adc_val(uint32_t * p);
void Get_Stop_CURET(void);
void Get_Start_CURET(void);
void USART_OUTPUT(void);
float get_temp(uint16_t para);

#define ADC2CURT    (float)(3.3f / 4.096f / 0.12f)      /* ADC²É¼¯Öµ * 3.3/4.096 £¨mv£© = 6 * ( 0.02*I ) */
#define ADC2VBUS    (float)(3.3f * 25 / 4096)  


#endif




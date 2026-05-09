#include "stm32f4xx.h"
#include "stdio.h"// Device header
#include "delay.h"
extern uint32_t Val ; 
#define ADC2CURT    (float)(3.3f / 4.096f / 0.12f)     
#define ADC2VBUS    (float)(3.3f * 25 / 4096)          
#define NUM_CLEAR(para,val)     {if(para >= val){para=0;}}
#define FirstOrderRC_LPF(Yn_1,Xn,a) Yn_1 = (1-a)*Yn_1 + a*Xn; 
uint32_t AD_Value[250];
uint32_t flag;
uint32_t g_adc_val[5];
uint32_t adc_amp_offset[3][51];

int16_t adc_amp_un[3];
uint32_t adc_amp_bus;

extern uint32_t Val ; 
float current[3]= {0.0f};
float current_lpf[4]= {0.0f};


const float Rp = 10000.0f;                  /* 10K */
const float T2 = (273.15f + 25.0f);         /* T2 */
const float Bx = 3380.0f;                   /* B */
const float Ka = 273.15f;


void ADC1_DMA_Init(void)
{

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_6| GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    ADC_InitTypeDef ADC_InitStructure;
    ADC_CommonInitTypeDef ADC_CommonInitStructure;
    ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;
    ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
    ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
    ADC_CommonInit(&ADC_CommonInitStructure);

    ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
    ADC_InitStructure.ADC_ScanConvMode = ENABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
    ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfConversion = 5;
    ADC_Init(ADC1, &ADC_InitStructure);

    ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_480Cycles);  // PA0
    ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 2, ADC_SampleTime_480Cycles);  // PA1
    ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 3, ADC_SampleTime_480Cycles);  // PA2
    ADC_RegularChannelConfig(ADC1, ADC_Channel_6, 4, ADC_SampleTime_480Cycles);  // PA6
    ADC_RegularChannelConfig(ADC1, ADC_Channel_7, 5, ADC_SampleTime_480Cycles);  // PA7

    DMA_InitTypeDef DMA_InitStructure;


    DMA_DeInit(DMA2_Stream0);

    DMA_InitStructure.DMA_Channel = DMA_Channel_0;
    DMA_InitStructure.DMA_PeripheralBaseAddr = ((uint32_t)&ADC1->DR);
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)AD_Value;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
    DMA_InitStructure.DMA_BufferSize = 250;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;

    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;

    DMA_Init(DMA2_Stream0, &DMA_InitStructure);
    DMA_Cmd(DMA2_Stream0, ENABLE);
	
	
	
		
	DMA_ITConfig(DMA2_Stream0, DMA_IT_TC, ENABLE);
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);  
	
	
    ADC_DMACmd(ADC1, ENABLE);
    ADC_Cmd(ADC1, ENABLE);
    ADC_SoftwareStartConv(ADC1);
}
void USART_Config(void)
{
	USART_InitTypeDef usart_structure;
	GPIO_InitTypeDef GPIO_structure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);
	
	GPIO_structure.GPIO_Mode=GPIO_Mode_AF;
	GPIO_structure.GPIO_OType=GPIO_OType_PP;
	GPIO_structure.GPIO_Pin=GPIO_Pin_9|GPIO_Pin_10;
	GPIO_structure.GPIO_PuPd=GPIO_PuPd_UP;
	GPIO_structure.GPIO_Speed=GPIO_Fast_Speed;
	
	GPIO_Init(GPIOA,&GPIO_structure);
	
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1);
	
	usart_structure.USART_BaudRate=9600;
	usart_structure.USART_HardwareFlowControl=USART_HardwareFlowControl_None;
	usart_structure.USART_Mode=USART_Mode_Rx|USART_Mode_Tx;
	usart_structure.USART_Parity=USART_Parity_No;
	usart_structure.USART_StopBits=USART_StopBits_1;
	usart_structure.USART_WordLength=USART_WordLength_8b;
	
	USART_Init(USART1,&usart_structure);
	
	USART_Cmd(USART1,ENABLE);
	
}
/* retarget the C library printf function to the USART */


void calc_adc_val(uint32_t * p)
{
    uint32_t temp[5] = {0,0,0};            
    int i,j;
    for(i = 0; i < 50; i++)             
    {
        for(j = 0; j < 5; j++)        
        {
            temp[j] += AD_Value[j+i*5];
        }
    }
    for(j = 0; j < 5; j++)
    {
        temp[j] /= 50;                       
        p[j] = temp[j];                     
    }
}
void Get_Stop_CURET(void)
{
	uint8_t i;
	uint32_t adc_amp_offset_p=0;
    uint32_t avg[3] = {0,0,0};
    adc_amp_offset[0][adc_amp_offset_p] = g_adc_val[2];     /* U */
    adc_amp_offset[1][adc_amp_offset_p] = g_adc_val[3];     /* V */
    adc_amp_offset[2][adc_amp_offset_p] = g_adc_val[4];     /* W */
    adc_amp_offset_p ++;
    NUM_CLEAR(adc_amp_offset_p,50);      
    for(i = 0; i < 50; i++)
    {
        avg[0] += adc_amp_offset[0][i];                    
        avg[1] += adc_amp_offset[1][i];
        avg[2] += adc_amp_offset[2][i];
     }
    for(i = 0; i < 3; i++)
    {
        avg[i] /= 50;                    
        adc_amp_offset[i][50] = avg[i];  
     }
	
}
void Get_Start_CURET(void)
{
	uint16_t i=0;
	int16_t adc_amp[3];
 
	int16_t adc_val_m1[3];
   
       for(i = 0; i < 3; i++)     
       {     
           adc_val_m1[i] = g_adc_val[i+2];     
           adc_amp[i] = adc_val_m1[i] - adc_amp_offset[i][50];      
           if(adc_amp[i] >= 0)                                                   
               adc_amp_un[i] = adc_amp[i];     
       }     

       if(Val == 0x04)     
       {     
           adc_amp_bus= (adc_amp_un[1] + adc_amp_un[2])*ADC2CURT;   /* UV */     
       }     
       else if(Val == 0x06)     
       {     
           adc_amp_bus= (adc_amp_un[0] + adc_amp_un[1])*ADC2CURT;   /* UW */     
       }     
       else if(Val == 0x02)     
       {     
           adc_amp_bus= (adc_amp_un[0] + adc_amp_un[2])*ADC2CURT;   /* VW */     
       }     
       else if(Val == 0x03)     
       {     
           adc_amp_bus= (adc_amp_un[1] + adc_amp_un[2])*ADC2CURT;   /* UV */     
       }     
       else if(Val== 0x01)     
       {     
           adc_amp_bus= (adc_amp_un[0] + adc_amp_un[1])*ADC2CURT;   /* WU */     
       }     
       else if(Val== 0x05)     
       {     
           adc_amp_bus= (adc_amp_un[0] + adc_amp_un[2])*ADC2CURT;   /* WV */     
       }       
	
}
float get_temp(uint16_t para)
{
    float Rt;
    float temp;
    Rt = 3.3f * 4700.0f / (para * 3.3f / 4096.0f) - 4700.0f;
    /* like this R=5000, T2=273.15+25,B=3470, RT=5000*EXP(3470*(1/T1-1/(273.15+25)) */
    temp = Rt / Rp;
    temp = log(temp);       /* ln(Rt/Rp) */
    temp /= Bx;             /* ln(Rt/Rp)/B */
    temp += (1.0f / T2);
    temp = 1.0f / (temp);
    temp -= Ka;
    return temp;
}
void USART_OUTPUT(void)
{
	
    current[0] = adc_amp_un[0]* ADC2CURT;             
    current[1] = adc_amp_un[1]* ADC2CURT;           
    current[2] = adc_amp_un[2]* ADC2CURT;             

    FirstOrderRC_LPF(current_lpf[0],current[0],0.1f);  
    FirstOrderRC_LPF(current_lpf[1],current[1],0.1f);   
    FirstOrderRC_LPF(current_lpf[2],current[2],0.1f);  
    FirstOrderRC_LPF(current_lpf[3],adc_amp_bus,0.1f); 

     if(GPIO_ReadInputDataBit(GPIOF,GPIO_Pin_6 == RESET) )
 {	
     current_lpf[0] = 0;           
     current_lpf[1] = 0;           
     current_lpf[2] = 0;           
     current_lpf[3] = 0;           
 }         
//    Delay_ms(500); 
 
            printf("Valtage:%.1fV \r\n", g_adc_val[0]*ADC2VBUS);
            printf("Temp:%.1fC \r\n", get_temp(g_adc_val[1]));
            printf("U相电流为：%.3fmA\r\n", (current_lpf[0]));
            printf("V相电流为：%.3fmA\r\n", (current_lpf[1]));
            printf("W相电流为：%.3fmA\r\n", (current_lpf[2]));
            printf("母线电流为：%.3fmA\r\n", (current_lpf[3]));
            printf("\r\n");
	
}
void DMA2_Stream0_IRQHandler(void)
{    
    if(DMA_GetITStatus(DMA2_Stream0,DMA_IT_TCIF0)!=RESET)
     {
		 flag=1;
		 DMA_Cmd(DMA2_Stream0, DISABLE);
		 calc_adc_val(g_adc_val);
		 DMA_Cmd(DMA2_Stream0, ENABLE);
        DMA_ClearITPendingBit(DMA2_Stream0,DMA_IT_TCIF0);    
     }
}



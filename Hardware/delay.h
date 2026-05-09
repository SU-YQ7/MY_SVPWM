#ifndef __delay_H
#define __delay_H

#ifndef __DELAY_H
#define __DELAY_H

#include "stm32f4xx.h"

/**
 * @brief  初始化SysTick
 * @param  无
 * @retval 无
 */
void Delay_Init(void);

/**
 * @brief  微秒级延时
 * @param  us: 延时的微秒数
 * @retval 无
 */
void Delay_us(uint32_t us);

/**
 * @brief  毫秒级延时
 * @param  ms: 延时的毫秒数
 * @retval 无
 */
void Delay_ms(uint32_t ms);

#endif /* __DELAY_H */


#endif





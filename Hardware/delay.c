#include "stm32f4xx.h"

static uint32_t g_fac_us = 0; // 微秒延时因子

/**
 * @brief  初始化SysTick
 * @param  无
 * @retval 无
 */
void Delay_Init(void)
{
    // 配置SysTick时钟源为AHB时钟（168MHz）
    SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK);
    
    // 计算微秒延时因子
    g_fac_us = SystemCoreClock / 1000000;
}

/**
 * @brief  微秒级延时
 * @param  us: 延时的微秒数
 * @retval 无
 */
void Delay_us(uint32_t us)
{
    uint32_t temp;
    
    // 设置重载值
    SysTick->LOAD = us * g_fac_us;
    
    // 清空当前值
    SysTick->VAL = 0x00;
    
    // 启动SysTick
    SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
    
    // 等待计数完成
    do
    {
        temp = SysTick->CTRL;
    } while ((temp & SysTick_CTRL_ENABLE_Msk) && !(temp & SysTick_CTRL_COUNTFLAG_Msk));
    
    // 关闭SysTick
    SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
    
    // 清空当前值
    SysTick->VAL = 0x00;
}

/**
 * @brief  毫秒级延时
 * @param  ms: 延时的毫秒数
 * @retval 无
 */
void Delay_ms(uint32_t ms)
{
    while (ms--)
    {
        Delay_us(1000);
    }
}




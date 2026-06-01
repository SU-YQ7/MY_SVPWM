#ifndef __i2c_H
#define __i2c_H

#include "stm32f4xx.h"
void GPIO_I2c(void);
void delay_ts(uint32_t i);
u16 AS5600_ReadRawAngleTwo(void);
u8 AS5600_ReadOneByte(u8 addr);
u8 IIC_Read_Byte(u8 ack);
void IIC_Send_Byte(u8 txd);
void IIC_NAck(void);
void IIC_Ack(void);
u8 IIC_Wait_Ack(void);
void IIC_Stop(void);
void IIC_Start(void);
#endif
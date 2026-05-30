#ifndef __my_svpwm_H
#define __my_svpwm_H

#include "stm32f4xx.h"

extern uint16_t rotor_angle;   //电角度 0~359
extern uint16_t mech_angle;    //机械角度 0~359 (相对上电位置)

void m_us_radius_calculate(void);
uint16_t hallsensor_get_state(void);
uint16_t m_rotor_angle_calculate(void);

#endif

#include "stm32f4xx.h"
extern uint16_t cnt;
extern uint16_t spd_time;
uint16_t q16_m_value;
void m_us_radius_calculate(void)
{
	uint16_t q16_adc_val = 0;
	uint16_t q16_spd_val=0;
	/*ADC滤波值：右移4位转换为0.16格式数据*/
	q16_adc_val = (uint16_t)(cnt << 4);
	
	/*Q16格式ADC值限幅*/
	q16_adc_val = (q16_adc_val < 512)?0:q16_adc_val;
	q16_adc_val = (q16_adc_val > 65535)?65535:q16_adc_val;
		
	q16_spd_val = q16_adc_val;
	
	if(!spd_time)
	{
		spd_time = 10;   //50us*10=500us
		if(q16_spd_val > q16_m_value)
		{
			if((q16_spd_val - q16_m_value) > 100)
			{
				q16_m_value += 100;
			}
			else if((q16_spd_val - q16_m_value) > 10)
			{
				q16_m_value += 10;
			}
			else 
			{
				q16_m_value = q16_spd_val;
			}
		}
		else
		{
			if((q16_m_value - q16_spd_val) > 100)
			{
				q16_m_value -= 100;
			}
			else if((q16_m_value - q16_spd_val) > 10)
			{
				q16_m_value -= 10;
			}
			else 
			{
				q16_m_value = q16_spd_val;
			}
		}
	}
	/*Us半径最大限幅：0.9*/
	if(q16_m_value > 58982)
	{
		q16_m_value = 58982;
	}
	/*Us半径最小限幅：0.01*/
	if(q16_m_value < 655)
	{
		q16_m_value = 655;
	}
}
void m_hall_value_get(void)
{
	uint8_t hall_u;
	uint8_t hall_v;
	uint8_t hall_w;
	
	/*hall_u*/
	R_IOPORT_PinRead(&g_ioport_ctrl, BSP_IO_PORT_11_PIN_05, &m_hall_unit.u_val);//PB05
	/*hall_v*/
	R_IOPORT_PinRead(&g_ioport_ctrl, BSP_IO_PORT_11_PIN_06, &m_hall_unit.v_val);//PB06
	/*hall_w*/
	R_IOPORT_PinRead(&g_ioport_ctrl, BSP_IO_PORT_11_PIN_07, &m_hall_unit.w_val);//PB07
	hall_u = (uint8_t)m_hall_unit.u_val;
	hall_v = (uint8_t)m_hall_unit.v_val;
	hall_w = (uint8_t)m_hall_unit.w_val;
#if 0	//适配不同类型电机	
	hall_u = !hall_u;
	hall_v = !hall_v;
	hall_w = !hall_w;
#endif		
	m_hall_unit.value = (uint8_t)((hall_u << 2) | (hall_v << 1) | (hall_w << 0));
	if((m_hall_unit.value > 0) && (m_hall_unit.value < 7))
	{
        if(m_hall_unit.value != m_hall_unit.value_last)
        {
            m_hall_unit.value_last = m_hall_unit.value;
            m_hall_unit.update_sign = true;
			
			/*测试程序*/
			if((m_hall_unit.hall_val_test_index >= 0) && (m_hall_unit.hall_val_test_index <= 5))
			{
				m_hall_unit.hall_val_test_buf[m_hall_unit.hall_val_test_index++] =  m_hall_unit.value;
			}
        }
	}
	else
	{
		//ERROR LOGIC
	}	
}

/**
  ******************************************************************************
  * @brief  转子位置角初始化
  * @param  None.
  * @retval None.
  ******************************************************************************/
void m_rotor_angle_init(void)
{
	m_hall_unit.time = 0;
	m_hall_unit.time_last = 0;
	m_hall_unit.start_sign = true;
	m_hall_unit.angle_60_time = 0;
	m_hall_unit.angle_60_time_filter1 = 0;
	m_hall_unit.angle_60_time_filter2 = 0;
	m_hall_unit.start_cnt = 0;
	
	hall_capture_unit.hall_capture_reset_func();
    m_hall_value_get();
	rotor_angle_inc.u32 = 0;
	
#ifdef ANGLE_ACCURACY_ENHANCEMENT_MODE
	/*将获取到的转子初始位置角放在Rotor_Angle变量的高16位*/
	rotor_angle.words.high = Rotor_ANGLE_INIT_TABLE[m_hall_unit.value];
#else
	rotor_angle.u32 = ROTOR_ANGLE_INIT_TABLE[m_hall_unit.value];
#endif
	m_hall_unit.hall_val_test_index = 0;
}

/**
  ******************************************************************************
  * @brief  自定义除法函数，返回商
  * @param  dividend：被除数
  * @param  divisor：除数
  * @retval quotient：商
  ******************************************************************************/
uint32_t m_custom_divide(uint32_t dividend, uint32_t divisor) 
{
    uint32_t quotient = 0;
    while(dividend >= divisor) 
	{
        dividend -= divisor;
        quotient++;
    }
    return quotient;
}

/**
  ******************************************************************************
  * @brief  自定义取余函数，返回余数
  * @param  dividend：被除数
  * @param  divisor：除数
  * @retval dividend：余数
  ******************************************************************************/
uint32_t m_custom_remainder(uint32_t dividend, uint32_t divisor) 
{
    while (dividend >= divisor) 
	{
        dividend -= divisor;
    }
    return dividend;
}

/**
  ******************************************************************************
  * @brief  转子位置角计算：间隔50us进行一次计算
  * @param  None.
  * @retval 转子位置角 Q16
  ******************************************************************************/
uint16_t m_rotor_angle_calculate(void)
{
	m_hall_value_get();
	/*m_hall_unit.update_sign=true:霍尔值有更新*/
	if(m_hall_unit.update_sign)
	{
		m_hall_unit.update_sign = false;
        m_hall_unit.update_cnt = 0;		
		
		switch(m_motor_ctrl.direction)
		{
			case CCW://逆时针
				#ifdef ANGLE_ACCURACY_ENHANCEMENT_MODE	
					rotor_angle.words.high = ROTOR_ANGLE_TABLE_CCW[m_hall_unit.value];
					rotor_angle.words.low = 0;  //将低16位清零
				#else
					/*获取霍尔沿跳变转子位置角*/
					rotor_angle.u32 = ROTOR_ANGLE_TABLE_CCW[m_hall_unit.value];
					monitor_rotor_angle.u32 = rotor_angle.u32; //转子位置角监测
				#endif
			break;
			case CW: //顺时针
				#ifdef ANGLE_ACCURACY_ENHANCEMENT_MODE	
					rotor_angle.words.high = ROTOR_ANGLE_TABLE_CW[m_hall_unit.value];
					rotor_angle.words.low = 0;  //将低16位清零
				#else
					/*获取霍尔沿跳变转子位置角*/
					rotor_angle.u32 = ROTOR_ANGLE_TABLE_CW[m_hall_unit.value];
					monitor_rotor_angle.u32 = rotor_angle.u32; //转子位置角监测
				#endif	
			break;
		}	
		
		/*检测到三相中任意一相沿跳变：捕捉到180°电角度时间*/
		if(*hall_capture_unit.u_sign || *hall_capture_unit.v_sign || *hall_capture_unit.w_sign)
		{
			hall_capture_unit.hall_capture_sign_clear_func();
		#if 0	
			m_hall_unit.time = m_custom_divide(hall_capture_unit.hall_u_capture_val_func() + \
										     hall_capture_unit.hall_v_capture_val_func() + \
										     hall_capture_unit.hall_w_capture_val_func(), 9);
		#else
			/*计算60°电角度持续时间*/
			m_hall_unit.time = (hall_capture_unit.hall_u_capture_val_func() + \
										     hall_capture_unit.hall_v_capture_val_func() + \
										     hall_capture_unit.hall_w_capture_val_func()) / 9;
		#endif	
		}
		m_hall_unit.angle_60_time = m_hall_unit.time;

		if (m_hall_unit.angle_60_time != 0)
		{
			m_hall_unit.angle_60_time_filter1 = LPF_Calc(m_hall_unit.angle_60_time, \
													   m_hall_unit.angle_60_time_filter1);
			m_hall_unit.angle_60_time_filter2 = LPF_Calc(m_hall_unit.angle_60_time_filter1, \
													   m_hall_unit.angle_60_time_filter2);
		}
		/*电机运行初始阶段：霍尔捕获电角度值未到稳定状态*/
		if(m_hall_unit.start_sign == true)
		{
			m_hall_unit.time = MIN_SPEED_HALL_TIME_VALUE;//50RPM     最低转速对应60°电角度值
			if(m_hall_unit.start_cnt++ >= 10)
			{
				m_hall_unit.start_sign = false;
			}
		}
		/*霍尔捕获电角度值未到稳定状态*/
		else
		{
			m_hall_unit.time = m_hall_unit.angle_60_time_filter2; 	//>50RPM <3000RPM对应60°电角度值
		}
		
		if (m_hall_unit.time <= MAX_SPEED_HALL_TIME_VALUE) //3000RPM 最高转速对应60°电角度值	
		{				
			m_hall_unit.time = MAX_SPEED_HALL_TIME_VALUE;
		}
		/*计算角度增量*/
		#ifdef ANGLE_ACCURACY_ENHANCEMENT_MODE	
			rotor_angle_inc.words.high = (uint16_t)m_custom_divide(DθR_DIFF_VALUE, m_hall_unit.time); 
			rotor_angle_inc.words.low = (uint16_t)m_custom_remainder(DθR_DIFF_VALUE, m_hall_unit.time);
		#else
			//rotor_angle_inc.u32 = m_custom_divide(DθR_DIFF_VALUE, m_hall_unit.time);
			rotor_angle_inc.u32 = DθR_DIFF_VALUE / m_hall_unit.time;  //Q16格式
		#endif
	}
	else
	{
		if (m_hall_unit.update_cnt < HALL_VALUE_TIMEOUT_THRESHOLD_VALUE)						//65535 x 50us = 3.276750s
		{	
			m_hall_unit.update_cnt++;
			switch(m_motor_ctrl.direction)
			{
				case CCW://逆时针
					rotor_angle.u32 += rotor_angle_inc.u32;
				break;
				case CW: //顺时针
					rotor_angle.u32 -= rotor_angle_inc.u32;
				break;
			}	
		}
		else
		{
			//异常处理
			m_hall_unit.update_cnt = 0;
		}
		
	}
#ifdef ANGLE_ACCURACY_ENHANCEMENT_MODE	
//	R_DAC_Write(&g_dac1_ctrl, (uint16_t)((float)rotor_angle.words.high / 65536.0f * 4095.0f));
//	R_DAC_Write(&g_dac2_ctrl, (uint16_t)((float)monitor_rotor_angle.u32 / 65536.0f * 4095.0f));
#else
//	R_DAC_Write(&g_dac1_ctrl, (uint16_t)((float)rotor_angle.words.low / 65536.0f * 4095.0f));
//	R_DAC_Write(&g_dac2_ctrl, (uint16_t)((float)monitor_rotor_angle.u32 / 65536.0f * 4095.0f));
#endif	
	
#ifdef ANGLE_ACCURACY_ENHANCEMENT_MODE	
	return (uint16_t)rotor_angle.words.high;   //最终计算的转子位置角
#else
	return (uint16_t)rotor_angle.words.low;   //最终计算的转子位置角
#endif
}

void m_us_sector_calculate(uint16_t theta)
{
    if 		((theta >= 0)         && (theta < EANGLE60))	m_svpwm_unit.sector = 1;
    else if ((theta >= EANGLE60)  && (theta < EANGLE120)) 	m_svpwm_unit.sector = 2;
    else if ((theta >= EANGLE120) && (theta < EANGLE180)) 	m_svpwm_unit.sector = 3;
    else if ((theta >= EANGLE180) && (theta < EANGLE240)) 	m_svpwm_unit.sector = 4;
    else if ((theta >= EANGLE240) && (theta < EANGLE300)) 	m_svpwm_unit.sector = 5;
    else 													m_svpwm_unit.sector = 6;
}

/**
  ******************************************************************************
  * @brief  获取x y z对应的值
  * @param  None.
  * @retval None.
  ******************************************************************************/
void m_ux_uy_uz_calculate(void)
{
    union_s32 value;
	
	/*x=sinθ*/
    m_svpwm_unit.q15_ux = math_unit.q15_sin;    
	
	/*√3/2cosθ*/
    value.s32 = SQRT3DIV2 * math_unit.q15_cos; //Q16 * Q15 = Q31
    
	/*y=1/2sinθ + √3/2cosθ*/
    m_svpwm_unit.q15_uy = (math_unit.q15_sin >> 1) + value.words.high;   //Q15
	/*z=1/2sinθ - √3/2cosθ*/
    m_svpwm_unit.q15_uz = (math_unit.q15_sin >> 1) - value.words.high;   //Q15
}

/**
  ******************************************************************************
  * @brief  计算第一矢量作用时间ta和第二矢量作用时间tb的值
  * @param  first_x_y_z:第一矢量作用时间对应的x y z中其中的一个值，做正负乘值
  * @param  second_x_y_z:第一矢量作用时间对应的x y z中其中的一个值，做正负乘值
  * @param  us_m:us M系数值
  * @retval None.
  ******************************************************************************/
void m_ta_tb_calculate(int16_t first_x_y_z, int16_t second_x_y_z, uint16_t us_m)
{
	union_u32 m_t_value;
	union_s32 ta;
	union_s32 tb;
	
	/*
		传参进来的first_x_y_z second_x_y_z值理论分析是都是>=0的
		因为定义传参值为Q15格式的，防止在0值附近计算出现负值
		需要做<0的处理
	*/
	if(first_x_y_z < 0)		first_x_y_z = 0;
	if(second_x_y_z < 0)	second_x_y_z = 0;
	
	/*计算M*T Q16*Q16=Q32*/
	m_t_value.u32 = us_m * PWM_PERIOD_T_VALUE;		//Q32
	
	/*计算ta Q16*Q15=Q31*/
	ta.s32 = m_t_value.words.high * first_x_y_z;	//Q31 
	m_svpwm_unit.q15_ta = ta.words.high;			//Q15
	
	/*计算tb Q16*Q15=Q31*/
	tb.s32 = m_t_value.words.high * second_x_y_z;	//Q31 
	m_svpwm_unit.q15_tb = tb.words.high;		 	//Q15
}

/**
  ******************************************************************************
  * @brief  计算taout tbout tcout
  * @param  None.
  * @retval None.
  ******************************************************************************/
void m_taout_tbout_tcout_calculate(void)
{
    uint16_t v1t = 0; 
	uint16_t v2t = 0;
	uint16_t ta_q15_to_q16 = 0;
	uint16_t tb_q15_to_q16 = 0;
	
	/*
		将Q15转换为Q16数据格式流程
		1.将Q15数据格式左移1位，将符号位移位到bit16。强制类型转换为uint32_t
		2.与0x0000FFFF相与，将高16位清零，低16位为Q16格式数据。
		3.强制类型转换uint16_t类型，数据为Q16格式
	*/
	ta_q15_to_q16 = (uint16_t)(((uint32_t)(m_svpwm_unit.q15_ta << 1)) & 0x0000FFFF); //Q16
	tb_q15_to_q16 = (uint16_t)(((uint32_t)(m_svpwm_unit.q15_tb << 1)) & 0x0000FFFF); //Q16
	
    v1t = ta_q15_to_q16 >> 1;   //ta/2
    v2t = tb_q15_to_q16 >> 1;   //tb/2

    m_svpwm_unit.q16_tc_out = ((PWM_PERIOD_T_VALUE >> 1) - v1t - v2t) >> 1;  //tcout:(T/2 - ta/2 - tb / 2) / 2
    m_svpwm_unit.q16_tb_out = m_svpwm_unit.q16_tc_out + v2t;  //tbout = tcout + tb/2
    m_svpwm_unit.q16_ta_out = m_svpwm_unit.q16_tb_out + v1t;  //taout = tbout + ta/2
}

/**
  ******************************************************************************
  * @brief  svpwm输出
  * @param  us_m:us M系数值
  * @retval None.
  ******************************************************************************/
void m_svpwm_duty_calculate(uint16_t us_m)
{
    three_phase_duty_cycle_t three_phase_duty_cycle = {0};

    switch (m_svpwm_unit.sector)
    {
    case 1:
		/*
			ta=t1   -z
			tb=t2	x
		*/
		m_ta_tb_calculate(-m_svpwm_unit.q15_uz, m_svpwm_unit.q15_ux, us_m);
		m_taout_tbout_tcout_calculate();
        m_svpwm_unit.u_duty_value = m_svpwm_unit.q16_ta_out;
        m_svpwm_unit.v_duty_value = m_svpwm_unit.q16_tb_out;
        m_svpwm_unit.w_duty_value = m_svpwm_unit.q16_tc_out;
        break;
    case 2:
		/*
			ta=t3   z
			tb=t2	y
		*/
		m_ta_tb_calculate(m_svpwm_unit.q15_uz, m_svpwm_unit.q15_uy, us_m);
		m_taout_tbout_tcout_calculate();
        m_svpwm_unit.u_duty_value = m_svpwm_unit.q16_tb_out;
        m_svpwm_unit.v_duty_value = m_svpwm_unit.q16_ta_out;
        m_svpwm_unit.w_duty_value = m_svpwm_unit.q16_tc_out;
        break;
    case 3:
		/*
			ta=t3   x
			tb=t4	-y
		*/
		m_ta_tb_calculate(m_svpwm_unit.q15_ux, -m_svpwm_unit.q15_uy, us_m);
		m_taout_tbout_tcout_calculate();
        m_svpwm_unit.u_duty_value = m_svpwm_unit.q16_tc_out;
        m_svpwm_unit.v_duty_value = m_svpwm_unit.q16_ta_out;
        m_svpwm_unit.w_duty_value = m_svpwm_unit.q16_tb_out;
        break;
    case 4:
		/*
			ta=t5   -x
			tb=t4	z
		*/
		m_ta_tb_calculate(-m_svpwm_unit.q15_ux, m_svpwm_unit.q15_uz, us_m);
		m_taout_tbout_tcout_calculate();
        m_svpwm_unit.u_duty_value = m_svpwm_unit.q16_tc_out;
        m_svpwm_unit.v_duty_value = m_svpwm_unit.q16_tb_out;
        m_svpwm_unit.w_duty_value = m_svpwm_unit.q16_ta_out;
        break;
    case 5:
		/*
			ta=t5   -y
			tb=t6	-z
		*/
		m_ta_tb_calculate(-m_svpwm_unit.q15_uy, -m_svpwm_unit.q15_uz, us_m);
		m_taout_tbout_tcout_calculate();
        m_svpwm_unit.u_duty_value = m_svpwm_unit.q16_tb_out;
        m_svpwm_unit.v_duty_value = m_svpwm_unit.q16_tc_out;
        m_svpwm_unit.w_duty_value = m_svpwm_unit.q16_ta_out;
        break;
    case 6:
		/*
			ta=t1   y
			tb=t6	-x
		*/
		m_ta_tb_calculate(m_svpwm_unit.q15_uy, -m_svpwm_unit.q15_ux, us_m);
		m_taout_tbout_tcout_calculate();
        m_svpwm_unit.u_duty_value = m_svpwm_unit.q16_ta_out;
        m_svpwm_unit.v_duty_value = m_svpwm_unit.q16_tc_out;
        m_svpwm_unit.w_duty_value = m_svpwm_unit.q16_tb_out;
        break;
    default:
        break;
    }

    /*最小占空比限制*/
    m_svpwm_unit.u_duty_value = (m_svpwm_unit.u_duty_value < MIN_DUTY_VALUE) ? MIN_DUTY_VALUE : m_svpwm_unit.u_duty_value;
    m_svpwm_unit.v_duty_value = (m_svpwm_unit.v_duty_value < MIN_DUTY_VALUE) ? MIN_DUTY_VALUE : m_svpwm_unit.v_duty_value;
    m_svpwm_unit.w_duty_value = (m_svpwm_unit.w_duty_value < MIN_DUTY_VALUE) ? MIN_DUTY_VALUE : m_svpwm_unit.w_duty_value;
    /*最大占空比限制*/
    m_svpwm_unit.u_duty_value = (m_svpwm_unit.u_duty_value > MAX_DUTY_VALUE) ? MAX_DUTY_VALUE : m_svpwm_unit.u_duty_value;
    m_svpwm_unit.v_duty_value = (m_svpwm_unit.v_duty_value > MAX_DUTY_VALUE) ? MAX_DUTY_VALUE : m_svpwm_unit.v_duty_value;
    m_svpwm_unit.w_duty_value = (m_svpwm_unit.w_duty_value > MAX_DUTY_VALUE) ? MAX_DUTY_VALUE : m_svpwm_unit.w_duty_value;

    //最终占空比给定  duty=period - period* y%
    three_phase_duty_cycle.duty[0] = (PWM_PERIOD_T_VALUE >> 1) - m_svpwm_unit.u_duty_value;  // U相占空比
    three_phase_duty_cycle.duty[1] = (PWM_PERIOD_T_VALUE >> 1) - m_svpwm_unit.v_duty_value;  // V相占空比
    three_phase_duty_cycle.duty[2] = (PWM_PERIOD_T_VALUE >> 1) - m_svpwm_unit.w_duty_value;  // W相占空比
    R_GPT_THREE_PHASE_DutyCycleSet(&g_three_phase0_ctrl, &three_phase_duty_cycle);

#if 1	
//  R_DAC_Write (&g_dac0_ctrl, (uint16_t)((float)(m_us_unit.q15_ux + 32768) / 65535.0f * 4095.0f));
    R_DAC_Write(&g_dac0_ctrl, m_svpwm_unit.u_duty_value);
	R_DAC_Write(&g_dac1_ctrl, m_svpwm_unit.v_duty_value);
	R_DAC_Write(&g_dac2_ctrl, m_svpwm_unit.w_duty_value);
#endif
}

/**
  ******************************************************************************
  * @brief  Us输出
  * @param  us_m:us M系数值
  * @param  us_angle:电压合成矢量当前角度
  * @retval None.
  ******************************************************************************/
void m_svpwm_generate(uint16_t us_m, uint16_t us_angle)
{
	/*第1步：通过调用math_xx.c文件里面的三角函数计算API接口计算我们
	Us矢量与0度位置的夹角θ对应的正弦值和余弦值
	*/
    math_sin_cos_calculate(us_angle);  
	/*第2步：计算扇区的编号*/
    m_us_sector_calculate(us_angle);
	/*第3步：计算x y z通用表达式的值*/
    m_ux_uy_uz_calculate();
	/*第4步：生成SVPWM波形*/
    m_svpwm_duty_calculate(us_m);
}
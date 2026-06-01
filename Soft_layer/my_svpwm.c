#include "stm32f4xx.h"
#include "my_svpwm.h"
#include "math.h"
extern uint16_t cnt;
uint16_t us_m=0;
uint16_t us_angle=0;
//三角函数计算......
#define PI 3.14159265358979f
#define SQRT3DIV2  					56756  //sqrt3/2 * 65536 = 56755.84 ≈ 56756  //0.16格式
#define PWM_PERIOD_T_VALUE   		(4200)  

/*死区时间：占空比最大值、最小值*/
#define DEAD_TIME 					240   		//2us  上升沿2us 下降沿2us
#define MIN_DUTY_VALUE				(DEAD_TIME)
#define MAX_DUTY_VALUE				((PWM_PERIOD_T_VALUE >> 1) - (DEAD_TIME + (DEAD_TIME >> 1)))
int16_t sin_x=0;
int16_t cos_x=0;
int16_t m_svpwm_unit_q15_uy=0;
int16_t m_svpwm_unit_q15_uz=0;
int16_t m_svpwm_unit_q15_ux=0;
uint16_t m_svpwm_unit_q16_tc_out=0;
uint16_t m_svpwm_unit_q16_ta_out=0;
uint16_t m_svpwm_unit_q16_tb_out=0;
uint16_t m_svpwm_unit_u_duty_value=0;
uint16_t m_svpwm_unit_v_duty_value=0;
uint16_t m_svpwm_unit_w_duty_value=0;
int16_t m_svpwm_unit_q15_ta=0;
int16_t m_svpwm_unit_q15_tb=0;


//模长检测变量......
extern uint16_t ADC_value;
uint16_t q16_m_value;
extern uint16_t spd_time;

uint16_t m_svpwm_unit_sector=0;

/*
  ******************************************************************************
  * @brief  ADC检测模长
  * @param  None.
  * @retval None.
  *****************************************************************************
*/
void m_us_radius_calculate(void)
{
uint16_t q16_adc_val = 0;
	uint16_t q16_spd_val=0;
	/*ADC滤波值：右移4位转换为0.16格式数据*/
	q16_adc_val = (uint16_t)(ADC_value << 4);
	
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

/*
  ******************************************************************************
  * @brief  扇区计算
  * @param  转子角度.
  * @retval None.
  *****************************************************************************
*/
void m_us_sector_calculate(uint16_t theta)
{
    if 		((theta >= 0)   && (theta < 60))	m_svpwm_unit_sector = 1;
    else if ((theta >= 60)  && (theta < 120)) 	m_svpwm_unit_sector = 2;
    else if ((theta >= 120) && (theta < 180)) 	m_svpwm_unit_sector = 3;
    else if ((theta >= 180) && (theta < 240)) 	m_svpwm_unit_sector = 4;
    else if ((theta >= 240) && (theta < 300)) 	m_svpwm_unit_sector = 5;
    else 													m_svpwm_unit_sector = 6;
}

/**
  ******************************************************************************
  * @brief  获取x y z对应的值
  * @param  None.
  * @retval None.
  ******************************************************************************/
void m_ux_uy_uz_calculate(void)
{
    int32_t value;
	/*x=sinθ*/
    m_svpwm_unit_q15_ux = sin_x;    
	/*√3/2cosθ*/
    value = SQRT3DIV2 * cos_x; //Q16 * Q15 = Q31
	/*y=1/2sinθ + √3/2cosθ*/
    m_svpwm_unit_q15_uy = (sin_x >> 1) + (value>>16);   //Q15
	/*z=1/2sinθ - √3/2cosθ*/
    m_svpwm_unit_q15_uz = (sin_x >> 1) - (value>>16);  //Q15
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
	uint32_t m_t_value;
	uint32_t ta;
	uint32_t tb;
	/*
		传参进来的first_x_y_z second_x_y_z值理论分析是都是>=0的
		因为定义传参值为Q15格式的，防止在0值附近计算出现负值
		需要做<0的处理
	*/
	if(first_x_y_z < 0)		first_x_y_z = 0;
	if(second_x_y_z < 0)	second_x_y_z = 0;
	
	/*计算M*T Q16*Q16=Q32*/
	m_t_value = us_m * PWM_PERIOD_T_VALUE;		//Q32
	
	/*计算ta Q16*Q15=Q31*/
	ta= (m_t_value>>16) * first_x_y_z;	//Q31
	m_svpwm_unit_q15_ta = ta>>16;			//Q15

	/*计算tb Q16*Q15=Q31*/
	tb= (m_t_value>>16) * second_x_y_z;	//Q31
	m_svpwm_unit_q15_tb = tb>>16;		 	//Q15
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
	
	/*s
		将Q15转换为Q16数据格式流程
		1.将Q15数据格式左移1位，将符号位移位到bit16。强制类型转换为uint32_t
		2.与0x0000FFFF相与，将高16位清零，低16位为Q16格式数据。
		3.强制类型转换uint16_t类型，数据为Q16格式
	*/
	ta_q15_to_q16 = (uint16_t)(((uint32_t)(m_svpwm_unit_q15_ta << 1)) & 0x0000FFFF); //Q16
	tb_q15_to_q16 = (uint16_t)(((uint32_t)(m_svpwm_unit_q15_tb << 1)) & 0x0000FFFF); //Q16
	
    v1t = ta_q15_to_q16 >> 1;   //ta/2
    v2t = tb_q15_to_q16 >> 1;   //tb/2

    m_svpwm_unit_q16_tc_out = ((PWM_PERIOD_T_VALUE >> 1) - v1t - v2t) >> 1;  //tcout:(T/2 - ta/2 - tb / 2) / 2
    m_svpwm_unit_q16_tb_out = m_svpwm_unit_q16_tc_out + v2t;  //tbout = tcout + tb/2
    m_svpwm_unit_q16_ta_out = m_svpwm_unit_q16_tb_out + v1t;  //taout = tbout + ta/2
}

/**
  ******************************************************************************
  * @brief  svpwm输出
  * @param  us_m:us M系数值
  * @retval None.
  ******************************************************************************/
void m_svpwm_duty_calculate(uint16_t us_m)
{
    switch (m_svpwm_unit_sector)
    {
    case 1:
		/*
			ta=t1   -z
			tb=t2	x
		*/
		m_ta_tb_calculate(-m_svpwm_unit_q15_uz, m_svpwm_unit_q15_ux, us_m);
		m_taout_tbout_tcout_calculate();
        m_svpwm_unit_u_duty_value = m_svpwm_unit_q16_ta_out;
        m_svpwm_unit_v_duty_value = m_svpwm_unit_q16_tb_out;
        m_svpwm_unit_w_duty_value = m_svpwm_unit_q16_tc_out;
        break;
    case 2:
		/*
			ta=t3   z
			tb=t2	y
		*/
		m_ta_tb_calculate(m_svpwm_unit_q15_uz, m_svpwm_unit_q15_uy, us_m);
		m_taout_tbout_tcout_calculate();
        m_svpwm_unit_u_duty_value = m_svpwm_unit_q16_tb_out;
        m_svpwm_unit_v_duty_value = m_svpwm_unit_q16_ta_out;
        m_svpwm_unit_w_duty_value = m_svpwm_unit_q16_tc_out;
        break;
    case 3:
		/*
			ta=t3   x
			tb=t4	-y
		*/
		m_ta_tb_calculate(m_svpwm_unit_q15_ux, -m_svpwm_unit_q15_uy, us_m);
		m_taout_tbout_tcout_calculate();
        m_svpwm_unit_u_duty_value = m_svpwm_unit_q16_tc_out;
        m_svpwm_unit_v_duty_value = m_svpwm_unit_q16_ta_out;
        m_svpwm_unit_w_duty_value = m_svpwm_unit_q16_tb_out;
        break;
    case 4:
		/*
			ta=t5   -x
			tb=t4	z
		*/
		m_ta_tb_calculate(-m_svpwm_unit_q15_ux, m_svpwm_unit_q15_uz, us_m);
		m_taout_tbout_tcout_calculate();
        m_svpwm_unit_u_duty_value = m_svpwm_unit_q16_tc_out;
        m_svpwm_unit_v_duty_value = m_svpwm_unit_q16_tb_out;
        m_svpwm_unit_w_duty_value = m_svpwm_unit_q16_ta_out;
        break;
    case 5:
		/*
			ta=t5   -y
			tb=t6	-z
		*/
		m_ta_tb_calculate(-m_svpwm_unit_q15_uy, -m_svpwm_unit_q15_uz, us_m);
		m_taout_tbout_tcout_calculate();
        m_svpwm_unit_u_duty_value = m_svpwm_unit_q16_tb_out;
        m_svpwm_unit_v_duty_value = m_svpwm_unit_q16_tc_out;
        m_svpwm_unit_w_duty_value = m_svpwm_unit_q16_ta_out;
        break;
    case 6:
		/*
			ta=t1   y
			tb=t6	-x
		*/
		m_ta_tb_calculate(m_svpwm_unit_q15_uy, -m_svpwm_unit_q15_ux, us_m);
		m_taout_tbout_tcout_calculate();
        m_svpwm_unit_u_duty_value = m_svpwm_unit_q16_ta_out;
        m_svpwm_unit_v_duty_value = m_svpwm_unit_q16_tc_out;
        m_svpwm_unit_w_duty_value = m_svpwm_unit_q16_tb_out;
        break;
    default:
        break;
    }
    /*最小占空比限制*/
    m_svpwm_unit_u_duty_value = (m_svpwm_unit_u_duty_value < MIN_DUTY_VALUE) ? MIN_DUTY_VALUE : m_svpwm_unit_u_duty_value;
    m_svpwm_unit_v_duty_value = (m_svpwm_unit_v_duty_value < MIN_DUTY_VALUE) ? MIN_DUTY_VALUE : m_svpwm_unit_v_duty_value;
    m_svpwm_unit_w_duty_value = (m_svpwm_unit_w_duty_value < MIN_DUTY_VALUE) ? MIN_DUTY_VALUE : m_svpwm_unit_w_duty_value;
    /*最大占空比限制*/
    m_svpwm_unit_u_duty_value = (m_svpwm_unit_u_duty_value > MAX_DUTY_VALUE) ? MAX_DUTY_VALUE : m_svpwm_unit_u_duty_value;
    m_svpwm_unit_v_duty_value = (m_svpwm_unit_v_duty_value > MAX_DUTY_VALUE) ? MAX_DUTY_VALUE : m_svpwm_unit_v_duty_value;
    m_svpwm_unit_w_duty_value = (m_svpwm_unit_w_duty_value > MAX_DUTY_VALUE) ? MAX_DUTY_VALUE : m_svpwm_unit_w_duty_value;
    TIM_SetCompare1(TIM1,m_svpwm_unit_u_duty_value);
    TIM_SetCompare2(TIM1,m_svpwm_unit_v_duty_value);
	  TIM_SetCompare3(TIM1,m_svpwm_unit_w_duty_value);
}

void math_sin_cos_calculate( uint16_t us_angle)
{
  float angle_rad = (float)us_angle * PI / 180.0f;
   sin_x=sin(angle_rad)*32768; // 计算正弦值
   cos_x=cos(angle_rad)*32768;
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

#include "stm32f4xx.h"
#include "my_svpwm.h"
#include "math.h"
extern uint16_t cnt;
uint16_t us_m=0;
uint16_t us_angle=0;
//三角函数计算......
#define PI 3.14159265358979f
#define SQRT3DIV2  					56756  //sqrt3/2 * 65536 = 56755.84 ≈ 56756  //0.16格式
#define PWM_PERIOD_T_VALUE   		(6000)  

/*死区时间：占空比最大值、最小值*/
#define DEAD_TIME 					240   		//2us  上升沿2us 下降沿2us
#define MIN_DUTY_VALUE				(DEAD_TIME)
#define MAX_DUTY_VALUE				((PWM_PERIOD_T_VALUE >> 1) - (DEAD_TIME + (DEAD_TIME >> 1)))
int16_t sin_x=0;
int16_t cos_x=0;
int16_t m_svpwm_unit_q15_ux=0;
int16_t m_svpwm_unit_q15_uy=0;
int16_t m_svpwm_unit_q15_uz=0;
uint16_t m_svpwm_unit_q16_tc_out=0;
uint16_t m_svpwm_unit_q16_ta_out=0;
uint16_t m_svpwm_unit_q16_tb_out=0;
uint16_t m_svpwm_unit_u_duty_value=0;
uint16_t m_svpwm_unit_v_duty_value=0;
uint16_t m_svpwm_unit_w_duty_value=0;
int16_t m_svpwm_unit_q15_ta=0;
int16_t m_svpwm_unit_q15_tb=0;



//霍尔更新变量......
uint16_t hall_upate=0;
uint16_t hall_capture_unit=0;
uint16_t start_sign=0;

//角度检测变量......

uint16_t enter_cnt=0;
uint16_t rotor_angle=0;       //电角度 0~359
uint16_t mech_angle=0;        //机械角度 0~359 (相对上电位置)
/*电机极对数：机械转一圈经过 3 个电周期(串口 3 次 0°)*/
#define POLE_PAIRS   3

//模长检测变量......
extern uint16_t ADC_value;
uint16_t q16_m_value;
extern uint16_t spd_time;

uint16_t m_svpwm_unit_sector=0;
/*霍尔状态(下标)→电角度，由开环标定实测：正转序列 4→6→2→3→1→5，间隔约60°*/
static const uint16_t  ROTOR_ANGLE_TABLE_CW[7]   = {0,157,47,103,282,218,339};
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
  * @brief  霍尔传感器
  * @param  None.
  * @retval 霍尔状态
  *****************************************************************************
*/
uint16_t hallsensor_get_state(void)
{
    __IO static uint16_t state=0 ;
	static uint16_t next_state =0;
	state=0;
  if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_5) != RESET) //U
  {
      state |= 0x1U;
  }
  if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_4) != RESET) //V
   {
      state |= 0x02U;
   }
  if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_3) != RESET) //W
   {
       state |= 0x04U;
   }
  if(next_state != state)
	 {
 		hall_upate=1;
		hall_capture_unit++;
		next_state =state;
   }
	 else
	 {
		hall_upate=0;
	 }
   return state;
}

/*
  ******************************************************************************
  * @brief  转子位置角计算：间隔50us进行一次计算
  * @param  None.
  * @retval 转子位置角 Q16
  *****************************************************************************
*/

uint16_t m_rotor_angle_calculate(void)
{
	uint32_t change_hall=0;
	static uint16_t sector_ticks  = 0;       //每 60° 经历的 50us tick 数
	static uint32_t angle_inc_q16 = 0;       //每个 tick 的角度增量, Q16(度)
	static uint32_t angle_q16     = 0;       //当前电角度, Q16(度)
	static uint16_t stall_ticks   = 0;       //距上次霍尔跳变的 tick 数
	static uint16_t cur_base      = 0;       //当前扇区电角度基准
	static uint16_t prev_base     = 0xFFFF;  //上一扇区电角度基准
	static uint8_t  elec_rev      = 0;       //已完成电周期数 0~(POLE_PAIRS-1)

	enter_cnt ++;
	change_hall=hallsensor_get_state();

	/*无效霍尔态(0 或 7)：故障，保持上一次角度*/
	if(change_hall < 1 || change_hall > 6)
	{
		return rotor_angle;
	}

	if(hall_upate)
	{
		uint16_t base;
		hall_upate=0;
		stall_ticks = 0;   //来了新沿，清失速计数

		/*本次霍尔沿对应的电角度扇区基准角*/
		base = ROTOR_ANGLE_TABLE_CW[change_hall];

		/*电周期回绕检测：基准角由 300 跳回 0(CW)说明走完一个电周期*/
		if(prev_base != 0xFFFF && base < prev_base)
		{
			if(++elec_rev >= POLE_PAIRS)
			{
				elec_rev = 0;
			}
		}
		prev_base = base;
		cur_base  = base;

		/*角度对齐到扇区基准角，消除累计误差*/
		angle_q16 = (uint32_t)base << 16;

		/*每 9 次跳变更新一次平均扇区时间*/
		if(hall_capture_unit==9)
		{
			sector_ticks = (uint16_t)(enter_cnt / 9);   //每 60° 的 tick 数
			start_sign=1;
			hall_capture_unit=0;
			enter_cnt=0;
		}

		/*电机运行初始阶段：霍尔尚未标定速度。
		  不再 return 0(否则矢量钉死在 0°，转子转不起来→永远标定不了→死锁)，
		  而是直接用霍尔扇区基准角 base 作为电角度强制换相(六步式粗换相)。
		  angle_q16 上面已对齐到 base，下方会据此输出 rotor_angle=base，
		  配合调用处 +90° 转矩角即可产生起动转矩。*/
		if(start_sign == 0)
		{
			rotor_angle = base;
		}

		/*每 tick 增量 = 60° / 每扇区 tick 数，用 Q16 保留小数*/
		if(sector_ticks != 0)
		{
			angle_inc_q16 = ((uint32_t)60 << 16) / sector_ticks;
		}
	}
	else
	{
		stall_ticks++;
		/*超过上次扇区时间的 2 倍仍无跳变 -> 判定停转/堵转，冻结角度*/
		if(sector_ticks != 0 && stall_ticks > (uint16_t)(sector_ticks * 2))
		{
			angle_inc_q16 = 0;   //停止插值推进，角度冻结在最后的真实扇区角
			start_sign    = 0;   //速度估计作废，重新起转时再标定
		}
		else
		{
			/*沿与沿之间线性插值推算角度，但不越过本扇区终点(等下一个霍尔沿翻扇区)*/
			uint32_t cap = ((uint32_t)(cur_base + 60)) << 16;
			angle_q16 += angle_inc_q16;
			if(angle_q16 > cap)
			{
				angle_q16 = cap;
			}
		}
	}

	/*电角度 0~359*/
	{
		uint16_t elec = (uint16_t)(angle_q16 >> 16);
		if(elec >= 360) elec -= 360;
		rotor_angle = elec;
	}

	/*机械角度 = (电周期数 x 360 + 电角度) / 极对数，相对上电位置*/
	{
		uint32_t total_elec = (uint32_t)elec_rev * 360 + (angle_q16 >> 16);
		uint16_t mech = (uint16_t)(total_elec / POLE_PAIRS);
		if(mech >= 360) mech -= 360;
		mech_angle = mech;
	}

	return rotor_angle;   //返回电角度(0~359)，供 SVPWM 电压矢量旋转使用
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

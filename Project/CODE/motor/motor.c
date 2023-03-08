/*******************************************************************************
	*	@file:		motor.c
	*	@brief:		
	*	@author:	whut
	*	@date:		2019-04-26
	*	@function:
							Motor_Init(void)										//初始化各个通道PWM
							Motor_Run(Moto_e moto, int16 duty)	//电机PWM控制函数
							motor_soft_start(int speed_normal)	//电机软起动
*******************************************************************************/
#include "motor.h"

/***********************************************************
	*	函数名称：Motor_Init(void)
	*	函数说明：初始化各个通道PWM
	*	函数参数：none
	*	参数返回：void
	*	修改日期：2019-11-23
***********************************************************/
void Motor_Init(void)
{
	pwm_init(PWM_TIM, PWM_CH1, motor_freq, duty_max);
	pwm_init(PWM_TIM, PWM_CH2, motor_freq, duty_max);
	pwm_init(PWM_TIM, PWM_CH3, motor_freq, duty_max);
	pwm_init(PWM_TIM, PWM_CH4, motor_freq, duty_max);

}

/***********************************************************
	*	函数名称：Motor_Run(Moto_e moto, int16 duty)
	*	函数说明：电机PWM控制函数
	*	函数参数：Motor_e	Moto_L、Moto_R
							duty	
	*	参数返回：void
	*	参数说明：Motor_L：左电机		duty>0：正转
							Motor_R：右电机		duty<0：反转
	*	修改日期：2019-11-23
***********************************************************/
void Motor_Run(Motor_e motor, int duty)
{
	if(ABS(duty)>=duty_max)
	{
		if(duty>=0)
			duty = duty_max;
		else
			duty = -duty_max;
	}
	
	switch(motor)
	{		
		case Motor_R:
			if(duty>=0)
			{
				pwm_duty_updata(PWM_TIM, PWM_CH1, PWM_DUTY_MAX);
				pwm_duty_updata(PWM_TIM, PWM_CH2, PWM_DUTY_MAX-duty);
			}
			else
			{
				pwm_duty_updata(PWM_TIM, PWM_CH1, PWM_DUTY_MAX+duty);
				pwm_duty_updata(PWM_TIM, PWM_CH2, PWM_DUTY_MAX);
			}	
			break;
			
		case Motor_L:
			if(duty>=0)
			{
				pwm_duty_updata(PWM_TIM, PWM_CH3, PWM_DUTY_MAX);
				pwm_duty_updata(PWM_TIM, PWM_CH4, PWM_DUTY_MAX-duty);
			}			
			else
			{
				pwm_duty_updata(PWM_TIM, PWM_CH3, PWM_DUTY_MAX+duty);
				pwm_duty_updata(PWM_TIM, PWM_CH4, PWM_DUTY_MAX);
			}
			break;
	
	}
	
}



extern int Speed_L;
extern int Speed_R;
/***********************************************************
	*	函数名称：motor_soft_start(int speed_normal)
	*	函数说明：电机软起动
	*	函数参数：int speed_normal
	*	参数返回：start_over_sign
	*	参数说明：
	*	修改日期：2019-04-26
***********************************************************/
char motor_soft_start(int speed_normal)
{
	static char start_over_sign=0;        //车辆软起动结束标志
	int normal_speed_pwm;
	if(start_over_sign == 0)            //如果软起动过程还没有结束
	{
		//normal_speed_pwm = (uint32)(1.67 * (float)(speed_normal) + 62.24);      
		normal_speed_pwm = (int)(187 * (speed_normal) + 700);
		if((Speed_L < (speed_normal - 5)) || (Speed_R < (speed_normal - 5)))
		{
			Motor_Run(Motor_L,normal_speed_pwm);
			Motor_Run(Motor_R,normal_speed_pwm);
		}
		else                            //如果软起动结束
		{
			start_over_sign = 1;
		}
	}
	
	return start_over_sign; 
}


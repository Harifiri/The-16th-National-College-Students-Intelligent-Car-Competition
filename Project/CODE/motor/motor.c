/*******************************************************************************
	*	@file:		motor.c
	*	@brief:		
	*	@author:	whut
	*	@date:		2019-04-26
	*	@function:
							Motor_Init(void)										//��ʼ������ͨ��PWM
							Motor_Run(Moto_e moto, int16 duty)	//���PWM���ƺ���
							motor_soft_start(int speed_normal)	//�������
*******************************************************************************/
#include "motor.h"

/***********************************************************
	*	�������ƣ�Motor_Init(void)
	*	����˵������ʼ������ͨ��PWM
	*	����������none
	*	�������أ�void
	*	�޸����ڣ�2019-11-23
***********************************************************/
void Motor_Init(void)
{
	pwm_init(PWM_TIM, PWM_CH1, motor_freq, duty_max);
	pwm_init(PWM_TIM, PWM_CH2, motor_freq, duty_max);
	pwm_init(PWM_TIM, PWM_CH3, motor_freq, duty_max);
	pwm_init(PWM_TIM, PWM_CH4, motor_freq, duty_max);

}

/***********************************************************
	*	�������ƣ�Motor_Run(Moto_e moto, int16 duty)
	*	����˵�������PWM���ƺ���
	*	����������Motor_e	Moto_L��Moto_R
							duty	
	*	�������أ�void
	*	����˵����Motor_L������		duty>0����ת
							Motor_R���ҵ��		duty<0����ת
	*	�޸����ڣ�2019-11-23
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
	*	�������ƣ�motor_soft_start(int speed_normal)
	*	����˵�����������
	*	����������int speed_normal
	*	�������أ�start_over_sign
	*	����˵����
	*	�޸����ڣ�2019-04-26
***********************************************************/
char motor_soft_start(int speed_normal)
{
	static char start_over_sign=0;        //�������𶯽�����־
	int normal_speed_pwm;
	if(start_over_sign == 0)            //������𶯹��̻�û�н���
	{
		//normal_speed_pwm = (uint32)(1.67 * (float)(speed_normal) + 62.24);      
		normal_speed_pwm = (int)(187 * (speed_normal) + 700);
		if((Speed_L < (speed_normal - 5)) || (Speed_R < (speed_normal - 5)))
		{
			Motor_Run(Motor_L,normal_speed_pwm);
			Motor_Run(Motor_R,normal_speed_pwm);
		}
		else                            //������𶯽���
		{
			start_over_sign = 1;
		}
	}
	
	return start_over_sign; 
}


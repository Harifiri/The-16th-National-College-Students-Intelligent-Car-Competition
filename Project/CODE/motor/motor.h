/*******************************************************************************
	*	@file:		motor.h
	*	@brief:		
	*	@author:	whut
	*	@date:		2019-08-25
	*	@function:
							Motor_Init(void)										//初始化各个通道PWM
							Motor_Run_L(int16 duty)             //电机PWM控制函数
							Motor_Run_R(int16 duty)							
							motor_soft_start(int speed_normal)	//电机软起动
*******************************************************************************/
#ifndef __MOTOR_H__
#define __MOTOR_H__
#include "headfile.h"

#define		motor_freq			      20000			    //定义电机PWM频率
#define		duty_max				      50000					//定义电机最大PWM值

#define PWM_TIM				TIM_2
#define PWM_CH1				TIM_2_CH3_A02
#define PWM_CH2				TIM_2_CH4_A03
#define PWM_CH3				TIM_2_CH2_A01
#define PWM_CH4				TIM_2_CH1_A00

typedef enum{
	Motor_L,
	Motor_R,
}Motor_e;

void Motor_Init(void);
void Motor_Run(Motor_e motor,int duty);
char motor_soft_start(int speed_normal);


#endif

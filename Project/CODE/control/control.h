#ifndef __CONTROL_H__
#define __CONTROL_H__

#include "headfile.h"

typedef struct//速度环
{
	float kp;
  float ki;
	float err_new;
	float err_integral;
	float err_integral_max;
	int speed_new;
	int speed_last;
	int speed_set;
	int speed_max;
	float control_out;
	float control_out_old;
	float control_out_new;
	float control_out_delta;
	float control_out_max;
	float control_out_min;
}SpeedCrl_Typedef;

typedef struct//直立环
{
	//外环参数
	float outer_kp;
	float outer_kd;
	float angle_offset;
	float angle_error;
	float angle_error_last;
	float inner_expect;
	//内环参数
	float inner_kp;
	float inner_ki;
	float gyro_error;
	float gyro_integral;
	int control_out;
}AngleCrl_Typedef;

typedef struct//方向环
{
	float outer_kp;
	float outer_kd;
	float err_new;
	float err_last;
	float inner_expect;
	float inner_expect_max;
	
	float inner_kp;
	float inner_ki;
	float gyro_now;
	float gyro_error;
	float gyro_integral;
	int control_out;
	int control_out_last;
	int control_delta;
	int control_delta_max;
	int control_out_max;
}DirectionCrl_Typedef;

typedef struct
{
	int LeftMotorOut;
	int RightMotorOut;
	int LeftDeadZone;
	int RightDeadZone;
}MotorCrl_Typedef;

void Control(void);                     //总体控制函数
void Control_init(void);                //各参数初始化
void SpeedConrtol(void);     	        //速度控制函数
void SpeedSmoothOutput(void);           //速度平滑输出函数
void AngleInnerConrtol(void);			//直立控制函数
void AngleOuterConrtol(void);	
void DirectionOuterConrtol(void);			//方向控制函数
void DirectionInnerConrtol(void);
void Brake(void);
void MotorConrtol(void);                //电机输出控制函数

#endif

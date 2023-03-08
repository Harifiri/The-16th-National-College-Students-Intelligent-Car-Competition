#ifndef __CONTROL_H__
#define __CONTROL_H__

#include "headfile.h"

typedef struct//�ٶȻ�
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

typedef struct//ֱ����
{
	//�⻷����
	float outer_kp;
	float outer_kd;
	float angle_offset;
	float angle_error;
	float angle_error_last;
	float inner_expect;
	//�ڻ�����
	float inner_kp;
	float inner_ki;
	float gyro_error;
	float gyro_integral;
	int control_out;
}AngleCrl_Typedef;

typedef struct//����
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

void Control(void);                     //������ƺ���
void Control_init(void);                //��������ʼ��
void SpeedConrtol(void);     	        //�ٶȿ��ƺ���
void SpeedSmoothOutput(void);           //�ٶ�ƽ���������
void AngleInnerConrtol(void);			//ֱ�����ƺ���
void AngleOuterConrtol(void);	
void DirectionOuterConrtol(void);			//������ƺ���
void DirectionInnerConrtol(void);
void Brake(void);
void MotorConrtol(void);                //���������ƺ���

#endif

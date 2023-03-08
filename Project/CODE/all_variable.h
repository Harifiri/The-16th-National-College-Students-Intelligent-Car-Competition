#ifndef __ALL_VARIABLE_H__
#define __ALL_VARIABLE_H__

#include "headfile.h"
#include "inductor.h"
#include "control.h"

extern uint8 LeftEdge_X[MT9V03X_H]; 										//存储左右边线横坐标
extern uint8 RightEdge_X[MT9V03X_H];
extern uint8 Middle_X[MT9V03X_H];						//储存中线

extern uint8 CrossFlag;
extern uint8 LeftRoundFlag;
extern uint8 RightRoundFlag;
extern uint8 ByroadFlag;
extern uint8 RampFlag;
extern uint8 ZebraFlag;
extern uint8 L_cross_flag,R_cross_flag;
extern uint8 start_flag;
extern uint8 L_state,R_state;
extern uint8 Bys_flag;

extern AngleCrl_Typedef 		Angle;
extern SpeedCrl_Typedef 		Speed;
extern DirectionCrl_Typedef Direction;

extern InductorTypedef AL;
extern InductorTypedef AM;
extern InductorTypedef AR;

extern int Speed_L;
extern int Speed_R;
extern int Road;

extern float Gyro_x,Gyro_y,Gyro_z;
extern float Acc_x,Acc_y,Acc_z;
extern float Pitch,Yaw,Roll;
extern float IMU_Kp,IMU_Ki;
extern float exInteg,eyInteg,ezInteg;
//extern float gyro_ratio;

extern float DataBuf[8];//发送给上位机的数组
extern uint8 byteBuf[8];
extern int16 int16Buf[8];


#endif
#ifndef __IMU_H__
#define __IMU_H__
#include "headfile.h"

//void IMUupdate(void);
void Get_gyro_data(void);
void Get_acc_data(void);
void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az);
#endif

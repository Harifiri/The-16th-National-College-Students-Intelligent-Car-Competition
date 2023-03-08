#ifndef __ENCODER_H__
#define __ENCODER_H__


#include "headfile.h"

#define ENCODER_L					TIM_3
#define ENCODER_L_A				TIM_3_ENC1_B04
#define ENCODER_L_B				TIM_3_ENC2_B05
#define ENCODER_R					TIM_4
#define ENCODER_R_A				TIM_4_ENC1_B06
#define ENCODER_R_B				TIM_4_ENC2_B07

void encoder_init(void);
void Get_speed(void);
	
#endif

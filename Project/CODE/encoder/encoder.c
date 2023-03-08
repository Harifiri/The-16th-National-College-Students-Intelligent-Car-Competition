#include "headfile.h"
#include "encoder.h"

/***********************************************************
	*	函数名称：encoder_init(void)
	*	函数说明：初始化编码器的两个通道
	*	函数参数：none
	*	参数返回：void
	*	修改日期：2019-11-28
***********************************************************/
void encoder_init(void)
{
	tim_encoder_init(ENCODER_L, ENCODER_L_A, ENCODER_L_B);							// 初始化正交编码器采集
	tim_encoder_init(ENCODER_R, ENCODER_R_A, ENCODER_R_B);							// 初始化正交编码器采集
}

/***********************************************************
	*	函数名称：Get_speed(void)
	*	函数说明：获取编码器速度
	*	函数参数：none
	*	参数返回：void
	*	修改日期：2019-11-28
***********************************************************/
int Speed_L = 0;
int Speed_R = 0;

uint8 start_buf[2]={0x20,0x21};
void Get_speed(void)
{
	Speed.speed_last = Speed.speed_new;
	
	Speed_R = -tim_encoder_get_count(ENCODER_R);					// 采集对应编码器数据	
	Speed_L = tim_encoder_get_count(ENCODER_L);					// 采集对应编码器数据
	
	Speed.speed_new = (Speed_L + Speed_R)>>1;
	//Speed.speed_new = 0.75f*((Speed_L + Speed_R)>>1)+0.25f*Speed.speed_last;
	
//	if( ABS(Speed.speed_new-Speed.speed_last)>700 && start_flag==3)
//		Speed.speed_new = Speed.speed_last;
	
	if( ABS(Speed.speed_new-Speed.speed_last)>20000 && start_flag<4)
		Speed.speed_new = Speed.speed_last;
	
	if(LeftRoundFlag==1 || LeftRoundFlag==5 || ByroadFlag==2 ||RampFlag==3)
		Road += (Speed_L+Speed_R)>>4;
	else
		Road = 0;
	
//     if((Speed_L+Speed_R)/2>=2000&&Road>=10000)
//		 {
//		 start_flag = 4;
//		 }
}

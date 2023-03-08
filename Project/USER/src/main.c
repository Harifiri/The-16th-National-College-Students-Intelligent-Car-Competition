/*********************************************************************************************************************
* COPYRIGHT NOTICE
* Copyright (c) 2019,逐飞科技
* All rights reserved.
* 技术讨论QQ群：一群：179029047(已满)  二群：244861897
*
* 以下所有内容版权均属逐飞科技所有，未经允许不得用于商业用途，
* 欢迎各位使用并传播本程序，修改内容时必须保留逐飞科技的版权声明。
*
* @file				main
* @company			成都逐飞科技有限公司
* @author			逐飞科技(QQ3184284598)
* @version			查看doc内version文件 版本说明
* @Software			IAR 8.3 or MDK 5.24
* @Target core		MM32F3277
* @Taobao			https://seekfree.taobao.com/
* @date				2021-02-22
********************************************************************************************************************/
#include "headfile.h"

uint8 start_flag=0;
uint8 isp_flag=0;

uint8 CrossFlag=0;
uint8 ZebraFlag=0;
uint8 LeftRoundFlag=0;
uint8 RightRoundFlag=0;
uint8 ByroadFlag=2;
uint8 RampFlag=0;
uint8 SpecialCount=0;
uint8 Bys_flag = 0;
//extern uint16 t1_ms,t1_s,t8_ms,t8_s;
//int16 x1,x2;

extern float q0,q1,q2,q3;
int main(void)
{
	uint8 i=0;
	board_init(true);																// 初始化 debug 输出串口

	allInit();
	openIsr();
	
	//Correction();
	
	while(true)
	{		
		if(mt9v03x_finish_flag!=0)
		{
			Get_AD_data();
			Get_Road();
			mt9v03x_finish_flag = 0;
			//send();
		}
		
		if(isp_flag || start_flag>=4)
		{
//			ips114_showint16(40,5,Speed_L);
//			ips114_showint16(40,6,Speed_R);
			ips114_showint16(0,5,LeftRoundFlag);
			ips114_showint16(50,5,RightRoundFlag);
			ips114_showint16(100,5,ByroadFlag);
			ips114_showint16(150,5,RampFlag);
			ips114_showint16(0,7,AM.Final);
			ips114_showint16(50,7,ZebraFlag);
			ips114_showint16(100,7,L_state);
			ips114_showint16(150,7,R_state);
			ips114_showfloat(192,3,Direction.err_new,3,1);
			ips114_showfloat(192,2,Pitch,3,1);
//			ips114_showfloat(192,3,Yaw,3,1);
//			ips114_showfloat(192,4,Roll,3,1);

//			ips114_showfloat(192,1,Direction.gyro_now,1,3);
//			ips114_showfloat(192,2,q1,1,3);
//			ips114_showfloat(192,3,q2,1,3);
//			ips114_showfloat(192,4,q3,1,3);
//			ips114_showfloat(192,0,Pitch,3,1);
//			ips114_showfloat(192,6,Yaw,3,1);
//			ips114_showfloat(192,7,Roll,3,1);
//			ips114_showint16(192,2,distance);
			ips114_showint16(0,6,SpecialCount);
//			ips114_showint16(192,6,RightStraightLineCount(15,45));
//			ips114_showint16(192,5,icm_gyro_x);
//			ips114_showint16(192,6,icm_gyro_y);
//			ips114_showint16(192,7,icm_gyro_z);
//			ips114_showfloat(192,4,Yaw,3,1);
//			ips114_showfloat(192,2,Acc_x,3,1);
//			ips114_showfloat(192,3,Acc_y,3,1);
//			ips114_showint16(40,5,(RightEdge_X[5]-LeftEdge_X[5]));
//			ips114_showint16(40,6,(RightEdge_X[55]-LeftEdge_X[55]));			
//			ips114_showfloat(192,2,Yaw,3,1);
//			ips114_showfloat(0,6,AL.Final,4,1);
//			ips114_showfloat(50,6,AR.Final,4,1);
//			ips114_showint16(192,0,AL.Average);
//			ips114_showint16(24,4,Pitch);
//			ips114_showint16(24,5,Yaw);
//			ips114_showint16(24,6,Roll);
//			ips114_showint16(192,3,AR.Average);
//			ips114_showfloat(192,5,Direction.err_new,4,1);
//			ips114_showfloat(0,7,AL.Final+AR.Final,4,1);
			ips114_displayimage032(image[0],MT9V03X_W,MT9V03X_H);
			for(i=60;i<=120;i++)
			{
				ips114_drawpoint(i,50,RED);
				ips114_drawpoint(i,30,RED);
				ips114_drawpoint(i,40,RED);
        ips114_drawpoint(i,20,RED);
			}
//			isp_flag=0;
		}

	}
}

//			ips114_showfloat(24,1,AL.Final,4,1);
//			ips114_showfloat(24,2,BL.Final,4,1);
//			ips114_showfloat(24,3,BR.Final,4,1);
//			ips114_showfloat(24,4,AR.Final,4,1);

//			ips114_showint16(24,1,AL.Average);
//			ips114_showint16(24,2,BL.Average);
//			ips114_showint16(24,3,BR.Average);
//			ips114_showint16(24,4,AR.Average);
//			ips114_showfloat(32,1,q0,2,3);
//			ips114_showfloat(32,2,q1,2,3);
//			ips114_showfloat(32,3,q2,2,3);
//			ips114_showfloat(32,4,q3,2,3);
//			ips114_showfloat(32,6,Yaw,3,1);
//			ips114_showint16(40,5,Speed_L);
//			ips114_showint16(40,6,Speed_R);
//ips114_showint16(0,7,t1_s);

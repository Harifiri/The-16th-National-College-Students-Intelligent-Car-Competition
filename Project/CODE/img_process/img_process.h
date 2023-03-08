/*******************************************************************************
	*	@file:		img_process.h
	*	@brief:		摄像头图像处理程序
	*	@author:	whut
	*	@date:		2019-04-26
*******************************************************************************/
#ifndef __IMG_PROCESS_H__
#define __IMG_PROCESS_H__

#include "headfile.h"
#include "SEEKFREE_MT9V03X.h"

void Correction(void);
void Get_Road(void);         //获取道路信息总函数
void grey_scale(void);
void VariableReset(void);    //变量清零
void TrackBothEdge(void);    //找左右边线

void bend_detect();
void LeftRound(void);
void RightRound(void);
void Byroad(void);
void Cross(void);
void Zebra(void);
void Ramp(void);
float erchengfa(int startline,int endline,uint8 type);
uint8 LeftStraightLineCount(uint8 start_row,uint8 final_row);
uint8 RightStraightLineCount(uint8 start_row,uint8 final_row);
uint8 Middle_LineCount(uint8 start_row,uint8 final_row);	
float err_calculation(uint8 start_row,uint8 final_row);

int regression(int startline,int endline,int16 y[]);//最小二乘法求斜率
int m_sqrt(int x);                                  //开方函数

#endif


/*******************************************************************************
	*	@file:		init.c
	*	@brief:		程序初始化主文件
	*	@author:	whut
	*	@date:		2019-7-3
	*	@function:
							allInit(void)		初始化总程序
							openisr (void)	开中断总程序
*******************************************************************************/

#include "headfile.h"


void allInit(void)
{
	Motor_Init(); 								//     TIM_2	
	ips114_init();
	mt9v03x_init();
	seekfree_wireless_init();
	Indoctor_init();
	encoder_init();								//    TIM_3 /	TIM_4
	icm20602_init_spi();
	Control_init();
	//laser_init();
	//key_init();
	systick_delay_ms(50);
}

/***********************************************************
	*	函数名称：openIsr (void)
	*	函数说明：开中断总程序
	*	函数参数：none
	*	参数返回：void
	*	修改日期：2019-11-28
***********************************************************/
void openIsr (void)
{
	tim_interrupt_init_ms(TIM_8, 1, 0x01, 0x01);
	//tim_interrupt_init_ms(TIM_6, 5, 0x02, 0x00);
}


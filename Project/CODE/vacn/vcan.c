/*******************************************************************************
	*	@file:		vcan.c
	*	@brief:		程序初始化主文件
	*	@author:	whut
	*	@date:		2019-12-17
	*	@function:
							vcan_init(void)		初始化串口2
							openisr (void)	开中断总程序
							vcan_sendware    发送数字到山外虚拟示波器
*******************************************************************************/
#include "vcan.h"
#include "headfile.h"


/***********************************************************
@函数名：vcan_sendware
@入口参数：unsigned char *wareaddr, uint32 waresize
@出口参数：无
功能描述：山外虚拟示波器发送
@作者：whut
@日期：2019年12月17日
*************************************************************/
float DataBuf[8];
uint8 byteBuf[8];
int16 int16Buf[8];
void vcan_sendware(uint8 *wareaddr, uint32 waresize)//山外发送波形
{
	uint8 num,i;
	
	#define CMD_WARE     3
	
  uint8 cmdf[2] = {CMD_WARE, ~CMD_WARE};//帧头
  uint8 cmdr[2] = {~CMD_WARE, CMD_WARE};//帧尾
	
  uart_putbuff(WIRELESS_UART, cmdf,sizeof(cmdf));
	
	if(waresize%4==0)
	{
		num=waresize/4;
		for(i=0;i<num;i++)
			uart_putbuff(WIRELESS_UART, wareaddr+i*4,4);
	}
	else
		uart_putbuff(WIRELESS_UART, wareaddr,waresize);
	
  uart_putbuff(WIRELESS_UART, cmdr,sizeof(cmdr));
}



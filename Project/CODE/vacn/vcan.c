/*******************************************************************************
	*	@file:		vcan.c
	*	@brief:		�����ʼ�����ļ�
	*	@author:	whut
	*	@date:		2019-12-17
	*	@function:
							vcan_init(void)		��ʼ������2
							openisr (void)	���ж��ܳ���
							vcan_sendware    �������ֵ�ɽ������ʾ����
*******************************************************************************/
#include "vcan.h"
#include "headfile.h"


/***********************************************************
@��������vcan_sendware
@��ڲ�����unsigned char *wareaddr, uint32 waresize
@���ڲ�������
����������ɽ������ʾ��������
@���ߣ�whut
@���ڣ�2019��12��17��
*************************************************************/
float DataBuf[8];
uint8 byteBuf[8];
int16 int16Buf[8];
void vcan_sendware(uint8 *wareaddr, uint32 waresize)//ɽ�ⷢ�Ͳ���
{
	uint8 num,i;
	
	#define CMD_WARE     3
	
  uint8 cmdf[2] = {CMD_WARE, ~CMD_WARE};//֡ͷ
  uint8 cmdr[2] = {~CMD_WARE, CMD_WARE};//֡β
	
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



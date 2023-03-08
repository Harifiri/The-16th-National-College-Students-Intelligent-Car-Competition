#ifndef __LASER_H__
#define __LASER_H__

#include "headfile.h"

#define LASER_UART				UART_3												// ����ת����ģ�� ��ʹ�õ��Ĵ���
#define LASER_UART_TX			UART3_TX_B10
#define LASER_UART_RX			UART3_RX_B11
#define LASER_UART_BAUD		115200

void laser_init(void);
void laser_uart_callback(void);
	
#endif

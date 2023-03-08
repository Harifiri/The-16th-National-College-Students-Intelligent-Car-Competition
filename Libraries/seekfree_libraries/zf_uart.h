/*********************************************************************************************************************
* COPYRIGHT NOTICE
* Copyright (c) 2019,��ɿƼ�
* All rights reserved.
* ��������QQȺ��һȺ��179029047(����)  ��Ⱥ��244861897
*
* �����������ݰ�Ȩ������ɿƼ����У�δ��������������ҵ��;��
* ��ӭ��λʹ�ò������������޸�����ʱ���뱣����ɿƼ��İ�Ȩ������
*
* @file				uart
* @company			�ɶ���ɿƼ����޹�˾
* @author			��ɿƼ�(QQ3184284598)
* @version			�鿴doc��version�ļ� �汾˵��
* @Software			IAR 8.3 or MDK 5.24
* @Target core		MM32F3277
* @Taobao			https://seekfree.taobao.com/
* @date				2021-02-22
********************************************************************************************************************/

#ifndef _zf_uart_h
#define _zf_uart_h

#include <stdio.h>
#include "common.h"
#include "hal_uart.h"

//ö�ٴ�������	��ö�ٶ��岻�����û��޸�
typedef enum
{
// UART 1 ���� TX
	UART1_TX_A09					= 0x709,										// 0x700[AF7] 0x000[A group] 0x009[pin  9]
	UART1_TX_B06					= 0x716,										// 0x700[AF7] 0x010[B group] 0x006[pin  6]
// UART 1 ���� RX
	UART1_RX_A10					= 0x70A,										// 0x700[AF7] 0x000[A group] 0x00A[pin 10]
	UART1_RX_B07					= 0x717,										// 0x700[AF7] 0x010[B group] 0x007[pin  7]

// UART 2 ���� TX
	UART2_TX_A02					= 0x702,										// 0x700[AF7] 0x000[A group] 0x002[pin  2]
	UART2_TX_D05					= 0x735,										// 0x700[AF7] 0x030[D group] 0x005[pin  5]
// UART 2 ���� RX
	UART2_RX_A03					= 0x703,										// 0x700[AF7] 0x000[A group] 0x003[pin  3]
	UART2_RX_D06					= 0x736,										// 0x700[AF7] 0x030[D group] 0x006[pin  6]

// UART 3 ���� TX
	UART3_TX_B10					= 0x71A,										// 0x700[AF7] 0x010[B group] 0x00A[pin 10]
	UART3_TX_C10					= 0x72A,										// 0x700[AF7] 0x020[C group] 0x00A[pin 10]
	UART3_TX_D08					= 0x738,										// 0x700[AF7] 0x030[D group] 0x008[pin  8]
// UART 3 ���� RX
	UART3_RX_B11					= 0x71B,										// 0x700[AF7] 0x010[B group] 0x00B[pin 11]
	UART3_RX_C11					= 0x72B,										// 0x700[AF7] 0x020[C group] 0x00B[pin 11]
	UART3_RX_D09					= 0x739,										// 0x700[AF7] 0x030[D group] 0x009[pin  9]

// UART 4 ���� TX
	UART4_TX_A00					= 0x800,										// 0x800[AF8] 0x000[A group] 0x000[pin  0]
	UART4_TX_C10					= 0x82A,										// 0x800[AF8] 0x020[C group] 0x00A[pin 10]
// UART 4 ���� RX
	UART4_RX_A01					= 0x801,										// 0x800[AF8] 0x000[A group] 0x001[pin  1]
	UART4_RX_C11					= 0x82B,										// 0x800[AF8] 0x020[C group] 0x00B[pin 11]

// UART 5 ���� TX
	UART5_TX_A04					= 0x804,										// 0x800[AF8] 0x000[A group] 0x004[pin  4]
	UART5_TX_C12					= 0x82C,										// 0x800[AF8] 0x020[C group] 0x00C[pin 12]
// UART 5 ���� RX
	UART5_RX_A05					= 0x805,										// 0x800[AF8] 0x000[A group] 0x005[pin  5]
	UART5_RX_D02					= 0x832,										// 0x800[AF8] 0x030[D group] 0x002[pin  2]

// UART 6 ���� TX
	UART6_TX_B00					= 0x810,										// 0x800[AF8] 0x010[B group] 0x000[pin  0]
	UART6_TX_C06					= 0x826,										// 0x800[AF8] 0x020[C group] 0x006[pin  6]
// UART 6 ���� RX
	UART6_RX_B01					= 0x811,										// 0x800[AF8] 0x010[B group] 0x001[pin  1]
	UART6_RX_C07					= 0x827,										// 0x800[AF8] 0x020[C group] 0x007[pin  7]

// UART 7 ���� TX
	UART7_TX_B06					= 0x816,										// 0x800[AF8] 0x010[B group] 0x006[pin  6]
	UART7_TX_E08					= 0x848,										// 0x800[AF8] 0x040[E group] 0x008[pin  8]
// UART 7 ���� RX
	UART7_RX_B07					= 0x817,										// 0x800[AF8] 0x010[B group] 0x007[pin  7]
	UART7_RX_E07					= 0x847,										// 0x800[AF8] 0x040[E group] 0x007[pin  7]

// UART 8 ���� TX
	UART8_TX_D00					= 0x830,										// 0x800[AF8] 0x030[D group] 0x000[pin  0]
	UART8_TX_E01					= 0x841,										// 0x800[AF8] 0x040[E group] 0x001[pin  1]
// UART 8 ���� RX
	UART8_RX_D01					= 0x831,										// 0x800[AF8] 0x030[D group] 0x001[pin  1]
	UART8_RX_E00					= 0x840,										// 0x800[AF8] 0x040[E group] 0x000[pin  0]
}UARTPIN_enum;

//ö�ٴ��ں�	��ö�ٶ��岻�����û��޸�
typedef enum
{
	UART_1,
	UART_2,
	UART_3,
	UART_4,
	UART_5,
	UART_6,
	UART_7,
	UART_8,
}UARTN_enum;

extern UART_TypeDef *uart_index[8];

void	uart_init		(UARTN_enum uartn, uint32 baud, UARTPIN_enum tx_pin, UARTPIN_enum rx_pin);
void	uart_putchar	(UARTN_enum uartn, uint8 dat);
void	uart_putbuff	(UARTN_enum uartn, uint8 *buff, uint32 len);
void	uart_putstr		(UARTN_enum uartn, const uint8 *str);
void	uart_getchar	(UARTN_enum uartn, uint8 *dat);
uint8	uart_query		(UARTN_enum uartn, uint8 *dat);
void	uart_tx_irq		(UARTN_enum uartn, uint32 status);
void	uart_rx_irq		(UARTN_enum uartn, uint32 status);

#endif

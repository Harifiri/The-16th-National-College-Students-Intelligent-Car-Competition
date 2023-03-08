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

#include "zf_uart.h"
#include "zf_gpio.h"

UART_TypeDef *uart_index[8] = {UART1, UART2, UART3, UART4, UART5, UART6, UART7, UART8};
IRQn_Type uart_irq[8] = {UART1_IRQn, UART2_IRQn, UART3_IRQn, UART4_IRQn, UART5_IRQn, UART6_IRQn, UART7_IRQn, UART8_IRQn};

//-------------------------------------------------------------------------------------------------------------------
// @brief		UART ���ų�ʼ�� �ڲ�����
// @param		tx_pin			ѡ�� TX ����
// @param		rx_pin			ѡ�� RX ����
// @return		void
// Sample usage:				uart_pin_init(tx_pin, rx_pin);
//-------------------------------------------------------------------------------------------------------------------
static void uart_pin_init (UARTPIN_enum tx_pin, UARTPIN_enum rx_pin)
{
	afio_init((PIN_enum)(tx_pin &0xff), GPO, (GPIOAF_enum)((tx_pin &0xf00)>>8), GPO_AF_PUSH_PUL);// ��ȡ��ӦIO���� AF���ܱ���
	afio_init((PIN_enum)(rx_pin &0xff), GPI, (GPIOAF_enum)((rx_pin &0xf00)>>8), GPI_FLOATING_IN);// ��ȡ��ӦIO���� AF���ܱ���
}

//-------------------------------------------------------------------------------------------------------------------
// @brief		���ڳ�ʼ��
// @param		uartn			����ģ���(USART_1,USART_2)
// @param		baud			���ڲ�����
// @param		tx_pin			���ڷ�������
// @param		rx_pin			���ڽ�������
// @return		void			
// Sample usage:				uart_init(USART_1,115200,UART1_TX_A09,UART1_RX_A10);			// ��ʼ������1 ������115200 ��������ʹ��PA09 ��������ʹ��PA10
//-------------------------------------------------------------------------------------------------------------------
void uart_init(UARTN_enum uartn, uint32 baud, UARTPIN_enum tx_pin, UARTPIN_enum rx_pin)
{
	uart_pin_init(tx_pin, rx_pin);																// ���ų�ʼ��

	switch(uartn)
	{
		case UART_1:
			RCC->APB2ENR |= RCC_APB2ENR_UART1;
			RCC->APB2RSTR |= (RCC_APB2RSTR_UART1);
			RCC->APB2RSTR &= ~(RCC_APB2RSTR_UART1);
			break;
		case UART_2:
			RCC->APB1ENR |= RCC_APB1ENR_UART2;
			RCC->APB1RSTR |= (RCC_APB1RSTR_UART2);
			RCC->APB1RSTR &= ~(RCC_APB1RSTR_UART2);
			break;
		case UART_3:
			RCC->APB1ENR |= RCC_APB1ENR_UART3;
			RCC->APB1RSTR |= (RCC_APB1RSTR_UART3);
			RCC->APB1RSTR &= ~(RCC_APB1RSTR_UART3);
			break;
		case UART_4:
			RCC->APB1ENR |= RCC_APB1ENR_UART4;
			RCC->APB1RSTR |= (RCC_APB1RSTR_UART4);
			RCC->APB1RSTR &= ~(RCC_APB1RSTR_UART4);
			break;
		case UART_5:
			RCC->APB1ENR |= RCC_APB1ENR_UART5;
			RCC->APB1RSTR |= (RCC_APB1RSTR_UART5);
			RCC->APB1RSTR &= ~(RCC_APB1RSTR_UART5);
			break;
		case UART_6:
			RCC->APB2ENR |= RCC_APB2ENR_UART6;
			RCC->APB2RSTR |= (RCC_APB2RSTR_UART6);
			RCC->APB2RSTR &= ~(RCC_APB2RSTR_UART6);
			break;
		case UART_7:
			RCC->APB1ENR |= RCC_APB1ENR_UART7;
			RCC->APB1RSTR |= (RCC_APB1RSTR_UART7);
			RCC->APB1RSTR &= ~(RCC_APB1RSTR_UART7);
			break;
		case UART_8:
			RCC->APB1ENR |= RCC_APB1ENR_UART8;
			RCC->APB1RSTR |= (RCC_APB1RSTR_UART8);
			RCC->APB1RSTR &= ~(RCC_APB1RSTR_UART8);
			break;
	}
	uart_index[uartn]->BRR = (SystemCoreClock / baud) / 16;										// ���ò�����
	uart_index[uartn]->FRA = (SystemCoreClock / baud) % 16;										// ���ò�����
	uart_index[uartn]->CCR |= UART_CCR_CHAR;													// 8bits ����λ
	uart_index[uartn]->GCR |= UART_GCR_TX | UART_GCR_RX | UART_GCR_UART;						// ʹ�� TX RX UART
}

//-------------------------------------------------------------------------------------------------------------------
// @brief		�����ֽ����
// @param		uartn			����ģ���(USART_1,USART_2)
// @param		dat				��Ҫ���͵��ֽ�
// @return		void        
// Sample usage:				uart_putchar(USART_1,0xA5);										// ����1����0xA5
//-------------------------------------------------------------------------------------------------------------------
void uart_putchar(UARTN_enum uartn, uint8 dat)
{
	uart_index[uartn]->TDR = dat;																// д�뷢������
	while(!(uart_index[uartn]->CSR & UART_CSR_TXC));											// �ȴ��������
}

//-------------------------------------------------------------------------------------------------------------------
// @brief		���ڷ�������
// @param		uartn			����ģ���(USART_1,USART_2)
// @param		*buff			Ҫ���͵������ַ
// @param		len				���ͳ���
// @return		void
// Sample usage:				uart_putbuff(USART_1,&a[0],5);
//-------------------------------------------------------------------------------------------------------------------
void uart_putbuff(UARTN_enum uartn, uint8 *buff, uint32 len)
{
	while(len)																					// ѭ����������
	{
		uart_index[uartn]->TDR = *buff++;														// д�뷢������
		while(!(uart_index[uartn]->CSR & UART_CSR_TXC));										// �ȴ��������
		len--;
	}
}

//-------------------------------------------------------------------------------------------------------------------
// @brief		���ڷ����ַ���
// @param		uartn			����ģ���(USART_1,USART_2)
// @param		*str			Ҫ���͵��ַ�����ַ
// @return		void
// Sample usage:				uart_putstr(USART_1,"i lvoe you"); 
//-------------------------------------------------------------------------------------------------------------------
void uart_putstr(UARTN_enum uartn, const uint8 *str)
{
	while(*str)																					// һֱѭ������β
	{
		uart_putchar(uartn, *str++);															// ��������
	}
}

//-------------------------------------------------------------------------------------------------------------------
// @brief		��ȡ���ڽ��յ����ݣ�whlie�ȴ���
// @param		uartn			����ģ���(USART_1,USART_2)
// @param		*dat			�������ݵĵ�ַ
// @return		void        
// Sample usage:				uint8 dat; uart_getchar(USART_1,&dat);							// ���մ���1����  ������dat������
//-------------------------------------------------------------------------------------------------------------------
void uart_getchar(UARTN_enum uartn, uint8 *dat)
{
	while(!(uart_index[uartn]->CSR & UART_CSR_RXAVL));											// �ȴ���ȡ��һ������
	*dat = (uint8)uart_index[uartn]->RDR;
}

//-------------------------------------------------------------------------------------------------------------------
// @brief		��ȡ���ڽ��յ����ݣ���ѯ���գ�
// @param		uartn			����ģ���(USART_1,USART_2)
// @param		*dat			�������ݵĵ�ַ
// @return		uint8			1�����ճɹ�   0��δ���յ�����
// Sample usage:				uint8 dat; uart_query(USART_1,&dat);							// ���մ���1����  ������dat������
//-------------------------------------------------------------------------------------------------------------------
uint8 uart_query(UARTN_enum uartn, uint8 *dat)
{
	if(uart_index[uartn]->CSR & UART_CSR_RXAVL)													// ��ȡ��һ������
	{
		*dat = (uint8)uart_index[uartn]->RDR;													// �洢һ������
		return 1;
	}
	return 0;
}

//-------------------------------------------------------------------------------------------------------------------
// @brief		���ڷ����ж�����
// @param		uartn			����ģ���(USART_1,USART_2)
// @param		status			1�����ж�   0���ر��ж�
// @return		void        
// Sample usage:				uart_tx_irq(USART_1,1);											// �򿪴���1��������ж�
//-------------------------------------------------------------------------------------------------------------------
void uart_tx_irq(UARTN_enum uartn, uint32 status)
{
	if(status)
		uart_index[uartn]->IER |= UART_IER_TX;													// ʹ�ܷ�������ж�
	else
		uart_index[uartn]->IER &= ~(UART_IER_TX);												// �رշ�������ж�

	nvic_init(uart_irq[uartn], 0x00, 0x00, (FunctionalState)status);
}

//-------------------------------------------------------------------------------------------------------------------
// @brief		���ڽ����ж�����
// @param		uartn			����ģ���(USART_1,USART_2)
// @param		status			1�����ж�   0���ر��ж�
// @return		void        
// Sample usage:				uart_rx_irq(USART_1,1);											// �򿪴���1��������ж�
//-------------------------------------------------------------------------------------------------------------------
void uart_rx_irq(UARTN_enum uartn, uint32 status)
{
	if(status)
		uart_index[uartn]->IER |= UART_IER_RX;													// ʹ�ܽ�������ж�
	else
		uart_index[uartn]->IER &= ~(UART_IER_RX);												// �رս�������ж�

	nvic_init(uart_irq[uartn], 0x00, 0x00, (FunctionalState)status);
}

//--------------------------------------------------------------------------------				// printf �ض��� �˲��ֲ������û�����
#if defined(__GNUC__)
#define PUTCHAR_PROTOTYPE s32 __io_putchar(s32 ch)
#else
#define PUTCHAR_PROTOTYPE s32 fputc(s32 ch, FILE *f)
#endif

#if defined(__ICCARM__)
PUTCHAR_PROTOTYPE {
	while((UART1->CSR & UART_CSR_TXC) == 0); //The loop is sent until it is finished
	UART1->TDR = (ch & (u16)0x00FF);
	return ch;
}
#else
s32 fputc(s32 ch, FILE* f)
{
	while((UART1->CSR & UART_CSR_TXC) == 0); //The loop is sent until it is finished
	UART1->TDR = (ch & (u16)0x00FF);
	return ch;
}

#endif
//--------------------------------------------------------------------------------			// printf �ض��� �˲��ֲ������û�����

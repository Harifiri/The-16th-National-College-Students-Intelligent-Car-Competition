/*********************************************************************************************************************
* COPYRIGHT NOTICE
* Copyright (c) 2019,��ɿƼ�
* All rights reserved.
* ��������QQȺ��һȺ��179029047(����)  ��Ⱥ��244861897
*
* �����������ݰ�Ȩ������ɿƼ����У�δ��������������ҵ��;��
* ��ӭ��λʹ�ò������������޸�����ʱ���뱣����ɿƼ��İ�Ȩ������
*
* @file				isr
* @company			�ɶ���ɿƼ����޹�˾
* @author			��ɿƼ�(QQ3184284598)
* @version			�鿴doc��version�ļ� �汾˵��
* @Software			IAR 8.3 or MDK 5.28
* @Target core		MM32F3277
* @Taobao			https://seekfree.taobao.com/
* @date				2021-02-22
********************************************************************************************************************/

#include "headfile.h"
#include "isr.h"

uint16 t8_ms=0,t1_s=0,t6_ms=0,t8_s=0;


extern uint8 isp_flag;
void TIM8_UP_IRQHandler (void)
{
	uint32 state = TIM8->SR;														// ��ȡ�ж�״̬

	Control();
		
	TIM8->SR &= ~state;																// ����ж�״̬
}

extern uint8 LeftEdge_X[MT9V03X_H]; 										//�洢���ұ��ߺ�����
extern uint8 RightEdge_X[MT9V03X_H];
void TIM6_IRQHandler (void)
{
	uint32 state = TIM6->SR;														// ��ȡ�ж�״̬
	
	//send();
	static uint8 row0=1;
	
	DataBuf[0] = row0;
	DataBuf[1] = (RightEdge_X[row0]-LeftEdge_X[row0])>>1;
	
	row0++;
	
	if(row0==60)
		row0 = 1;
	
	vcan_sendware((uint8 *)DataBuf,8);

	TIM6->SR &= ~state;																// ����ж�״̬
}

void EXTI4_IRQHandler(void)
{
	// ���������жϱ�־���Ը���ʵ��Ӧ�ý���ɾ��
	EXTI_ClearFlag(EXTI_Line4);														// ��� line4 ������־
}

void EXTI9_5_IRQHandler(void)
{
	// ���������жϱ�־���Ը���ʵ��Ӧ�ý���ɾ��
	if(EXTI_GetITStatus(EXTI_Line5))												// ��� line5 �Ƿ񴥷�
	{
		EXTI_ClearFlag(EXTI_Line5);													// ��� line5 ������־
	}
	if(EXTI_GetITStatus(EXTI_Line6))												// ��� line6 �Ƿ񴥷�
	{
		EXTI_ClearFlag(EXTI_Line6);													// ��� line6 ������־
	}
	if(EXTI_GetITStatus(EXTI_Line7))												// ��� line7 �Ƿ񴥷�
	{
		EXTI_ClearFlag(EXTI_Line7);													// ��� line8 ������־
	}
	if(EXTI_GetITStatus(EXTI_Line8))												// ��� line8 �Ƿ񴥷�
	{
		mt9v03x_vsync();		
		EXTI_ClearFlag(EXTI_Line8);													// ��� line8 ������־
	}
	if(EXTI_GetITStatus(EXTI_Line9))												// ��� line9 �Ƿ񴥷�
	{
		EXTI_ClearFlag(EXTI_Line9);													// ��� line9 ������־
	}
}


void UART8_IRQHandler(void)
{
	if(UART8->ISR & UART_ISR_TX_INTF)												// ���ڷ��ͻ�����ж�
	{
		UART8->ICR |= UART_ICR_TXICLR;												// ����жϱ�־λ
	}
	if(UART8->ISR & UART_ISR_RX_INTF)												// ���ڽ��ջ����ж�
	{
		UART8->ICR |= UART_ICR_RXICLR;												// ����жϱ�־λ
		mt9v03x_uart_callback();											// ���������Ĵ��ڽ��մ���	
	}
}

void UART3_IRQHandler(void)
{
	if(UART3->ISR & UART_ISR_TX_INTF)												// ���ڷ��ͻ�����ж�
	{
		UART3->ICR |= UART_ICR_TXICLR;												// ����жϱ�־λ
	}
	if(UART3->ISR & UART_ISR_RX_INTF)												// ���ڽ��ջ����ж�
	{
		//wireless_uart_callback();
		UART3->ICR |= UART_ICR_RXICLR;												// ����жϱ�־λ
	}
}

void UART4_IRQHandler(void)
{
	if(UART4->ISR & UART_ISR_TX_INTF)												// ���ڷ��ͻ�����ж�
	{
		UART4->ICR |= UART_ICR_TXICLR;												// ����жϱ�־λ
	}
	if(UART4->ISR & UART_ISR_RX_INTF)												// ���ڽ��ջ����ж�
	{
		wireless_uart_callback();
		UART4->ICR |= UART_ICR_RXICLR;												// ����жϱ�־λ
	}
}

void TIM1_UP_IRQHandler(void)
{
	uint32 state = TIM1->SR;														// ��ȡ�ж�״̬	
	TIM1->SR &= ~state;																// ����ж�״̬
}

void TIM2_IRQHandler (void)
{
	uint32 state = TIM2->SR;														// ��ȡ�ж�״̬
	TIM2->SR &= ~state;																// ����ж�״̬
}

void TIM5_IRQHandler (void)
{
	uint32 state = TIM5->SR;														// ��ȡ�ж�״̬
	TIM5->SR &= ~state;																// ����ж�״̬
}

void TIM3_IRQHandler (void)
{
	uint32 state = TIM3->SR;														// ��ȡ�ж�״̬
	TIM3->SR &= ~state;																// ����ж�״̬
}

void TIM4_IRQHandler (void)
{
	uint32 state = TIM4->SR;														// ��ȡ�ж�״̬
	TIM4->SR &= ~state;																// ����ж�״̬
}

void TIM7_IRQHandler (void)
{
	uint32 state = TIM7->SR;														// ��ȡ�ж�״̬
	TIM7->SR &= ~state;																// ����ж�״̬
}

void UART1_IRQHandler(void)
{
	if(UART1->ISR & UART_ISR_TX_INTF)												// ���ڷ��ͻ�����ж�
	{
		UART1->ICR |= UART_ICR_TXICLR;												// ����жϱ�־λ
	}
	if(UART1->ISR & UART_ISR_RX_INTF)												// ���ڽ��ջ����ж�
	{
		UART1->ICR |= UART_ICR_RXICLR;												// ����жϱ�־λ
	}
}

void UART2_IRQHandler(void)
{
	if(UART2->ISR & UART_ISR_TX_INTF)												// ���ڷ��ͻ�����ж�
	{
		UART2->ICR |= UART_ICR_TXICLR;												// ����жϱ�־λ
	}
	if(UART2->ISR & UART_ISR_RX_INTF)												// ���ڽ��ջ����ж�
	{
		UART2->ICR |= UART_ICR_RXICLR;												// ����жϱ�־λ
	}
}

void UART5_IRQHandler(void)
{
	if(UART5->ISR & UART_ISR_TX_INTF)												// ���ڷ��ͻ�����ж�
	{
		UART5->ICR |= UART_ICR_TXICLR;												// ����жϱ�־λ
	}
	if(UART5->ISR & UART_ISR_RX_INTF)												// ���ڽ��ջ����ж�
	{
		UART5->ICR |= UART_ICR_RXICLR;												// ����жϱ�־λ
	}
}

void UART6_IRQHandler(void)
{
	if(UART6->ISR & UART_ISR_TX_INTF)												// ���ڷ��ͻ�����ж�
	{
		UART6->ICR |= UART_ICR_TXICLR;												// ����жϱ�־λ
	}
	if(UART6->ISR & UART_ISR_RX_INTF)												// ���ڽ��ջ����ж�
	{
		UART6->ICR |= UART_ICR_RXICLR;												// ����жϱ�־λ
	}
}

void UART7_IRQHandler(void)
{
	if(UART7->ISR & UART_ISR_TX_INTF)												// ���ڷ��ͻ�����ж�
	{
		UART7->ICR |= UART_ICR_TXICLR;												// ����жϱ�־λ
	}
	if(UART7->ISR & UART_ISR_RX_INTF)												// ���ڽ��ջ����ж�
	{
		UART7->ICR |= UART_ICR_RXICLR;												// ����жϱ�־λ
	}
}

void EXTI0_IRQHandler(void)
{
	// ���������жϱ�־���Ը���ʵ��Ӧ�ý���ɾ��
	EXTI_ClearFlag(EXTI_Line0);														// ��� line0 ������־
}

void EXTI1_IRQHandler(void)
{
	// ���������жϱ�־���Ը���ʵ��Ӧ�ý���ɾ��
	EXTI_ClearFlag(EXTI_Line1);														// ��� line1 ������־
}

void EXTI2_IRQHandler(void)
{
	// ���������жϱ�־���Ը���ʵ��Ӧ�ý���ɾ��
	EXTI_ClearFlag(EXTI_Line2);														// ��� line2 ������־
}

void EXTI3_IRQHandler(void)
{
	// ���������жϱ�־���Ը���ʵ��Ӧ�ý���ɾ��
	EXTI_ClearFlag(EXTI_Line3);														// ��� line3 ������־
}

void EXTI15_10_IRQHandler (void)
{
	// ���������жϱ�־���Ը���ʵ��Ӧ�ý���ɾ��
	if(EXTI_GetITStatus(EXTI_Line10))												// ��� line10 �Ƿ񴥷�
	{
		EXTI_ClearFlag(EXTI_Line10);												// ��� line10 ������־
	}
	if(EXTI_GetITStatus(EXTI_Line11))												// ��� line11 �Ƿ񴥷�
	{
		EXTI_ClearFlag(EXTI_Line11);												// ��� line11 ������־
	}
	if(EXTI_GetITStatus(EXTI_Line12))												// ��� line12 �Ƿ񴥷�
	{
		EXTI_ClearFlag(EXTI_Line12);												// ��� line12 ������־
	}
	if(EXTI_GetITStatus(EXTI_Line13))												// ��� line13 �Ƿ񴥷�
	{
		EXTI_ClearFlag(EXTI_Line13);												// ��� line13 ������־
	}
	if(EXTI_GetITStatus(EXTI_Line14))												// ��� line14 �Ƿ񴥷�
	{
		EXTI_ClearFlag(EXTI_Line14);												// ��� line14 ������־
	}
	if(EXTI_GetITStatus(EXTI_Line15))												// ��� line15 �Ƿ񴥷�
	{
		EXTI_ClearFlag(EXTI_Line15);												// ��� line15 ������־
	}
}

void DMA1_Channel1_IRQHandler(void)
{
	if(SET == DMA_GetFlagStatus(DMA1_FLAG_TC1))										// �жϴ���ͨ��
	{
		DMA_ClearFlag(DMA1_FLAG_TC1);												// ��ո�ͨ���жϱ�־
	}
}

void DMA1_Channel2_IRQHandler(void)
{
	if(SET == DMA_GetFlagStatus(DMA1_FLAG_TC2))										// �жϴ���ͨ��
	{
		DMA_ClearFlag(DMA1_FLAG_TC2);												// ��ո�ͨ���жϱ�־
	}
}

void DMA1_Channel3_IRQHandler(void)
{
	if(SET == DMA_GetFlagStatus(DMA1_FLAG_TC3))										// �жϴ���ͨ��
	{
		DMA_ClearFlag(DMA1_FLAG_TC3);												// ��ո�ͨ���жϱ�־
	}
}

void DMA1_Channel4_IRQHandler(void)
{
	if(SET == DMA_GetFlagStatus(DMA1_FLAG_TC4))										// �жϴ���ͨ��
	{
		DMA_ClearFlag(DMA1_FLAG_TC4);												// ��ո�ͨ���жϱ�־
	
		// �����
		mt9v03x_dma();
	}
}

void DMA1_Channel5_IRQHandler(void)
{
	if(SET == DMA_GetFlagStatus(DMA1_FLAG_TC5))										// �жϴ���ͨ��
	{
		DMA_ClearFlag(DMA1_FLAG_TC5);												// ��ո�ͨ���жϱ�־
	}
}

void DMA1_Channel6_IRQHandler(void)
{
	if(SET == DMA_GetFlagStatus(DMA1_FLAG_TC6))										// �жϴ���ͨ��
	{
		DMA_ClearFlag(DMA1_FLAG_TC6);												// ��ո�ͨ���жϱ�־
	}
}

void DMA1_Channel7_IRQHandler(void)
{
	if(SET == DMA_GetFlagStatus(DMA1_FLAG_TC7))										// �жϴ���ͨ��
	{
		DMA_ClearFlag(DMA1_FLAG_TC7);												// ��ո�ͨ���жϱ�־
	}
}

void DMA2_Channel1_IRQHandler(void)
{
	if(SET == DMA_GetFlagStatus(DMA2_FLAG_TC1))										// �жϴ���ͨ��
	{
		DMA_ClearFlag(DMA2_FLAG_TC1);												// ��ո�ͨ���жϱ�־
	}
}

void DMA2_Channel2_IRQHandler(void)
{
	if(SET == DMA_GetFlagStatus(DMA2_FLAG_TC2))										// �жϴ���ͨ��
	{
		DMA_ClearFlag(DMA2_FLAG_TC2);												// ��ո�ͨ���жϱ�־
	}
}

void DMA2_Channel3_IRQHandler(void)
{
	if(SET == DMA_GetFlagStatus(DMA2_FLAG_TC3))										// �жϴ���ͨ��
	{
		DMA_ClearFlag(DMA2_FLAG_TC3);												// ��ո�ͨ���жϱ�־
	}
}

void DMA2_Channel4_IRQHandler(void)
{
	if(SET == DMA_GetFlagStatus(DMA2_FLAG_TC4))										// �жϴ���ͨ��
	{
		DMA_ClearFlag(DMA2_FLAG_TC4);												// ��ո�ͨ���жϱ�־
	}
}

void DMA2_Channel5_IRQHandler(void)
{
	if(SET == DMA_GetFlagStatus(DMA2_FLAG_TC5))										// �жϴ���ͨ��
	{
		DMA_ClearFlag(DMA2_FLAG_TC5);												// ��ո�ͨ���жϱ�־
	}
}

#ifdef Will_never_be_defined
WWDG_IRQHandler
PVD_IRQHandler
TAMPER_IRQHandler
RTC_IRQHandler
FLASH_IRQHandler
RCC_CRS_IRQHandler
EXTI0_IRQHandler
EXTI1_IRQHandler
EXTI2_IRQHandler
EXTI3_IRQHandler
EXTI4_IRQHandler
DMA1_Channel1_IRQHandler
DMA1_Channel2_IRQHandler
DMA1_Channel3_IRQHandler
DMA1_Channel4_IRQHandler
DMA1_Channel5_IRQHandler
DMA1_Channel6_IRQHandler
DMA1_Channel7_IRQHandler
ADC1_2_IRQHandler
FlashCache_IRQHandler
CAN1_RX_IRQHandler
EXTI9_5_IRQHandler
TIM1_BRK_IRQHandler
TIM1_UP_IRQHandler
TIM1_TRG_COM_IRQHandler
TIM1_CC_IRQHandler
TIM2_IRQHandler
TIM3_IRQHandler
TIM4_IRQHandler
I2C1_IRQHandler
I2C2_IRQHandler
SPI1_IRQHandler
SPI2_IRQHandler
UART1_IRQHandler
UART2_IRQHandler
UART3_IRQHandler
EXTI15_10_IRQHandler
RTCAlarm_IRQHandler
OTG_FS_WKUP_IRQHandler
TIM8_BRK_IRQHandler
TIM8_UP_IRQHandler
TIM8_TRG_COM_IRQHandler
TIM8_CC_IRQHandler
ADC3_IRQHandler
SDIO_IRQHandler
TIM5_IRQHandler
SPI3_IRQHandler
UART4_IRQHandler
UART5_IRQHandler
TIM6_IRQHandler
TIM7_IRQHandler
DMA2_Channel1_IRQHandler
DMA2_Channel2_IRQHandler
DMA2_Channel3_IRQHandler
DMA2_Channel4_IRQHandler
DMA2_Channel5_IRQHandler
ETH_IRQHandler
COMP1_2_IRQHandler
OTG_FS_IRQHandler
UART6_IRQHandler
UART7_IRQHandler
UART8_IRQHandler
#endif

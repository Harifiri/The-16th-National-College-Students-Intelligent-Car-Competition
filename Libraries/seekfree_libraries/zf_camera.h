/*********************************************************************************************************************
* COPYRIGHT NOTICE
* Copyright (c) 2019,��ɿƼ�
* All rights reserved.
* ��������QQȺ��һȺ��179029047(����)  ��Ⱥ��244861897
*
* �����������ݰ�Ȩ������ɿƼ����У�δ��������������ҵ��;��
* ��ӭ��λʹ�ò������������޸�����ʱ���뱣����ɿƼ��İ�Ȩ������
*
* @file				camera
* @company			�ɶ���ɿƼ����޹�˾
* @author			��ɿƼ�(QQ3184284598)
* @version			�鿴doc��version�ļ� �汾˵��
* @Software			IAR 8.3 or MDK 5.28
* @Target core		MM32F3277
* @Taobao			https://seekfree.taobao.com/
* @date				2021-02-22
********************************************************************************************************************/

#ifndef _zf_camera_h
#define _zf_camera_h

#include "common.h"
#include "hal_dma.h"
#include "hal_rcc.h"

#include "zf_tim.h"

// ö�� TIM_ETR ��������	��ö�ٶ��岻�����û��޸�
typedef enum																		// ö��ADCͨ��
{
	TIM_1_ETR_A12			= 0x010C,												// 0x0000[TIM1] 0x0100[AF1] 0x0000[A group] 0x000C[pin 12]
	TIM_1_ETR_E07			= 0x0147,												// 0x0000[TIM1] 0x0100[AF1] 0x0040[E group] 0x0007[pin  7]

	TIM_8_ETR_A00			= 0x1300,												// 0x1000[TIM8] 0x0300[AF3] 0x0000[A group] 0x0000[pin  0]

	TIM_2_ETR_A00			= 0x2100,												// 0x2000[TIM2] 0x0100[AF1] 0x0000[A group] 0x0000[pin  0]
	TIM_2_ETR_A05			= 0x2105,												// 0x2000[TIM2] 0x0100[AF1] 0x0000[A group] 0x0005[pin  5]
	TIM_2_ETR_A15			= 0x210F,												// 0x2000[TIM2] 0x0100[AF1] 0x0000[A group] 0x000F[pin 15]

	TIM_3_ETR_D02			= 0x4232,												// 0x4000[TIM3] 0x0200[AF2] 0x0030[D group] 0x0002[pin  2]

	TIM_4_ETR_E00			= 0x5240,												// 0x5000[TIM4] 0x0200[AF2] 0x0040[E group] 0x0000[pin  0]
}TIM_ETR_PIN_enum;

extern CAMERA_TYPE_enum camera_type;

void camera_dma_init(DMA_Channel_TypeDef* dma_ch,uint32 src_addr, uint32 des_addr, uint32 size);
void camera_tim_etr_init(TIM_ETR_PIN_enum pin, CAMERA_TYPE_enum camera_type);

#endif

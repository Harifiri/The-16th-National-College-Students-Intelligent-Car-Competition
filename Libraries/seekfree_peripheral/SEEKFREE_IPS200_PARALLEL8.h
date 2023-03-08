/*********************************************************************************************************************
* COPYRIGHT NOTICE
* Copyright (c) 2020,��ɿƼ�
* All rights reserved.
* ��������QQȺ����Ⱥ��824575535
*
* �����������ݰ�Ȩ������ɿƼ����У�δ��������������ҵ��;��
* ��ӭ��λʹ�ò������������޸�����ʱ���뱣����ɿƼ��İ�Ȩ������
*
* @file       		2.0��IPS��Ļ
* @company	   		�ɶ���ɿƼ����޹�˾
* @author     		��ɿƼ�(QQ3184284598)
* @version    		�鿴doc��version�ļ� �汾˵��
* @Software 		ADS v1.2.2
* @Target core		TC264D
* @Taobao   		https://seekfree.taobao.com/
* @date       		2020-3-23
* @note
********************************************************************************************************************/

#ifndef _SEEKFREE_IPS200_PARALLEL8_H
#define _SEEKFREE_IPS200_PARALLEL8_H

#include "common.h"
#include "SEEKFREE_FONT.h"

     
// ������ɫ��SEEKFREE_FONT.h�ļ��ж���
#define IPS200_BGCOLOR		WHITE	//������ɫ
#define IPS200_PENCOLOR		RED		//������ɫ

// ��������  FSMC�̶����� ���ɸ���
#define IPS200_RS_PIN  	D13
#define IPS200_CS_PIN  	D7

// �������� �����޸�
#define IPS200_RD_PIN  	D4
#define IPS200_WR_PIN  	D6
#define IPS200_RST_PIN 	D5
#define IPS200_BL_PIN  	D11

// �������� FSMC�̶����� ���ɸ���
#define IPS200_D0_PIN 		E11
#define IPS200_D1_PIN 		E12
#define IPS200_D2_PIN 		E13
#define IPS200_D3_PIN 		E14
#define IPS200_D4_PIN 		E15
#define IPS200_D5_PIN 		D8
#define IPS200_D6_PIN 		D9
#define IPS200_D7_PIN 		D10

// ���ݶ�Ӧ��ַ
#define IPS200_DATA_ADD	0x60080000
#define IPS200_CMD_ADD		0x60000000

// �������
#define IPS200_RD(x)		(x? (GPIO_PIN_SET(IPS200_RD_PIN)): (GPIO_PIN_RESET(IPS200_RD_PIN)))
#define IPS200_WR(x)     	(x? (GPIO_PIN_SET(IPS200_WR_PIN)): (GPIO_PIN_RESET(IPS200_WR_PIN)))
#define IPS200_RST(x)     	(x? (GPIO_PIN_SET(IPS200_RST_PIN)): (GPIO_PIN_RESET(IPS200_RST_PIN)))
#define IPS200_BL(x)		(x? (GPIO_PIN_SET(IPS200_BL_PIN)): (GPIO_PIN_RESET(IPS200_BL_PIN)))


// ��Ļ�ֱ��� �����޸�
#define IPS200_W			240
#define IPS200_H			320


//������ʾ����
//0 ����ģʽ
//1 ����ģʽ  ��ת180��
//2 ����ģʽ
//3 ����ģʽ  ��ת180��
#define IPS200_DISPLAY_DIR 0

#if (0==IPS200_DISPLAY_DIR || 1==IPS200_DISPLAY_DIR)

#define	IPS200_X_MAX	IPS200_W	//Һ��X�����
#define IPS200_Y_MAX	IPS200_H	//Һ��Y�����

#elif (2==IPS200_DISPLAY_DIR || 3==IPS200_DISPLAY_DIR)

#define	IPS200_X_MAX	IPS200_H	//Һ��X�����
#define IPS200_Y_MAX	IPS200_W	//Һ��Y�����

#else

#error "IPS200_DISPLAY_DIR �������"

#endif

void ips200_init(void); //��ʼ��Ӳ��
void ips200_w_data(uint8 dat);
void ips200_wr_reg(uint8 com);
void ips200_wr_data(uint8 dat);
void ips200_wr_data16(uint16 dat);
void ips200_w_reg(uint8 com,uint8 dat);
void ips200_address_set(uint16 x1,uint16 y1,uint16 x2,uint16 y2);
void ips200_clear(uint16 color);
void ips200_drawpoint(uint16 x,uint16 y,uint16 color);
void ips200_showchar(uint16 x,uint16 y,const int8 dat);
void ips200_showstr(uint16 x,uint16 y,const int8 dat[]);


void ips200_showint8(uint16 x,uint16 y,int8 dat);
void ips200_showuint8(uint16 x,uint16 y,uint8 dat);
void ips200_showint16(uint16 x,uint16 y,int16 dat);
void ips200_showuint16(uint16 x,uint16 y,uint16 dat);
void ips200_showint32(uint16 x,uint16 y,int dat,uint8 num);
void ips200_showfloat(uint16 x,uint16 y,double dat,int8 num,int8 pointnum);
void ips200_showimage(uint16 x,uint16 y,uint16 w,uint16 l,const unsigned char *p);

void ips200_displayimage032(uint8 *p, uint16 width, uint16 height);
void ips200_displayimage032_zoom(uint8 *p, uint16 width, uint16 height, uint16 dis_width, uint16 dis_height);
void ips200_displayimage032_zoom1(uint8 *p, uint16 width, uint16 height, uint16 start_x, uint16 start_y, uint16 dis_width, uint16 dis_height);
void ips200_displayimage8660_zoom(uint16 *p, uint16 width, uint16 height, uint16 dis_width, uint16 dis_height);
void ips200_displayimage8660_zoom1(uint16 *p, uint16 width, uint16 height, uint16 start_x, uint16 start_y, uint16 dis_width, uint16 dis_height);
void ips200_displayimage8660_grayscale_zoom(uint16 *p, uint16 width, uint16 height, uint16 dis_width, uint16 dis_height);
void ips200_displayimage7725(uint8 *p, uint16 width, uint16 height);
void ips200_display_chinese(uint16 x, uint16 y, uint8 size, const uint8 *p, uint8 number, uint16 color);

#endif


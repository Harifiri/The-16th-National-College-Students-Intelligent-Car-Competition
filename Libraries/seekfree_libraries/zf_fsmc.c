/*********************************************************************************************************************
* COPYRIGHT NOTICE
* Copyright (c) 2019,��ɿƼ�
* All rights reserved.
* ��������QQȺ��һȺ��179029047(����)  ��Ⱥ��244861897
*
* �����������ݰ�Ȩ������ɿƼ����У�δ��������������ҵ��;��
* ��ӭ��λʹ�ò������������޸�����ʱ���뱣����ɿƼ��İ�Ȩ������
*
* @file				fsmc
* @company			�ɶ���ɿƼ����޹�˾
* @author			��ɿƼ�(QQ3184284598)
* @version			�鿴doc��version�ļ� �汾˵��
* @Software			IAR 8.3 or MDK 5.28
* @Target core		MM32F3277
* @Taobao			https://seekfree.taobao.com/
* @date				2021-02-22
********************************************************************************************************************/

#include "zf_fsmc.h"

//-------------------------------------------------------------------------------------------------------------------
// @brief		����ͷDMA��ʼ��
// @return		void
// Sample usage:					fsmc_init();
//-------------------------------------------------------------------------------------------------------------------
void fsmc_init (void)
{
	FSMC_InitTypeDef  FSMC_InitStructure;
	FSMC_NORSRAM_Bank_InitTypeDef  FSMC_BankInitStructure;

	FSMC_NORSRAM_BankStructInit(&FSMC_BankInitStructure);
	FSMC_NORSRAMStructInit(&FSMC_InitStructure);

	RCC_AHB3PeriphClockCmd(RCC_AHB3ENR_FSMC, ENABLE);

    FSMC_BankInitStructure.FSMC_SMReadPipe    = 0;							// �������������
    FSMC_BankInitStructure.FSMC_ReadyMode     = 0;							// Ӧ���ź����� 0-�ڲ�FSMC 1-�ⲿDevice
    FSMC_BankInitStructure.FSMC_WritePeriod   = 0x2;						// д������
    FSMC_BankInitStructure.FSMC_WriteHoldTime = 1;							// д��ı���ʱ��
    FSMC_BankInitStructure.FSMC_AddrSetTime   = 3;							// ��ַ������ʱ��
    FSMC_BankInitStructure.FSMC_ReadPeriod    = 0x1;						// ��ȡ����
    FSMC_BankInitStructure.FSMC_DataWidth     = FSMC_DataWidth_16bits;		// ����λ��
    FSMC_NORSRAM_Bank_Init(&FSMC_BankInitStructure, FSMC_NORSRAM_BANK0);

    FSMC_InitStructure.FSMC_Mode = FSMC_Mode_NorFlash;						// ���ô洢������Э��ģʽ
    FSMC_InitStructure.FSMC_TimingRegSelect = FSMC_TimingRegSelect_0;		// ʱ��������üĴ�����
    FSMC_InitStructure.FSMC_MemSize = FSMC_MemSize_64MB;					// �洢����С
    FSMC_InitStructure.FSMC_MemType = FSMC_MemType_NorSRAM;					// �洢������
    FSMC_InitStructure.FSMC_AddrDataMode = FSMC_AddrDataMUX;				// ��������ģʽ
    FSMC_NORSRAMInit(&FSMC_InitStructure);
}

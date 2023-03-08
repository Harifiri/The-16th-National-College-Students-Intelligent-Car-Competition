#ifndef _INDUCTOR_H
#define _INDUCTOR_H

#include "zf_adc.h"

#define AL_PIN	 ADC2_CH10_C00 //ADC2_CH10_C00 ADC2_CH11_C01 ADC2_CH07_A07
#define AM_PIN	 ADC2_CH07_A07
#define AR_PIN	 ADC2_CH11_C01

//#define Read_AM  adc_convert(ADC_2,ADC2_CH10_C00)
#define Read_AM	 adc_convert(ADC_2,ADC2_CH07_A07)
//#define Read_AM  adc_convert(ADC_2,ADC2_CH11_C01)
//#define Read_AM  adc_convert(ADC_2,ADC2_CH10_C00)

typedef struct//电感结构体数据类型
{ 
  uint16 Max;
	uint16 Min;
	uint16 BandWidth;
	uint16 Average;
	uint16 Sum;
	int Final;
}InductorTypedef;

void Indoctor_init(void);
void Get_AD_data(void);

float newtonSqrt(float a);

#endif


#include "headfile.h"
#include "inductor.h"

//���У�������������ΪAL,BL,CL
//�ҵ�У�������������ΪAR,BR,CR
InductorTypedef AL;
InductorTypedef AM;
InductorTypedef AR;

uint16 InductorAL[6];
uint16 InductorAM[5];
uint16 InductorAR[6];

float Indoctor_error;

//----------------------------------------------------------------    
//  @brief      ���ADC��ʼ��   
//  @date       2021/1/14   
//----------------------------------------------------------------  
void Indoctor_init(void)
{
	adc_init(ADC_2, AL_PIN, ADC_12BIT);
	adc_init(ADC_2, AM_PIN, ADC_12BIT);
	adc_init(ADC_2, AR_PIN, ADC_12BIT);

//	AL.Min = 10;
//	AL.Max = 1500;
//	
	AM.Min = 700;
	AM.Max = 3000;
//	
//	AR.Min = 10;
//	AR.Max = 1500;
//	
//	AL.BandWidth = AL.Max - AL.Min;
	AM.BandWidth = AM.Max - AM.Min;
//	AR.BandWidth = AR.Max - AR.Min;
}

void Get_AD_data(void)
{ 
	uint8 i,j,N=5;
  uint16 AD_temp;
   
	for(i=0;i<N;i++)
	{
//		InductorAL[i] = Read_AL;
		InductorAM[i] = Read_AM;
//		InductorAR[i] = Read_AR;
		systick_delay_us(1);
	}
	
//	for(i=0;i<N;i++)//N����������
//	{
//		for(j=i+1;j<N;j++)
//		{
////			if(InductorAL[i] > InductorAL[j])//ǰ��ıȺ���Ĵ�  ����н���
////			{
////				 AD_temp = InductorAL[i];
////				 InductorAL[i] =InductorAL[j];
////				 InductorAL[j] = AD_temp;
////			}
////			
//			if(InductorAM[i] > InductorAM[j])
//			{
//				 AD_temp = InductorAM[i];
//				 InductorAM[i] =InductorAM[j];
//				 InductorAM[j] = AD_temp;
//			}
////			
////			if(InductorAR[i] > InductorAR[j])
////			{
////				 AD_temp = InductorAR[i];
////				 InductorAR[i] =InductorAR[j];
////				 InductorAR[j] = AD_temp;
////			}
//		}
//	}
	//ƽ��ֵ
//	AL.Sum = InductorAL[1]+InductorAL[2]+InductorAL[3]+InductorAL[4];
	AM.Sum = InductorAM[0]+InductorAM[1]+InductorAM[2]+InductorAM[3]+InductorAM[4];
//	AR.Sum = InductorAR[1]+InductorAR[2]+InductorAR[3]+InductorAR[4];	
	
//	AL.Average=AL.Sum/4;
	AM.Average=AM.Sum/5;
//	AR.Average=AR.Sum/4;
	
	//��һ��������·��Average����2
//  AL.Final  = 1000*(AL.Average-AL.Min)/AL.BandWidth;
	AM.Final  = 1000*(AM.Average-AM.Min)/AM.BandWidth;
//	AR.Final  = 1000*(AR.Average-AR.Min)/AR.BandWidth;

	//����С�������
//	if(AL.Final<0)
//		AL.Final=0;

	if(AM.Final<0)
		AM.Final=0;
//		
//	if(AR.Final<0)
//		AR.Final=0;
	
}

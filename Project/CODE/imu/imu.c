#include "imu.h"

const float RtA = 57.2957795f; //RtA = 180.0f/pi
const float halfT	= 0.0025f;//T=0.005
float IMU_Kp = 0.9f;		  //��Ԫ����ʼֵ IMU_Kp = 0.0008f;
float IMU_Ki = 0.0005f;		//���ٶȲ������ٶȻ���ֵ��ʼֵ 0.00005f;	
float exInteg=0,eyInteg=0,ezInteg=0;
float q0=-0.029f,q1=0.068f,q2=-0.997f,q3=0.000f;
//----------------------------------------------------------------    
//  @brief      ��̬����         
//  @date       2021/1/10
//---------------------------------------------------------------- 
float Gyro_x,Gyro_y,Gyro_z;
float Acc_x,Acc_y,Acc_z;
float Pitch,Yaw,Roll;

void Get_gyro_data(void)
{
	get_icm20602_gyro_spi();

	Gyro_x=(float)icm_gyro_z/16.384f;//ת����ϵ:gyro/(2^16/2)*2000��/s, 16.384
	Gyro_y=(float)icm_gyro_y/16.384f;
	Gyro_z=-(float)icm_gyro_x/16.384f;
}

#define AccFilterNum 8
void Get_acc_data(void)
{
	uint8 i;
	int32 acc_sum[3]={0};

	for(i=0;i<AccFilterNum;i++)
	{
		get_icm20602_accdata_spi();
		acc_sum[0]  += icm_acc_z;
		acc_sum[1]  += icm_acc_y;
		acc_sum[2]  -= icm_acc_x;
		systick_delay_us(1);
	}
	
	Acc_x=(float)acc_sum[0]/3343.67347f;//ת����ϵ:acc/(2^16/2)*8g = acc_sum/AccFilterNum/(2^16/2)*8g, gȡ9.8
	Acc_y=(float)acc_sum[1]/3343.67347f;
	Acc_z=(float)acc_sum[2]/3343.67347f;
	

	
//	acc_angle = RtA*(float)(atan2(Acc_z,Acc_x));
	
//	if(acc_angle<-10.0f)
//		acc_angle=-10.0f;
//	else if(acc_angle>37.0f)
//		acc_angle=37.0f;
}

//void IMUupdate(void)
//{  
//	float gyro_ratio = 0.99963f;//0.999325f    //�����Ǳ���<0.99975
//	float gyro_angle;     
//	
////	if(start_flag==4 || RampFlag!=0)
////		gyro_ratio = 1.0f;
//		
//	gyro_angle = Pitch + Gyro_y*0.005f;
//	Pitch = gyro_angle*gyro_ratio + acc_angle*(1.0f-gyro_ratio); 

//	if(LeftRoundFlag!=0 || RightRoundFlag!=0||ByroadFlag!=0)
//	{
//		if(Gyro_x>0 && Gyro_z>0)
//			Yaw += sqrt(Gyro_x*Gyro_x + Gyro_z*Gyro_z) * 0.005f;
//		else if(Gyro_x<0 && Gyro_z<0)
//			Yaw -= sqrt(Gyro_x*Gyro_x + Gyro_z*Gyro_z) * 0.005f;
//	}
//	else
//		Yaw = 0;
//}

void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az)
{
	float norm;
	float vx, vy, vz;
	float ex, ey, ez;

	norm = sqrt(ax*ax + ay*ay + az*az);	//�Ѽ��ٶȼƵ���ά����ת�ɵ�ά����   
	ax = ax / norm;
	ay = ay / norm;
	az = az / norm;

	//	�����ǰ���Ԫ������ɡ��������Ҿ����еĵ����е�����Ԫ�ء� 
	//	�������Ҿ����ŷ���ǵĶ��壬��������ϵ������������ת����������ϵ��������������Ԫ��
	//	���������vx vy vz����ʵ���ǵ�ǰ��ŷ���ǣ�����Ԫ�����Ļ����������ϵ�ϣ����������
	//	������λ������
	vx = 2*(q1*q3 - q0*q2);
	vy = 2*(q0*q1 + q2*q3);
	vz = q0*q0 - q1*q1 - q2*q2 + q3*q3 ;

	ex = (ay*vz - az*vy);
	ey = (az*vx - ax*vz);
	ez = (ax*vy - ay*vx);

	exInteg = exInteg*0.9f + ex * IMU_Ki;
	eyInteg = eyInteg*0.9f + ey * IMU_Ki;
	ezInteg = ezInteg*0.9f + ez * IMU_Ki;

	gx = gx + IMU_Kp*ex + exInteg;
	gy = gy + IMU_Kp*ey + eyInteg;
	gz = gz + IMU_Kp*ez + ezInteg;

	q0 = q0 + (-q1*gx - q2*gy - q3*gz) * halfT;
	q1 = q1 + ( q0*gx + q2*gz - q3*gy) * halfT;
	q2 = q2 + ( q0*gy - q1*gz + q3*gx) * halfT;
	q3 = q3 + ( q0*gz + q1*gy - q2*gx) * halfT;

	norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
	q0 = q0 / norm;
	q1 = q1 / norm;
	q2 = q2 / norm;
	q3 = q3 / norm;

	Pitch = asin(-2*(q0*q2 - q1*q3)) * RtA; // ����   ����ɶ�
//	Yaw = asin(2*(q0*q1 + q2*q3 )) * RtA; // ���
//	Roll = atan(2*(q1*q2 + q0*q3 ) / (q0*q0 + q1*q1 - q2*q2 -q3*q3)) * RtA;

//	Pitch += Gyro_y * 0.005f; // ����   ����ɶ�
//	Yaw += Gyro_x * 0.005f; // ���
//	Roll += Gyro_z * 0.005f;
	
	if(LeftRoundFlag!=0 || RightRoundFlag!=0||ByroadFlag!=0)
	{
		if(Gyro_x>0 && Gyro_z<0)
			Yaw += sqrt(Gyro_x*Gyro_x + Gyro_z*Gyro_z) * 0.005f;
		else if(Gyro_x<0 && Gyro_z>0)
			Yaw -= sqrt(Gyro_x*Gyro_x + Gyro_z*Gyro_z) * 0.005f;
	}
	else
		Yaw = 0;
}

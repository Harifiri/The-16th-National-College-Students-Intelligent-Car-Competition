#include "headfile.h"
#include "control.h"

extern float DataBuf[8];//发送给上位机的数组

//定义结构体
AngleCrl_Typedef Angle;
SpeedCrl_Typedef Speed;
DirectionCrl_Typedef Direction;
MotorCrl_Typedef Motor;

float Angle_Kp,Angle_Ki;

void Control_init(void)
{
	//滤波递归环
	Angle_Kp = 0.075;//0.12,0.08
	Angle_Ki = 0.0005;//0.0005
	
	//速度环
	Speed.kp = 0.00042f;//0.005
	Speed.ki = 0.00009f;//0.001
	Speed.speed_max = 2800;
	Speed.speed_set = Speed.speed_max;
	Speed.err_integral = 0;
	Speed.control_out_old = 0;
 	Speed.control_out_max = 5.0f;
	Speed.control_out_min = -7.5f;
	
	//直立环
	Angle.inner_kp = 293.0f;//300
	Angle.inner_ki = 7.6f;//7.7

	Angle.outer_kp = 3.7f;//<2.9
	Angle.outer_kd = 1.1f;//<0.5
	
	Angle.angle_offset = 28.3f;//37.1
	
	//转向环
	Direction.outer_kp = 8.5f;//1.3
	Direction.outer_kd = 2.4f;//1.0
	
	Direction.inner_kp = 170;//50
	Direction.inner_ki = 5.0f;//3
	Direction.inner_expect_max = 350;
//	Direction.control_delta_max = 2500;
	Direction.control_out_max = 20000;

	//电机参数
	Motor.LeftDeadZone=100;  //1250										
	Motor.RightDeadZone=100;  //750
}

/***********************************************************
	*	函数名称：SpeedConrtol(void)
	*	函数说明：PID速度控制函数，周期20ms
	*	函数参数：none
	*	参数返回：void
	*	修改日期：2020-10-19
***********************************************************/
void SpeedConrtol(void)
{
	static uint8 max_flag=0,max_count=0;

	Speed.control_out_old = Speed.control_out;

	Speed.err_new = Speed.speed_new - Speed.speed_set;//正反馈

	if(max_flag==0)
	{
		if(max_count<=250)
			max_count++;

		if(max_count>=7 || Pitch>25)
		{
			Bys_flag=1;
			
			Direction.outer_kp = 8.3f;//8.3，无U弯8.1
			Direction.outer_kd = 2.3f;//2.2，无U弯2.1
		}
			

		if(max_count>=5&&ABS(Speed.err_new)<1500)
		{
			Angle.angle_offset = 34.3f-6.0f*(ABS(Speed.err_new)-500)/1000;
		}

		if(ABS(Speed.err_new)<500 || max_count>245)
		{
			Angle.angle_offset = 34.3f;
			max_flag=1;
		}
	}
	
	Speed.control_out_new = Speed.kp*Speed.err_new;

	if(Speed.control_out_new>Speed.control_out_max)
		Speed.control_out_new = Speed.control_out_max;
	else if(Speed.control_out_new<Speed.control_out_min)
		Speed.control_out_new = Speed.control_out_min;
	
	Speed.control_out_delta = Speed.control_out_new - Speed.control_out_old;
}

/***********************************************************
	*	函数名称：SpeedConrtol(void)
	*	函数说明：速度平滑输出函数
	*	函数参数：none
	*	参数返回：void
	*	修改日期：2020-10-20
***********************************************************/
void SpeedSmoothOutput(void)
{
	static uint8 ts=0;
	ts++;
	
	Speed.control_out = Speed.control_out_old + Speed.control_out_delta*ts/20;
	
	if(ts==20) ts=0;
}

//----------------------------------------------------------------    
//  @brief      角度环外环PID        
//  @date       2021/1/10
//----------------------------------------------------------------  
void AngleOuterConrtol(void){
	
	//IMUupdate();
	IMUupdate(Gyro_x*0.0174533f, Gyro_y*0.0174533f, Gyro_z*0.0174533f, Acc_x, Acc_y, Acc_z);

//	if(RampFlag!=0)
//		Angle.angle_error = ramp_angle - Pitch;
//	else
	Angle.angle_error = (Angle.angle_offset + Speed.control_out) - Pitch;
	
	//d项误差不要用角速度，这样和速度环串起来后就改变期望就不能快速响应了
	Angle.inner_expect = Angle.outer_kp*Angle.angle_error + Angle.outer_kd*(Angle.angle_error-Angle.angle_error_last);
	
	Angle.angle_error_last = Angle.angle_error;
	
	Angle.gyro_integral *= 0.975f;
}

//----------------------------------------------------------------    
//  @brief      角度环内环PID        
//  @date       2021/1/10
//----------------------------------------------------------------  

void AngleInnerConrtol(void)
{
	Angle.gyro_error = Angle.inner_expect - Gyro_y;
	Angle.gyro_integral += Angle.gyro_error;

	Angle.control_out = Angle.inner_kp * Angle.gyro_error + Angle.inner_ki * Angle.gyro_integral;
}

//----------------------------------------------------------------    
//  @brief      转向环外环PID        
//  @date       2021/1/20
//----------------------------------------------------------------  

void DirectionOuterConrtol(void)
{
//	static uint8 start_flag=0; 
//	float dir_radio;//转向调节系数，速度越慢，转向输出越小，范围：>0.75
	
//	if(Speed.speed_new<2000)
//	{
//		Direction.outer_kp = 5.3f;
//		Direction.outer_kd = 3.7f;
//		Direction.err_new = err_calculation(25,45);
//	}
//	else if(Speed.speed_new>2000 && Speed.speed_new<2500) 
//	{
//	Direction.outer_kp = 7.5f;//1.3
//	Direction.outer_kd = 4.7f;//1.0
//		Direction.err_new = err_calculation(15,45);
//	}
	
	if(ZebraFlag==2)
		Direction.err_new = 0;
	else
		Direction.err_new = err_calculation(30,40);
	
	Direction.inner_expect = Direction.outer_kp*Direction.err_new + Direction.outer_kd*(Direction.err_new-Direction.err_last);
	Direction.err_last=Direction.err_new;
	
	if(RampFlag==0)
	{
		if(Direction.inner_expect>Direction.inner_expect_max)
			Direction.inner_expect=Direction.inner_expect_max;
		else if(Direction.inner_expect<-Direction.inner_expect_max)
			Direction.inner_expect=-Direction.inner_expect_max;
	}
	if(RampFlag!=0)
	{
		if(RampFlag==1)
		{
			if(Direction.inner_expect>20)
				Direction.inner_expect=20;
			else if(Direction.inner_expect<-20)
				Direction.inner_expect=-20;
	  }
		else
		{
			if(Direction.inner_expect>30)
				Direction.inner_expect=30;
			else if(Direction.inner_expect<-30)
				Direction.inner_expect=-30;
	  }
  }
	
}

//----------------------------------------------------------------    
//  @brief      转向环内环PID        
//  @date       2021/1/20
//----------------------------------------------------------------  
void DirectionInnerConrtol(void)
{
	Direction.control_out_last = Direction.control_out;
	
	if(Gyro_x>0 && Gyro_z<0)
		Direction.gyro_now = sqrt(Gyro_x*Gyro_x + Gyro_z*Gyro_z);
	else if(Gyro_x<0 && Gyro_z>0)
		Direction.gyro_now = -sqrt(Gyro_x*Gyro_x + Gyro_z*Gyro_z);
	else
		Direction.gyro_now = 0;
	
	Direction.gyro_error = Direction.inner_expect - Direction.gyro_now;
	
	Direction.gyro_integral *= 0.7f;
	Direction.gyro_integral += Direction.gyro_error;
	Direction.control_out = Direction.inner_kp * Direction.gyro_error + Direction.inner_ki * Direction.gyro_integral;

	Direction.control_delta = Direction.control_out - Direction.control_out_last;
	
//	if(Direction.control_delta>Direction.control_delta_max)
//		Direction.control_out=Direction.control_out_last+Direction.control_delta_max;
//	else if(Direction.control_delta<-Direction.control_delta_max)
//		Direction.control_out=Direction.control_out_last-Direction.control_delta_max;
	
	//限幅
	if(Direction.control_out>Direction.control_out_max)
		Direction.control_out=Direction.control_out_max;
	else if(Direction.control_out<-Direction.control_out_max)
		Direction.control_out=-Direction.control_out_max;
//	if(RampFlag!=0)
//	{
//		if(RampFlag==1)
//		{
//			if(Direction.control_out>2000)
//				Direction.control_out=2000;
//			else if(Direction.control_out<-2000)
//				Direction.control_out=-2000;
//	  }
//		else
//		{
//			if(Direction.control_out>7000)
//				Direction.control_out=7000;
//			else if(Direction.control_out<-7000)
//				Direction.control_out=-7000;
//	  }
//  }
}

/***********************************************************
	*	函数名称：MotorConrtol(void)
	*	函数说明：
	*	函数参数：none
	*	参数返回：void
	*	修改日期：2020-10-20
***********************************************************/
void MotorConrtol(void)
{
	Motor.LeftMotorOut  = Angle.control_out - Direction.control_out;
	Motor.RightMotorOut = Angle.control_out + Direction.control_out;
	
//	if(Direction.control_out>=0)
//	{
//		Motor.LeftMotorOut  = Angle.control_out - 1.05f*Direction.control_out;
//		Motor.RightMotorOut = Angle.control_out + 0.95f*Direction.control_out;
//	}
//	else if(Direction.control_out<0)
//	{
//		Motor.LeftMotorOut  = Angle.control_out - 0.95f*Direction.control_out;
//		Motor.RightMotorOut = Angle.control_out + 1.05f*Direction.control_out;
//	}
	
	//限制死区
	if(Motor.LeftMotorOut>0)
		Motor.LeftMotorOut+=Motor.LeftDeadZone;
	else
		Motor.LeftMotorOut-=Motor.LeftDeadZone;
	if(Motor.RightMotorOut>0)
		Motor.RightMotorOut+=Motor.RightDeadZone;
	else
		Motor.RightMotorOut-=Motor.RightDeadZone;

	//输出
//Motor.RightMotorOut = 20000;
//Motor.LeftMotorOut = 20000;
	Motor_Run(Motor_R, Motor.RightMotorOut);
	Motor_Run(Motor_L, Motor.LeftMotorOut);
}

void Brake(void)
{
	int Brake_kp=20,Brake_ki=1;
	int Brake_err,Brake_out_L,Brake_out_R;
	static int BrakeIntegral_L=0,BrakeIntegral_R=0;

	Brake_err = 0-Speed_L;//负反馈
	BrakeIntegral_L += Brake_err;
	Brake_out_L = Brake_kp*Brake_err + Brake_ki*BrakeIntegral_L;
	
	Brake_err = 0-Speed_R;//负反馈
	BrakeIntegral_R += Brake_err;
	Brake_out_R = Brake_kp*Brake_err + Brake_ki*BrakeIntegral_R;
	
	Motor_Run(Motor_R, Brake_out_L);
	Motor_Run(Motor_L, Brake_out_R);
	
	if(ABS(Speed.speed_new)<100)
	{
		Motor_Run(Motor_L, 0);
		Motor_Run(Motor_R, 0);
		start_flag=5;//完成
	}
}

void Control(void)
{
	static uint8 t=0;

	//改变各周期比例关系后记得把平滑输出的被除数改了
	if(start_flag==2)//正常行驶
	{
		if(t>=100)
		{
			Get_gyro_data();
			Get_acc_data();
			
			Get_speed();
			SpeedConrtol();
			SpeedSmoothOutput();
			
			AngleOuterConrtol();
			AngleInnerConrtol();
			
			DirectionInnerConrtol();
			t=0;
		}
		else if(t%20==0)
		{
			Get_gyro_data();
			Get_acc_data();
			
			Get_speed();
			SpeedSmoothOutput();
			
			AngleOuterConrtol();
			AngleInnerConrtol();
			
			DirectionInnerConrtol();
		}
		else if(t%10==0)
		{
			Get_gyro_data();
			Get_acc_data();
			
			SpeedSmoothOutput();
			
			AngleOuterConrtol();
			AngleInnerConrtol();

			DirectionInnerConrtol();
		}
		else if(t%5==0)
		{
			Get_gyro_data();
			Get_acc_data();
			
			SpeedSmoothOutput();
			
			AngleOuterConrtol();
			AngleInnerConrtol();
		}
		else if(t%2==0)
		{	
			Get_gyro_data();
			AngleInnerConrtol();
			
			DirectionInnerConrtol();
		}
		else
		{
			Get_gyro_data();
			AngleInnerConrtol();
		}
		
	  MotorConrtol();
	}
	
	else if(start_flag==4)//刹车
	{
		if(t%5==0)
		{
			t=0;
			Get_gyro_data();
			Get_acc_data();
			IMUupdate(Gyro_x*0.0174533f, Gyro_y*0.0174533f, Gyro_z*0.0174533f, Acc_x, Acc_y, Acc_z);
			Get_speed();
			Brake();
			uart_putchar(WIRELESS_UART,0x23);
		}
	}
	
	else if(start_flag==5)//刹车
	{
		static uint8 t5 = 0;
		if(t%5==0)
		{
			Get_speed();
			Get_gyro_data();
			Get_acc_data();
			IMUupdate(Gyro_x*0.0174533f, Gyro_y*0.0174533f, Gyro_z*0.0174533f, Acc_x, Acc_y, Acc_z);
			uart_putchar(WIRELESS_UART,0x23);
		}
		
		if(t%250==0)
		{
			t=0;
			if(t5>=15)
			{
				t5 = 0;
				IMU_Kp = 0.9f;
				IMU_Ki = 0.0005f;
			}
			else
				t5++;
		}
	}
	
	else if(start_flag==1)//等待启动
	{
		static uint8 t1 = 0;
		
		if(t%5==0)
		{	
			Get_gyro_data();
			Get_acc_data();
			
			Get_speed();
			
			if( (Speed_L+Speed_R) > 10 )
			{
				start_flag=2;	
				exInteg = 0;
				eyInteg = 0;
				ezInteg = 0;
				IMU_Kp = Angle_Kp;
				IMU_Ki = Angle_Ki;
			}
			else
			{
				Motor_Run(Motor_L, 0);
				Motor_Run(Motor_R, 0);
			}
			
//			if(start_flag==2 || t1>=15)
//			{
//				exInteg = 0;
//				eyInteg = 0;
//				ezInteg = 0;
//				IMU_Kp = Angle_Kp;
//				IMU_Ki = Angle_Ki;
//			}
			
			IMUupdate(Gyro_x*0.0174533f, Gyro_y*0.0174533f, Gyro_z*0.0174533f, Acc_x, Acc_y, Acc_z);
		}
		
		if(t%250==0)
		{
			t=0;
			
			if(t1<=200)
				t1++;
		}
		
	}
	
	else if(start_flag==0)
	{
		static uint8 tr=0;
		
		if(t%5==0)
		{
			Get_gyro_data();
			Get_acc_data();
			IMUupdate(Gyro_x*0.0174533f, Gyro_y*0.0174533f, Gyro_z*0.0174533f, Acc_x, Acc_y, Acc_z);
			Get_speed();
		}
		
		if(t>=100)
		{
			t=0;

			tr++;
			
			if(tr>=20)
				start_flag=1;
			
		}
		Motor_Run(Motor_L, 0);
		Motor_Run(Motor_R, 0);
	}
	
	t++;
}


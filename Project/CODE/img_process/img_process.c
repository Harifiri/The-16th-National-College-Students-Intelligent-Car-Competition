/*******************************************************************************
	*	@file:		img_process.c
	*	@brief:		摄像头图像处理程序
	*	@author:	whut
	*	@date:		2019-11-26
	*	@function:
							Get_Road(void)	道路图像处理
							reosion()				腐蚀运算
							gray2bit(void)	灰度图像二值化
							regression()		最小二乘法计算斜率
*******************************************************************************/

#include "img_process.h"
#include "common.h"
#include "all_variable.h"

/************获取赛道的图像信息参数*************/
#define MIDDLE_ROAD 89 //屏幕的中间值
uint8 row;                     								//函数执行所需要的变量
uint8 line;

uint8	break_row=2;														//扫描的图像最远有效行
uint8	end_row=5;														  //需要处理的图像最远有效行

uint8 Half_Width[MT9V03X_H]=
{
	13, 14, 15, 16, 17, 18, 19, 20, 22, 23, 
	24, 25, 26, 27, 28, 30, 31, 32, 33, 34, 
	35, 36, 38, 39, 40, 41,	42, 43, 44, 45, 
	47, 48, 49, 50, 51, 52,	53, 54, 56, 57, 
	58, 59, 61, 62, 63, 64,	65, 66, 67, 68, 
	69, 70, 71, 72, 73, 74, 75, 77, 78, 79,

};//45°时基准宽度

/******************

27度：20：43         40：67
30度：20：34         40：57
32度：20：29         40：51

斜率k=3，注意极性别反

Middle_X[i] = LeftEdge_X[i] + ( Half_Width[i] + 3*( 30-(int)Pitch) );
******************/

uint8 LeftEdge_X[MT9V03X_H] ={0}; 										//存储左右边线横坐标
uint8 RightEdge_X[MT9V03X_H]={MT9V03X_W-1};
uint8 Middle_X[MT9V03X_H]={MT9V03X_W/2-1};						//储存中线

uint8 StartScanPoint_line= MT9V03X_W/2-1;   					//赛道起始扫描点
uint8 StartScanPoint_row = MT9V03X_H-1;
int16 LeftStart_line= MT9V03X_W/2-1;   								//赛道起始扫描点
int16 RightStart_line = MT9V03X_W/2-1;

uint8 LeftEdge_GetFlag[MT9V03X_H] ={0}; 							//存储左右边线横坐标
uint8 RightEdge_GetFlag[MT9V03X_H]={0}; 
uint8 LeftLostNum=0;		  //左边丢边数量
uint8 RightLostNum=0;	    //右边丢边数量
uint8 BothLostNum=0;	    //两边都丢边数量

//斑马线方向标志位
uint8 garage_dir = 1;//0——左车库

//三叉标志位
uint8 Byroad_Num = 1;
uint8 Byroad_dir[3] = {1,0,1};//0——三叉沿左边

//环岛标志位
uint8 LeftRoundCount=0;
uint8 RightRoundCount=0;
uint8 LargeLeftRoundFlag[5]={1,1,1,1,1};
uint8 LargeRightRoundFlag[5]={1,1,1,1,1};
//环岛补线系数，越小打角越狠
float LeftRoundRadio_2 = 1.1f;
float LeftRoundRadio_4 = 1.1f;
float RightRoundRadio_2 = 0.9f;
float RightRoundRadio_4 = 1.1f;

uint8 split_flag=0;
uint8 L_state=0,R_state=0;
extern uint8 SpecialCount;
enum {
	Left,
	Right,
	Middle
};

void Correction(void)
{
	uint8 i;
	
	start_flag = 4;
	
	for(;;)
	{
		ips114_showfloat(192,0,Pitch,3,1);
		ips114_displayimage032(image[0],MT9V03X_W,MT9V03X_H);
		
		for(i=1;i<=59;i++)
		{
			ips114_drawpoint(90+Half_Width[i],i,RED);
			ips114_drawpoint(90-Half_Width[i],i,RED);
		}
	}
		
}
void Get_Road(void)
{
	VariableReset();                //参数清零
	grey_scale();
	TrackBothEdge();							  //找初始左右边缘
  bend_detect();
	
	/****************全元素识别*****************/
	if(LeftRoundFlag==0 && RightRoundFlag==0 && ZebraFlag==0 && ByroadFlag==0)
	//小S弯误判
		Ramp();
	if(RampFlag==0)
	{
		Zebra();
		
		if(LeftRoundFlag==0 && RightRoundFlag==0 && ZebraFlag==0)
			Byroad();
		if(ByroadFlag==0 && LeftRoundFlag==0 && ZebraFlag==0)
			RightRound();
		if(ByroadFlag==0 && RightRoundFlag==0 && ZebraFlag==0)
			LeftRound();
		if(ZebraFlag!=0)
		{
			ByroadFlag = 0;
			LeftRoundFlag = 0;
			RightRoundFlag = 0;
		}
  }
	/****************全元素识别*****************/
	
	/****************正跑顺序识别*****************/
//	if(SpecialCount==0)
//		Byroad();
//	else if(SpecialCount==1)
//		LeftRound();
//	else if(SpecialCount==2)
//		Zebra();
//	else if(SpecialCount==3)
//		Ramp();
//	else if(SpecialCount==4)
//		Byroad();
	/****************正跑顺序识别*****************/
	
	DirectionOuterConrtol();
		

}



/***********************************************************
	*	函数名称：void VariableReset(void)
	*	函数说明：变量清零
	*	函数参数：none
	*	参数返回：void
	*	修改日期：2019-11-26
***********************************************************/
void VariableReset(void)
{
	StartScanPoint_row = MT9V03X_H-1;
	
	L_state = 0;
	R_state = 0;
	LeftLostNum=0;
	RightLostNum=0;
	
	for(row=0;row<MT9V03X_H;row++)
	{
		Middle_X[row] = MT9V03X_W/2-1;
		LeftEdge_X[row]=0;
		RightEdge_X[row]=MT9V03X_W-1;
		LeftEdge_GetFlag[row] =0;
		RightEdge_GetFlag[row]=0;
	}
	
}

void grey_scale(void)
{
	uint8 i=0,line_detect=0;
	
	for(i=MT9V03X_H-5;i>MT9V03X_H-20;i--)
	{
		if(image[i][0]>200&&image[i][150]>200&&image[i][180]>200&&image[i][30]>200&&image[i][60]>200&&image[i][90]>200&&image[i][120]>200)
			line_detect++;
	}
	if(line_detect>=7)
	{
		line_detect = 0;
		split_flag = 1;
	}
	else
	{
		line_detect = 0;
		split_flag = 0;
	}
}

void bend_detect(void)
{
	uint8 i=0;

	for(row=MT9V03X_H-5;row>=10;row=row-2)
	{
		L_state += LeftEdge_GetFlag[row];
		R_state += RightEdge_GetFlag[row];
	}
}

/***********************************************************
	*	函数名称：void TrackBothEdge(void)
	*	函数说明：找左右两边线
	*	函数参数：none
	*	参数返回：void
	*	修改日期：2019-11-26
***********************************************************/
#define MaxValue 20
void TrackBothEdge(void)
{
	int16 value_confirm=0;
	static uint8 start=90,inherit = 0;
	uint8 launch_flag=0;
	int launch_row=0;

	for(row=StartScanPoint_row;row>=1;row--)
	{
		if(row==StartScanPoint_row)
		{
			for(line=start;line>1;line--)//左边缘
			{
				if((image[row][line] - image[row][line-2])>MaxValue)
				{
					LeftEdge_X[row] = line;
					LeftEdge_GetFlag[row]=1;
					break;
				}
			}
		
			for(line=start;line<MT9V03X_W-2;line++)//右边缘
			{
				if((image[row][line] - image[row][line+2])>MaxValue)
				{
					RightEdge_X[row] = line;
					RightEdge_GetFlag[row]=1;
					break;
				}
			}
		}
		
		else
		{
			for(line=LeftStart_line;line>1;line--)//左边缘
			{
				if(line>=4)
				  value_confirm = image[row][line] - image[row][line-4];
				else
					value_confirm = image[row][line] - image[row][0];
				if((image[row][line] - image[row][line-2])>MaxValue && value_confirm>20)
				{
					LeftEdge_X[row] = line;
					LeftEdge_GetFlag[row]=1;
					break;
				}
			}
		
			for(line=RightStart_line;line<MT9V03X_W-2;line++)//右边缘
			{
				if(line<MT9V03X_W-4)
				  value_confirm = image[row][line] - image[row][line+4];
				else
					value_confirm = image[row][line] - image[row][180];
				if((image[row][line] - image[row][line+2])>MaxValue && value_confirm>20)
				{
					RightEdge_X[row] = line;
					RightEdge_GetFlag[row]=1;
					break;
				}
			}	 
		}

		if(row<58)
		{
			if(ABS(image[row][(RightEdge_X[row]+LeftEdge_X[row])/2]-image[row+2][(RightEdge_X[row+2]+LeftEdge_X[row+2])/2])>40)//||image[row][(RightEdge_X[row]+LeftEdge_X[row])/2]<130
			{
				LeftEdge_GetFlag[row]=0;
				RightEdge_GetFlag[row]=0;
			}
	  }
	
		if(((RightEdge_GetFlag[row]==0&&LeftEdge_GetFlag[row]==0&&(RightEdge_GetFlag[row+1]==1||LeftEdge_GetFlag[row+1]==1))||RightEdge_X[row]-LeftEdge_X[row]<20)&&split_flag==0&&ABS(Middle_X[row+1]-90)>20&&L_cross_flag==0&&R_cross_flag==0)
		{
			launch_flag=1;
			launch_row=Middle_X[row+1];
		}
		
		if(launch_flag==1)
			Middle_X[row]=launch_row;
		
		if(row==1)
			launch_flag=0;
	
		if(launch_flag==0||row>40)
		{
			if((LeftEdge_GetFlag[row]==1||RightEdge_GetFlag[row]==1))       //中线处理
			{
				Middle_X[row] = (RightEdge_X[row] + LeftEdge_X[row])/2;
			}
			
	  }
		else
			Middle_X[row] = Middle_X[row+1];
		
			if(LeftEdge_GetFlag[row]==1)                            //起点设置
				LeftStart_line = LeftEdge_X[row] + 10;
			else if(split_flag==1)
				LeftStart_line = Middle_X[row] + 20;
			else
			{
				if(LeftEdge_GetFlag[row+1]==1&&RightEdge_GetFlag[row+1]==1)
					LeftStart_line = Middle_X[row+1];
				else
					LeftStart_line = Middle_X[row];
			}
			
			if(RightEdge_GetFlag[row]==1)
				RightStart_line = RightEdge_X[row] - 10;
			else if(split_flag==1)
				RightStart_line = Middle_X[row] - 20;
			else
			{
				if(LeftEdge_GetFlag[row+1]==1&&RightEdge_GetFlag[row+1]==1)
					RightStart_line = Middle_X[row+1];
				else
					RightStart_line = Middle_X[row];
			}
			
			if(LeftStart_line>MT9V03X_W-1)                           //限制出界
				LeftStart_line = MT9V03X_W-1;
			if(LeftStart_line>RightEdge_X[row])
				LeftStart_line = RightEdge_X[row] - 5;
			if(RightStart_line<1)
				RightStart_line = 1;
			if(RightStart_line<LeftEdge_X[row])
				RightStart_line = LeftEdge_X[row] + 5;
			if(LeftStart_line>RightStart_line)
			{
				line = LeftStart_line;
				LeftStart_line=RightStart_line;
				RightStart_line=line;
			}

	}
	
	start = Middle_X[MT9V03X_H-2];
}

void Zebra(void)
{
	int i;	
	uint8 jump=0,jump1=0,jump2=0,zebra_jump=0;
	int count=0,up_row=0,down_row=0,zebra_endrow,end_row=0;
	static uint8 ZebraCount=0,stage=0,through_flag=0,start_line = 90,t = 0;
	static float k;
	float p_error = 0;
  uint8 get=0;
	int boundary=0,middle=0;
	p_error = ( 34-(int)Pitch);
	if(ZebraFlag==0)
	{
		for(line=120;line>60;line=line-2)
		{
			if(ABS(image[15][line] - image[15][line-2])>20)
			{
				jump++;
			}
		}
		
		if(jump>=8)
		{
			ZebraFlag = 1;
		}

	}

	
	else if(ZebraFlag==2)
	{		
		for(line=120;line>=60;line--)
		{
			if(ABS(image[45][line] - image[45][line-2])>20)
			{
				jump++;
			}
			
			if(ABS(image[50][line] - image[50][line-2])>20)
			{
				jump1++;
			}	
			
			if(ABS(image[55][line] - image[55][line-2])>20)
			{
				jump2++;
			}
		}
		
		if(jump<4&&jump1<4&&jump2<4&&t==0)//
		{
			t = 1;	
		}
		if(t!=0)
		{
			t++;
			if(t>=3)
			{
				t = 0;
				k = 0;
				stage=0;
				through_flag=0;
				ZebraFlag = 0;
				SpecialCount++;
			}
		}
	}
//	else if(ZebraFlag==3)
//	{
//		t++;
//		if(t>20)
//		{
//			t = 0;
//			k = 0;
//			stage=0;
//			through_flag=0;
//			ZebraFlag = 0;
//		}
//	}
	
	if(ZebraFlag!=0)
	{
		if(garage_dir==0)
		{
			if(through_flag!=2)
			{
				for(row=59;row>5;row--)
				{
					for(line=start_line+30;line>start_line-30;line=line-2)
					{
						if(ABS(image[row][line] - image[row][line-2])>20)
						{
							zebra_jump++;
						}
					}
					if(zebra_jump>6)
					{
						if(get==0)
							down_row = row+6;
						get++;
						zebra_jump = 0;
					}
					else if(get!=0&&down_row>30)
					{
						up_row = row-5;
						break;
					}
			 }
		 }
		
//		if(down_row>50&&through_flag==0)
//			through_flag = 1;
		if((up_row>30)&&through_flag==0)
		{
			ZebraFlag = 2;
			through_flag = 2;
		}
//			ips114_showint16(50,7,down_row);
//			ips114_showint16(100,7,up_row);
		if(through_flag==0)
		{
			for(row=59;row>=down_row;row--)
			{
				if(RightEdge_GetFlag[row]==1)
				{
					end_row = row-1;
					break;
				}
			}
//			ips114_showint16(150,7,end_row);
			if(down_row!=0)
			{
				if(end_row!=0)
				  k = erchengfa(down_row,end_row,Right);
				else
					k = erchengfa(down_row,down_row+10,Right);
				for(row=20;row<=40;row++)
				{
					RightEdge_X[row] = RightEdge_X[down_row] + k * (row-down_row);
					Middle_X[row] = RightEdge_X[row] - ( Half_Width[row] + 3*p_error );
				}
			}
	  }
		
//		if(through_flag==1)
		else
		{
//			k = erchengfa(up_row-10,up_row-5,Right);
			for(row=20;row<=40;row++)
			{
//				RightEdge_X[row] = RightEdge_X[up_row-10] + k * (row-(up_row-10));
				Middle_X[row] = 89;
			}
		}
//		if(through_flag==2)
//		{
//			k = erchengfa(20,40,Right);
//			for(row=20;row<=40;row++)
//			{
//				RightEdge_X[row] = RightEdge_X[20] + k * (row-20);
//				Middle_X[row] = RightEdge_X[row] - ( Half_Width[row] + 3*( 30-(int)Pitch) );
//			}
//		}
//		start_line = Middle_X[40];

		}
		
		if(garage_dir==1)
		{
			if(through_flag!=2)
			{
				for(row=59;row>5;row--)
				{
					for(line=start_line+30;line>start_line-30;line=line-2)
					{
						if(ABS(image[row][line] - image[row][line-2])>20)
						{
							zebra_jump++;
						}
					}
					if(zebra_jump>6)
					{
						if(get==0)
							down_row = row+6;
						get++;
						zebra_jump = 0;
					}
					else if(get!=0&&down_row>30)
					{
						up_row = row-5;
						break;
					}
			 }
		 }
			
//		if(down_row>50&&through_flag==0)
//			through_flag = 1;
		
		if((up_row>30)&&through_flag==0)
		{
			ZebraFlag = 2;
			through_flag = 2;
		}

		if(through_flag==0)
		{
			for(row=59;row>=down_row;row--)
			{
				if(LeftEdge_GetFlag[row]==1)
				{
					end_row = row-1;
					break;
				}
			}
//			if(down_row!=0&&up_row>10)
//			{
//				k = (float)(LeftEdge_X[down_row] - LeftEdge_X[up_row])/(down_row-up_row);
//				for(row=15;row<=45;row++)
//				{
//					LeftEdge_X[row] = LeftEdge_X[down_row] + k * (row-down_row);
//					Middle_X[row] = LeftEdge_X[row] + (Road_Width[row]+(int)((float)(180.0f-4.0f*Pitch)))/2;
//				}
//			}
			if(down_row!=0)
			{
				if(end_row!=0)
				  k = erchengfa(down_row,end_row,Left);
				else
				  k = erchengfa(down_row,down_row+10,Left);
				for(row=20;row<=40;row++)
				{
					boundary = LeftEdge_X[down_row] + k * (row-down_row);
					if(boundary<1)
						boundary = 1;
					LeftEdge_X[row] = boundary;
					Middle_X[row] = LeftEdge_X[row] + ( Half_Width[row] + 3*p_error );
				}
			}
	  }

//		if(through_flag==1)
		else
		{
//			k = erchengfa(up_row-15,up_row-8,Left);
			for(row=20;row<=40;row++)
			{
//				LeftEdge_X[row] = LeftEdge_X[up_row-15] + k * (row-(up_row-15));
				Middle_X[row] = 89;
			}
		}
//		if(through_flag==2)
//		{
//			k = erchengfa(20,40,Left);
//			for(row=20;row<=50;row++)
//			{
//				LeftEdge_X[row] = LeftEdge_X[20] + k * (row-20);
//				Middle_X[row] = LeftEdge_X[row] + ( Half_Width[row] + 3*( 30-(int)Pitch) );
//			}
//		}
//		start_line = Middle_X[40];

		}
	}
}

int Road = 0;
void Byroad(void)
{
	static uint8 ByroadCount=0,t=0;
	uint8 angle_row=0,L_angle_row = 0,R_angle_row = 0,angle_line = 0;
	uint8 a1=0,a2=0,i=0,detect_line=0,stage=0,j=0;
	float p_error = 0;
	p_error = ( 34-(int)Pitch);
	if(ByroadFlag==0)              
	{
		for(row=40;row>10;row--)
		{
			if((LeftEdge_X[row]-LeftEdge_X[row-2])>=1&&(LeftEdge_X[row]-LeftEdge_X[row-2])<40&&LeftEdge_X[row]-LeftEdge_X[row+2]>=1&&(LeftEdge_X[row]-LeftEdge_X[row+2])<40&&LeftEdge_GetFlag[row-2]==1)//
			{
				L_angle_row = row-1;
//				if((image[row-4][LeftEdge_X[row]-4]-image[row][LeftEdge_X[row]-4])>MaxValue)
					a1=1;			
			}
			if((RightEdge_X[row]-RightEdge_X[row-2])<=-1&&(RightEdge_X[row]-RightEdge_X[row-2])>-40&&RightEdge_X[row]-RightEdge_X[row+2]<=-1&&(RightEdge_X[row]-RightEdge_X[row+2])>-40&&RightEdge_GetFlag[row-2]==1)//
			{
				R_angle_row = row-1;
//				if((image[row-4][RightEdge_X[row]+4]-image[row][RightEdge_X[row]+4])>MaxValue)
					a2=1;
			}
			if(a1==1&&a2==1)
			{
				angle_row = row;
			  break;
			}
		}
		a1 = 0;
		a2 = 0;
		
		if(angle_row!=0)
		{
			for(i=L_angle_row;i>=L_angle_row-5;i--)
			{
				if(LeftEdge_GetFlag[i]==0)
				{
					L_angle_row = 0;
					break;
				}
			}
			if(L_angle_row!=0)
			for(i=R_angle_row;i>=R_angle_row-5;i--)
			{
				if(RightEdge_GetFlag[i]==0)
				{
					R_angle_row = 0;
					break;
				}
			}
			if(L_angle_row!=0&&R_angle_row!=0)
			{
				ByroadCount++;
				ByroadFlag = 1;
			}
	  }
		if(ByroadFlag==0)
		{
			angle_row = 0;
			if(L_state<3&&R_state<3)
			{
				for(row=15;row>2;row--)
				{			
					if((image[row][90]-image[row-2][90]>20)) 
					{
						angle_row = row;
//						angle_line = 90;
						break;
					}
				}
				
			}
			if(angle_row!=0)
			{
//				for(i=angle_row;i>2;i--)
//				{
//					for(j=90;j<179;j++)
//					{
//						if(image[i][j]-image[i][j+2]>20)
//						{
//							angle_row = 0;
//							break;
//						}
//					}
//					if(angle_row!=0)
//					for(j=90;j>2;j--)
//					{
//						if(image[i][j]-image[i][j-2]>20)
//						{
//							angle_row = 0;
//							break;
//						}
//					}
//					if(angle_row==0)
//						break;
//				}
        if(angle_row!=0)
				{
					for(i=50;i<130;i++)
					{
						for(j=angle_row;j>2;j--)
						{
							if(image[j][i]-image[j-2][i]>20)
							detect_line++;
						}
					}
					if(detect_line>70)
					{
//						start_flag = 4;
							ByroadFlag = 1;
							ByroadCount++;
					}
			 }
			}
		}
		if(ByroadFlag==0)
		{
			for(row=40;row<=59;row++)
			{
				if(LeftEdge_X[row]-LeftEdge_X[row-1]>=1&&LeftEdge_X[row]-LeftEdge_X[row-1]<40&&LeftEdge_GetFlag[row-1]==1)//
				{
					a1=1;
				}
				if(RightEdge_X[row]-RightEdge_X[row-1]<=-1&&RightEdge_X[row]-RightEdge_X[row-1]>-40&&RightEdge_GetFlag[row-1]==1)//
				{
					a2=1;
				}
				if(a1==1&&a2==1)
				{
					 stage = 3;
					 angle_row = row+2;
					 if(angle_row>59)
						 angle_row = 59;
					 break;
				}
			}

			if(stage==3)
			{
				for(i=angle_row;i>angle_row-10;i--)
				{
					if((RightEdge_X[row]-LeftEdge_X[row])<(RightEdge_X[row+1]-LeftEdge_X[row+1]))
					{
						angle_row = 0;
						break;
					}
					
				}
				if(angle_row!=0)
				for(i=25;i>5;i--)
				{
					if((image[i][Middle_X[angle_row]]-image[i-2][Middle_X[angle_row]]>20)) 
					{
						ByroadFlag = 1;
						ByroadCount++;
						break;
					}
				}	
			}
		}
	}
	
	if(ByroadFlag!=0)
	{
		if(ByroadFlag==2)
		{
			if((AM.Final)>600&&Pitch>24)
			{
				SpecialCount++;
				ByroadFlag = 0;
			}
	  }
		
		if(ByroadFlag==1)
		{
			//start_flag = 4;
			if( (AM.Final)<200 )
				ByroadFlag = 2;
		}

		if(Bys_flag==1)
		{
			if(ByroadCount==0)
			{
				if(Byroad_dir[ByroadCount]==0)
					for(row=40;row>=20;row--)
						Middle_X[row] = LeftEdge_X[row]  + ( Half_Width[row] + 3*p_error );
				else
					for(row=40;row>=20;row--)
					  Middle_X[row] = RightEdge_X[row] - ( Half_Width[row] + 3*p_error );	
			}
			else if(ByroadCount==1)
			{
				if(Byroad_Num==ByroadCount)
				{
					if(Byroad_dir[ByroadCount]==0)
						for(row=40;row>=20;row--)
							Middle_X[row] = LeftEdge_X[row]  + 1.1f*( Half_Width[row] + 3*p_error );	
					else
						for(row=40;row>=20;row--)
							Middle_X[row] = RightEdge_X[row] - 1.1f*( Half_Width[row] + 3*p_error );
					
					if(Road>1000 || start_flag != 4)
						uart_putchar(WIRELESS_UART,0x23);
					if(Road>7500)
						start_flag = 4;
				}
				else
				{
					if(Byroad_dir[ByroadCount]==0)
						for(row=40;row>=20;row--)
							Middle_X[row] = LeftEdge_X[row]  + ( Half_Width[row] + 3*p_error );
					else
						for(row=40;row>=20;row--)
							Middle_X[row] = RightEdge_X[row] - ( Half_Width[row] + 3*p_error );	
				}
			}
			else if(ByroadCount==2)
			{
				if(Byroad_Num==ByroadCount)
				{
					if(Byroad_dir[ByroadCount]==0)
						for(row=40;row>=20;row--)
							Middle_X[row] = LeftEdge_X[row]  + 1.1f*( Half_Width[row] + 3*p_error );	
					else
						for(row=40;row>=20;row--)
							Middle_X[row] = RightEdge_X[row] - 1.1f*( Half_Width[row] + 3*p_error );
					
					if(Road>1000 || start_flag != 4)
						uart_putchar(WIRELESS_UART,0x23);
					if(Road>7500)
						start_flag = 4;
				}
				else
				{
					if(Byroad_dir[ByroadCount]==0)
						for(row=40;row>=20;row--)
							Middle_X[row] = LeftEdge_X[row]  + ( Half_Width[row] + 3*p_error );
					else
						for(row=40;row>=20;row--)
							Middle_X[row] = RightEdge_X[row] - ( Half_Width[row] + 3*p_error );	
				}
			}
			else if(ByroadCount==3)
			{
				if(Byroad_Num==ByroadCount)
				{
					if(Byroad_dir[ByroadCount]==0)
						for(row=40;row>=20;row--)
							Middle_X[row] = LeftEdge_X[row]  + 1.1f*( Half_Width[row] + 3*p_error );	
					else
						for(row=40;row>=20;row--)
							Middle_X[row] = RightEdge_X[row] - 1.1f*( Half_Width[row] + 3*p_error );
					
					if(Road>1000 || start_flag != 4)
						uart_putchar(WIRELESS_UART,0x23);
					if(Road>7500)
						start_flag = 4;
				}
				else
				{
					if(Byroad_dir[ByroadCount]==0)
						for(row=40;row>=20;row--)
							Middle_X[row] = LeftEdge_X[row]  + ( Half_Width[row] + 3*p_error );
					else
						for(row=40;row>=20;row--)
							Middle_X[row] = RightEdge_X[row] - ( Half_Width[row] + 3*p_error );	
				}
			}
		}
	}
}

uint8 L_cross_flag = 0,R_cross_flag = 0;
void Cross(void)
{
	int min = RightEdge_X[50],max = RightEdge_X[50];
	int min_row = 50,max_row = 50;
   for(row=50;row>=10;row--)
	{
	    if(RightEdge_X[row]<min)
			{
			  min = RightEdge_X[row];
				min_row = row;
			}
			if(LeftEdge_X[row]>max)
			{
				max = LeftEdge_X[row];
				max_row = row;
			}			
	}
	if(L_state<=3&&min_row>=20&&min_row<=40&&(LeftEdge_GetFlag[min_row]||RightEdge_GetFlag[min_row])&&RightEdge_X[min_row]-RightEdge_X[min_row+3]<-1&&RightEdge_X[min_row]-RightEdge_X[min_row+3]>=-8&&RightEdge_X[min_row]-RightEdge_X[min_row-3]<-1&&RightEdge_X[min_row]-RightEdge_X[min_row-3]>=-8)
	{
   // L_cross_flag=1;
	  for(row=min_row;row>=10;row--)
		{
		  RightEdge_X[row] = RightEdge_X[min_row]-10;
		}
	
	}
	if(R_state<=3&&max_row>=20&&max_row<=40&&(LeftEdge_GetFlag[max_row]||RightEdge_GetFlag[max_row])&&LeftEdge_X[max_row]-LeftEdge_X[max_row+3]>1&&LeftEdge_X[max_row]-LeftEdge_X[max_row+3]<=8&&LeftEdge_X[max_row]-LeftEdge_X[max_row-3]>1&&LeftEdge_X[max_row]-LeftEdge_X[max_row-3]<=8)
	{
		
		  for(row=max_row;row>=10;row--)
		{
		  LeftEdge_X[row] = LeftEdge_X[max_row] + 10;
		}
	}


}

float ramp_angle = 0;
void Ramp(void)
{
	uint8 ramp_count=0;
	static uint8 t = 0,stage = 0;
	float p_error = 0;
	int i=0,L_endrow=0,R_endrow=0;
	p_error = ( 34-(int)Pitch);

	if(RampFlag==0)
	{
		if((Speed_R+Speed_L)/2>1500)
			stage = 1;
		if(stage==1)
		{
			for(i=59;i>15;i--)
			{
			if(RightEdge_GetFlag[i]==1)
			{
				R_endrow = i;
				break;
			}
		  }
			for(i=59;i>15;i--)
			{
			if(LeftEdge_GetFlag[i]==1)
			{
				L_endrow = i;
				break;
			}
		  }
			if(LeftStraightLineCount(10,L_endrow)<3&&RightStraightLineCount(10,R_endrow)<3)
			{
	//			if(ABS(erchengfa(20,40,Right)*100+erchengfa(20,40,Left)*100)<20&&(erchengfa(20,40,Left)!=0||erchengfa(20,40,Right)!=0))
	//			{
					for(row=20;row>=5;row--)
				   if(LeftEdge_GetFlag[row]==1&&RightEdge_GetFlag[row]==1)
						if( (RightEdge_X[row]-LeftEdge_X[row])-( ( Half_Width[row] + 3*p_error )*2 ) >(50-row/2) )
							ramp_count++;
						
					if(ramp_count>=12)
					{
//						start_flag = 4;
						RampFlag=1;
					}
					ramp_count = 0;
	//		  }
			}
	  }
	}

	if(RampFlag==1)
	{
		if(AM.Final>1000)
			stage = 2;
		if(AM.Final<800&&stage==2)
		{
			RampFlag = 2;
		}
	}
	if(RampFlag==2)
	{
		t++;
		if(t>17)
		{
			t=0;
			RampFlag=0;
			SpecialCount++;
		}
//		for(row=59;row>=40;row--)
//			if( ( (( Half_Width[row] + 3*p_error )*2) - (RightEdge_X[row]-LeftEdge_X[row]) ) <10 )
//				ramp_count++;
//			
//		if(ramp_count>=10)
//		{
//			RampFlag=0;
//		}
	}

}


void LeftRound(void)
{
	int i=0;
	uint8 LeftRound_row=0,LeftEnd_row=0,RightEnd_row=0,up_row=0,down_row=0;
	uint8 detect_line=0,line_d=0,angle_row=0;;
	static uint8 stage = 0,step = 0;
	static float t=0;
	float p_error = 0;
	p_error = ( 34-(int)Pitch);
	
	if(LeftRoundFlag==0)
	{
		for(row=57;row>=20;row--)
		{
			if(LeftEdge_GetFlag[row]==1&&RightEdge_GetFlag[row]==1&&LeftEdge_GetFlag[row-1]==0&&LeftEdge_GetFlag[row-2]==0&&RightEdge_GetFlag[row-1]==1&&RightEdge_GetFlag[row-2]==1)
			{
			  LeftRound_row=row-1;
				break;
			}

		}
		if(LeftRound_row!=0)
		{
			for(row=LeftRound_row-1;row>=LeftRound_row-10;row--)
			{
				if(LeftEdge_GetFlag[row]==1)
				{
					LeftRound_row = 0;
					break;
				}
			}
			if(LeftRound_row!=0)
			{
				for(row=MT9V03X_H-1;row>=20;row--)
				{
					if(RightEdge_GetFlag[row]==1)
					{
						RightEnd_row = row;
						break;
					}
				}
				if(RightStraightLineCount(5,RightEnd_row)<3)
				{
					
					LeftRoundFlag=1;
				}
	  	}
		}
  }
	
	else if(LeftRoundFlag==1)
	{
		if(stage==0)
		{
				if(AM.Final>900)
				{
					stage = 1;
				}
		}
		if(LargeLeftRoundFlag[LeftRoundCount]==1)  //大环岛
		{
			if(stage==1)
			{
				for(row=30;row>10;row--)
				{
					if( (RightEdge_X[row]-LeftEdge_X[row])-( ( Half_Width[row] + 3*p_error )*2 ) >(50-row/2) )
							detect_line++;
				}
				if(detect_line>10)
				{
					LeftRoundFlag = 2;
				}
			}
		}
		else
		{
			if(stage==1)
			{
				for(row=20;row<=50;row++)
				{
					if(LeftEdge_X[row]-LeftEdge_X[row-4]>=1)
					{
						LeftRoundFlag = 2;
						break;
					}
				}

			}
	  }
	}

  else if(LeftRoundFlag==2)
	{
		if(ABS(Yaw)>40)
			LeftRoundFlag = 3;
	}
	else if(LeftRoundFlag==3)
	{
		for(row=30;row>15;row--)
		{
			if(RightEdge_GetFlag[row]==0)
				break;
			if((image[row-4][RightEdge_X[row]+2]-image[row][RightEdge_X[row]+2]>20)&&ABS(Yaw)>200) //||split_flag==1
			{
					LeftRoundFlag = 4;
					break;
			}
		}
	}
	else if(LeftRoundFlag==4&&ABS(Yaw)>340)
	{
//		start_flag = 4;
		LeftRoundFlag = 5;
	}

	else if(LeftRoundFlag==5)
	{	
		stage = 1;
		for(i=30;i>20;i--)
		{
			if(LeftEdge_GetFlag[i]==0)
			{
				stage = 2;
				break;
			}
		}
		if(stage==1)
		{
			stage = 0;
			t=0;
			SpecialCount++;
			LeftRoundCount++;
			LeftRoundFlag = 0;
		}		
//		for(i=59;i>20;i--)
//		{

//				if(LeftEdge_GetFlag[i]==0&&LeftEdge_GetFlag[i-1]==1)
//					if(i>40)
//					{
//						stage = 0;
//						t=0;
//			      SpecialCount++;
//						LeftRoundCount++;
//						LeftRoundFlag = 0;
//						break;
//					}
//		}
	}
	
	if(LeftRoundFlag==1)
	{
		for(row=20;row<=40;row++)
			Middle_X[row]=RightEdge_X[row] - 1.1f*( Half_Width[row] + 3*p_error );
	}
	else if(LeftRoundFlag==2)
	{
		for(row=20;row<=40;row++)
		{
			if(LeftEdge_X[row]-LeftEdge_X[59]>50)
				LeftEdge_X[row] = 0;
			Middle_X[row] = LeftEdge_X[row] + (int)(LeftRoundRadio_2*( Half_Width[row] + 3*p_error ));
		}
	}
	else if(LeftRoundFlag==4)
		for(row=20;row<=40;row++)
			Middle_X[row] = LeftEdge_X[row] + (int)(LeftRoundRadio_4*( Half_Width[row] + 3*p_error ));
	else if(LeftRoundFlag==5)
	{
			for(row=20;row<=40;row++)
				Middle_X[row] = RightEdge_X[row] -  ( Half_Width[row] + 3*p_error );
	}
}

void RightRound(void)
{
	int i=0;
	uint8 RightRound_row=0,LeftEnd_row=0,RightEnd_row=0,down_row=0,up_row=0;
	uint8 detect_line=0,line_d=0,angle_row=0;
	static int stage = 0,step=0;
	static float t=0;
  float p_error = 0;
	p_error = ( 34-(int)Pitch);

	if(RightRoundFlag==0)
	{
		for(row=57;row>=20;row--)
		{
			if(RightEdge_GetFlag[row]==1&&LeftEdge_GetFlag[row]==1&&RightEdge_GetFlag[row-1]==0&&RightEdge_GetFlag[row-2]==0&&LeftEdge_GetFlag[row-1]==1&&LeftEdge_GetFlag[row-2]==1)
			{
				RightRound_row=row-1;
				break;
			}
		}
		if(RightRound_row!=0)
		{
			for(row=RightRound_row-1;row>=RightRound_row-10;row--)
			{
				if(RightEdge_GetFlag[row]==1)
				{
					RightRound_row = 0;
					break;
				}
			}
		if(RightRound_row!=0)
		{
			for(row=MT9V03X_H-1;row>=40;row--)
			{
				if(LeftEdge_GetFlag[row]==1)
				{
					LeftEnd_row = row;
					break;
				}
			}
			if(LeftStraightLineCount(5,LeftEnd_row)<3)
				RightRoundFlag=1;
		}
		}
//		if(RightRoundFlag==0)
//		{
//			if(step==0)
//			{
//				for(i=10;i<=59;i++)
//				{
//					if(RightEdge_GetFlag[i]==1)
//					{
//						angle_row = 1;
//						break;
//					}
//				}
//				if(angle_row==0)
//				if(LeftStraightLineCount(10,50)<3)
//					step = 1;
//		 }
//			if(step==1)
//			{
//				for(row=30;row>=10;row--)
//				{
//					if(RightEdge_GetFlag[row]==0&&RightEdge_GetFlag[row-1]==1)
//					{
//						line_d = 1;
//						RightEnd_row=row-1;
//						break;
//					}
//				}
//				for(row=RightEnd_row;row>=3;row--)
//				{
//					if(RightEdge_GetFlag[row]==0)
//					{
//						angle_row=row+1;
//						break;
//					}
//				}
//				if(RightEnd_row!=0&&line_d==1&&angle_row==0)
//					if(RightStraightLineCount(3,RightEnd_row)>=10)
//					{
//						RightRoundFlag = 1;
//						start_flag = 4;
//					}


//			}
//			if(RightEdge_GetFlag[30]==1)
//					step = 0;
			
//		}
  }
	
	else if(RightRoundFlag==1)
	{
		if(stage==0)
		{
			 	if(AM.Final>900)
				{
				  stage = 1;
				}
		}
		if(stage==1)
		{
			if(LargeRightRoundFlag[RightRoundCount]==1)  //大环岛
			{
				if(stage==1)
				{
					for(row=30;row>10;row--)
					{
						if( (RightEdge_X[row]-LeftEdge_X[row])-( ( Half_Width[row] + 3*p_error )*2 ) >(50-row/2) )
								detect_line++;
					}
					if(detect_line>10)
					{
						RightRoundFlag = 2;
					}
				}
			}
		else
		{
			for(row=20;row<=40;row++)
			{
				if(RightEdge_X[row]-RightEdge_X[row+4]>=1)
				{
					RightRoundFlag = 2;
					break;
				}
			}
		}
	}
	}
	
  else if(RightRoundFlag==2)
	{
		if(ABS(Yaw)>40)
			RightRoundFlag = 3;
//		for(row=20;row<59;row++)
//		{
//			if(LeftEdge_GetFlag[row]==1&&LeftEdge_GetFlag[row+1]==0) //||split_flag==1
//			{
//				if(LeftEdge_X[row]-LeftEdge_X[row+1]>20)
//					stage = 2;
//				if(row>45&&stage==2)
//				{
//					RightRoundFlag = 3;
//					break;
//				}
//			}
//		}
		
	}
	else if(RightRoundFlag==3)
	{
		for(row=30;row>10;row--)
		{
			if(LeftEdge_GetFlag[row]==0)
				break;
			if((image[row-4][LeftEdge_X[row]-2]-image[row][LeftEdge_X[row]-2]>20)&&ABS(Yaw)>200) //||split_flag==1
			{
					RightRoundFlag = 4;
					break;
			}
		}
	}
	else if(RightRoundFlag==4&&ABS(Yaw)>340)//(AM.Final>1100)
	{
			RightRoundFlag = 5;
		  t = 0;
	}

	else if(RightRoundFlag==5)
	{
		stage = 1;
		for(i=30;i>20;i--)
		{
			if(RightEdge_GetFlag[i]==0)
			{
				stage = 2;
				break;
			}
		}
		if(stage==1)
		{
			stage = 0;
			t=0;
			SpecialCount++;
			RightRoundCount++;
			RightRoundFlag = 0;
		}
//		for(i=59;i>20;i--)
//		{
////			if(RightEdge_GetFlag[i]==0)
////				stage = 3;
////			if(stage==3)
////			{
//				if(RightEdge_GetFlag[i]==0&&RightEdge_GetFlag[i-1]==1)
//					if(i>40)
//					{
////						start_flag = 4;
//						stage = 0;
//						t=0;
//			      SpecialCount++;
//						RightRoundCount++;
//						RightRoundFlag = 0;
//						break;
//					}
////			}
//		}
	}
	
	if(RightRoundFlag==1)
	{
		for(row=20;row<=40;row++)
		{
				Middle_X[row] = LeftEdge_X[row] + 1.1*( Half_Width[row] + 3*p_error );
		}
	}
	else if(RightRoundFlag==2)
	{
		for(row=20;row<=40;row++)
		{
			if(RightEdge_X[row]-RightEdge_X[59]<-50)
				RightEdge_X[row] = 180;
			Middle_X[row] = RightEdge_X[row] -  (int)( RightRoundRadio_2*( Half_Width[row] + 3*p_error) );
		}
		
	}
	else if(RightRoundFlag==4)
		for(row=20;row<=40;row++)
			Middle_X[row] = RightEdge_X[row] - (int)( RightRoundRadio_4*( Half_Width[row] + 3*p_error ));
	else if(RightRoundFlag==5)
	{
		for(row=20;row<=40;row++)
			Middle_X[row] = LeftEdge_X[row] + ( Half_Width[row] + 3*p_error );
	}
}

uint8 RightStraightLineCount(uint8 start_row,uint8 final_row)
{
	float Line_fitting_k=0;
	int error_row=0;
	Line_fitting_k=(float)(RightEdge_X[start_row]-RightEdge_X[final_row])/(start_row-final_row);
	if(RightEdge_GetFlag[start_row]==0||RightEdge_GetFlag[final_row]==0)
		error_row = 100;
	else
	{
		for(row=start_row-1;row<final_row;row++)
		if(ABS(RightEdge_X[row]-(Line_fitting_k*(row-start_row)+RightEdge_X[start_row]))>=2)
		error_row++;
	}
  return error_row;
//	float krs=0;
//	int error=0;
//	if(RightEdge_GetFlag[start_row]==0||RightEdge_GetFlag[final_row]==0)
//		error = 100;
//	else
//	{
//	krs=(float)(RightEdge_X[start_row]-RightEdge_X[final_row])/(start_row-final_row);
//	
//	for(row=start_row;row<=final_row;row++)
//		error+=ABS(RightEdge_X[row]-(krs*(row-start_row)+RightEdge_X[start_row]));
//	}
//  return error;
}

uint8 Middle_LineCount(uint8 start_row,uint8 final_row)
{
	float Line_fitting_k=0;
	int error_row=0;
	Line_fitting_k = erchengfa(start_row,final_row,Middle);;
	if(RightEdge_GetFlag[start_row]==0||RightEdge_GetFlag[final_row]==0||LeftEdge_GetFlag[start_row]==0||LeftEdge_GetFlag[final_row]==0)
		error_row = 100;
	else
	{
		for(row=start_row-1;row<=final_row;row++)
		if(ABS(Middle_X[row]-(Line_fitting_k*(row-start_row)+Middle_X[start_row]))>=2)
		error_row++;
	}
  return error_row;

}

uint8 LeftStraightLineCount(uint8 start_row,uint8 final_row)
{
	float Line_fitting_k=0;
	int error_row=0;
	Line_fitting_k=(float)(LeftEdge_X[start_row]-LeftEdge_X[final_row])/(start_row-final_row);
	if(LeftEdge_GetFlag[start_row]==0||LeftEdge_GetFlag[final_row]==0)
		error_row = 100;
	else
	{
	  for(row=start_row-1;row<final_row;row++)
	  if(ABS(LeftEdge_X[row]-(Line_fitting_k*(row-start_row)+LeftEdge_X[start_row]))>=2)
		  error_row++;
  }
	
  return error_row;
//	float kls=0;
//	int error=0;
//	if(LeftEdge_GetFlag[start_row]==0||LeftEdge_GetFlag[final_row]==0)
//		error = 100;
//	else
//	{
//	kls=(float)(LeftEdge_X[start_row]-LeftEdge_X[final_row])/(start_row-final_row);

//	for(row=start_row;row<=final_row;row++)
//		error+=ABS(LeftEdge_X[row]-(kls*(row-start_row)+LeftEdge_X[start_row]));
//	}
//	return error;
}

float err_calculation(uint8 start_row,uint8 final_row)//start_row<end_row
{
	float err=0;
	
	for(row=start_row;row<final_row;row++)
		err+= (MIDDLE_ROAD-Middle_X[row]);

	return (float)err/(final_row-start_row);
}


/*************************************************************
	*	@brief			简单求根公式,二分法求解，六次运算
	*	@param			x：待开根号的值
	*	@return     开根号后的值
	* @date				2019-05-14
	*	Sample usage:			x = m_sqrt(16);
*************************************************************/

int m_sqrt(int x)
{
	 unsigned char ans=0,p=0x80;
	 while(p!=0)
	 {
			ans+=p;
			if(ans*ans>x)
			{
				ans-=p;
			}
			p=( unsigned char)(p/2);
	 }
		return(ans);
}

float erchengfa(int startline,int endline,uint8 type)
{
  int i;
  int sumX=0,sumY=0,sumXY=0,sumX2=0;
	float a=0,b=0;
  int num=0;


	for(i=startline;i<=endline;i++)
	{
		num++;
		sumX+=i;
		sumX2 += i*i;
		if(type==0)
		{
			sumXY += i*LeftEdge_X[i];
		  sumY+=LeftEdge_X[i];
		}
		else if(type==1)
		{
			sumXY += i*RightEdge_X[i];
			sumY+=RightEdge_X[i];
		}
		else if(type==2)
		{
			sumXY += i*Middle_X[i];
			sumY+=Middle_X[i];
		}
	}
	
	a = (float)(num*sumXY - sumX*sumY)/(num*sumX2 - sumX*sumX);
//	b = (float)(sumX2*sumY - sumX*sumXY)/(num*sumX2 - sumX*sumX);
	return a;
//	for(i=startline;i<=endline;i++)
//	{       
//     Middle_X[i] = (int)(a*i + b);
//	}
}


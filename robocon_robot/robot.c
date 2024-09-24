/**
  ******************************************************************************
  * @file    robot.c
  * @author  Py
  * @version V1.0.0
  * @date    2024/5/9
  * @brief   
  ******************************************************************************
  */ 
/* Includes -------------------------------------------------------------------*/
#include "robot.h"
#include "motor.h"
#include "calculation.h"
#include "driver_usart.h"
#include "hardware.h"
#include "MIT.h"
#include "communicate.h"
#include "FSM.h"
#include "remote.h"

/* Private  variables ---------------------------------------------------------*/
int rising_round=0;
static float rising_level=130;//!!!!!!调试测量抬升所需转动圈数！！！！！！(0-160左右)
static float down_level=0;
float Last_left=0;
float Last_right=0;
float temp=0;

// unisgned short Move_Signal=0;//1:车动 0：车停
robot_move Robot_Move;


//开启吸球
void Absorb_Ball(int t)
 {
//	 if(v0!=0&&MOTOR_REAL_INFO[0].velocity_planning.flag==1&&MOTOR_REAL_INFO[1].velocity_planning.flag==1)
//	 {
//		Velocity_Planning_setpos(&MOTOR_REAL_INFO[0],MOTOR_REAL_INFO[0].REAL_ANGLE,MOTOR_REAL_INFO[0].REAL_ANGLE+30,10,20,0,0.2,0.2);
//		VelocityPlanningMODE(&MOTOR_REAL_INFO[0]);
//		 
//		 Velocity_Planning_setpos(&MOTOR_REAL_INFO[1],MOTOR_REAL_INFO[1].REAL_ANGLE,MOTOR_REAL_INFO[0].REAL_ANGLE-30,10,20,0,0.2,0.2);
//		VelocityPlanningMODE(&MOTOR_REAL_INFO[1]);
//	 }
	 
//	if(Data_mid==0)
//	{
//		USART4_FLAG=0;
//		Robot_Move.Absorb_Flag=0;
//		return;
//	}
	 
//	Velocity_Planning_setpos(&MOTOR_REAL_INFO[0],0,30,20,30,0,0.1,0.2);
//	Velocity_Planning_setpos(&MOTOR_REAL_INFO[1],0,-30,20,30,0,0.1,0.2);
//	VelocityPlanningMODE(&MOTOR_REAL_INFO[0]);
//	VelocityPlanningMODE(&MOTOR_REAL_INFO[1]);
//	if(MOTOR_REAL_INFO[0].velocity_planning.flag==1)
//	{
//	    VelCrl(&MOTOR_REAL_INFO[0],0);
//		VelCrl(&MOTOR_REAL_INFO[1],0);
//		Robot_Move.Absorb_Flag=0;
//	}
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_RESET);//夹住
	
	VelCrl(&MOTOR_REAL_INFO[0],50);
	VelCrl(&MOTOR_REAL_INFO[1],-50);
	HAL_Delay(1000);
	VelCrl(&MOTOR_REAL_INFO[0],0);
	VelCrl(&MOTOR_REAL_INFO[1],0);
	Mode=STOP;
//	HAL_Delay(2000);
//	shoot(5000);
//	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_SET);//放开
//	HAL_Delay(2000);
//	 VelCrl(&MOTOR_REAL_INFO[0],20);
//	 VelCrl(&MOTOR_REAL_INFO[1],-20);
//	 if(fabsf(MOTOR_REAL_INFO[0].REAL_ANGLE - t)<3)CurrentCrl(&MOTOR_REAL_INFO[0],0);
//	 if(fabsf(MOTOR_REAL_INFO[1].REAL_ANGLE - t)<3)CurrentCrl(&MOTOR_REAL_INFO[1],0);
//	 if(MOTOR_REAL_INFO[0].TARGET_CURRENT==0&&MOTOR_REAL_INFO[1].TARGET_CURRENT==0)temp=t;

	 
//	 Velocity_Planning_setpos(&MOTOR_REAL_INFO[0],temp,t,10,20,0,0.2,0.2);
//	VelocityPlanningMODE(&MOTOR_REAL_INFO[0]);
//	 
//	 
//	 if(MOTOR_REAL_INFO[0].FIRST_ANGLE_INTEGRAL_FLAG)
//	 {
//		VelCrl(&MOTOR_REAL_INFO[0],0);
//		temp=MOTOR_REAL_INFO[0].REAL_ANGLE;
//	 }

 }
 
 
////停止吸球
// void absorb_stop(void)
// {
//	MOTOR_REAL_INFO[0].unitMode=MOTO_OFF;
//	MOTOR_REAL_INFO[1].unitMode=MOTO_OFF;
// }
// 

/*
 *  函数名：ShootTurn
 *  功能描述：旋转射球
 *  输入参数：射球速度
 *  输出参数：无
 *  返回值：无
*/
 void ShootTurn(float v0)
 {
	
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_RESET);
	 
	if(ZONE_Choice==Blue)ctrl_motor2(DM43_ID3,0.85,3);
	else if(ZONE_Choice==Red)ctrl_motor2(DM43_ID3,-0.85,3);
	HAL_Delay(500);
	
	VelCrl(&MOTOR_REAL_INFO[2],v0);
	VelCrl(&MOTOR_REAL_INFO[3],-v0);
	HAL_Delay(500);
	VelCrl(&MOTOR_REAL_INFO[0],v0);
	VelCrl(&MOTOR_REAL_INFO[1],-v0);
	HAL_Delay(500);
	VelCrl(&MOTOR_REAL_INFO[2],0);
	VelCrl(&MOTOR_REAL_INFO[3],0);
	VelCrl(&MOTOR_REAL_INFO[0],0);
	VelCrl(&MOTOR_REAL_INFO[1],0);
	ctrl_motor2(DM43_ID3,0,3);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_SET);
 }
 
/*
  *  函数名：shoot
  *  功能描述：以给定速度发射球
  *  输入参数：初速度
  *  输出参数：无
  *  返回值：无
 */
 void shoot(float v0)
 {
//	VelCrl(&MOTOR_REAL_INFO[2],v0);
//	VelCrl(&MOTOR_REAL_INFO[3],-v0);
//	VelCrl(&MOTOR_REAL_INFO[0],v0);
//	VelCrl(&MOTOR_REAL_INFO[1],-v0);	
//	HAL_Delay(1000);
//	VelCrl(&MOTOR_REAL_INFO[2],0);
//	VelCrl(&MOTOR_REAL_INFO[3],0);
//	VelCrl(&MOTOR_REAL_INFO[0],0);
//	VelCrl(&MOTOR_REAL_INFO[1],0);
//	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_RESET);
//	USART4_FLAG=0; 
	 
	 
	 /*-------------------------------------------*/
	VelCrl(&MOTOR_REAL_INFO[2],v0);
	VelCrl(&MOTOR_REAL_INFO[3],-v0);
//	 HAL_Delay(500);
	VelCrl(&MOTOR_REAL_INFO[0],v0);
	VelCrl(&MOTOR_REAL_INFO[1],-v0);
	 
	HAL_Delay(500);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_RESET);
	HAL_Delay(500);
	VelCrl(&MOTOR_REAL_INFO[2],0);
	VelCrl(&MOTOR_REAL_INFO[3],0);
	VelCrl(&MOTOR_REAL_INFO[0],0);
	VelCrl(&MOTOR_REAL_INFO[1],0);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_SET);
	ctrl_motor2(DM43_ID3,0,2);
//	HAL_Delay(1000);
	Mode=STOP;
	USART4_FLAG=0;
	
	
	 
 }
 
static float angleup=60;
float angle_adjust=0;
 
 

 
 void self_inspection(void)
 {	
//	if(MOTOR_REAL_INFO[5].CURRENT!=0&&MOTOR_REAL_INFO[4].CURRENT!=0)
	if(MOTOR_REAL_INFO[4].CURRENT!=0)
	{
		if(MOTOR_REAL_INFO[5].self_inspection.inspection_flag==0)
		{
				VelCrl(&MOTOR_REAL_INFO[5],-250);

				if(MOTOR_REAL_INFO[5].CURRENT<-3400)
				{
					VelCrl(&MOTOR_REAL_INFO[5],0);
					MOTOR_REAL_INFO[5].LAST_ANGLE=MOTOR_REAL_INFO[5].FIRST_ANGLE_INTEGRAL_FLAG=MOTOR_REAL_INFO[5].REAL_ANGLE=0;
					MOTOR_REAL_INFO[5].self_inspection.inspection_flag=1;
					Robot_Move.R_R_Flag=DOWN;
				}
		}
		
		if(MOTOR_REAL_INFO[4].self_inspection.inspection_flag==0)
		{
				VelCrl(&MOTOR_REAL_INFO[4],250);

				if(MOTOR_REAL_INFO[4].CURRENT>3000)
				{
					VelCrl(&MOTOR_REAL_INFO[4],0);
					MOTOR_REAL_INFO[4].LAST_ANGLE=MOTOR_REAL_INFO[4].FIRST_ANGLE_INTEGRAL_FLAG=MOTOR_REAL_INFO[4].REAL_ANGLE=0;
					MOTOR_REAL_INFO[4].self_inspection.inspection_flag=1;
					Robot_Move.R_L_Flag=DOWN;
				}
		}
		
		//云台U8自检
		if(!MIT_DRIVER_REAL_INFO[2].DM43_inspection.inspection_flag)
		{
			DM43_control_cmd(DM43_ID3, 0x01);
			HAL_Delay(10);
//			ctrl_motor2(DM43_ID3,0,1);
			Robot_Move.Cloud_State=BACK;
			MIT_DRIVER_REAL_INFO[2].DM43_inspection.inspection_flag=1;
			

		}
		//左手DM自检
		if(!MIT_DRIVER_REAL_INFO[1].DM43_inspection.inspection_flag&&MOTOR_REAL_INFO[4].self_inspection.inspection_flag==1)
		{
			DM43_control_cmd(DM43_ID2, 0x01);
			HAL_Delay(10);
			ctrl_motor2(DM43_ID2,1.2,1);
			Robot_Move.Claw_State=Close;
			MIT_DRIVER_REAL_INFO[1].DM43_inspection.inspection_flag=1;
		} 
		//右手DM自检
		if(!MIT_DRIVER_REAL_INFO[0].DM43_inspection.inspection_flag&&MOTOR_REAL_INFO[5].self_inspection.inspection_flag==1)
		{

//			ctrl_motor2(DM43_ID1,-7,2);//-7.5
//			HAL_Delay(10);
//			if(MIT_DRIVER_REAL_INFO[1].REAL_ANGLE>=0)ctrl_motor2(DM43_ID2,-4,-2);//-6.5
//			else ctrl_motor2(DM43_ID2,-4,2);
//			ctrl_motor2(DM43_ID2,2,2);
//			HAL_Delay(10);
			DM43_control_cmd(DM43_ID1, 0x01);
			HAL_Delay(10);
			ctrl_motor2(DM43_ID1,0,1);
			Robot_Move.Claw_State=Close;
//			if(MIT_DRIVER_REAL_INFO[1].V_angle==0)
//			{
//				HAL_Delay(2000);
//				ctrl_motor2(DM43_ID2,-8.2,1);
//			ctrl_motor2(DM43_ID2,-3.7,1);
				MIT_DRIVER_REAL_INFO[0].DM43_inspection.inspection_flag=1; 
//			}
		}
		
		//右手抬升电机自检还未加入
		if(MOTOR_REAL_INFO[4].self_inspection.inspection_flag==1&&MIT_DRIVER_REAL_INFO[0].DM43_inspection.inspection_flag==1&&MIT_DRIVER_REAL_INFO[1].DM43_inspection.inspection_flag&&MIT_DRIVER_REAL_INFO[2].DM43_inspection.inspection_flag==1)
			Move_State.Inspection_Flag=OK;
		
	}	
}

//void left_self_inspection(void)
// {	

//	if(MOTOR_REAL_INFO[4].self_inspection.inspection_flag==0)
//	{
//			VelCrl(&MOTOR_REAL_INFO[4],250);

//			if(MOTOR_REAL_INFO[4].CURRENT>3500)
//			{
//				VelCrl(&MOTOR_REAL_INFO[4],0);
//				MOTOR_REAL_INFO[4].LAST_ANGLE=MOTOR_REAL_INFO[4].FIRST_ANGLE_INTEGRAL_FLAG=MOTOR_REAL_INFO[4].REAL_ANGLE=0;
//				MOTOR_REAL_INFO[4].self_inspection.inspection_flag=1;
//			}
//	}
//}
 //!!!!!!!!!!!可以从velocity_planning.flag得知是否完成！！！！！！！！！
 
unsigned short Rising_Count=0;
 /*
  *  函数名：rising_right
  *  功能描述：将右夹爪变动至设定值
  *  输入参数：
  *  输出参数：无
  *  返回值：无
 */
void  rising_right(int level)
{
	
	
//		switch(level)
//		{
//			case 130:
//				if(Robot_Move.Claw_State==Open)
//				{
//					//夹住
//					HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_SET);
//					HAL_Delay(200);
//					Robot_Move.Claw_State=Close;
//				}
//				
//				Velocity_Planning_setpos(&MOTOR_REAL_INFO[5],0,140,200,1500,0,0.2,0.2);
//				VelocityPlanningMODE(&MOTOR_REAL_INFO[5]);
//				if(MOTOR_REAL_INFO[5].velocity_planning.flag)
//				{
//					ctrl_motor2(DM43_ID1,1.7,4);
//				}
//			break;

//			case 0:
//				Velocity_Planning_setpos(&MOTOR_REAL_INFO[5],140,0,200,1500,0,0.2,0.3);
//				VelocityPlanningMODE(&MOTOR_REAL_INFO[5]);
//				
//					if(MOTOR_REAL_INFO[5].velocity_planning.flag||MOTOR_REAL_INFO[5].CURRENT<-3000)
//					{
//						if(Robot_Move.Claw_State==Close)
//						{
//							HAL_Delay(500);
//							HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_RESET);//松开
//							Robot_Move.Claw_State=Open ;
//						}
//					}
//			break;
//		}

switch(level)
	{
		case 130:
			if(Robot_Move.R_R_Flag==DOWN)
			{
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_SET);
				HAL_Delay(200);
				VelCrl(&MOTOR_REAL_INFO[5],1000);
				if(MOTOR_REAL_INFO[5].CURRENT>4000)
				{
					ctrl_motor2(DM43_ID1,1.7,5);
					Rising_Count++;
					VelCrl(&MOTOR_REAL_INFO[5],0);
					Robot_Move.R_R_Flag=UP;
				}
			}
			
		break;
			

		case 0:
			if(Robot_Move.R_R_Flag==UP)
			{
				VelCrl(&MOTOR_REAL_INFO[5],-1000);
				if(MOTOR_REAL_INFO[5].CURRENT<-10000)
				{
					VelCrl(&MOTOR_REAL_INFO[5],0);
					Robot_Move.R_R_Flag=DOWN;
					HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_RESET);
				}
			}
			//松开
//			Send_MoveFlag(Move_Flag);
			
		
		break;
	}
	
		
//	if(MOTOR_REAL_INFO[4].velocity_planning.flag==1)Last_SWD=SWD;

		
}

	//level：：130是拿取苗 0是放苗


//测试2024.7.3
void left_test2(int level)
{
	switch(level)
	{
		case 130:
			if(Robot_Move.Claw_State==Open)
			{
				//夹住
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_SET);
				HAL_Delay(200);
				Robot_Move.Claw_State=Close;
			}
			else
			{
				if(Robot_Move.R_L_Flag==DOWN)
				{
					Velocity_Planning_setpos(&MOTOR_REAL_INFO[4],0,-140,200,1500,0,0.2,0.2);
					VelocityPlanningMODE(&MOTOR_REAL_INFO[4]);
					if(MOTOR_REAL_INFO[4].velocity_planning.flag)
					{
	//					Send_MoveFlag(Move_Flag);
						ctrl_motor2(DM43_ID2,-1,4);
						Rising_Count++;
						if(Rising_Count==1)
						{
							if(MIT_DRIVER_REAL_INFO[1].V_angle==0)Robot_Move.R_L_Flag=UP;
						}
						else 
						{
							if(MIT_DRIVER_REAL_INFO[1].ANGLE>-1.1)Robot_Move.R_L_Flag=UP;
						}
	//					Send_MoveFlag(Move_Flag);
						
						
					}
				}
				
				
				if(Robot_Move.R_L_Flag==UP)
				{
					Velocity_Planning_setpos(&MOTOR_REAL_INFO[4],-140,-35,100,1000,0,0.2,0.2);
					VelocityPlanningMODE(&MOTOR_REAL_INFO[4]);
					if(MOTOR_REAL_INFO[4].velocity_planning.flag)
					{
						
	//					Last_SWA=SWA;
					}
				}
			}
			
		break;
			

		case 0:
			
//			Send_MoveFlag(Move_Flag);
			Velocity_Planning_setpos(&MOTOR_REAL_INFO[4],-35,0,200,1500,0,0.2,0.2);
			VelocityPlanningMODE(&MOTOR_REAL_INFO[4]);
			if(MOTOR_REAL_INFO[4].velocity_planning.flag&&Robot_Move.R_L_Flag==UP)
			{
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_RESET);//松开
	//			HAL_Delay(2000);
	//			ctrl_motor2(DM43_ID2,-4.4,4);
				VelCrl(&MOTOR_REAL_INFO[4],0);
	//			Send_MoveFlag(Move_Flag);
	//			Last_SWA=SWA;
				Robot_Move.R_L_Flag=DOWN;
			}
			
		
		break;
	}
}


void  left_Rise(int level)
{

	switch(level)
	{
		case 130:
			if(Robot_Move.Claw_State==Open)
			{
				//夹住
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_SET);
				HAL_Delay(200);
				Robot_Move.Claw_State=Close;
			}
			else
			{
				if(Robot_Move.R_L_Flag==DOWN)
				{
					Velocity_Planning_setpos(&MOTOR_REAL_INFO[4],0,-140,300,1500,0,0.2,0.2);
					VelocityPlanningMODE(&MOTOR_REAL_INFO[4]);
					if(MOTOR_REAL_INFO[4].velocity_planning.flag)
					{
	//					Send_MoveFlag(Move_Flag);
						ctrl_motor2(DM43_ID2,-1,4);
						Rising_Count++;
						if(Rising_Count==1)
						{
							if(MIT_DRIVER_REAL_INFO[1].V_angle==0)Robot_Move.R_L_Flag=UP;
						}
						else 
						{
							if(MIT_DRIVER_REAL_INFO[1].ANGLE>-2)Robot_Move.R_L_Flag=UP;
						}
	//					Send_MoveFlag(Move_Flag);
						
						
					}
				}
				
				
				if(Robot_Move.R_L_Flag==UP)
				{
					Velocity_Planning_setpos(&MOTOR_REAL_INFO[4],-140,0,300,1500,0,0.2,0.2);
					VelocityPlanningMODE(&MOTOR_REAL_INFO[4]);
					if(MOTOR_REAL_INFO[4].velocity_planning.flag)
					{
						
	//					Last_SWA=SWA;
					}
				}
			}
			
		break;
			

		case 0:
			//松开
//			Send_MoveFlag(Move_Flag);
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_RESET);
//			HAL_Delay(2000);
//			ctrl_motor2(DM43_ID2,-4.4,4);
			VelCrl(&MOTOR_REAL_INFO[4],0);
//			Send_MoveFlag(Move_Flag);
//			Last_SWA=SWA;
			Robot_Move.R_L_Flag=DOWN;
		
		break;
	}
	
		
//	if(MOTOR_REAL_INFO[4].velocity_planning.flag==1)Last_SWD=SWD;
}


void  right_test(int level)
{

	switch(level)
	{
	case 130:
			if(Robot_Move.Claw_State==Open)
			{
				//夹住
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_SET);
				HAL_Delay(200);
				Robot_Move.Claw_State=Close;
			}
			else
			{
				if(Robot_Move.R_R_Flag==DOWN)
				{
					Velocity_Planning_setpos(&MOTOR_REAL_INFO[5],0,150,300,1500,0,0.2,0.2);
					VelocityPlanningMODE(&MOTOR_REAL_INFO[5]);
					if(MOTOR_REAL_INFO[5].velocity_planning.flag)
					{
	//					Send_MoveFlag(Move_Flag);
						ctrl_motor2(DM43_ID1,1.7,5);
						Rising_Count++;
						if(Rising_Count==1)
						{
							if(MIT_DRIVER_REAL_INFO[0].V_angle==0)Robot_Move.R_R_Flag=UP;
						}
						else 
						{
							if(MIT_DRIVER_REAL_INFO[0].ANGLE<1.8)Robot_Move.R_R_Flag=UP;
						}
	//					Send_MoveFlag(Move_Flag);
						
						
					}
				}
				
				
				if(Robot_Move.R_R_Flag==UP)
				{
					Velocity_Planning_setpos(&MOTOR_REAL_INFO[5],150,40,100,1000,0,0.2,0.2);
					VelocityPlanningMODE(&MOTOR_REAL_INFO[5]);
					if(MOTOR_REAL_INFO[5].velocity_planning.flag)
					{
						
	//					Last_SWA=SWA;
					}
				}
			}
			
		break;
			

		case 0:
			
//			Send_MoveFlag(Move_Flag);
		if(Robot_Move.R_R_Flag==UP)
		{
			Velocity_Planning_setpos(&MOTOR_REAL_INFO[5],40,0,300,1500,0,0.2,0.2);
			VelocityPlanningMODE(&MOTOR_REAL_INFO[5]);
			if(MOTOR_REAL_INFO[5].velocity_planning.flag)
			{
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_RESET);//松开
	//			HAL_Delay(2000);
	//			ctrl_motor2(DM43_ID2,-4.4,4);
				VelCrl(&MOTOR_REAL_INFO[5],0);
	//			Send_MoveFlag(Move_Flag);
	//			Last_SWA=SWA;
				Robot_Move.R_R_Flag=DOWN;
			}
		}
			
		
		break;
	}
}
	
		
//	if(MOTOR_REAL_INFO[4].velocity_planning.flag==1)Last_SWD=SWD;




unsigned char Claw_State=GPIO_PIN_RESET;
 /*
  *  函数名：rising_left
  *  功能描述：将右夹爪变动至设定值
  *  输入参数：void
  *  输出参数：无
  *  返回值：无
 */
void  rising_left(int level)
{

	switch(level)
	{
		case 130:
			//夹住
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_SET);
			Claw_State=GPIO_PIN_SET;
			Velocity_Planning_setpos(&MOTOR_REAL_INFO[4],0,-150,200,1500,0,0.2,0.2);
			VelocityPlanningMODE(&MOTOR_REAL_INFO[4]);
			if(MOTOR_REAL_INFO[4].velocity_planning.flag)
			{
//				Send_MoveFlag(Move_Flag);
				MOTOR_REAL_INFO[4].TARGET_RPM=0;
				ctrl_motor2(DM43_ID2,-3.8,4);
//				Mode=0;
				if(MIT_DRIVER_REAL_INFO[1].ANGLE==-2)move_flag++;
				
			}
		break;
		
		case 0:
		
			Velocity_Planning_setpos(&MOTOR_REAL_INFO[4],-150,0,200,1500,0,0.2,0.2);
			VelocityPlanningMODE(&MOTOR_REAL_INFO[4]);
			
			if(MOTOR_REAL_INFO[4].velocity_planning.flag)
			{
				if(Claw_State==GPIO_PIN_SET)
				{
					HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_RESET);//松开
					Claw_State=GPIO_PIN_RESET;
					move_flag++;
				}
				
				
//				Send_MoveFlag(Move_Flag);
				
//				Mode=0;
			};
		break;
	}
	
		
}
	



/*
 *  函数名：Retry(void)
 *  功能描述：重试
 *  输入参数：
 *  输出参数：无
 *  返回值：无
*/
void Retry_State(void)
{
	MOTOR_REAL_INFO[5].self_inspection.inspection_flag=0;
	MOTOR_REAL_INFO[4].self_inspection.inspection_flag=0;
	MIT_DRIVER_REAL_INFO[1].DM43_inspection.inspection_flag=0;
	MIT_DRIVER_REAL_INFO[0].DM43_inspection.inspection_flag=0;
}




//int  rising_left2(int level)
//{

//	if(left_level<level)
//	{
//		if(last_round<MOTOR_REAL_INFO[4].ANGLE)left_round++;
//		last_round=MOTOR_REAL_INFO[4].ANGLE;
//		
//		PID_incremental_PID_calculation(&MOTOR_PID_RPM[4], MOTOR_REAL_INFO[4].RPM, MOTOR_PID_POS[4].output);//速度环
//		PID_position_PID_calculation(&MOTOR_PID_POS[4], left_level, level);//位置环
//		left_level=(left_round*8191+MOTOR_REAL_INFO[4].ANGLE)/19;
//	}
//	
//	else if(left_level>level)
//	{
//		if(last_round>MOTOR_REAL_INFO[4].ANGLE)left_round--;
//		last_round=MOTOR_REAL_INFO[4].ANGLE;
//		PID_incremental_PID_calculation(&MOTOR_PID_RPM[4], MOTOR_REAL_INFO[4].RPM, MOTOR_PID_POS[4].output);//速度环
//		PID_position_PID_calculation(&MOTOR_PID_POS[4], left_level, level);//位置环
//		left_level=(left_round*8191+MOTOR_REAL_INFO[4].ANGLE)/19;
//	}
//}



 
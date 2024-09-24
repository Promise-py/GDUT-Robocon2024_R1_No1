/**
  ******************************************************************************
  * @file    FSM.c
  * @author  Py
  * @version V1.0.0
  * @date    2024/5/10
  * @brief   上层状态机
  ******************************************************************************
  */ 
/* Includes -------------------------------------------------------------------*/
#include "FSM.h"
#include "hardware.h"
#include "robot.h"
#include "communicate.h"
#include "MIT.h"
#include "remote.h"
/* Private  variables ---------------------------------------------------------*/

int Last_SWA=1000;
int Last_SWB=1000;
int Last_SWC=1000;
int Last_SWD=1000;


float World_Vx=0;
float World_Vy=0;
float World_w=0;

float DM_left=0;
float DM_right=0;
float U8_angle=0;

uint8_t Seedling_Count=0;


MOVE_STATE Move_State;

unsigned short last_mode=STOP;
unsigned short Hand_Position=Insp;//
unsigned short ZONE=Remote_ZONE1;
unsigned char Remote_SendFlag=0;
uint8_t NowZONE=ZONE_1;
uint8_t ClorSend=0;//场地选择发送标志位
float Cloud_Angle=0.85; 
void Move_Blue(void);


// unsigned short Move_Flag=0;
void move(void)
{
	switch(SWC)
	{
		case 1000:
					World_Vx=-(ROCK_L_X-1500)*0.05f;
					World_Vy=-(ROCK_L_Y-1500)*0.05f;
					World_w=(ROCK_R_X-1500)*0.05f;
					if(ZONE==Remote_ZONE1)Usart4_SendData(World_Vx,World_Vy,0,ZONE);
					else if(ZONE==Remote_ZONE2)Usart4_SendData(World_Vx,World_Vy,World_w,ZONE);
					HAL_Delay(1);//根据实际情况改变频率
				
		break;
		
		case 1500:
				// U8_angle=(float)(ROCK_L_X-1500);
				// if(U8_angle<0)ctrl_motor2(DM43_ID3,-2,-U8_angle*0.001);
				// else if(U8_angle>=0)ctrl_motor2(DM43_ID3,2,U8_angle*0.001);
				
		break;
		
		case 2000:
				DM_left=(float)(ROCK_L_X-1500);
				DM_right=(float)(ROCK_R_X-1500);
		
				if(DM_left<=0)ctrl_motor2(DM43_ID2,-4.4,-DM_left*0.01);
				else ctrl_motor2(DM43_ID2,0,DM_left*0.01);
		
		
				if(DM_right<=0)ctrl_motor2(DM43_ID1,2,-DM_right*0.01);
				else ctrl_motor2(DM43_ID1,7.5,DM_right*0.01);
		break;
	}
//		switch(Mode)
//		{
//			case Shoot:
//				shoot(5000);
//			break;
//		
//			case Rise:
//				if(Data_right==1)rising_left(-Data_left);
////				if(Data_left==1)rising_right(Data_right);
//			break;
//		
//			case Absorb:
//				Absorb_Ball(30);
//			break;
//			
////			case Turn:
////			
////				if(Data_left==Data_right==1)
////				{
////					if(Data_mid<0)ctrl_motor2(DM43_ID3,-2,-Data_mid*0.001);
////					else if(Data_mid>=0)ctrl_motor2(DM43_ID3,2,Data_mid*0.001);
////				}
////			break;	
//				
//			case Inspection:
//				 if(!move_flag)
//				 {
//					ctrl_motor2(DM43_ID2,-4.4,3);
//					move_flag=1;
//				 }
//			break;
//		}

		
}


//void rising_test(void)
//{
//	switch(SWA)
//	{
//		case 1000:
//			down_right();
//		break;
//		
//		case 2000:
//			rising_right(Data_right);
//		break;
//	}

//}

//void move_FSM(void)
//{
//	switch(ACTION_MODE.MoveMode)
//	{
//		case SHOOT_Action:
//			shoot(10);//!!!!!!!!!!!!测量并计算射出速度！！！！！！
//			break;
//		
//			
//	
//	}

//}

// void shoot_test(void)
// {
// 	if(MOTOR_REAL_INFO[5].self_inspection.inspection_flag==1&&MOTOR_REAL_INFO[4].self_inspection.inspection_flag==1&&MIT_DRIVER_REAL_INFO[0].DM43_inspection.inspection_flag==1)
// 	{
// 		if(move_flag==1)
// 		{
			
			
// 				switch(SWA)
// 					{
						
						
// 							case 1000:
// 	//								rising_left(0);
// 	//								rising_right(0);
// 									left_test(0);

// 								break;
								
// 							case 2000:
// 	//								rising_left(130);
// 	//								rising_right(130);
// 									left_test(130);
// 								break;
// 					}
			
// 		}
// 	}
// 		if(Last_SWD!=SWD)
// 		{	
// 			switch(SWD)
// 			{
				
// 					case 1000:
// 							ctrl_motor2(DM43_ID2,-2,3);
// 							ZONE=Remote_ZONE2;
// 							Last_SWD=SWD;
// //							rising_right(0);

// 						break;
						
// 					case 2000:
// 							ctrl_motor2(DM43_ID2,-4.4,3);
// 							ZONE=Remote_ZONE1;
// 							Hand_Position=Grab;
// 							move_flag=1;
// 							Last_SWD=SWD;
// //							rising_right(130);
// 						break;
// 				}
// 		}
// 	if(Last_SWB!=SWB&&ZONE==Remote_ZONE2)
// 	{
// 		switch(SWB)
// 		{
// 			case 1000:
// 					Absorb_Ball(30);
// 					Last_SWB=SWB;
// 			break;
			
// 			case 1500:
// 					Last_SWB=SWB;
// 			break;
			
// 			case 2000:
// 					shoot(5000);
// 					Last_SWB=SWB;
// 			break;
// 		}
// 	}
	

	
	
	
// 	if(ROCK_R_Y<1200)
// 	{
// 		ctrl_motor2(DM43_ID2,-4.4,4);
// 		Hand_Position=Grab;
// 	}

	
// }


void ActionReceive_FSM(void)
{
	if(Mode==ACTION_Flag)Action_flag=OK;
	
	if(!Remote_SendFlag)
	{
		Send_Inspection();
	}
	
	if(!Remote_SendFlag&&Move_State.Inspection_Flag&&Action_flag)
	{
		Send_Inspection();
		Remote_SendFlag=1;
	}
	
}

uint8_t last_Data1;
uint8_t last_Data2;
uint8_t Retry_Finsh=0;
int a1;

/*
 *  函数名：Remote_FSM(void)
 *  功能描述：遥控状态机
 *  输入参数：
 *  输出参数：无
 *  返回值：无
*/
void Remote_FSM(void)
{
	switch(Remote_Data1)
	{
		case Absorb:
			Absorb_Ball(30);
			Remote_Data1=0;
		break;
		
		case Shoot:
			shoot(5000);
			Remote_Data1=0;
		break;
		
		case Retry:
			
			ctrl_motor2(DM43_ID2,-2,3);
			ctrl_motor2(DM43_ID1,8.3,3);
			Seedling_Count++;//夹苗计数
//				Send_Data(Seedling,Seedling_Count);
			HAL_Delay(10);
			if(Robot_Move.Claw_State==Close)
			{
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_RESET);//松开
				Robot_Move.Claw_State=Open;
				Move_State.Send_Inspection=OK;//停止向底盘发送自检成功标志
				Remote_SendFlag=OK;//停止向遥控发送自检成功标志
			}
			if(Robot_Move.Cloud_State==BACK)
			{
				HAL_Delay(500);
				HAL_GPIO_WritePin(GPIOD,GPIO_PIN_14,GPIO_PIN_SET);
				Robot_Move.Cloud_State=FRONT;
			}
			
			move_flag=1;
//			Send_Data(Retry,Seedling_Count);
//		    a1=1;
//			if(Remote_Data2==0&&last_Data2!=Remote_Data2)
//			{
//				a1=2;
//				Usart4_SendData(0,0,0,Retry);
//				Retry_State();
//				last_Data2=Remote_Data2;
//			}
//			else if(Remote_Data2==1&&last_Data2!=Remote_Data2)
//			{
//				Usart4_SendData(0,0,1,Retry);
//				last_Data2=Remote_Data2;
//			}
			
		break;
		
		case Seedling:
			Usart4_SendData(0,0,Remote_Data2,Seedling);
		break;
		
		case AbsorbChoice:
			if(NowZONE==ZONE_2)
			{
				if(ZONE_Choice==Red)
				{
					if(Remote_Data2=='C')
					{
						ctrl_motor2(DM43_ID3,-0.9,2);
						HAL_Delay(1000);
						shoot(6000);
	//					Usart4_SendData(0,0,10.0f,0);
	//					uint8_t Date=0x0A;
	//					HAL_UART_Transmit(&huart4,&Date, 1, HAL_MAX_DELAY);
						Remote_Data2=0;
					}
					else if(Remote_Data2=='A')
					{
						ctrl_motor2(DM43_ID3,-0.9,2);
						HAL_Delay(1000);
						shoot(6000);
	//					Usart4_SendData(0,0,11.0f,0);
	//					HAL_Delay(10);
						Remote_Data2=0;
					}
					else if(Remote_Data2=='B')
					{
						ctrl_motor2(DM43_ID3,-0.9,2);
						HAL_Delay(1000);
						shoot(6000);
	//					Usart4_SendData(0,0,12.0f,0);
	//					HAL_Delay(10);
						Remote_Data2=0;
					}
					else if(Remote_Data2>48&&Remote_Data2<58)
					{
						switch(Remote_Data2-48)
						{
							case 1:
								ctrl_motor2(DM43_ID3,-0.5,2);
								HAL_Delay(1000);
								shoot(5000);
								Remote_Data2=0;
							break;
							
							case 2:
								ctrl_motor2(DM43_ID3,-0.7,2);
								HAL_Delay(1000);
								shoot(5000);
								Remote_Data2=0;
							break;
							
							case 3:
								ctrl_motor2(DM43_ID3,-0.9,2);
								HAL_Delay(1000);
								shoot(5000);
								Remote_Data2=0;
							break;
							
							case 4:
								ctrl_motor2(DM43_ID3,-0.9,2);
								HAL_Delay(1000);
								shoot(5000);
								Remote_Data2=0;
							break;
							
							case 5:
								ctrl_motor2(DM43_ID3,-0.9,2);
								HAL_Delay(1000);
								shoot(6000);
								Remote_Data2=0;
							break;
							
							case 6:
								ctrl_motor2(DM43_ID3,-0.9,2);
								HAL_Delay(1000);
								shoot(6000);
								Remote_Data2=0;
							break;
							
							case 7:
								ctrl_motor2(DM43_ID3,-0.9,2);
								HAL_Delay(1000);
								shoot(6000);
								Remote_Data2=0;
							break;
							
							case 8:
								ctrl_motor2(DM43_ID3,-0.9,2);
								HAL_Delay(1000);
								shoot(6000);
								Remote_Data2=0;
							break;
							
							case 9:
								ctrl_motor2(DM43_ID3,-0.9,2);
								HAL_Delay(1000);
								shoot(6000);
								Remote_Data2=0;
							break;
						}
					}
//					Usart4_SendData(0,0,Remote_Data2-48,0);
//					HAL_Delay(10);
				}
				
				
			}
//				if(Remote_Data2>52&&Remote_Data2<58)ctrl_motor2(DM43_ID3,0.85-0.28*(Remote_Data2-51),1);
//				else if(Remote_Data2=='A'||Remote_Data2=='B'||Remote_Data2=='C')ctrl_motor2(DM43_ID3,0.85-0.28*(Remote_Data2-67),1);
//				else ctrl_motor2(DM43_ID3,0.85,1);
			
//			switch(Remote_Data2)
//			{
//				case 'I':
//					Usart4_SendData(0,0,10.0f,AbsorbChoice);
//					Remote_Data2=0;
//				break;
//				case 'A':
//					Usart4_SendData(0,0,11.0f,AbsorbChoice);
//					Remote_Data2=0;
//				break;
//				case 'B':
//					Usart4_SendData(0,0,12.0f,AbsorbChoice);
//					Remote_Data2=0;
//				break;
//				
//				case 0:
//					
//				break;
//				
//				default:
//					Usart4_SendData(0,0,(Remote_Data2-48),AbsorbChoice);
//					Remote_Data2=0;
//				break;
//			}
		break;
	}
}

void Move_FSM(void)
{
	if(HAL_GPIO_ReadPin(Red_GPIO_Port,Red_Pin)==RESET&&!ClorSend)
	{
		HAL_Delay(10);
		if(HAL_GPIO_ReadPin(Red_GPIO_Port,Red_Pin)==RESET)ZONE_Choice=Red;
	}
	else if(HAL_GPIO_ReadPin(Blue_GPIO_Port,Blue_Pin)==RESET&&!ClorSend)
	{
		HAL_Delay(10);
		if(HAL_GPIO_ReadPin(Blue_GPIO_Port,Blue_Pin)==RESET)ZONE_Choice=Blue;
	}
	
	switch(ZONE_Choice)
	{
		case Red:
			Move_Red();
			if(!ClorSend)
			{
				Usart4_SendData(0,0,Red,Clor_Choice);
				ClorSend=1;
			}
			
		break;
		
		case Blue:
			Move_Blue();
			if(!ClorSend)
			{
				Usart4_SendData(0,0,Blue,Clor_Choice);
				ClorSend=1;
			}
		break;
	}
	
}

//红方场地
void Move_Red(void)
{
	
		if(Move_State.Inspection_Flag==OK&&Move_State.Send_Inspection!=OK)
		{
//			Usart4_SendData(0,0,0,Inspection);
			move_flag=1;
		}
		
		switch(Mode)
		{
			case Shoot:
//				Absorb_Ball(30);
//				HAL_Delay(1000);
			if(Data_left==1)ShootTurn(Data_mid);
			else shoot(Data_mid);
			
			break;
		
			case Rise:
				if(Data_right==1)left_test2(Data_left);
//				if(Data_right==1)left_Rise(Data_left);
//				if(Data_right==1)rising_left(Data_left);
//				if(Data_left==1)rising_right(Data_right);
			break;
		
			case Absorb:
				Absorb_Ball(30);
				
			
//				HAL_Delay(300);
//				if(Data_left==2)shoot(6000);
//				else shoot(5000);
			break;
			
			case Turn:
				if(Data_mid==1)ctrl_motor2(DM43_ID3,0.15,2);
				else if(Data_mid==2)ctrl_motor2(DM43_ID3,0.3,2);
				else if(Data_mid==3)ctrl_motor2(DM43_ID3,0.45,2);
				else if(Data_mid==4)ctrl_motor2(DM43_ID3,0.6,2);
				else if(Data_mid==5)ctrl_motor2(DM43_ID3,0.75,2);
				else if(Data_mid==6)ctrl_motor2(DM43_ID3,0.9,2);
//				ctrl_motor2(DM43_ID3,Data_mid,2);
//				if(Data_right==1)ctrl_motor2(DM43_ID2,Data_left,4);
//				if(Data_mid<0)ctrl_motor2(DM43_ID3,0.85,-Data_mid*0.002);
//				else if(Data_mid>=0)ctrl_motor2(DM43_ID3,0,Data_mid*0.002);
			break;	
				
			case Inspection:
				HAL_Delay(500);
				ctrl_motor2(DM43_ID2,-7,7);
				ctrl_motor2(DM43_ID1,2,3);
				Seedling_Count++;//夹苗计数
//				Send_Data(Seedling,Seedling_Count);
				HAL_Delay(10);
				if(Robot_Move.Claw_State==Close)
				{
					HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_RESET);//松开
					Robot_Move.Claw_State=Open;
					Move_State.Send_Inspection=OK;//停止向底盘发送自检成功标志
					Remote_SendFlag=OK;//停止向遥控发送自检成功标志
				}
				if(Robot_Move.Cloud_State==BACK)
				{
					HAL_Delay(500);
					HAL_GPIO_WritePin(GPIOD,GPIO_PIN_14,GPIO_PIN_SET);
					Robot_Move.Cloud_State=FRONT;
				}
				
				move_flag=1;
				 
			break;
				
			case ZONE2_Inspection:
				ctrl_motor2(DM43_ID2,-1,4);
				ctrl_motor2(DM43_ID1,8.2,4);
				HAL_Delay(500);
				if(Robot_Move.Cloud_State==FRONT)
				{
					HAL_Delay(500);
					HAL_GPIO_WritePin(GPIOD,GPIO_PIN_14,GPIO_PIN_RESET);
					HAL_Delay(500);
					HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_SET);
					Robot_Move.Cloud_State=BACK;
				}	
//				ctrl_motor2(DM43_ID3,0.85,1);
				NowZONE=ZONE_2;
			break;
				
			case Retry:
				
				if(Robot_Move.Cloud_State==FRONT)
				{
					HAL_Delay(500);
					HAL_GPIO_WritePin(GPIOD,GPIO_PIN_14,GPIO_PIN_RESET);//将云台推后
					Robot_Move.Cloud_State=BACK;
				}
				
				DM43_Init();
			    ctrl_motor2(DM43_ID3,0,2);
				ctrl_motor2(DM43_ID2,0,4);
				ctrl_motor2(DM43_ID1,0,4);
				
				//关爪子
				if(Robot_Move.Claw_State==Open)
				{
					HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_SET);//关紧
					Robot_Move.Claw_State=Close;
				}
			break;
				
			case Open:
				
					HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_SET);
					Robot_Move.Claw_State=Open;
			break;
		}
}


//蓝方场地
void Move_Blue(void)
{
	
		if(Move_State.Inspection_Flag==OK&&Move_State.Send_Inspection!=OK)
		{
//			Usart4_SendData(0,0,0,Inspection);
			move_flag=1;
			
		}
		
		switch(Mode)
		{
			case Shoot:
//				Absorb_Ball(30);
//				HAL_Delay(1000);
				if(Data_left==1)ShootTurn(Data_mid);
				else shoot(Data_mid);
			break;
		
			case Rise:
//				rising_right(Data_left);
				right_test(Data_left);
//				if(Data_right==1)left_test(Data_left);
//				if(Data_right==1)rising_left(Data_left);
//				if(Data_left==1)rising_right(Data_right);
			break;
		
			case Absorb:
				Absorb_Ball(30);
				
			
//				HAL_Delay(300);
//				if(Data_left==2)shoot(6000);
//				else shoot(5000);
			break;
			
			case Turn:
//				if(Data_right==1)ctrl_motor2(DM43_ID2,Data_left,4);
//				if(Data_mid<0)ctrl_motor2(DM43_ID3,0.85,-Data_mid*0.002);
//				else if(Data_mid>=0)ctrl_motor2(DM43_ID3,0,Data_mid*0.002);
			break;	
				
			case Inspection:
				HAL_Delay(500);
				ctrl_motor2(DM43_ID1,8.2,7);
				ctrl_motor2(DM43_ID2,-1,4);
				Seedling_Count++;//夹苗计数
//				Send_Data(Seedling,Seedling_Count);
				HAL_Delay(10);
				if(Robot_Move.Claw_State==Close)
				{
					HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_RESET);//松开
					Robot_Move.Claw_State=Open;
					Move_State.Send_Inspection=OK;//停止向底盘发送自检成功标志
					Remote_SendFlag=OK;//停止向遥控发送自检成功标志
				}
				if(Robot_Move.Cloud_State==BACK)
				{
					HAL_Delay(500);
					HAL_GPIO_WritePin(GPIOD,GPIO_PIN_14,GPIO_PIN_SET);
					Robot_Move.Cloud_State=FRONT;
				}
				
				move_flag=1;
				 
			break;
				
			case ZONE2_Inspection:
				ctrl_motor2(DM43_ID2,-1,4);
				ctrl_motor2(DM43_ID1,2,4);
				HAL_Delay(500);
				if(Robot_Move.Cloud_State==FRONT)
				{
					HAL_Delay(500);
					HAL_GPIO_WritePin(GPIOD,GPIO_PIN_14,GPIO_PIN_RESET);
					HAL_Delay(500);
					HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_SET);
					Robot_Move.Cloud_State=BACK;
				}	
				NowZONE=ZONE_2;
			break;
				
			case Retry:
				if(Robot_Move.Cloud_State==FRONT)
				{
					HAL_Delay(500);
					HAL_GPIO_WritePin(GPIOD,GPIO_PIN_14,GPIO_PIN_RESET);//将云台推后
					Robot_Move.Cloud_State=BACK;
				}		
				//关爪子
				if(Robot_Move.Claw_State==Open)
				{
					HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_SET);//关紧
					Robot_Move.Claw_State=Close;
				}
				Retry_State();
			break;
				
			case Open: 
				
					//
					HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_SET);
					Robot_Move.Claw_State=Open;
			
			break;
		}
}





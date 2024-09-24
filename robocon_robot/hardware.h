
#ifndef HARDWARE_H
#define HARDWARE_H

#include "stm32f4xx_hal.h"
#include "motor.h"

//define
#define SWA		PPM_buf[4]				//AUX4 1000~2000//没用
#define SWB		PPM_buf[5]				//AUX2 1000-1500-2000
#define SWD		PPM_buf[7]			//AUX1 1000~2000
#define SWC		PPM_buf[6]				//AUX3 1000-1500-2000

#define ROCK_R_X			PPM_buf[0]					//YAW  1000-1500-2000
#define ROCK_R_Y			PPM_buf[1]					//THR  1000-1500-2000

#define ROCK_L_Y			PPM_buf[2]				//ROLL 1000-1500-2000//未知bug
#define	ROCK_L_X		  PPM_buf[3]				//PITCH 1000-1500-2000P


extern uint16_t PPM_buf[10];

typedef struct
{
	struct  //遥控原始数据，8通道
	{
	 uint16_t roll;			//右摇杆
	 uint16_t pitch;		//
	 uint16_t thr;
	 uint16_t yaw;
	 uint16_t AUX1;
	 uint16_t AUX2;
	 uint16_t AUX3;
	 uint16_t AUX4; 
	 uint16_t BUX1;
	 uint16_t BUX2;		
	}Remote; 

}Air_Contorl;

typedef struct KEY_Type//发射数据
{
  
	int KEY_armtop;//armtop
	int KEY_armbottom;//armbottom
	int KEY_push;//push
}KEY_Type;


/* Action读到的数据 */
// 东大全场定位模块定位的位置
typedef struct ACTION_GL_POS
{
	float ANGLE_Z;
	float ANGLE_X;
	float ANGLE_Y;	
	float POS_X;
	float POS_Y;
	float W_Z;
	
	float LAST_POS_X;
	float LAST_POS_Y;
	
	float DELTA_POS_X;
	float DELTA_POS_Y;	
	
	float REAL_X;
	float REAL_Y;
}ACTION_GL_POS;


/* 机器人的真实位置 */
typedef struct ROBOT_CHASSIS
{

	float World_V[3]; // X , Y , W
	float Robot_V[3];
	float Position[2];
	float Motor_RPM[3];
	float expect_angle ;
	float Angle;
} ROBOT_CHASSIS;

/*激光数据*/
typedef struct Laser_Data
{
	float Laser1;
	float Laser2;
	unsigned char Error_Flag1;
	unsigned char Error_Flag2;
} Laser_Data;



extern unsigned char UART2_Receiver;
extern unsigned char UART3_Receiver;
extern ACTION_GL_POS ACTION_GL_POS_INFO;
extern ROBOT_CHASSIS ROBOT_REAL_POS_INFO;
extern Laser_Data Laser_Real_Data;

void Laser_ReadData2(float* Laser_Data);
void remote_control(void);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
float Laser_Resolution(uint8_t rx_Data[9]);
void Laser_ReadData(float* Laser_Data);
#endif



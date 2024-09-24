/*!
 *****************************************************************************
 *
 *  Copyright © 2017-2018 Keten. All Rights Reserved.
 *
 * \file      motor.h
 * \author    Keten
 * \version   1.0
 * \date      2024年4月29日
 * \brief     motor.c 的头文件
 *
 *----------------------------------------------------------------------------
 * \attention
 *
 *
 *****************************************************************************
 */

/*****************************************************************************
 change history: 
    1.date  : 2024年4月29日
      author: Keten
      change: create file

*****************************************************************************/
#ifndef MOTOR_H
#define MOTOR_H

#include "stm32f4xx_hal.h"
#include "pid.h"

#define  SPEED_CONTROL_MODE			1
#define  POSITION_CONTROL_MODE		2
#define  CURRENT_MODE				3
#define  MOTO_OFF					4
				
/* m3508电机id */
#define M3508_CHASSIS_MOTOR_ID_1      0x201
#define M3508_CHASSIS_MOTOR_ID_2      0x202
#define M3508_CHASSIS_MOTOR_ID_3      0x203

#define M3508_CHASSIS_MOTOR_ID_4      0x204
#define M3508_CHASSIS_MOTOR_ID_5      0x205
#define M3508_CHASSIS_MOTOR_ID_6      0x206

#define Rising   1
#define Down     0


extern int itemp;



typedef enum
{ 
  RM_3508   = 1, 
  M_2006    = 2,
	NONE      = 3  //none表示没有接电机

}MotorType_TypeDef;

typedef struct VELOCITY_PLANNING //速度规划
{
	float Distance;
	float Pstart;        //开始位置
	float Pend;          //结束位置
	float Vstart;        //开始的速度           // 单位：RPM 绝对值
	float Vmax;          //最大的速度
	float Vend;          //末尾的速度
	float Rac;           //加速路程的比例
	float Rde;           //减速路程的比例
	int flag;            //完成标志位，电机停下来的时候置1
}VELOCITY_PLANNING;

typedef struct motor_test
{
	uint8_t inspection_flag;//自检完成标志位 0未完成 1完成
	uint8_t inspection_state;//自检状态 1rising 0down
}motor_test;

typedef struct MOTO_REAL_INFO
{
	// 电机模式
	uint32_t unitMode;//电机模式
		//POSITION_CONTROL_MODE位置模式
		//SPEED_CONTROL_MODE速度模式
		//CURRENT_MODE电流控制模式
	MotorType_TypeDef type;//电机类型：m3508、m2006
	uint16_t ANGLE;   		//采样角度						
	int16_t  RPM;			//速度值			
	int16_t  CURRENT;     //电流值
	int16_t  TARGET_CURRENT;//目标电流值
	int16_t  TARGET_POS;//目标角度
	float    TARGET_RPM;//目标转速
	
	// 角度积分时用到下面变量
	float		 REAL_ANGLE;         //处理过的真实角度
	uint8_t	 FIRST_ANGLE_INTEGRAL_FLAG;  //?
	uint16_t LAST_ANGLE;   //?
	int16_t filter_RPM;
	VELOCITY_PLANNING velocity_planning;//速度规划
	
	uint8_t current_state;//当前夹爪状态（抬起了/放下了）
	motor_test self_inspection;

}MOTO_REAL_INFO;



/* 电机的目标速度 */
/* 目前先掌握控制3台电机 */
typedef struct MOTOR_RPM
{
	float MOTOR1_RPM;
	float MOTOR2_RPM;
	float MOTOR3_RPM;
}MOTOR_RPM;


/* 电机的目标位置 */
typedef struct MOTOR_POS
{
	float MOTOR1_POS;
	float MOTOR2_POS;
	float MOTOR3_POS;
}MOTOR_POS;


/* 电机初始化函数 */
void m3508_Init(void);
/* can1 接收对数据处理 */
void m3508_update_info(CAN_RxHeaderTypeDef *msg,uint8_t	can1_RxData[8]);
/* pid计算后，打包can报文发送回电机 */
void chassis_m3508_send_motor_currents(void);
void MotorCtrl(void);
float VelCrl(MOTO_REAL_INFO *MOTOR_REAL_INFO,float target_vel);
float CurrentCrl(MOTO_REAL_INFO *MOTOR_REAL_INFO,float target_current);
void down_update_info(CAN_RxHeaderTypeDef *msg,uint8_t	can2_RxData[8]);
float MOTOR_STOP(MOTO_REAL_INFO *MOTOR_REAL_INFO,pid* pp);

extern MOTO_REAL_INFO MOTOR_REAL_INFO[11]; // 1-4分别对应顺时针方向的底盘电机
extern pid MOTOR_PID_RPM[11];	//速度pid信息 1-4底盘电机 5-6发射电机
extern pid MOTOR_PID_POS[11];	//位置pid信息

extern MOTOR_RPM MOTOR_TARGET_RPM;
extern MOTOR_POS MOTOR_TARGET_POS;



#endif /* MOTOR_H */


#ifndef CALCULATION_H
#define CALCULATION_H

#include "stm32f4xx_hal.h"
#include "motor.h"

#define number 						1.732050807568877  //!< //The square root of three By: Keten 2024年3月27日
#define RADIUS 						0.1730520 //!< 轮子（中心）到底盘圆心的距离（近似） By: Keten 2024年4月5日
#define PI 							3.1415926


void Update_Action_gl_position(float value[6]);
void M3508AngleIntegral(MOTO_REAL_INFO *M3508_MOTOR);
void Velocity_Planning_setpos(MOTO_REAL_INFO *M3508_MOTOR,float Pstart,float Pend,float Vstart,float Vmax,float Vend,float Rac,float Rde);
void VelocityPlanningMODE(MOTO_REAL_INFO *M3508_MOTOR);


#endif



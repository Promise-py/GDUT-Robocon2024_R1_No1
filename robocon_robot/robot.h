/*!
 *****************************************************************************
 *
 *
 * \file      robot.h
 * \author    Py
 * \version   1.0
 * \date      2024年5月9日
 * \brief     robot.c的头文件
 *
 *----------------------------------------------------------------------------
 * \attention
 *
 *
 *****************************************************************************
 */

/*****************************************************************************
 change history: 
    1.date  : 2024年5月9日
      author: py
      change: create file

*****************************************************************************/
#ifndef ROBOT_H
#define ROBOT_H
#include "stm32f4xx_hal.h"

#define UP 0x01
#define DOWN 0x02

#define BACK 0
#define FRONT 0x01

#define Close 0
#define Open 0x01

extern float temp;
typedef struct robot_move
{
	unsigned short Absorb_Flag;
  unsigned short Shoot_Flag;
  unsigned short R_L_Flag;
  unsigned short R_R_Flag;
  unsigned char Claw_State;
  unsigned char Cloud_State;
}robot_move;


extern robot_move Robot_Move;
//开启吸球
void Absorb_Ball(int t);

void shoot(float v0);

void rising_right_test(void);

extern float angle_adjust;

void  rising_right(int level);
void  rising_left(int level);
void Retry_State(void);
void self_inspection(void);
void  left_Rise(int level);
void  right_test(int level);
void left_test2(int level);
void ShootTurn(float v0);
extern float max_level;
extern float min_level;

//int rising_left1(int level);
//int rising_left2(int level);

#endif
/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "motor.h"
#include "driver_usart.h"
#include "hardware.h"
#include "calculation.h"
#include "hardware.h"
#include "FSM.h"
#include "robot.h"
#include "communicate.h"
#include "MIT.h"
#include "can.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
 
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
static uint8_t g_printf=1;
extern uint16_t PPM_buf[10];

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for motor_control */
osThreadId_t motor_controlHandle;
const osThreadAttr_t motor_control_attributes = {
  .name = "motor_control",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for robot_state */
osThreadId_t robot_stateHandle;
const osThreadAttr_t robot_state_attributes = {
  .name = "robot_state",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for robot_move */
osThreadId_t robot_moveHandle;
const osThreadAttr_t robot_move_attributes = {
  .name = "robot_move",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void MotorControl(void *argument);
void RobotState(void *argument);
void RobotMove(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of motor_control */
  motor_controlHandle = osThreadNew(MotorControl, NULL, &motor_control_attributes);

  /* creation of robot_state */
  robot_stateHandle = osThreadNew(RobotState, NULL, &robot_state_attributes);

  /* creation of robot_move */
  robot_moveHandle = osThreadNew(RobotMove, NULL, &robot_move_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
	static portTickType move_xLastWakeTime;
	const portTickType move_xFrequency = pdMS_TO_TICKS(100); // ÑÓÊ±10ms
  /* Infinite loop */
  for(;;)
  {
	  ActionReceive_FSM();
	 
	self_inspection();
	
//	 MOTOR_REAL_INFO[0].TARGET_RPM=300; 
	 
//	  
	  
////	absorb_ball(100);
////	Usart4_SendData(-1.0f,-2.0f,-3.0f,-1);  
////	vTaskDelayUntil(&move_xLastWakeTime, move_xFrequency); // ¾ø¶ÔÑÓÊ±
////	  printf("%f,%f,%f,%d\n",Data_left,Data_right,Data_right,o);
////	 osDelay(10);
//	  MOTOR_REAL_INFO[3].TARGET_RPM=-3000;
	  osDelay(1);
	 
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_MotorControl */
/**
* @brief Function implementing the motor_control thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_MotorControl */
void MotorControl(void *argument)
{
  /* USER CODE BEGIN MotorControl */
  /* Infinite loop */
  for(;;)
  {
 
//	  DM43_Init();
//	ctrl_motor2(DM43_ID1, 10, 3);
//    vTaskDelay(10);
	  MotorCtrl();
	  vTaskDelay(10);
  }
  /* USER CODE END MotorControl */
}

/* USER CODE BEGIN Header_RobotState */
/**
* @brief Function implementing the robot_state thread. 
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_RobotState */
void RobotState(void *argument)
{
  /* USER CODE BEGIN RobotState */
  /* Infinite loop */
  for(;;)
  {
//	  left_self_inspection();
//	  rising_test();
//   printf("%f,%hd\n",MOTOR_REAL_INFO[1].TARGET_RPM,MOTOR_REAL_INFO[1].RPM);
//	  move();
//  shoot_test();
	 
//	 Usart4_SendData(0,0,0,move_flag);
	 Move_FSM();
//	Remote_FSM();	  
//	move_test();
//  HAL_UART_Transmit(&huart4,&move_flag,1,0xff);		
  vTaskDelay(10);
  }
  /* USER CODE END RobotState */
}

/* USER CODE BEGIN Header_RobotMove */
/**
* @brief Function implementing the robot_move thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_RobotMove */
void RobotMove(void *argument)
{
  /* USER CODE BEGIN RobotMove */
  /* Infinite loop */
	for(;;)
  {
	  
//	Move_Red();
//	 Absorb_Ball(30);
//	  shoot(4000);
	vTaskDelay(1);
  }
  /* USER CODE END RobotMove */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */


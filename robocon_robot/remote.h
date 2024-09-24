/**
  ******************************************************************************
  * @file    remote.h
  * @author  Catalyst
  * @version V1.0.0
  * @date    2024/6/9
  * @brief   
  ******************************************************************************
  */ 
  
#include "stm32f4xx_hal.h"
#ifndef _REMOTE_H
#define _REMOTE_H

#define Red 0x01
#define Blue 0x02
#define Retry 0x09
#define Seedling 0x10
#define AbsorbChoice 0xAB
void Remote_Data(uint8_t * receiveData);
void Send_BT(void);
void Send_Inspection(void);
void Send_Data(uint8_t Data1,uint8_t Data2);
void receivePacket(uint8_t * rx_temp_rm) ;
void handlePacket(void) ;
extern uint8_t Remote_Data1;
extern uint8_t Remote_Data2;
extern uint8_t ZONE_Choice;


extern uint8_t rx_temp;
extern uint8_t rx_index ;
extern uint8_t receiving ;
extern uint32_t crc_received;
extern uint32_t crc_calculated;
extern uint8_t flag_check;
extern uint8_t flag_code;
extern uint8_t flag_start;//遥控发送的启动标志位
extern uint8_t start_flag;//向遥控发送的启动确认
extern uint8_t Now_Area;  //当前区域
extern uint8_t Action_flag;
extern uint8_t flag_other;
#endif


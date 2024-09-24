/**
  ******************************************************************************
  * @file    remote.c
  * @author  Catalyst
  * @version V1.0.0
  * @date    2024/6/9
  * @brief   
  ******************************************************************************
  */ 
#include "remote.h"
#include "main.h"
//#include "crc.h"
#include <string.h>
#include "usart.h"
#include "robot.h"
#include "motor.h"
#include "MIT.h"
#include "FSM.h"
#define BUFFER_SIZE_RM 4
#define PACKET_LENGTH 4 // 定义数据包长度
uint8_t dataPacket[PACKET_LENGTH];// 数据包缓冲区

uint8_t rx_buffer[BUFFER_SIZE_RM];
uint8_t rx_temp;
uint8_t rx_index = 0;
uint8_t receiving = 0;
uint32_t crc_received;
uint32_t crc_calculated;
uint8_t flag_check;
uint8_t flag_code;
uint8_t flag_start;//遥控发送的启动标志位
uint8_t start_flag;//向遥控发送的启动确认
uint8_t Now_Area;  //当前区域
uint8_t Action_flag;
uint8_t flag_other;
uint8_t ZONE_Choice;
uint8_t Remote_Data1;
uint8_t Remote_Data2;
int ka = 0;
float RockerX, RockerY;

void Remote_Data(uint8_t * rx_temp_rm)
{
	rx_temp = *rx_temp_rm;
	if (receiving) {
			rx_buffer[rx_index++] = rx_temp;
			//检测是否填充完毕
			if (rx_temp == 0xFC && rx_index == BUFFER_SIZE_RM) 
			{
//				crc_calculated = HAL_CRC_Calculate(&hcrc, (uint32_t *)(rx_buffer+1), 12);
				memcpy(&crc_received, rx_buffer + 13, sizeof(crc_received));
				
				if (crc_received == 0x11451410) 
				{
						ka++;
						flag_check  = rx_buffer[1];//检查标志位
						flag_start  = rx_buffer[2];//一键启动
						flag_code   = rx_buffer[3];//代码更换位
						memcpy(&RockerX, rx_buffer + 5, sizeof(RockerX));//遥感x轴
						memcpy(&RockerY, rx_buffer + 9, sizeof(RockerY));//遥感y轴
				} 
				else 
				{
					//校验失败的操作
				}
				receiving = 0;  
				rx_index = 0;   
			} 
			else if(rx_temp != 0xFC && rx_index == BUFFER_SIZE_RM)
			{
				receiving = 0;
				rx_index = 0;
			}
			else if (rx_index >= BUFFER_SIZE_RM) 
			{
					receiving = 0;
					rx_index = 0;
			}
			
	} 
	else if (rx_temp == 0xAC)
	{
			receiving = 1;
			rx_index = 0;
			rx_buffer[rx_index++] = rx_temp;
	}
	else if(rx_temp==0x30)
	{
		ZONE_Choice=Red;
		rx_index=0;
	}
			
	else if(rx_temp==0x31)
	{
		ZONE_Choice=Blue;
		rx_index=0;
	}
	
}
uint8_t Choice_flag=0;
uint8_t bufferIndex=0;
uint8_t buffer[4];
uint8_t packetAvailable=0;
void receivePacket(uint8_t * rx_temp_rm) 
{
	uint8_t byteReceived = *rx_temp_rm;
	if(byteReceived==0x30&&Choice_flag==0)
	{
		ZONE_Choice=Red;
		Choice_flag=1;
		return;
	}
			
	else if(byteReceived==0x31&&Choice_flag==0)
	{
		ZONE_Choice=Blue;
		Choice_flag=1;
		return;
	}	
    
	
    if (bufferIndex == 0 && byteReceived != 0xAC) {
      return ; // Ignore bytes until the start byte is received
    }
    buffer[bufferIndex++] = byteReceived;
    if (bufferIndex == 4) {
      if (buffer[PACKET_LENGTH - 1] == 0xFC) {
        packetAvailable = 1;
      }
      bufferIndex = 0;
    }
  
}

void handlePacket(void) 
{
  if (packetAvailable) 
  {
    packetAvailable = 0;
    Remote_Data1=buffer[1];
	Remote_Data2=buffer[2];
  }
}


uint8_t exampleData[16] = {0xAC, 0xAC, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,

                           0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10};


						   
						   
uint8_t InspectionData[4]={0xAC,0,0,0xFC};	

/*
 *  函数名：Send_Inspection
 *  功能描述：向遥控发送自检成功标志
 *  输入参数：
 *  输出参数：无
 *  返回值：无
*/
void Send_Inspection(void)
{
	InspectionData[1]=Move_State.Inspection_Flag;
	InspectionData[2]=Action_flag;
	HAL_UART_Transmit(&huart3, InspectionData, 4, HAL_MAX_DELAY);
}


uint8_t send_buffer[4];
/*
 *  函数名：Send_Data
 *  功能描述：向遥控发送自检成功标志
 *  输入参数：两个要发送的系数
 *  输出参数：无
 *  返回值：无
*/
void Send_Data(uint8_t Data1,uint8_t Data2)
{
	send_buffer[1]=Data1;
	send_buffer[2]=Data2;
	HAL_UART_Transmit(&huart3, send_buffer, 4, HAL_MAX_DELAY);
}



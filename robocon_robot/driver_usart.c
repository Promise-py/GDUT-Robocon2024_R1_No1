#include "driver_usart.h"
#include "usart.h"
#include "main.h"
#include "ring_buffer.h"
#include "pid_adjust.h"
#include "pid.h"
#include <stdio.h>
#include "calculation.h"
#include "hardware.h"
#include "FSM.h"
#include "communicate.h"
#include "can.h"
#include "remote.h"
static volatile uint8_t txcplt_flag = 0;    // 发送完成标志，1完成0未完成
static volatile uint8_t rx_data = 0;
uint8_t UART3_Receiver_RM = 0;

float pos_x =0;
float pos_y=0;
float zangle =0;
float xangle = 0;
float yangle =0;
float w_z = 0;


/*
 *  函数名：EnableDebugIRQ
 *  功能描述：使能USART1的中断+使能UART4的中断
 *  输入参数：无
 *  输出参数：无
 *  返回值：无
*/
void EnableDebugIRQ(void)
{
    HAL_NVIC_SetPriority(USART1_IRQn, 0, 1);    // 设置USART1中断的优先级
    HAL_NVIC_EnableIRQ(USART1_IRQn);            // 使能USART1的中断
    
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_TC | UART_IT_RXNE);   // 使能USRAT1的发送和接收中断

	
	HAL_NVIC_SetPriority(USART2_IRQn, 0, 1);    // 设置USART2中断的优先级
    HAL_NVIC_EnableIRQ(USART2_IRQn);            // 使能USART2的中断
    
    __HAL_UART_ENABLE_IT(&huart2, UART_IT_TC | UART_IT_RXNE);   // 使能USART2的发送和接收中断
    
	
	HAL_NVIC_SetPriority(USART3_IRQn, 0, 2);    // 设置USART3中断的优先级
    HAL_NVIC_EnableIRQ(USART3_IRQn);            // 使能USART3的中断
    
    __HAL_UART_ENABLE_IT(&huart3, UART_IT_TC | UART_IT_RXNE);   // 使能USART3的发送和接收中断
    
	HAL_UART_Receive_IT(&huart3, RxBuffer_for4, 1);//每接收一个数据，就打开一次串口中断接收
	
	
	
	HAL_NVIC_SetPriority(UART4_IRQn, 0, 1);    // 设置UART4中断的优先级
    HAL_NVIC_EnableIRQ(UART4_IRQn);            // 使能UART4的中断
    
    __HAL_UART_ENABLE_IT(&huart4, UART_IT_TC | UART_IT_RXNE );   // 使能Uuart4的发送和接收中断
}

/*
 *  函数名：DisableDebugIRQ
 *  功能描述：失能USART1的中断
 *  输入参数：无
 *  输出参数：无
 *  返回值：无
*/
void DisableDebugIRQ(void)
{
    __HAL_UART_DISABLE_IT(&huart1, UART_IT_TC | UART_IT_RXNE);      // 失能USRAT1的发送和接收中断
    
    HAL_NVIC_DisableIRQ(USART1_IRQn);   // 失能USART1的中断
}

/*
 *  函数名：fputc
 *  功能描述：printf/putchar 标准输出函数的底层输出函数
 *  输入参数：ch --> 要输出的数据
 *  输出参数：无
 *  返回值：无
*/
int fputc(int ch, FILE *f)
{
    //txcplt_flag = 0;
    //HAL_UART_Transmit_IT(&huart1, (uint8_t*)&ch, 1);
    //while(txcplt_flag==0);
    uint8_t data = (uint8_t)ch;
    HAL_UART_Transmit(&huart1, &data, 1, HAL_MAX_DELAY);
    return ch;
}
/*
 *  函数名：fgetc
 *  功能描述：scanf/getchar 标准输出函数的底层输出函数
 *  输入参数：
 *  输出参数：无
 *  返回值：接收到的数据
*/
//int fgetc(FILE *f)
//{
    //uint8_t data = (uint8_t)ch;
    //HAL_UART_Transmit(&huart1, &data, 1, HAL_MAX_DELAY);
    //return ch;
//}

/*
 *  函数名：USART1_IRQHandler
 *  功能描述：USART1的中断服务函数
 *  输入参数：无
 *  输出参数：无
 *  返回值：无
*/
//void USART1_IRQHandler(void)
//{
    //unsigned char c = 0;
    //if((USART1->SR &(1<<5)) != 0)   // 判断USART1的状态寄存器的第五位即RXNE位是否被置位
    //{
        //c = USART1->DR; // RXNE=1，表明DR寄存器有值，就将它读出来保存到临时变量中；
        //ring_buffer_write(c, &test_buffer); // 将数据保存到环形缓冲区中
    //}
    //HAL_UART_IRQHandler(&huart1);   // HAL库中的UART统一中断服务函数，通过形参判断是要处理谁的中断
//}



uint8_t fer[12];

/*
 *  函数名：HAL_UART_RxCpltCallback
 *  功能描述：HAL库中的UART接收完成回调函数
 *  输入参数：huart --> UART的设备句柄，用以指明UART设备是哪一个UART
 *  输出参数：无
 *  返回值：无
*/

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
   
   if(huart==&huart1)
   {
	   RxLine++;					  //每接收到一个数据，进入回调数据长度加1
	   DataBuff[RxLine-1]=RxBuffer[0];	//把每次接收到的数据保存到缓存数组
	   if(RxBuffer[0]=='!') 		  //接收结束标志位
	   {
//		   printf("RXLen=%d\r\n",RxLine);
		   for(int i=0;i<RxLine;i++)
			   printf("UART DataBuff[%d] = %c\r\n",i,DataBuff[i]);
		   	 
		   USART_PID_Adjust(1);//数据解析和参数赋值函数
		   memset(DataBuff,0,sizeof(DataBuff));  //清空缓存数组
		   RxLine=0;  //清空接收长度
	   }
	   RxBuffer[0]=0;
	   HAL_UART_Receive_IT(&huart1, (uint8_t *)RxBuffer, 1);//每接收一个数据，就打开一次串口中断接收
   }
   
   if(huart==&huart2)
   {
        Laser_ReadData(&Laser_Real_Data.Laser1);
        HAL_UART_Receive_IT(&huart2,&UART2_Receiver,1);
	   
   }
   
   if(huart==&huart3)
   {
	   HAL_UART_Receive_IT(&huart3,&UART3_Receiver_RM,1);
       receivePacket(&UART3_Receiver_RM);
	   handlePacket();
	   o++;
   }
   
   if(huart==&huart4)
   {
		uart4_ReceiveData(&Data_left,&Data_right,&Data_mid,&Mode);
	    HAL_UART_Receive_IT(&huart4, &UART4_Receiver,1);
	   USART4_FLAG++;
	   	
   }
   

}

/*
 *  函数名：HAL_UART_TxCpltCallback
*  功能描述：HAL库中的UART发送完成回调函数
 *  输入参数：huart --> UART的设备句柄，用以指明UART设备是哪一个UART
 *  输出参数：无
 *  返回值：无
*/
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART1)   // 判断进来的是否是USART1这个UART设备
    {
        txcplt_flag = 1;    // 进入此回调函数表明发送指定长度的数据已经完成，将标志置一
    }
}

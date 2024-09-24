#ifndef __DRIVER_USART_H
#define __DRIVER_USART_H

#include "stm32f4xx_hal.h"

#define SHOOT   0x0B
#define Rising  0x0C
#define Turn    0x0D

/*
 *  函数名：EnableDebugIRQ
 *  功能描述：使能USART1的中断
 *  输入参数：无
 *  输出参数：无
 *  返回值：无
*/
extern void EnableDebugIRQ(void);
extern uint8_t UART3_Receiver_RM;
/*
 *  函数名：DisableDebugIRQ
 *  功能描述：失能USART1的中断
 *  输入参数：无
 *  输出参数：无
 *  返回值：无
*/
extern void DisableDebugIRQ(void);

#endif /* __DRIVER_USART_H */

#include "hardware.h"
#include "string.h"
#include "driver_usart.h"
#include "calculation.h"
#include "motor.h"
#include "usart.h"

Air_Contorl  Device;

ACTION_GL_POS ACTION_GL_POS_INFO;
ROBOT_CHASSIS ROBOT_REAL_POS_INFO;

//定义变量

uint16_t Time_Sys[4]={0};
uint16_t Microsecond_Cnt=0;
uint16_t PPM_buf[10]={0};

unsigned char UART2_Receiver= 0;
unsigned char UART3_Receiver= 0;




uint8_t ppm_update_flag=0;
uint32_t now_ppm_time_send=0;
uint32_t TIME_ISR_CNT=0,LAST_TIME_ISR_CNT=0;

#define Hour         3
#define Minute       2
#define Second       1
#define MicroSecond  0
 

/**
  * 函数功能: 按键外部中断回调函数
  * 输入参数: GPIO_Pin：中断引脚
  * 返 回 值: 无
  * 说    明: 无
 */


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	
	static uint32_t last_ppm_time=0,now_ppm_time=0;
	static uint8_t ppm_ready=0,ppm_sample_cnt=0;
	static uint16_t ppm_time_delta=0;//得到上升沿与下降沿的时间
	
	if(GPIO_Pin==GPIO_PIN_6)//判断是否为接收器产生的中断，例程设置为PIN8
	{
		//系统运行时间获取，单位us
		last_ppm_time=now_ppm_time;//获取上一次的当前时间作为上次时间
		
		now_ppm_time_send=now_ppm_time=10000*TIME_ISR_CNT+TIM2->CNT;//us
		
		ppm_time_delta=now_ppm_time-last_ppm_time;//相减得到一个周期时间
		
		//PPM解析开始
		if(ppm_ready==1)//判断帧结束时，开始解析新的一轮PPM
		{
			if(ppm_time_delta>=2100)//帧结束电平至少2ms=2000us，由于部分老版本遥控器、//接收机输出PPM信号不标准，当出现解析异常时，尝试改小此值，该情况仅出现一例：使用天地飞老版本遥控器
			{
				//memcpy(PPM_Databuf,PPM_buf,ppm_sample_cnt*sizeof(uint16));
				ppm_ready = 1;
				ppm_sample_cnt=0;//对应的通道值
				ppm_update_flag=1;
			} 
			else if(ppm_time_delta>=9&&ppm_time_delta<=2050)//单个PWM脉宽在1000-2000us，这里设定900-2100，应该是为了提升容错
			{         
				PPM_buf[ppm_sample_cnt]=ppm_time_delta;//对应通道写入缓冲区，cnt++计算有多少个元素
				ppm_sample_cnt++;
				
				if(ppm_sample_cnt>=10)//单次解析结束0-7表示8个通道。我这里可以显示10个通道，故这个值应该为0-9！！待修改
				{
					//memcpy(PPM_Databuf,PPM_buf,ppm_sample_cnt*sizeof(uint16_t));
					ppm_ready=0;
					ppm_sample_cnt=0;
				}
			}
			
			else  ppm_ready=0;
			
		}
		
		else if(ppm_time_delta>=2100)//帧结束电平至少2ms=2000us
		{
			ppm_ready=1;
			ppm_sample_cnt=0;
			ppm_update_flag=0;
		}
		
		if(ROCK_L_X>1460&&ROCK_L_X<1540)ROCK_L_X=1500;
		if(ROCK_L_Y>1460&&ROCK_L_Y<1540)ROCK_L_Y=1500;
		
		if(ROCK_R_X>1460&&ROCK_R_X<1540)ROCK_R_X=1500;
		if(ROCK_R_Y>1460&&ROCK_R_X<1540)ROCK_R_Y=1500;
		
		if(SWA>900&&SWA<1100)SWA=1000;
		if(SWA>1900&&SWA<2100)SWA=2000;
		
		if(SWD>900&&SWD<1100)SWD=1000;
		if(SWD>1900&&SWD<2100)SWD=2000;
		
		if(SWB>900&&SWB<1100)SWB=1000;
		if(SWB>1450&&SWB<1550)SWB=1500;
		if(SWB>1900&&SWB<2100)SWB=2000;
		
		if(SWC>900&&SWC<1100)SWC=1000;
		if(SWC>1450&&SWC<1550)SWC=1500;
		if(SWC>1900&&SWC<2100)SWC=2000;
	
	}
}
	
//	//上升下降沿均触发
//	if(GPIO_Pin==GPIO_PIN_11)
//	{
//		KEY_DATA.KEY_armtop=HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_11);
//	}
//	if(GPIO_Pin==GPIO_PIN_12)
//	{
//		KEY_DATA.KEY_armbottom=HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_12);
//	}
//	if(GPIO_Pin==GPIO_PIN_13)
//	{
//		KEY_DATA.KEY_push=HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_13);
//	}
	

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM2)  // 检查是否是TIM2定时器中断
  {
    LAST_TIME_ISR_CNT = TIME_ISR_CNT;
    TIME_ISR_CNT++;
    Microsecond_Cnt++;
    if (Microsecond_Cnt >= 100)
    {
      Microsecond_Cnt = 0;
      Time_Sys[Second]++;
      if (Time_Sys[Second] >= 60)
      {
        Time_Sys[Second] = 0;
        Time_Sys[Minute]++;
        if (Time_Sys[Minute] >= 60)
        {
          Time_Sys[Minute] = 0;
          Time_Sys[Hour]++;
        }
      }
    }
    Time_Sys[MicroSecond] = Microsecond_Cnt;
  }
}


void remote_control(void)
{
	
	float vx=(ROCK_L_X-1500)*5.0f;
	float vy=-(ROCK_L_Y-1500)*5.0f;
	float W=(ROCK_R_X-1500)*2.0f;

	for(int i=0;i<3;i++)
	{
		
		
	}
}


/*---------------------------------------------------------激光通讯--------------------------------------------------------------*/
Laser_Data Laser_Real_Data=0;
// float Last_Data1=0;//上一次的数据
/**
  * 函数功能: 激光数据解算
  * 输入参数: 激光接收数组
  * 返 回 值: 处理后数据
  * 说    明: 无
 */
float Laser_Resolution(uint8_t rx_Data[9])
{

		float Laser_Data=0;
		int Count_2E=0;
		Laser_Data=Laser_Data+(rx_Data[1]-48)*100;
		Laser_Data=Laser_Data+(rx_Data[2]-48)*10;
		Laser_Data=Laser_Data+(rx_Data[3]-48);
		
		Laser_Data=Laser_Data+(rx_Data[5]-48)*0.1f;
		Laser_Data=Laser_Data+(rx_Data[6]-48)*0.01f;
		Laser_Data=Laser_Data+(rx_Data[7]-48)*0.001f;
		Laser_Data=Laser_Data+(rx_Data[8]-48)*0.0001f;
		return Laser_Data;


}


float ID=0x80;
int Head[2]={0x06,0x83};

uint8_t Laser1[9]={0};
unsigned short count=0;
uint8_t Refer_ID=0;
uint8_t Refer1=0;
uint8_t Refer2=0;

uint8_t Laser2[9]={0};
unsigned short count2=0;
uint8_t Refer_ID2=0;
uint8_t Refer3=0;
uint8_t Refer4=0;

/**
  * 函数功能: 1号激光读取函数
  * 输入参数: 激光的值
  * 返 回 值: 无
  * 说    明: 无
 */
void Laser_ReadData(float* Laser_Data)
{
	
	HAL_UART_Receive_IT(&huart2, &UART2_Receiver, 1); // 继续监听

	if(UART2_Receiver==0x80)Refer_ID=0x80;
	else if(UART2_Receiver==0x06)Refer1=0x06;
	else if(UART2_Receiver==0x83)Refer2=0x83;
	if(Refer_ID==0x80&&Refer1==0x06&&Refer2==0x83)
	{
		if(UART2_Receiver==0x45)//Error情况
		{
			Laser_Real_Data.Error_Flag1=1;
			Refer_ID=0;
			Refer1=0;
			Refer2=0;
		}
		else 
		{
			Laser_Real_Data.Error_Flag1=0;
			Laser1[count]=UART2_Receiver;
			count++;
			if(count==9)
			{
				*Laser_Data=Laser_Resolution(Laser1);
				count=0;
				Refer_ID=0;
				Refer1=0;
				Refer2=0;
			}
		}
	}
}



/**
  * 函数功能: 2号激光读取函数
  * 输入参数: 激光的值
  * 返 回 值: 无
  * 说    明: 无
 */
void Laser_ReadData2(float* Laser_Data)
{
	
	HAL_UART_Receive_IT(&huart3, &UART3_Receiver, 1); // 继续监听

	if(UART3_Receiver==0x80)Refer_ID2=0x80;
	else if(UART3_Receiver==0x06)Refer3=0x06;
	else if(UART3_Receiver==0x83)Refer4=0x83;
	if(Refer_ID2==0x80&&Refer3==0x06&&Refer4==0x83)
	{
		if(UART3_Receiver==0x45)
		{
			Laser_Real_Data.Error_Flag2=1;
			Refer_ID2=0;
			Refer3=0;
			Refer4=0;
		}
		else 
		{
			Laser_Real_Data.Error_Flag2=0;
			Laser2[count2]=UART3_Receiver;
			count2++;
			if(count2==9)
			{
				*Laser_Data=Laser_Resolution(Laser2);
				count2=0;
				Refer_ID2=0;
				Refer3=0;
				Refer4=0;
			}
		}
	}
}
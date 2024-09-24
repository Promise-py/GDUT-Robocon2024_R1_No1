/**
  ******************************************************************************
  * @file    communicate.c
  * @author  Py
  * @version V1.0.0
  * @date    2024/5/12
  * @brief   
  ******************************************************************************
  */ 
/* Includes -------------------------------------------------------------------*/
#include "communicate.h"
#include "usart.h"
/* Private  variables ---------------------------------------------------------*/


unsigned char UART4_Receiver= 0;
unsigned char header[2]  = {0x55, 0xaa};
const unsigned char ender[2]   = {0x0d, 0x0a};
unsigned short USART4_FLAG=0;


float Data_left=0;
float Data_right=0;
float Data_mid=0;
int Mode=0;
uint8_t move_flag=0;


int a=0;




/*
 *  ��������UART4_Send_String
 *  ����������usart4�����ַ���
 *  �����������λ�ַ������ַ�������
 *  �����������
 *  ����ֵ����
*/
void UART4_Send_String(uint8_t *p,uint16_t sendSize)
{
	static int length=0;//��̬������ֹ���ݶ�ʧ
	while(length<sendSize)
	{
		while(!(UART4->SR&(0x01<<7)));//���ͻ�����Ϊ��(�������ݻ���λӦ��Ϊ�ڰ�λ)
		UART4->DR=*p;
		p++;
		length++;
	}
	length=0;
}


/**
  * @brief  �����λѭ������У�飬��usartSendData��usartReceiveOneData��������
  * @param   �����ַ�������С
  * @retval 
  */
unsigned char getCrc8(unsigned char *ptr, unsigned short len)
{
	unsigned char crc;
	unsigned char i;
	crc = 0;
	while(len--)
	{
		crc ^= *ptr++;
		for(i = 0; i < 8; i++)
		{
			if(crc&0x01)
                crc=(crc>>1)^0x8C;
			else 
                crc >>= 1;
		}
	}
	return crc;
}



/**
  * @brief  ʹ��uart4�͵���ͨѶ�����ܣ�
  * @param   
  * @retval 
  */
//����������
unsigned char  receiveBuff_u4[22] = {0};
union uart4_ReceiveData
{
	float d;
	unsigned char data[4];
}real_x,real_y,real_w;

union uart4_ReceiveData_int
{
	int d;
	unsigned char data[4];
}chassic_flag; 


int uart4_ReceiveData(float *action_x,float *action_y,float *action_w,int *flag)
{

	static unsigned char checkSum             = 0;
	static unsigned char USARTBufferIndex     = 0;
	static short j=0,k=0;
	static unsigned char USARTReceiverFront   = 0;
	static unsigned char Start_Flag           = START;      //һ֡���ݴ��Ϳ�ʼ��־λ
	static short dataLength                   = 0;

	HAL_UART_Receive_IT(&huart4, &UART4_Receiver, 1); // ��������
	//������Ϣͷ
	if(Start_Flag == START)
	{
		if(UART4_Receiver == 0xaa)                             //buf[1]
		{  
			if(USARTReceiverFront == 0x55)         //����ͷ��λ //buf[0]
			{
				Start_Flag = !START;              //�յ�����ͷ����ʼ��������
				receiveBuff_u4[0]=header[0];         //buf[0]
				receiveBuff_u4[1]=header[1];         //buf[1]
				USARTBufferIndex = 0;             //��������ʼ��
				checkSum = 0x00;				  //У��ͳ�ʼ��
			}
		}
		else 
		{
			USARTReceiverFront = UART4_Receiver;  
		}
	}
	else
    { 
		switch(USARTBufferIndex)
		{
			case 0://�������ݵĳ���
				receiveBuff_u4[2] = UART4_Receiver;
				dataLength     = receiveBuff_u4[2];            //buf[2]
				USARTBufferIndex++;
				break;
			
			case 1://�����������ݣ�����ֵ���� 
				receiveBuff_u4[j + 3] = UART4_Receiver;        //buf[3] buf[4]/buf[5] buf[6]	/	buf[7] buf[8]/		buf[9] buf[10]\	
				j++;
				if(j >= dataLength-1)                         
				{
					j = 0;
					USARTBufferIndex++;
				} 
				break;
				
			case 2://����У��ֵ��Ϣ(�趨Ϊ0x07)
				receiveBuff_u4[2 + dataLength] = UART4_Receiver;
				checkSum = getCrc8(receiveBuff_u4, 3 + dataLength);
				USARTBufferIndex++;
				break;
				
			case 3://������Ϣβ
				if(k==0)
				{
					k++;
				}
				else if (k==1)
				{				
					 for(k = 0; k < 4; k++)
					{
						real_x.data[k]  = receiveBuff_u4[k + 3]; //buf[3]  buf[4] buf[5]  buf[6]
						real_y.data [k] = receiveBuff_u4[k + 7]; //buf[7]  buf[8] buf[9]  buf[10]
						real_w.data [k]  = receiveBuff_u4[k + 11]; //buf[11]  buf[12] buf[13]  buf[14]
						chassic_flag.data[k] = receiveBuff_u4[k + 15]; //buf[15]  buf[16] buf[17]  buf[18]
					}				
					
					//��ֵ����
					*action_x = real_x.d;
					*action_y = real_y.d;
					*action_w = real_w.d;
					*flag = chassic_flag.d;
					//-----------------------------------------------------------------
					//���һ�����ݰ��Ľ��գ���ر������㣬�ȴ���һ�ֽ�����
					USARTBufferIndex   = 0;
					USARTReceiverFront = 0;
					Start_Flag         = START;
					checkSum           = 0;
					dataLength         = 0;
					j = 0;
					k = 0;
					//-----------------------------------------------------------------					
				}
				break;
			 default:break;
		}		
	}
	return 0;
}


/**
  * @brief  ʹ��uart4�͵���ͨѶ�����ͣ�
* @param   
  * @retval 
  */
//unsigned char buf[22]={0};//���ݻ�����

union Uart4_SendData//�������ݵĹ�����
{
	float d;
	unsigned char data[4];
}uart4_vx,uart4_vy,uart4_vw;

union Uart4_SendData_int//�������ݵĹ�����
{
	int d;
	unsigned char data[4];
}uart4_flag;



void Usart4_SendData(float X,float Y,float W,int flag)
{
	
	int i,length = 0;
	unsigned char  buf_u4[22] = {0};
	
	 //memset(buf,0,50);//�������
	uart4_vx.d = X;
	uart4_vy.d = Y;
	uart4_vw.d = W;
	uart4_flag.d = flag;
	for(i=0;i<2;i++)
	{
		buf_u4[i]=header[i];//Э������ͷ
	}
	length = 17;
	buf_u4[2] =length;//sizeof
	
	for(i=0;i<4;i++)
	{
		buf_u4[i+3]=uart4_vx.data[i];
		buf_u4[i+7]=uart4_vy.data[i];
		buf_u4[i+11]=uart4_vw.data[i];
		buf_u4[i+15]=uart4_flag.data[i];
	}

	buf_u4[3+length-1]=getCrc8(buf_u4,3+length);
	buf_u4[3+length]=ender[0];
	buf_u4[3+length+1]=ender[1];
	
	UART4_Send_String(buf_u4,sizeof(buf_u4));//�����ַ������ͺ�����������
}


//int Send_MoveFlag(unsigned short flag)
//{
//	if(Data_mid!=Receive)
//	{
//		Usart4_SendData(0,0,0,Move_Flag);   
//		return 0;
//	}
//	
//	else return 1;
//	
//	
//}

void Send_MoveFlag(unsigned short flag)
{
	if(flag==Stop_Flag)Usart4_SendData(0,0,0,Stop_Flag);
	else if(flag==Move_Flag)Usart4_SendData(0,0,0,Move_Flag);
}

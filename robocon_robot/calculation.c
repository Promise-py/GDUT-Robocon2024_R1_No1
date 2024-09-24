#include "calculation.h"
#include "main.h"
#include "math.h"
#include "motor.h"
#include "hardware.h"

void Update_Action_gl_position(float value[6])
{
	//������һ�ε�ֵ
	ACTION_GL_POS_INFO.LAST_POS_X = ACTION_GL_POS_INFO.POS_X;
	ACTION_GL_POS_INFO.LAST_POS_Y = ACTION_GL_POS_INFO.POS_Y;

	//��¼�˴ε�ֵ
	ACTION_GL_POS_INFO.ANGLE_Z = value[0]; // �Ƕȣ�-180~180
	ACTION_GL_POS_INFO.ANGLE_X = value[1];
	ACTION_GL_POS_INFO.ANGLE_Y = value[2];
	ACTION_GL_POS_INFO.POS_X = value[3]; // ����
	ACTION_GL_POS_INFO.POS_Y = value[4]; // ����
	ACTION_GL_POS_INFO.W_Z = value[5];//���ٶ�
	
	ROBOT_REAL_POS_INFO.Robot_V[w]=ACTION_GL_POS_INFO.W_Z ;

	// �������
	ACTION_GL_POS_INFO.DELTA_POS_X = ACTION_GL_POS_INFO.POS_X - ACTION_GL_POS_INFO.LAST_POS_X;
	ACTION_GL_POS_INFO.DELTA_POS_Y = ACTION_GL_POS_INFO.POS_Y - ACTION_GL_POS_INFO.LAST_POS_Y;
	
	
	//�ۼӵó�������ʵλ��
	ACTION_GL_POS_INFO.REAL_X += (-ACTION_GL_POS_INFO.DELTA_POS_X);                       //action��װʱ����������ϵ��һ���任
	ACTION_GL_POS_INFO.REAL_Y += (-ACTION_GL_POS_INFO.DELTA_POS_Y);
	
	//�任����������
	ROBOT_REAL_POS_INFO.Position[x] =  ACTION_GL_POS_INFO.REAL_X-161.86f * sin(ROBOT_REAL_POS_INFO.Angle* PI / 180) ;
	ROBOT_REAL_POS_INFO.Position[y] =  ACTION_GL_POS_INFO.REAL_Y+161.86f * cos(ROBOT_REAL_POS_INFO.Angle* PI / 180) ;

	//ƫ����ֱ�Ӹ�ֵ
	ROBOT_REAL_POS_INFO.Angle = ACTION_GL_POS_INFO.ANGLE_Z;
}


/*
 *  ��������M3508AngleIntegral
 *  ����������3508������ٶȻ���
 *  ���������MOTO_REAL_INFO
 *  �����������
 *  ����ֵ����
*/
void M3508AngleIntegral(MOTO_REAL_INFO *M3508_MOTOR)
{
	float delta_pos = 0;
	
	// ��¼��һ�ν���ʱ������
	if(!M3508_MOTOR->FIRST_ANGLE_INTEGRAL_FLAG)
	{
		M3508_MOTOR->LAST_ANGLE = M3508_MOTOR->ANGLE;
		M3508_MOTOR->FIRST_ANGLE_INTEGRAL_FLAG = 1;
		return;
	}
	
	// ����仯�ĽǶ�
	if(M3508_MOTOR->RPM >= 0)
	{
		if(M3508_MOTOR->ANGLE < M3508_MOTOR->LAST_ANGLE)
		{
			if(ABS(8191 + M3508_MOTOR->ANGLE - M3508_MOTOR->LAST_ANGLE) < 1250)  // ��������CAN����ʱ�������ת���ǶȽ����˲�
			{
				delta_pos = ((float)(8191 + M3508_MOTOR->ANGLE - M3508_MOTOR->LAST_ANGLE) / 8191.0f) * 360.0f;
				delta_pos = delta_pos / 19;	//���ٱ�
			}
		}
		
		else
		{
			delta_pos = ((float)(M3508_MOTOR->ANGLE - M3508_MOTOR->LAST_ANGLE) / 8191.0f) * 360.0f;
			delta_pos = delta_pos / 19;	//���ٱ�
		}
		
		// �˲�
		if(delta_pos > 0) 
			M3508_MOTOR->REAL_ANGLE += delta_pos;  // ����	
	}
	
	else
	{
		if(M3508_MOTOR->ANGLE > M3508_MOTOR->LAST_ANGLE)
		{
			if(ABS(8191 - M3508_MOTOR->ANGLE + M3508_MOTOR->LAST_ANGLE) < 1250)  // ��������CAN����ʱ�������ת���ǶȽ����˲�			
			{
				delta_pos = ((float)(8191 - M3508_MOTOR->ANGLE + M3508_MOTOR->LAST_ANGLE) / 8191.0f) * 360.0f;
				delta_pos = delta_pos /19;	//���ٱ�
			}
		}	
		else
		{
			delta_pos = ((float)(M3508_MOTOR->ANGLE - M3508_MOTOR->LAST_ANGLE) / 8191.0f) * 360.0f;
			delta_pos = delta_pos / 19;	//���ٱ�
		}
		
		// �˲�
		if(delta_pos < 0) 
			M3508_MOTOR->REAL_ANGLE += delta_pos;  // ����
	}

	// �洢�Ƕ�ֵ 
	M3508_MOTOR->LAST_ANGLE = M3508_MOTOR->ANGLE;
}



/**
  * @brief  �ٶȹ滮
	* @param  
	* @retval none
  */
//                                                   ��ʼλ��   ����λ��    ��ʼ���ٶ�(RPM ����ֵ)  �����ٶ�	ĩβ���ٶ�   ����·�̵ı���   ����·�̵ı���
void VelocityPlanningMODE(MOTO_REAL_INFO *M3508_MOTOR)	
{
	static int cnt;//��ʱ��
	float Ssu;   //��·��
	float Sac;   //����·��
	float Sde;   //����·��
	float Sco;   //����·��
	float Aac;   //���ټ��ٶ�
	float Ade;   //���ټ��ٶ�
	float S;     //��ǰ·��
	// �����������������ִ���ٶȹ滮		
	if((M3508_MOTOR->velocity_planning.Rac > 1) || (M3508_MOTOR->velocity_planning.Rac < 0) ||		//����·�̵ı���
		 (M3508_MOTOR->velocity_planning.Rde > 1) || (M3508_MOTOR->velocity_planning.Rde < 0) ||	//����·�̵ı���
		 (M3508_MOTOR->velocity_planning.Vmax < M3508_MOTOR->velocity_planning.Vstart) )			//�����ٶ�<��ʼ���ٶ� 
	{
		M3508_MOTOR->TARGET_RPM = 0;  // ���צ���˶�
		return;
	}
	// ����ģʽ
	if(M3508_MOTOR->velocity_planning.Pstart == M3508_MOTOR->velocity_planning.Pend)	//��ʼλ��=����λ��
	{
		M3508_MOTOR->TARGET_RPM = M3508_MOTOR->velocity_planning.Vstart * M3508_MOTOR->velocity_planning.Vmax;	//��ʼ���ٶ�*�����ٶ�
		return;
	}
	
	// ����һЩ����
	Ssu = ABS(M3508_MOTOR->velocity_planning.Pend - M3508_MOTOR->velocity_planning.Pstart); 	//��·��   
	Sac = Ssu * M3508_MOTOR->velocity_planning.Rac;		//����·�� =	��·�� * ����·�̵ı���
	Sde = Ssu * M3508_MOTOR->velocity_planning.Rde;		//����·�� =	��·�� * ����·�̵ı���
	Sco = Ssu - Sac - Sde;		//����·�� = ��·�� - ����·�� - ����·��
	Aac = (M3508_MOTOR->velocity_planning.Vmax * M3508_MOTOR->velocity_planning.Vmax - M3508_MOTOR->velocity_planning.Vstart * M3508_MOTOR->velocity_planning.Vstart) / (2.0f * Sac);	//���ټ��ٶ� (�����ٶ�*�����ٶ� - ��ʼ���ٶ� *��ʼ���ٶ� ) / (2.0f * ����·��)
	Ade = (M3508_MOTOR->velocity_planning.Vend * M3508_MOTOR->velocity_planning.Vend -   M3508_MOTOR->velocity_planning.Vmax *   M3508_MOTOR->velocity_planning.Vmax) / (2.0f * Sde);	  
	
	// �����쳣���
	if(((M3508_MOTOR->velocity_planning.Pend > M3508_MOTOR->velocity_planning.Pstart) && (M3508_MOTOR->REAL_ANGLE < M3508_MOTOR->velocity_planning.Pstart)) ||		//[(����λ�� > ��ʼλ��) && (���������ʵ�Ƕ�pos <��ʼλ��)]	||
		 ((M3508_MOTOR->velocity_planning.Pend < M3508_MOTOR->velocity_planning.Pstart) && (M3508_MOTOR->REAL_ANGLE > M3508_MOTOR->velocity_planning.Pstart)))		//	[(����λ�� < ��ʼλ��) && (���������ʵ�Ƕ�pos >��ʼλ��)]
	{
		M3508_MOTOR->TARGET_RPM = M3508_MOTOR->velocity_planning.Vstart;	//TARGET_RPM = ��ʼ���ٶ�
	}
	else if(((M3508_MOTOR->velocity_planning.Pend > M3508_MOTOR->velocity_planning.Pstart) && (M3508_MOTOR->REAL_ANGLE > M3508_MOTOR->velocity_planning.Pend)) ||
		      ((M3508_MOTOR->velocity_planning.Pend < M3508_MOTOR->velocity_planning.Pstart) && (M3508_MOTOR->REAL_ANGLE < M3508_MOTOR->velocity_planning.Pend)))
	{
		M3508_MOTOR->TARGET_RPM = M3508_MOTOR->velocity_planning.Vend;	//TARGET_RPM = ĩβ���ٶ�
	}
	else
	{
		S = ABS(M3508_MOTOR->REAL_ANGLE - M3508_MOTOR->velocity_planning.Pstart);      //��ʼλ��
		
		// �滮RPM
		if     (S < Sac)       M3508_MOTOR->TARGET_RPM = sqrt(2.0f * Aac * S + M3508_MOTOR->velocity_planning.Vstart * M3508_MOTOR->velocity_planning.Vstart);               // ���ٽ׶�
		else if(S < (Sac+Sco)) M3508_MOTOR->TARGET_RPM = M3508_MOTOR->velocity_planning.Vmax;                                                        // ���ٽ׶�
		else                   M3508_MOTOR->TARGET_RPM = sqrt(M3508_MOTOR->velocity_planning.Vend * M3508_MOTOR->velocity_planning.Vend - 2.0f * Ade * ABS(Ssu - S));  // ���ٽ׶�
	}
	 
	// ������ʵ�������
	if(M3508_MOTOR->velocity_planning.Pend < M3508_MOTOR->velocity_planning.Pstart) M3508_MOTOR->TARGET_RPM = -M3508_MOTOR->TARGET_RPM;
	//�ж��Ƿ����
	if((fabsf(M3508_MOTOR->REAL_ANGLE - M3508_MOTOR->velocity_planning.Pend)) < 5)
		M3508_MOTOR->velocity_planning.flag = 1;//���ñ�־λ			
						
   if((fabsf(M3508_MOTOR->REAL_ANGLE - M3508_MOTOR->velocity_planning.Pend)) > 5)
			M3508_MOTOR->velocity_planning.flag = 0;
}

/**
  * @brief  �����ٶȹ滮�Ĳ����������ٶȹ滮����
	* @param  
	* float Pstart;        //��ʼλ��
	* float Pend;          //����λ��
	* float Vstart;        //��ʼ���ٶ�           // ��λ��RPM ����ֵ
	* float Vmax;          //�����ٶ�
	* float Vend;          //ĩβ���ٶ�
	* float Rac;           //����·�̵ı���
	* float Rde;           //����·�̵ı���
	* @retval none
  */
void Velocity_Planning_setpos(MOTO_REAL_INFO *M3508_MOTOR,float Pstart,float Pend,float Vstart,float Vmax,float Vend,float Rac,float Rde)
{
	
//	M3508_MOTOR->unitMode = VELOCITY_PLANNING_MODE;
	M3508_MOTOR->velocity_planning.Pstart = Pstart;
	M3508_MOTOR->velocity_planning.Pend = Pend;
	M3508_MOTOR->velocity_planning.Vstart = Vstart;
	M3508_MOTOR->velocity_planning.Vmax = Vmax;
	M3508_MOTOR->velocity_planning.Vend = Vend;
	M3508_MOTOR->velocity_planning.Rac = Rac;
	M3508_MOTOR->velocity_planning.Rde = Rde;
}




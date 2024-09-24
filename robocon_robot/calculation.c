#include "calculation.h"
#include "main.h"
#include "math.h"
#include "motor.h"
#include "hardware.h"

void Update_Action_gl_position(float value[6])
{
	//储存上一次的值
	ACTION_GL_POS_INFO.LAST_POS_X = ACTION_GL_POS_INFO.POS_X;
	ACTION_GL_POS_INFO.LAST_POS_Y = ACTION_GL_POS_INFO.POS_Y;

	//记录此次的值
	ACTION_GL_POS_INFO.ANGLE_Z = value[0]; // 角度，-180~180
	ACTION_GL_POS_INFO.ANGLE_X = value[1];
	ACTION_GL_POS_INFO.ANGLE_Y = value[2];
	ACTION_GL_POS_INFO.POS_X = value[3]; // 有用
	ACTION_GL_POS_INFO.POS_Y = value[4]; // 有用
	ACTION_GL_POS_INFO.W_Z = value[5];//角速度
	
	ROBOT_REAL_POS_INFO.Robot_V[w]=ACTION_GL_POS_INFO.W_Z ;

	// 差分运算
	ACTION_GL_POS_INFO.DELTA_POS_X = ACTION_GL_POS_INFO.POS_X - ACTION_GL_POS_INFO.LAST_POS_X;
	ACTION_GL_POS_INFO.DELTA_POS_Y = ACTION_GL_POS_INFO.POS_Y - ACTION_GL_POS_INFO.LAST_POS_Y;
	
	
	//累加得出最终真实位置
	ACTION_GL_POS_INFO.REAL_X += (-ACTION_GL_POS_INFO.DELTA_POS_X);                       //action安装时跟场地坐标系有一个变换
	ACTION_GL_POS_INFO.REAL_Y += (-ACTION_GL_POS_INFO.DELTA_POS_Y);
	
	//变换到底盘中心
	ROBOT_REAL_POS_INFO.Position[x] =  ACTION_GL_POS_INFO.REAL_X-161.86f * sin(ROBOT_REAL_POS_INFO.Angle* PI / 180) ;
	ROBOT_REAL_POS_INFO.Position[y] =  ACTION_GL_POS_INFO.REAL_Y+161.86f * cos(ROBOT_REAL_POS_INFO.Angle* PI / 180) ;

	//偏航角直接赋值
	ROBOT_REAL_POS_INFO.Angle = ACTION_GL_POS_INFO.ANGLE_Z;
}


/*
 *  函数名：M3508AngleIntegral
 *  功能描述：3508电机的速度积分
 *  输入参数：MOTO_REAL_INFO
 *  输出参数：无
 *  返回值：无
*/
void M3508AngleIntegral(MOTO_REAL_INFO *M3508_MOTOR)
{
	float delta_pos = 0;
	
	// 记录第一次进入时的数据
	if(!M3508_MOTOR->FIRST_ANGLE_INTEGRAL_FLAG)
	{
		M3508_MOTOR->LAST_ANGLE = M3508_MOTOR->ANGLE;
		M3508_MOTOR->FIRST_ANGLE_INTEGRAL_FLAG = 1;
		return;
	}
	
	// 计算变化的角度
	if(M3508_MOTOR->RPM >= 0)
	{
		if(M3508_MOTOR->ANGLE < M3508_MOTOR->LAST_ANGLE)
		{
			if(ABS(8191 + M3508_MOTOR->ANGLE - M3508_MOTOR->LAST_ANGLE) < 1250)  // 利用两次CAN接收时间电机最大转动角度进行滤波
			{
				delta_pos = ((float)(8191 + M3508_MOTOR->ANGLE - M3508_MOTOR->LAST_ANGLE) / 8191.0f) * 360.0f;
				delta_pos = delta_pos / 19;	//减速比
			}
		}
		
		else
		{
			delta_pos = ((float)(M3508_MOTOR->ANGLE - M3508_MOTOR->LAST_ANGLE) / 8191.0f) * 360.0f;
			delta_pos = delta_pos / 19;	//减速比
		}
		
		// 滤波
		if(delta_pos > 0) 
			M3508_MOTOR->REAL_ANGLE += delta_pos;  // 积分	
	}
	
	else
	{
		if(M3508_MOTOR->ANGLE > M3508_MOTOR->LAST_ANGLE)
		{
			if(ABS(8191 - M3508_MOTOR->ANGLE + M3508_MOTOR->LAST_ANGLE) < 1250)  // 利用两次CAN接收时间电机最大转动角度进行滤波			
			{
				delta_pos = ((float)(8191 - M3508_MOTOR->ANGLE + M3508_MOTOR->LAST_ANGLE) / 8191.0f) * 360.0f;
				delta_pos = delta_pos /19;	//减速比
			}
		}	
		else
		{
			delta_pos = ((float)(M3508_MOTOR->ANGLE - M3508_MOTOR->LAST_ANGLE) / 8191.0f) * 360.0f;
			delta_pos = delta_pos / 19;	//减速比
		}
		
		// 滤波
		if(delta_pos < 0) 
			M3508_MOTOR->REAL_ANGLE += delta_pos;  // 积分
	}

	// 存储角度值 
	M3508_MOTOR->LAST_ANGLE = M3508_MOTOR->ANGLE;
}



/**
  * @brief  速度规划
	* @param  
	* @retval none
  */
//                                                   开始位置   结束位置    开始的速度(RPM 绝对值)  最大的速度	末尾的速度   加速路程的比例   减速路程的比例
void VelocityPlanningMODE(MOTO_REAL_INFO *M3508_MOTOR)	
{
	static int cnt;//记时用
	float Ssu;   //总路程
	float Sac;   //加速路程
	float Sde;   //减速路程
	float Sco;   //匀速路程
	float Aac;   //加速加速度
	float Ade;   //减速加速度
	float S;     //当前路程
	// 如果所配数据有误，则不执行速度规划		
	if((M3508_MOTOR->velocity_planning.Rac > 1) || (M3508_MOTOR->velocity_planning.Rac < 0) ||		//加速路程的比例
		 (M3508_MOTOR->velocity_planning.Rde > 1) || (M3508_MOTOR->velocity_planning.Rde < 0) ||	//减速路程的比例
		 (M3508_MOTOR->velocity_planning.Vmax < M3508_MOTOR->velocity_planning.Vstart) )			//最大的速度<开始的速度 
	{
		M3508_MOTOR->TARGET_RPM = 0;  // 令夹爪不运动
		return;
	}
	// 匀速模式
	if(M3508_MOTOR->velocity_planning.Pstart == M3508_MOTOR->velocity_planning.Pend)	//开始位置=结束位置
	{
		M3508_MOTOR->TARGET_RPM = M3508_MOTOR->velocity_planning.Vstart * M3508_MOTOR->velocity_planning.Vmax;	//开始的速度*最大的速度
		return;
	}
	
	// 计算一些变量
	Ssu = ABS(M3508_MOTOR->velocity_planning.Pend - M3508_MOTOR->velocity_planning.Pstart); 	//总路程   
	Sac = Ssu * M3508_MOTOR->velocity_planning.Rac;		//加速路程 =	总路程 * 加速路程的比例
	Sde = Ssu * M3508_MOTOR->velocity_planning.Rde;		//减速路程 =	总路程 * 减速路程的比例
	Sco = Ssu - Sac - Sde;		//匀速路程 = 总路程 - 加速路程 - 减速路程
	Aac = (M3508_MOTOR->velocity_planning.Vmax * M3508_MOTOR->velocity_planning.Vmax - M3508_MOTOR->velocity_planning.Vstart * M3508_MOTOR->velocity_planning.Vstart) / (2.0f * Sac);	//加速加速度 (最大的速度*最大的速度 - 开始的速度 *开始的速度 ) / (2.0f * 加速路程)
	Ade = (M3508_MOTOR->velocity_planning.Vend * M3508_MOTOR->velocity_planning.Vend -   M3508_MOTOR->velocity_planning.Vmax *   M3508_MOTOR->velocity_planning.Vmax) / (2.0f * Sde);	  
	
	// 过滤异常情况
	if(((M3508_MOTOR->velocity_planning.Pend > M3508_MOTOR->velocity_planning.Pstart) && (M3508_MOTOR->REAL_ANGLE < M3508_MOTOR->velocity_planning.Pstart)) ||		//[(结束位置 > 开始位置) && (处理过的真实角度pos <开始位置)]	||
		 ((M3508_MOTOR->velocity_planning.Pend < M3508_MOTOR->velocity_planning.Pstart) && (M3508_MOTOR->REAL_ANGLE > M3508_MOTOR->velocity_planning.Pstart)))		//	[(结束位置 < 开始位置) && (处理过的真实角度pos >开始位置)]
	{
		M3508_MOTOR->TARGET_RPM = M3508_MOTOR->velocity_planning.Vstart;	//TARGET_RPM = 开始的速度
	}
	else if(((M3508_MOTOR->velocity_planning.Pend > M3508_MOTOR->velocity_planning.Pstart) && (M3508_MOTOR->REAL_ANGLE > M3508_MOTOR->velocity_planning.Pend)) ||
		      ((M3508_MOTOR->velocity_planning.Pend < M3508_MOTOR->velocity_planning.Pstart) && (M3508_MOTOR->REAL_ANGLE < M3508_MOTOR->velocity_planning.Pend)))
	{
		M3508_MOTOR->TARGET_RPM = M3508_MOTOR->velocity_planning.Vend;	//TARGET_RPM = 末尾的速度
	}
	else
	{
		S = ABS(M3508_MOTOR->REAL_ANGLE - M3508_MOTOR->velocity_planning.Pstart);      //开始位置
		
		// 规划RPM
		if     (S < Sac)       M3508_MOTOR->TARGET_RPM = sqrt(2.0f * Aac * S + M3508_MOTOR->velocity_planning.Vstart * M3508_MOTOR->velocity_planning.Vstart);               // 加速阶段
		else if(S < (Sac+Sco)) M3508_MOTOR->TARGET_RPM = M3508_MOTOR->velocity_planning.Vmax;                                                        // 匀速阶段
		else                   M3508_MOTOR->TARGET_RPM = sqrt(M3508_MOTOR->velocity_planning.Vend * M3508_MOTOR->velocity_planning.Vend - 2.0f * Ade * ABS(Ssu - S));  // 减速阶段
	}
	 
	// 分配合适的正负号
	if(M3508_MOTOR->velocity_planning.Pend < M3508_MOTOR->velocity_planning.Pstart) M3508_MOTOR->TARGET_RPM = -M3508_MOTOR->TARGET_RPM;
	//判断是否完成
	if((fabsf(M3508_MOTOR->REAL_ANGLE - M3508_MOTOR->velocity_planning.Pend)) < 5)
		M3508_MOTOR->velocity_planning.flag = 1;//设置标志位			
						
   if((fabsf(M3508_MOTOR->REAL_ANGLE - M3508_MOTOR->velocity_planning.Pend)) > 5)
			M3508_MOTOR->velocity_planning.flag = 0;
}

/**
  * @brief  设置速度规划的参数，开启速度规划控制
	* @param  
	* float Pstart;        //开始位置
	* float Pend;          //结束位置
	* float Vstart;        //开始的速度           // 单位：RPM 绝对值
	* float Vmax;          //最大的速度
	* float Vend;          //末尾的速度
	* float Rac;           //加速路程的比例
	* float Rde;           //减速路程的比例
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




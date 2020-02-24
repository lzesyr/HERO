#include "gimbal.h"
#include "filter.h"
#include "kalman_filter.h"
#include "vision.h"
typedef enum
{
	MACH = 0,
 	GYRO = 1,
}eGimbalCtrlMode;
eGimbalCtrlMode  modeGimbal;

typedef enum
{
	Gimbal_NORMAL  = 0, //正常模式
	Gimbal_Turn    = 1, //转身模式
	GIMBAL_AUTO    = 2, //自瞄模式
	Gimbal_Diao    = 3, //桥头吊射模式
	Gimbal_HIGHT   = 4, //抬头模式
	Gimbal_Duanya  = 5, //断崖吊射
}eactGimbal;
eactGimbal actGimbal;

/***************自瞄******************/
//角度补偿
float Offset_Angle_Yaw;//随目标距离和瞄准模式实时改变
float Offset_Angle_Pitch;

//误差
float Auto_Error_Yaw[2];//    now/last
float Auto_Error_Pitch[2];
float Auto_Distance;//距离单目

//自瞄突然开启,卡尔曼滤波开启延时
uint16_t Auto_KF_Delay = 0;

/*************卡尔曼滤波**************/
/*一阶卡尔曼*/
//云台角度误差卡尔曼
extKalman_t Gimbal_Pitch_Mach_Error_Kalman;//定义一个kalman指针
extKalman_t Gimbal_Pitch_Gyro_Error_Kalman;//定义一个kalman指针
extKalman_t Gimbal_Pitch_Auto_Error_Kalman;//定义一个kalman指针
extKalman_t Gimbal_Yaw_Mach_Error_Kalman;//定义一个kalman指针
extKalman_t Gimbal_Yaw_Gyro_Error_Kalman;//定义一个kalman指针
extKalman_t Gimbal_Yaw_Auto_Error_Kalman;//定义一个kalman指针

extKalman_t Vision_Distance_Kalman;


typedef struct  //视觉目标速度测量
{
  int delay_cnt;//计算相邻两帧目标不变持续时间,用来判断速度是否为0
  int freq;
  int last_time;//上次受到目标角度的时间
  float last_position;//上个目标角度
  float speed;//速度
  float last_speed;//上次速度
  float processed_speed;//速度计算结果
}speed_calc_data_t;

speed_calc_data_t Vision_Yaw_speed_Struct;
speed_calc_data_t Vision_Pitch_speed_Struct;

kalman_filter_init_t yaw_kalman_filter_para = {
  .P_data = {2, 0, 0, 2},
  .A_data = {1, 0.002, 0, 1},//采样时间间隔
  .H_data = {1, 0, 0, 1},
  .Q_data = {1, 0, 0, 1},
  .R_data = {200, 0, 0, 700}
};//初始化yaw的部分kalman参数

kalman_filter_init_t pitch_kalman_filter_para = {
  .P_data = {2, 0, 0, 2},
  .A_data = {1, 0.002, 0, 1},//采样时间间隔
  .H_data = {1, 0, 0, 1},
  .Q_data = {1, 0, 0, 1},
  .R_data = {400, 0, 0, 1000}
};//初始化pitch的部分kalman参数

kalman_filter_t  yaw_kalman_filter;
kalman_filter_t  pitch_kalman_filter;

//float kKalman_Q;//测量置信度
//float kKalman_R;//预测置信度

/*自动打弹用的一些标志位*/
bool Mobility_Prediction_Yaw = FALSE;//预测是否开启标志位
bool Mobi_Pre_Yaw_Fire = FALSE;//默认预测没到位，禁止开枪
uint16_t mobpre_yaw_left_delay = 0;//向左预测延时判断可开火消抖
uint16_t mobpre_yaw_right_delay = 0;//向右预测延时判断可开火消抖
uint16_t mobpre_yaw_stop_delay = 0;//预测关闭延时判断可开火消抖

/* 云台控制参数 */ 
float outP[3][2]={1.5,-4.5,0,3.5,3,4},outI[3][2],inP[3][2]={-290,100,0,180,-250,170},inI[3][2]={150,150,0,150,200,280};         //PID参数
float Gim_Pterm[3][2],Gim_Iterm[3][2],Gimbal_ErrorSum[3][2],Gimbal_ErrorSum_Max[3][2];        //PID中间值
float AngleError[3][2],RateError[3][2],Target[2],aimAngle[3][2],aimRate[3][2];
float Mouse_Yaw[2]={3,2.5}, Mouse_PIT[2]={3,2}, Slow_G_speed_PITCH, Slow_V_speed_PITCH;  //鼠标速度计算量
float Calc_yaw;
Critical_t  Critical_Gyro_yaw;

/* 转弯调头 */
float Q_TurnAngle, E_TurnAngle, C_TurnAngle;     //转身角度
bool  TurnLeft, TurnRight, TurnBack;             //转身标志位 

bool  sentry_flag;  //是否击打哨兵标志位

 /* 吊射参数  */
float Base_Yaw_Comp_Gimbal = -60; //吊射左右校准
float Base_Error_Yaw;  //获取像素误差
float yaw_luck_time, Yaw_Luck_Time = 50;
bool  Base_Yaw_ready_flag = 0;

void Gimbal_Control(void)
{
	if( modeGimbal == GYRO )
		Calc_yaw = Critical_Handle( &Critical_Gyro_yaw, yaw * 20 );	
	
	if(Get_SystemState() == SYSTEM_STARTING)
		{
			Gimbal_InitControl();
		}
	else
		{
			if( Get_remoteMode() == RC )
				{
					RC_Gimbal_control();
				}
			else
				{
					KEY_Gimbal_control();
				}
		}
		
	CAN1_Gimbal_Send();
}
 
void Gimbal_InitControl(void)
{
	static signed char AngleRecord = FALSE;
	static uint32_t Currenttime = 0;
	
	if(micros() - Currenttime > 100000)
		{
			AngleRecord = FALSE;
		}
	
	Currenttime = micros();//复位时间
	
	if(AngleRecord == FALSE)
		{
			AngleRecord = TRUE;

			aimAngle[M][PITCH] = Motor_angel[PITCH]; //记录当前电机机械角
			aimAngle[M][YAW] = Motor_angel[YAW];
		}
	
	modeGimbal = MACH;//默认启动模式为机械模式
	
	XIANWEI_Angle( OPEN_ANGLE );	//打开舵机，防止枪管有弹堵转舵机
		
	aimAngle[M][PITCH] = slope_Control(P_M_MID, aimAngle[M][PITCH], 4);
	aimAngle[M][YAW] = slope_Control(Y_M_MID, aimAngle[M][YAW], 4);
		
	PITCH_Mach_PID();
	YAW_Mach_PID();
}
	
void RC_Gimbal_control()
{
	if(RC_Ctl.rc.s2 == RC_SW_MID)  //右开关居中
		{
			modeGimbal = GYRO;
		}
	else
		{
			modeGimbal = MACH;
		}
				
	if( modeGimbal == MACH )
		{
			aimAngle[M][PITCH] -= (-RC_Ctl.rc.ch1 + RC_CH_VALUE_OFFSET) * 0.01;
			aimAngle[M][YAW] = Y_M_MID;
				
			PITCH_Mach_PID();
			YAW_Mach_PID();
		}
	else if( modeGimbal == GYRO )
		{
			if( ((Motor_angel[YAW] >= Y_M_MAX - 600) && (RC_Ctl.rc.ch0 > RC_CH_VALUE_OFFSET))
						|| ((Motor_angel[YAW] <= Y_M_MIN + 600) && (RC_Ctl.rc.ch0 < RC_CH_VALUE_OFFSET)) )
				{
					aimAngle[G][YAW] += (-RC_Ctl.rc.ch0 + RC_CH_VALUE_OFFSET) * 0.010;
				}
			else
				{
					aimAngle[G][YAW]  += (-RC_Ctl.rc.ch0 + RC_CH_VALUE_OFFSET) * 0.020;
				}
				
			aimAngle[M][PITCH] -= (-RC_Ctl.rc.ch1 + RC_CH_VALUE_OFFSET) * 0.01;
				
			PITCH_Mach_PID();
			YAW_Gyro_PID();
		}
}
	
void KEY_Gimbal_control(void)
{	
	switch (actGimbal)
		{
			case Gimbal_NORMAL:
			if( IF_KEY_PRESSED_CTRL || Get_Chassis_Mode() == 4 || Get_Chassis_Mode() == 5 )//取弹模式和断崖吊射模式进入机械模式
				modeGimbal = MACH;
			else
				modeGimbal = GYRO;
			
			//模式切换	
			if( !IF_KEY_PRESSED_CTRL && !IF_MOUSE_PRESSED_RIGH && (IF_KEY_PRESSED_Q || IF_KEY_PRESSED_E || IF_KEY_PRESSED_C) )
				{
					actGimbal = Gimbal_Turn;
					if( IF_KEY_PRESSED_Q )
						{
							TurnLeft = 1;
						}
					else if( IF_KEY_PRESSED_E )
						{
							TurnRight = 1;
						}
					else if( IF_KEY_PRESSED_C )
						{
							TurnBack = 1;
						}
				}
			
			else if( IF_MOUSE_PRESSED_RIGH && RC_Ctl.rc.ch1 != RC_SW_MID )
				{
					actGimbal = GIMBAL_AUTO;
				}
				
			else if( IF_KEY_PRESSED_G )
				{
					actGimbal = Gimbal_Diao;
				}
				
			else if( IF_KEY_PRESSED_V && !IF_MOUSE_PRESSED_RIGH )
				{
					actGimbal = Gimbal_Duanya;
				}
				
			else if( IF_MOUSE_PRESSED_LEFT && Fric42_ready == 0 ) 
				{
					actGimbal = Gimbal_HIGHT;
				}
				
			else
				{
					if( modeGimbal == MACH )
						{
							aimAngle[M][PITCH] -= MOUSE_Y_MOVE_SPEED/Mouse_PIT[M];
							aimAngle[M][YAW] = Y_M_MID;
							
							PITCH_Mach_PID();
							YAW_Mach_PID();
						}
					else if( modeGimbal == GYRO )
						{
							if( ((Motor_angel[YAW] >= Y_M_MAX - 300) && (MOUSE_X_MOVE_SPEED > 0))
								|| ((Motor_angel[YAW] <= Y_M_MIN + 300) && (MOUSE_X_MOVE_SPEED < 0)) )
								{
									MOUSE_X_MOVE_SPEED = 0;
								}
							
							aimAngle[M][PITCH] -= MOUSE_Y_MOVE_SPEED/Mouse_PIT[G];
							aimAngle[G][YAW] += -MOUSE_X_MOVE_SPEED/Mouse_Yaw[G];
								
							PITCH_Mach_PID();
							YAW_Gyro_PID();
						}
				}
			break;
				
			case Gimbal_Turn:
			modeGimbal = GYRO;
		
			if( (!IF_KEY_PRESSED_Q) && (TurnLeft == 1) )
				{
					Q_TurnAngle += 20;
					if( Q_TurnAngle < 1800 )
						{
							aimAngle[G][YAW] += 20;
						}
					else
						{
							Q_TurnAngle = 0;
							TurnLeft = 0;
							actGimbal = Gimbal_NORMAL;
						}
				}
				
			if( (!IF_KEY_PRESSED_E) && (TurnRight == 1) )
				{
					E_TurnAngle += 20;
					if( E_TurnAngle < 1800 )
						{
							aimAngle[G][YAW] -= 20;
						}
					else
						{
							E_TurnAngle = 0;
							TurnRight = 0;
							actGimbal = Gimbal_NORMAL;
						}
				}
				
			if( (!IF_KEY_PRESSED_C) && (TurnBack == 1) )
				{
					C_TurnAngle += 25;
					if( C_TurnAngle < 3600 )
						{
							aimAngle[G][YAW] += 25;
						}
					else
						{
							C_TurnAngle = 0;
							TurnBack = 0;
							actGimbal = Gimbal_NORMAL;
						}
				}
				
				PITCH_Mach_PID();
				YAW_Gyro_PID();
			break;
				
			case GIMBAL_AUTO:
			modeGimbal = GYRO;
			
			if( !IF_MOUSE_PRESSED_RIGH )
				{
					actGimbal = Gimbal_NORMAL;
					
					//自瞄目标偏差清零,避免切换时云台跳动
					VisionRecvData.identify_target = FALSE;
					Auto_KF_Delay = 0;//清零给下次延迟预测用
					Mobility_Prediction_Yaw = FALSE;//标记预测没开启
					Mobi_Pre_Yaw_Fire = FALSE;//默认标记预测没到位，禁止开火
				
					mobpre_yaw_left_delay  = 0;//重置左预测的开火延迟
					mobpre_yaw_right_delay = 0;//重置右预测的开火延迟	
					mobpre_yaw_stop_delay = 0;//停止预测开火延时重置
				}
			else
				{
					GIMBAL_AUTO_Mode_Ctrl();
				}
			break;
				
			case Gimbal_Diao:
//				modeGimbal = GYRO;
			if( !IF_KEY_PRESSED_G )
				{
					actGimbal = Gimbal_NORMAL;
					Slow_G_speed_PITCH = 0;
					Base_Yaw_ready_flag = 0;
					yaw_luck_time = 0;
				}
			else
				{
					GIMBAL_BASE_Mode_Ctrl();
				}
			break;
				
			case Gimbal_HIGHT:
				modeGimbal = MACH;
			
			if( Fric42_ready == 1 )
				{
					actGimbal = Gimbal_NORMAL;
					aimAngle[M][PITCH] = P_M_MID;
				}
			else
				{
					aimAngle[M][PITCH] = slope_Control( FRC_MACH_PIT, aimAngle[M][PITCH], 5);
				}
				
			aimAngle[M][YAW] = Y_M_MID;
				
			PITCH_Mach_PID();
			YAW_Mach_PID();
			break;
				
			case Gimbal_Duanya:
				modeGimbal = MACH;
			
			if( !IF_KEY_PRESSED_V || IF_MOUSE_PRESSED_RIGH )
				{
					actGimbal = Gimbal_NORMAL;
					Slow_V_speed_PITCH = 0;
				}
			else
				{
					Slow_V_speed_PITCH += MOUSE_Y_MOVE_SPEED/Mouse_PIT[G]/5;
					
					aimAngle[M][PITCH] = 4540 - Slow_V_speed_PITCH;
					aimAngle[M][YAW]   = Y_M_MID;
					
					PITCH_Mach_PID();
					YAW_Mach_PID();
				}
			break;
		}	
		
	KlamanCalc();
}


float Target_Speed_Calc(speed_calc_data_t *S, uint32_t time, float position);
void GIMBAL_KalmanParameter_Init(void);
/**
  * @brief  自瞄控制函数
  * @param  void
  * @retval void
  * @attention 中间坐标(0,0),左正右负,上负下正
  *            yaw为陀螺仪模式,pitch为机械模式(其实pitch全程都在用机械模式)
  *            只有当新数据来了才在当前实时角度上累加误差当成目标角度
  *            新数据一来,目标角度就实时更新
  */
float debug_yaw_sk = 76;//85;//移动预测系数,越大预测越多
float debug_pitch_sk = 30;

uint32_t Vision_Time[2];// NOW/LAST
float vision_time_js;

float Vision_Angle_Speed_Yaw;//卡尔曼滤波yaw速度测量值
float Vision_Angle_Speed_Pitch;//卡尔曼滤波pitch速度测量值
float *yaw_kf_result;//二阶卡尔曼yaw滤波结果,0角度 1速度
float *pitch_kf_result;//二阶卡尔曼pitch滤波结果,0角度 1速度

float error_yaw_k   = 11;//误差放大
float error_pitch_k = 6; //误差放大

float debug_auto_yaw_err = 14;   //yaw角度过大关闭预测
float debug_auto_pitch_err = 4; //pitch角度过大关闭预测
float debug_kf_delay = 100;//80;  //预测延时开启

float debug_kf_yaw_speed = 0.2;  //速度过低关闭预测
float debug_kf_yaw_speed_h = 5;//6.5;//速度过高关闭预测
float debug_kf_yaw_angle;
float debug_kf_pitch_speed = 0.08;
float debug_kf_pitch_speed_h = 3;
float debug_kf_pitch_angle;
float debug_kf_yaw_angcon = 250;//yaw预测量限幅
float debug_kf_pitch_angcon = 80;//pitch预测量限幅
float debug_yaw_speed, debug_pitch_speed;

float timeaaa = 80;
float yaw_angle_raw;//卡尔曼滤波角度测量值
float pitch_angle_raw;
float yaw_angle_ref;//记录目标角度
float pitch_angle_ref;//记录目标角度
void GIMBAL_AUTO_Mode_Ctrl(void)
{
	GIMBAL_KalmanParameter_Init();
	
	Mobility_Prediction_Yaw = FALSE; //默认标记预测没开启
	Mobi_Pre_Yaw_Fire       = FALSE; //默认标记预测没到位，禁止开火
	
	if(VisionRecvData.identify_target == TRUE)//识别到了目标
	{
		Auto_KF_Delay++;//滤波延时开启
		
		/*预测开启条件*/
		if( fabs(Auto_Error_Yaw[NOW]) < debug_auto_yaw_err 
				&& Auto_KF_Delay > debug_kf_delay 
					&& fabs(yaw_kf_result[1]) > debug_kf_yaw_speed 
						&& fabs(yaw_kf_result[1]) < debug_kf_yaw_speed_h && VisionRecvData.In_Your_Face == 0)
			{
//				debug_kf_yaw_angcon = 71.282f * abs(debug_yaw_speed) + 3;
//				debug_kf_yaw_angcon = constrain( debug_kf_yaw_angcon, -330, 330 );
				
				if( yaw_kf_result[1] < 0 )//速度向右，为负，则预测量为减（更右）
				{
					debug_kf_yaw_angle = debug_yaw_sk * (yaw_kf_result[1] - debug_kf_yaw_speed);
					debug_kf_yaw_angle = constrain(debug_kf_yaw_angle, -debug_kf_yaw_angcon, debug_kf_yaw_angcon);
				}
				else if( yaw_kf_result[1] > 0 )//速度向左，为正，预测量为加（更左）
				{
					debug_kf_yaw_angle = debug_yaw_sk * (yaw_kf_result[1] + debug_kf_yaw_speed);
					debug_kf_yaw_angle = constrain(debug_kf_yaw_angle, -debug_kf_yaw_angcon, debug_kf_yaw_angcon);
				}
				
				aimAngle[Auto][YAW] = yaw_kf_result[0] + debug_kf_yaw_angle;
				
				/*↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓预测到位判断↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓*/
				/*yaw_kf_result[1]左移正，右移负，debug看出来的*/
				/*给自动打弹范围做个小标记*/
				if( (yaw_kf_result[1]>0) //目标向左移且误差值显示说目标在右边，则说明预测到位置，可打弹
					&& (Auto_Error_Yaw[NOW] < 0) )
					{
						mobpre_yaw_right_delay = 0;//向右预测开火延时重置

						mobpre_yaw_left_delay++;
						if(mobpre_yaw_left_delay > 80)//稳定240ms
							{
								Mobi_Pre_Yaw_Fire = TRUE;//预测到位，可开火
							}
						else
							{
								Mobi_Pre_Yaw_Fire = FALSE;//预测没到位，不可开火
							}	
					}
				else if( (yaw_kf_result[1] < 0) //目标向右移且误差值显示说目标在左边，则说明预测到位置，可打弹
						&& ( Auto_Error_Yaw[NOW] > 0) )
					{
						mobpre_yaw_left_delay = 0;//向左预测开火延时重置
				
						mobpre_yaw_right_delay++;
						if(mobpre_yaw_right_delay > 80)//稳定240ms
							{
								Mobi_Pre_Yaw_Fire = TRUE;//预测到位，可开火
							}
						else
							{
								Mobi_Pre_Yaw_Fire = FALSE;//预测没到位，不可开火
							}
					}
				else
					{
						Mobi_Pre_Yaw_Fire = FALSE;//标记预测没到位，禁止开火
				
						mobpre_yaw_left_delay = 0;//向左预测开火延时重置
						mobpre_yaw_right_delay = 0;//向右预测开火延时重置
					}
					/*↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑预测到位判断↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑*/
				Mobility_Prediction_Yaw = TRUE;//标记预测已开启

				mobpre_yaw_stop_delay = 0;//重置静止时的开火延迟
			}
		else			/*预测条件没达到，关闭预测*/
			{
				aimAngle[Auto][YAW] = yaw_angle_ref;  //用斜坡量的时候会有问题，画弧线到达目标值，故直接加
				Mobility_Prediction_Yaw = FALSE;//标记预测没开启
				mobpre_yaw_left_delay  = 0;//重置左预测的开火延迟
				mobpre_yaw_right_delay = 0;//重置右预测的开火延迟	
			
				mobpre_yaw_stop_delay++;
				if(mobpre_yaw_stop_delay > 50)//停止稳定150ms
					{
						Mobi_Pre_Yaw_Fire = TRUE;//此时根据视觉开火标志位做射击判断，记得一定要置TRUE
					}
				else
					{
						Mobi_Pre_Yaw_Fire = FALSE;//标记没回复到位，禁止开火
					}
			}
			
		if(( fabs(Auto_Error_Pitch[NOW]) < debug_auto_pitch_err && Auto_KF_Delay > debug_kf_delay //pitch轴开启预测条件
					&& fabs(pitch_kf_result[1]) > debug_kf_pitch_speed && fabs(pitch_kf_result[1]) < debug_kf_pitch_speed_h ))
			{
				if( pitch_kf_result[1] < 0 )//速度向上，为负，预测量为减（更上）
				{
					debug_kf_pitch_angle = debug_pitch_sk * (pitch_kf_result[1] - debug_kf_pitch_speed);
					debug_kf_pitch_angle = constrain(debug_kf_pitch_angle, -debug_kf_pitch_angcon, debug_kf_pitch_angcon);
				}
				else if( pitch_kf_result[1] > 0 )//速度向下，为正，预测量为加（更下）
				{
					debug_kf_pitch_angle = debug_pitch_sk * (pitch_kf_result[1] + debug_kf_pitch_speed);
					debug_kf_pitch_angle = constrain(debug_kf_pitch_angle, -debug_kf_pitch_angcon, debug_kf_pitch_angcon);
				}
				
				aimAngle[Auto][PITCH] = pitch_kf_result[0] + debug_kf_pitch_angle;
			}
		else
			{
				aimAngle[Auto][PITCH] = pitch_angle_ref;
			}

		PITCH_Auto_PID();
		YAW_Auto_PID();
	}
	else//未识别到目标,可随意控制云台
	{
		if( modeGimbal == MACH )
			{
				aimAngle[M][PITCH] -= MOUSE_Y_MOVE_SPEED/Mouse_PIT[M];
				aimAngle[M][YAW] = Y_M_MID;
							
				PITCH_Mach_PID();
				YAW_Mach_PID();
			}
		else if( modeGimbal == GYRO )
			{
				if( ((Motor_angel[YAW] <= Y_M_MIN + 300) && (MOUSE_X_MOVE_SPEED > 0))
					|| ((Motor_angel[YAW] >= Y_M_MAX - 300) && (MOUSE_X_MOVE_SPEED < 0)) )
					{
						MOUSE_X_MOVE_SPEED = 0;
					}
							
				aimAngle[M][PITCH] -= MOUSE_Y_MOVE_SPEED/Mouse_PIT[G];
				aimAngle[G][YAW] += -MOUSE_X_MOVE_SPEED/Mouse_Yaw[G];
								
				PITCH_Mach_PID();
				YAW_Gyro_PID();
			}
		Auto_KF_Delay = 0;	

		mobpre_yaw_left_delay  = 0;//重置左预测的开火延迟
		mobpre_yaw_right_delay = 0;//重置右预测的开火延迟	
		mobpre_yaw_stop_delay = 0;//停止预测开火延时重置		
	}
}

/**
  * @brief  是否开启自瞄
  * @param  void
  * @retval TRUE开启  FALSE关闭
  * @attention 
  */
bool GIMBAL_IfAutoHit(void)
{
  if(actGimbal == GIMBAL_AUTO)//鼠标右键按下
	{
		return TRUE;
	}
	else
	{
		return FALSE;
	}
}

float speed_threshold = 5.0f;//速度过快
float debug_speed;//左正右负,一般都在1左右,debug看
float Target_Speed_Calc(speed_calc_data_t *S, uint32_t time, float position)
{
	S->delay_cnt++;

	if (time != S->last_time)
	{
		S->speed = (position - S->last_position) / (time - S->last_time) * 2;//计算速度

//		if ((S->speed - S->processed_speed) < -speed_threshold)
//		{
//			S->processed_speed = S->processed_speed - speed_threshold;//速度斜坡变化
//		}
//		else if ((S->speed - S->processed_speed) > speed_threshold)
//		{
//			S->processed_speed = S->processed_speed + speed_threshold;//速度斜坡变化
//		}

		S->processed_speed = S->speed;

		S->last_time = time;
		S->last_position = position;
		S->last_speed = S->speed;
		S->delay_cnt = 0;
	}

	if(S->delay_cnt > 100) // delay 200ms speed = 0
	{
		S->processed_speed = 0;//时间过长则认为速度不变
	}
	debug_speed = S->processed_speed;
	return S->processed_speed;//计算出的速度
}

void GIMBAL_KalmanParameter_Init(void)
{
	if(Shoot_State == BIGBOMB_MODE)
	{
		if( Motor_angel[PITCH] > P_M_MID + 300 ) //抬头一定角度开自瞄，就认为是在打哨兵
		{
			debug_pitch_sk = 25;
		}
		else // 不打哨兵时
		{
			debug_pitch_sk = 32;
		}
		
		debug_yaw_sk = 84;
	}
	
	else if( Shoot_State == XIAOBOMB_MODE )
	{
		debug_pitch_sk = 20;
		debug_yaw_sk = 63;
	}
	
	if( IF_KEY_PRESSED_V || (VisionRecvData.In_Your_Face == 1 && sentry_flag == 0) ) //自瞄按V时，或者贴脸，减小预测
	{
		debug_yaw_sk = 30;
	}
}

void KlamanCalc(void)
{
	if(VisionRecvData.identify_target == TRUE)//识别到了目标
		{
			/*↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓数据更新↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓*/
			if(Vision_If_Update() == TRUE)//视觉数据更新了
				{		
					//获取角度偏差量,欧拉角类型,过分依赖于视觉的精准度
					Vision_Error_Angle_Yaw(&Auto_Error_Yaw[NOW]);
					Vision_Error_Angle_Pitch(&Auto_Error_Pitch[NOW]);
					Vision_Get_Distance(&Auto_Distance);
				
					Auto_Distance = KalmanFilter(&Vision_Distance_Kalman, Auto_Distance);
				
					//更新目标角度//记录当前时刻的目标位置,为卡尔曼做准备
					yaw_angle_ref   = Calc_yaw + Auto_Error_Yaw[NOW] * error_yaw_k;
					pitch_angle_ref = Motor_angel[PITCH] + Auto_Error_Pitch[NOW] * error_pitch_k;
				
					Vision_Clean_Update_Flag();//一定要记得清零,否则会一直执行
					
					if(Auto_Error_Yaw[NOW] != 0 && Auto_Error_Pitch[NOW] !=0)
						{
							Vision_Time[NOW] = micros();//获取新数据到来时的时间
						}
				}
			/*↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑数据更新↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑*/	
				
			/*↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓二阶卡尔曼计算↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓*/
			if(Vision_Time[NOW] != Vision_Time[LAST])//更新新数据到来的时间
				{
					vision_time_js = Vision_Time[NOW] - Vision_Time[LAST];//计算视觉延迟
					yaw_angle_raw  = yaw_angle_ref;//更新二阶卡尔曼滤波测量值
					pitch_angle_raw = pitch_angle_ref;
					Vision_Time[LAST] = Vision_Time[NOW];
				}
							
			//目标速度解算
			Vision_Angle_Speed_Yaw = Target_Speed_Calc(&Vision_Yaw_speed_Struct, Vision_Time[NOW], yaw_angle_raw);
			Vision_Angle_Speed_Pitch = Target_Speed_Calc(&Vision_Pitch_speed_Struct, Vision_Time[NOW], pitch_angle_raw);
			
			if( GIMBAL_IfAutoHit() == FALSE )	//对角度和速度进行二阶卡尔曼滤波融合,0位置,1速度
				{
					yaw_kf_result = kalman_filter_calc(&yaw_kalman_filter, yaw_angle_raw, 0);
					pitch_kf_result = kalman_filter_calc(&pitch_kalman_filter, pitch_angle_raw, 0);
				}
			else
				{
					yaw_kf_result = kalman_filter_calc(&yaw_kalman_filter, yaw_angle_raw, Vision_Angle_Speed_Yaw);
					pitch_kf_result = kalman_filter_calc(&pitch_kalman_filter, pitch_angle_raw, Vision_Angle_Speed_Pitch);
				}
			/*↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑二阶卡尔曼计算↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑*/
			debug_yaw_speed = yaw_kf_result[1];
			debug_pitch_speed = pitch_kf_result[1];
		}
}


/**
  * @brief 	返回云台的动作模式
  * @param  void
  * @retval float
  */
float GIMBAL_ActionMode(void)
{
	return actGimbal;
}


/**
  * @brief 	返回自瞄识别目标
  * @param  void
  * @retval int16_t sentry_flag
	* @attention 抬头角大于水平200机械角时，认为是在自瞄哨兵，发送给视觉
  */
int16_t Get_Attack_attention_Mode(void)
{
	if( Motor_angel[PITCH] > P_M_MID + 200 )  //哨兵抬头补偿
		{
			sentry_flag = 1;
		}
		else
		{
			sentry_flag = 0;
		}
		
	return sentry_flag;
}


/**
  * @brief  桥头吊射模式
  * @param  void
  * @retval void
  * @attention 像素点，只有yaw锁定，pitch给固定机械角加调节
  *            因为抬头后视觉识别不到，所以先锁定左右，再抬头
  */
void GIMBAL_BASE_Mode_Ctrl(void)
{
	static float yaw_base_angle_raw = 0;
	float base_yaw_gyro = 0;
	Vision_Error_Yaw( &Base_Error_Yaw );
	
	if( Vision_If_Update() == TRUE )
		{
			base_yaw_gyro = Base_Error_Yaw + Base_Yaw_Comp_Gimbal;
			
			yaw_base_angle_raw = Calc_yaw + base_yaw_gyro;
			
			Vision_Clean_Update_Flag();  //清零
			Vision_Time[NOW] = micros(); //获取新数据到来的时间
		}
/*   抬头后看得到灯条的做法		
//	if( VisionRecvData.identify_target == 1 )
//		{
//			aimAngle[G][YAW] = yaw_base_angle_raw;
//		}
//	else
//		{
//			aimAngle[G][YAW] += -MOUSE_X_MOVE_SPEED / Mouse_Yaw[G];
//		}

//	Slow_speed_PITCH += MOUSE_Y_MOVE_SPEED/Mouse_PIT[G]/5;			
//	aimAngle[M][PITCH] = 4136 - Slow_speed_PITCH;
//		
//										
//	PITCH_Mach_PID();
//	YAW_Gyro_PID();
*/
		if( Base_Yaw_ready_flag == 0 )
		{
			modeGimbal = GYRO;
			
			if( VisionRecvData.identify_target == 1 )
				{
					aimAngle[G][YAW] = yaw_base_angle_raw;
					
					if( abs(AngleError[G][YAW]) < 20 )
						{
							yaw_luck_time++;
							
							if( yaw_luck_time > Yaw_Luck_Time )
								{
									Base_Yaw_ready_flag = 1;
								}		
						}
				}
			else
				{
					aimAngle[M][PITCH] -= MOUSE_Y_MOVE_SPEED/Mouse_PIT[G];
					aimAngle[G][YAW] += -MOUSE_X_MOVE_SPEED / Mouse_Yaw[G];
				}
				
			YAW_Gyro_PID();
		}
		else
		{
			modeGimbal = MACH;
			
			aimAngle[M][PITCH] = Y_M_MID;
			
			Slow_G_speed_PITCH += MOUSE_Y_MOVE_SPEED/Mouse_PIT[G]/5;			
      aimAngle[M][PITCH] = 4265 - Slow_G_speed_PITCH;
			
			YAW_Mach_PID();
		}
	
		PITCH_Mach_PID();		
}



/**
  * @brief 	临界值结构体初始化
  * @param  critical:临界值结构体指针
  *    			get:当前读取到的角度（陀螺仪或机械角度）
  * @retval void
  */
void Critical_Handle_Init(Critical_t *critical, float get)
{
	critical->AngleSum = 0;
	critical->CurAngle = get;
	critical->LastAngle = get;
}


/**
  * @brief  返回经过临界处理后的实际角度，本质是利用当前角度减去上一次读取的电机或陀螺仪角度
  * 	 			得出新的反馈值，送去PID处理
  * @param  critical:临界值结构体指针
  *	     		get:当前读取到的角度（陀螺仪的话是放大后的角度）
  * @retval float，陀螺仪参考反馈值
  */
float Critical_Handle(Critical_t *critical, float get)
{
	if( modeGimbal == GYRO ) //陀螺仪模式
	{
		critical->CurAngle = get - critical->LastAngle;
		critical->LastAngle = get;
		
		if(critical->CurAngle < -3600)		//此处是陀螺仪180放大20倍的临界值
				critical->CurAngle += 7200;
		if(critical->CurAngle > 3600)
				critical->CurAngle -= 7200;
		critical->AngleSum += critical->CurAngle;
	}
	
	return critical->AngleSum;
}

/*--------------------------------------------------------------------------------------------*/
/*----------------------------------------云台串级PID-----------------------------------------*/
void YAW_Mach_PID(void)   //yaw轴机械模式PID
{
	aimAngle[M][YAW]   = constrain( aimAngle[M][YAW], Y_M_MIN, Y_M_MAX );
	 
	AngleError[M][YAW] = aimAngle[M][YAW] - Motor_angel[YAW];
	
	AngleError[M][YAW] = KalmanFilter(&Gimbal_Yaw_Mach_Error_Kalman, AngleError[M][YAW]);
	 
  aimRate[M][YAW]    = constrain( AngleError[M][YAW] * outP[M][YAW], -30000, 30000 );
	
	RateError[M][YAW]  = constrain( aimRate[M][YAW] + gyroz * 0.5, -30000, 30000 );
	
  Gim_Pterm[M][YAW]  = inP[M][YAW] *  RateError[M][YAW];
	 
	Gimbal_ErrorSum[M][YAW] += RateError[M][YAW];
	Gim_Iterm[M][YAW] = Gimbal_ErrorSum[M][YAW] * inI[M][YAW] * 0.001f;
	Gim_Iterm[M][YAW] = constrain( Gim_Iterm[M][YAW], -18000, 18000 );
	 
	if( Gim_Pterm[M][YAW] * Gim_Iterm[M][YAW] < 0 )
		{
			Gimbal_ErrorSum_Max[M][YAW] = 20000 / inI[M][YAW] / 0.002f;
			Gimbal_ErrorSum[M][YAW] = constrain( Gimbal_ErrorSum[M][YAW], -Gimbal_ErrorSum_Max[M][YAW], Gimbal_ErrorSum_Max[M][YAW] );
		}
	
	Gimbal_PIDterm[YAW] = -constrain( Gim_Pterm[M][YAW] + Gim_Iterm[M][YAW], -30000, 30000 );
		
	if( Gimbal_PIDterm[YAW] * Gim_Iterm[M][YAW] > 0 )
		{
			Gimbal_ErrorSum[M][YAW] = 0;
		}
		
	aimAngle[G][YAW] = 20 * yaw; //将欧拉角赋值给陀螺仪模式
	Critical_Gyro_yaw.CurAngle = Critical_Gyro_yaw.AngleSum = Critical_Gyro_yaw.LastAngle = yaw * 20;
}


void YAW_Gyro_PID(void)  //yaw轴陀螺仪模式PID
{
	if( GIMBAL_ActionMode() == 3 && VisionRecvData.identify_target == 1 )
		{
			outP[G][YAW] = 0.65;
			inP[G][YAW]  = 200;
			inI[G][YAW]  = 200;
		}
	else
		{
			outP[G][YAW] = 3.5;
			inP[G][YAW]  = 170;
			inI[G][YAW]  = 150;
		}
	
	AngleError[G][YAW] = 	aimAngle[G][YAW] - Calc_yaw;
	
	AngleError[G][YAW] = KalmanFilter(&Gimbal_Yaw_Gyro_Error_Kalman, AngleError[G][YAW]); //一阶卡尔曼滤波
	 
	aimRate[G][YAW] = constrain(  AngleError[G][YAW] * outP[G][YAW], -30000, 30000 );
		
	RateError[G][YAW]  = constrain( aimRate[G][YAW] + gyroz * 0.35, -30000, 30000 );
	 
	Gim_Pterm[G][YAW]  = inP[G][YAW] *  RateError[G][YAW];
		
	Gimbal_ErrorSum[G][YAW] += RateError[G][YAW];
  Gim_Iterm[G][YAW] = Gimbal_ErrorSum[G][YAW] * inI[G][YAW] * 0.001f;
	Gim_Iterm[G][YAW] = constrain( Gim_Iterm[G][YAW], -18000, 18000 );
		
	if( Gim_Pterm[G][YAW] * Gim_Iterm[G][YAW] < 0 )
		{
			Gimbal_ErrorSum_Max[G][YAW] = 20000 / inI[G][YAW] / 0.002f;
			Gimbal_ErrorSum[G][YAW] = constrain( Gimbal_ErrorSum[G][YAW], -Gimbal_ErrorSum_Max[G][YAW], Gimbal_ErrorSum_Max[G][YAW] );
		}
	
	if( Gimbal_PIDterm[YAW] * Gim_Iterm[G][YAW] > 0 )
		{
			Gimbal_ErrorSum[G][YAW] = 0;
		}	
  
	Gimbal_PIDterm[YAW] = -constrain( Gim_Pterm[G][YAW] + Gim_Iterm[G][YAW], -30000, 30000 );
		
	aimAngle[M][YAW] = Y_M_MID;  //将机械模式的yaw定在中间
}

void YAW_Auto_PID(void)  //自瞄yaw轴PID
{
	AngleError[Auto][YAW] = aimAngle[Auto][YAW] -  Calc_yaw;
	
	AngleError[Auto][YAW] = KalmanFilter(&Gimbal_Yaw_Auto_Error_Kalman, AngleError[Auto][YAW]); //一阶卡尔曼滤波

	aimRate[Auto][YAW] = constrain(  AngleError[Auto][YAW] * outP[Auto][YAW], -30000, 30000 );
		
	RateError[Auto][YAW]  = constrain( aimRate[Auto][YAW] + gyroz * 0.35, -30000, 30000 );
	 
	Gim_Pterm[Auto][YAW]  = inP[Auto][YAW] *  RateError[Auto][YAW];
		
	Gimbal_ErrorSum[Auto][YAW] += RateError[Auto][YAW];
  Gim_Iterm[Auto][YAW] = Gimbal_ErrorSum[Auto][YAW] * inI[Auto][YAW] * 0.001f;
	Gim_Iterm[Auto][YAW] = constrain( Gim_Iterm[Auto][YAW], -18000, 18000 );
		
	if( Gim_Pterm[Auto][YAW] * Gim_Iterm[Auto][YAW] < 0 )
		{
			Gimbal_ErrorSum_Max[Auto][YAW] = 20000 / inI[Auto][YAW] / 0.002f;
			Gimbal_ErrorSum[Auto][YAW] = constrain( Gimbal_ErrorSum[Auto][YAW], -Gimbal_ErrorSum_Max[Auto][YAW], Gimbal_ErrorSum_Max[Auto][YAW] );
		}
	
	if( Gimbal_PIDterm[YAW] * Gim_Iterm[Auto][YAW] > 0 )
		{
			Gimbal_ErrorSum[Auto][YAW] = 0;
		}	
  
	Gimbal_PIDterm[YAW] = -constrain( Gim_Pterm[Auto][YAW] + Gim_Iterm[Auto][YAW], -30000, 30000 );
		
	aimAngle[M][YAW] = Y_M_MID;  //将机械模式的yaw定在中间
	aimAngle[G][YAW] = aimAngle[Auto][YAW]; 
}

int32_t debug_pitch_Pterm, debug_pitch_Iterm, debug_pitch_PIDterm;
void PITCH_Mach_PID(void)   //PITCH轴机械模式PID
{	
	aimAngle[M][PITCH] = constrain( aimAngle[M][PITCH], P_M_MIN, P_M_MAX );
	 
	AngleError[M][PITCH] = aimAngle[M][PITCH] - Motor_angel[PITCH];
	 
	AngleError[M][PITCH] = KalmanFilter(&Gimbal_Pitch_Mach_Error_Kalman, AngleError[M][PITCH]); //一阶卡尔曼滤波
	
	aimRate[M][PITCH] = constrain( AngleError[M][PITCH] * outP[M][PITCH], -30000, 30000 );
	 
	RateError[M][PITCH] = constrain( aimRate[M][PITCH] - gyrox * 0.10, -30000, 30000 );
	 
	Gim_Pterm[M][PITCH] = RateError[M][PITCH] * inP[M][PITCH];
	 
	if( Gim_Pterm[M][PITCH] * Gim_Iterm[M][PITCH] > 0 )
		{
			Gimbal_ErrorSum_Max[M][PITCH] = 20000 / inI[M][PITCH] / 0.002f;
			Gimbal_ErrorSum[M][PITCH] = constrain( Gimbal_ErrorSum[M][PITCH], -Gimbal_ErrorSum_Max[M][PITCH], Gimbal_ErrorSum_Max[M][PITCH] );
		}
		
	Gimbal_ErrorSum[M][PITCH] += RateError[M][PITCH];
	Gim_Iterm[M][PITCH] = Gimbal_ErrorSum[M][PITCH] * inI[M][PITCH] * 0.001f;
	Gim_Iterm[M][PITCH] = constrain( Gim_Iterm[M][PITCH], -18000, 18000 );
	 	
	if( Gimbal_PIDterm[PITCH] * Gim_Iterm[M][PITCH] > 0 )
		{
			Gimbal_ErrorSum[M][PITCH] = 0;
		}
	
	Gimbal_PIDterm[PITCH] = -constrain( Gim_Pterm[M][PITCH] + Gim_Iterm[M][PITCH], -30000, 30000 );
		
	debug_pitch_Pterm = Gim_Pterm[M][PITCH];
	debug_pitch_Iterm = Gim_Iterm[M][PITCH];
	debug_pitch_PIDterm = Gimbal_PIDterm[PITCH];
}	 

void PITCH_Auto_PID(void) //自瞄Pitch轴PID
{
	aimAngle[Auto][PITCH] = constrain( aimAngle[Auto][PITCH], P_M_MIN, P_M_MAX );
	 
	AngleError[Auto][PITCH] = aimAngle[Auto][PITCH] - Motor_angel[PITCH];
	
	AngleError[Auto][PITCH] = KalmanFilter(&Gimbal_Pitch_Auto_Error_Kalman, AngleError[Auto][PITCH]); //一阶卡尔曼滤波
	
	aimRate[Auto][PITCH] = constrain( AngleError[Auto][PITCH] * outP[Auto][PITCH], -30000, 30000 );
	 
	RateError[Auto][PITCH] = constrain( aimRate[Auto][PITCH] - gyrox * 0.10, -30000, 30000 );
	 
	Gim_Pterm[Auto][PITCH] = RateError[Auto][PITCH] * inP[Auto][PITCH];
	 
	if( Gim_Pterm[Auto][PITCH] * Gim_Iterm[Auto][PITCH] < 0 )
		{
			Gimbal_ErrorSum_Max[Auto][PITCH] = 20000 / inI[Auto][PITCH] / 0.002f;
			Gimbal_ErrorSum[Auto][PITCH] = constrain( Gimbal_ErrorSum[Auto][PITCH], -Gimbal_ErrorSum_Max[Auto][PITCH], Gimbal_ErrorSum_Max[Auto][PITCH] );
		}
		
	Gimbal_ErrorSum[Auto][PITCH] += RateError[Auto][PITCH];
	Gim_Iterm[Auto][PITCH] = Gimbal_ErrorSum[Auto][PITCH] * inI[Auto][PITCH] * 0.001f;
	Gim_Iterm[Auto][PITCH] = constrain( Gim_Iterm[Auto][PITCH], -18000, 18000 );
	 	
	if( Gimbal_PIDterm[PITCH] * Gim_Iterm[Auto][PITCH] > 0 )
		{
			Gimbal_ErrorSum[Auto][PITCH] = 0;
		}

	Gimbal_PIDterm[PITCH] = -constrain( Gim_Pterm[Auto][PITCH] + Gim_Iterm[Auto][PITCH], -30000, 30000 );
		
	aimAngle[M][PITCH] = aimAngle[Auto][PITCH];
}

/*-------------------------------------------------------------------*/
/*---------------------------斜坡函数--------------------------------*/
float slope_Control(float final,float aimAngle,int tilt) //斜坡函数
{
	float buffer;
	buffer = final - aimAngle;
	if(buffer > 0)
		{
			if(buffer > tilt)
				{
					aimAngle += tilt;
				}
			else
				{
					aimAngle += buffer; 
				}
		}
	else
		{
			if(buffer < -tilt)
				{
					aimAngle += -tilt;
				}
			else
				{
					aimAngle += buffer;
				}
		}
	return aimAngle;
}


/* 云台参数初始化 */
void GIMBAL_InitArgument(void)
{
	//卡尔曼滤波器初始化
	/*PID角度误差卡尔曼,一阶*/
	KalmanCreate(&Gimbal_Pitch_Mach_Error_Kalman, 1, 40);
	KalmanCreate(&Gimbal_Pitch_Auto_Error_Kalman, 1, 40);
	
	KalmanCreate(&Gimbal_Yaw_Mach_Error_Kalman, 1, 40);
	KalmanCreate(&Gimbal_Yaw_Gyro_Error_Kalman, 1, 50);
	KalmanCreate(&Gimbal_Yaw_Auto_Error_Kalman, 1, 60);
	
	KalmanCreate(&Vision_Distance_Kalman, 1, 2000);
	
	/*自瞄卡尔曼滤波,二阶*/
	mat_init(&yaw_kalman_filter.Q,2,2, yaw_kalman_filter_para.Q_data);
	mat_init(&yaw_kalman_filter.R,2,2, yaw_kalman_filter_para.R_data);
	kalman_filter_init(&yaw_kalman_filter, &yaw_kalman_filter_para);

	mat_init(&pitch_kalman_filter.Q,2,2, pitch_kalman_filter_para.Q_data);
	mat_init(&pitch_kalman_filter.R,2,2, pitch_kalman_filter_para.R_data);
	kalman_filter_init(&pitch_kalman_filter, &pitch_kalman_filter_para);
	
	Critical_Handle_Init( &Critical_Gyro_yaw, 20 * yaw );
}


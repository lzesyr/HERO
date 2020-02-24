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
	Gimbal_NORMAL  = 0, //����ģʽ
	Gimbal_Turn    = 1, //ת��ģʽ
	GIMBAL_AUTO    = 2, //����ģʽ
	Gimbal_Diao    = 3, //��ͷ����ģʽ
	Gimbal_HIGHT   = 4, //̧ͷģʽ
	Gimbal_Duanya  = 5, //���µ���
}eactGimbal;
eactGimbal actGimbal;

/***************����******************/
//�ǶȲ���
float Offset_Angle_Yaw;//��Ŀ��������׼ģʽʵʱ�ı�
float Offset_Angle_Pitch;

//���
float Auto_Error_Yaw[2];//    now/last
float Auto_Error_Pitch[2];
float Auto_Distance;//���뵥Ŀ

//����ͻȻ����,�������˲�������ʱ
uint16_t Auto_KF_Delay = 0;

/*************�������˲�**************/
/*һ�׿�����*/
//��̨�Ƕ�������
extKalman_t Gimbal_Pitch_Mach_Error_Kalman;//����һ��kalmanָ��
extKalman_t Gimbal_Pitch_Gyro_Error_Kalman;//����һ��kalmanָ��
extKalman_t Gimbal_Pitch_Auto_Error_Kalman;//����һ��kalmanָ��
extKalman_t Gimbal_Yaw_Mach_Error_Kalman;//����һ��kalmanָ��
extKalman_t Gimbal_Yaw_Gyro_Error_Kalman;//����һ��kalmanָ��
extKalman_t Gimbal_Yaw_Auto_Error_Kalman;//����һ��kalmanָ��

extKalman_t Vision_Distance_Kalman;


typedef struct  //�Ӿ�Ŀ���ٶȲ���
{
  int delay_cnt;//����������֡Ŀ�겻�����ʱ��,�����ж��ٶ��Ƿ�Ϊ0
  int freq;
  int last_time;//�ϴ��ܵ�Ŀ��Ƕȵ�ʱ��
  float last_position;//�ϸ�Ŀ��Ƕ�
  float speed;//�ٶ�
  float last_speed;//�ϴ��ٶ�
  float processed_speed;//�ٶȼ�����
}speed_calc_data_t;

speed_calc_data_t Vision_Yaw_speed_Struct;
speed_calc_data_t Vision_Pitch_speed_Struct;

kalman_filter_init_t yaw_kalman_filter_para = {
  .P_data = {2, 0, 0, 2},
  .A_data = {1, 0.002, 0, 1},//����ʱ����
  .H_data = {1, 0, 0, 1},
  .Q_data = {1, 0, 0, 1},
  .R_data = {200, 0, 0, 700}
};//��ʼ��yaw�Ĳ���kalman����

kalman_filter_init_t pitch_kalman_filter_para = {
  .P_data = {2, 0, 0, 2},
  .A_data = {1, 0.002, 0, 1},//����ʱ����
  .H_data = {1, 0, 0, 1},
  .Q_data = {1, 0, 0, 1},
  .R_data = {400, 0, 0, 1000}
};//��ʼ��pitch�Ĳ���kalman����

kalman_filter_t  yaw_kalman_filter;
kalman_filter_t  pitch_kalman_filter;

//float kKalman_Q;//�������Ŷ�
//float kKalman_R;//Ԥ�����Ŷ�

/*�Զ����õ�һЩ��־λ*/
bool Mobility_Prediction_Yaw = FALSE;//Ԥ���Ƿ�����־λ
bool Mobi_Pre_Yaw_Fire = FALSE;//Ĭ��Ԥ��û��λ����ֹ��ǹ
uint16_t mobpre_yaw_left_delay = 0;//����Ԥ����ʱ�жϿɿ�������
uint16_t mobpre_yaw_right_delay = 0;//����Ԥ����ʱ�жϿɿ�������
uint16_t mobpre_yaw_stop_delay = 0;//Ԥ��ر���ʱ�жϿɿ�������

/* ��̨���Ʋ��� */ 
float outP[3][2]={1.5,-4.5,0,3.5,3,4},outI[3][2],inP[3][2]={-290,100,0,180,-250,170},inI[3][2]={150,150,0,150,200,280};         //PID����
float Gim_Pterm[3][2],Gim_Iterm[3][2],Gimbal_ErrorSum[3][2],Gimbal_ErrorSum_Max[3][2];        //PID�м�ֵ
float AngleError[3][2],RateError[3][2],Target[2],aimAngle[3][2],aimRate[3][2];
float Mouse_Yaw[2]={3,2.5}, Mouse_PIT[2]={3,2}, Slow_G_speed_PITCH, Slow_V_speed_PITCH;  //����ٶȼ�����
float Calc_yaw;
Critical_t  Critical_Gyro_yaw;

/* ת���ͷ */
float Q_TurnAngle, E_TurnAngle, C_TurnAngle;     //ת��Ƕ�
bool  TurnLeft, TurnRight, TurnBack;             //ת���־λ 

bool  sentry_flag;  //�Ƿ�����ڱ���־λ

 /* �������  */
float Base_Yaw_Comp_Gimbal = -60; //��������У׼
float Base_Error_Yaw;  //��ȡ�������
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
	
	Currenttime = micros();//��λʱ��
	
	if(AngleRecord == FALSE)
		{
			AngleRecord = TRUE;

			aimAngle[M][PITCH] = Motor_angel[PITCH]; //��¼��ǰ�����е��
			aimAngle[M][YAW] = Motor_angel[YAW];
		}
	
	modeGimbal = MACH;//Ĭ������ģʽΪ��еģʽ
	
	XIANWEI_Angle( OPEN_ANGLE );	//�򿪶������ֹǹ���е���ת���
		
	aimAngle[M][PITCH] = slope_Control(P_M_MID, aimAngle[M][PITCH], 4);
	aimAngle[M][YAW] = slope_Control(Y_M_MID, aimAngle[M][YAW], 4);
		
	PITCH_Mach_PID();
	YAW_Mach_PID();
}
	
void RC_Gimbal_control()
{
	if(RC_Ctl.rc.s2 == RC_SW_MID)  //�ҿ��ؾ���
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
			if( IF_KEY_PRESSED_CTRL || Get_Chassis_Mode() == 4 || Get_Chassis_Mode() == 5 )//ȡ��ģʽ�Ͷ��µ���ģʽ�����еģʽ
				modeGimbal = MACH;
			else
				modeGimbal = GYRO;
			
			//ģʽ�л�	
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
					
					//����Ŀ��ƫ������,�����л�ʱ��̨����
					VisionRecvData.identify_target = FALSE;
					Auto_KF_Delay = 0;//������´��ӳ�Ԥ����
					Mobility_Prediction_Yaw = FALSE;//���Ԥ��û����
					Mobi_Pre_Yaw_Fire = FALSE;//Ĭ�ϱ��Ԥ��û��λ����ֹ����
				
					mobpre_yaw_left_delay  = 0;//������Ԥ��Ŀ����ӳ�
					mobpre_yaw_right_delay = 0;//������Ԥ��Ŀ����ӳ�	
					mobpre_yaw_stop_delay = 0;//ֹͣԤ�⿪����ʱ����
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
  * @brief  ������ƺ���
  * @param  void
  * @retval void
  * @attention �м�����(0,0),�����Ҹ�,�ϸ�����
  *            yawΪ������ģʽ,pitchΪ��еģʽ(��ʵpitchȫ�̶����û�еģʽ)
  *            ֻ�е����������˲��ڵ�ǰʵʱ�Ƕ����ۼ�����Ŀ��Ƕ�
  *            ������һ��,Ŀ��ǶȾ�ʵʱ����
  */
float debug_yaw_sk = 76;//85;//�ƶ�Ԥ��ϵ��,Խ��Ԥ��Խ��
float debug_pitch_sk = 30;

uint32_t Vision_Time[2];// NOW/LAST
float vision_time_js;

float Vision_Angle_Speed_Yaw;//�������˲�yaw�ٶȲ���ֵ
float Vision_Angle_Speed_Pitch;//�������˲�pitch�ٶȲ���ֵ
float *yaw_kf_result;//���׿�����yaw�˲����,0�Ƕ� 1�ٶ�
float *pitch_kf_result;//���׿�����pitch�˲����,0�Ƕ� 1�ٶ�

float error_yaw_k   = 11;//���Ŵ�
float error_pitch_k = 6; //���Ŵ�

float debug_auto_yaw_err = 14;   //yaw�Ƕȹ���ر�Ԥ��
float debug_auto_pitch_err = 4; //pitch�Ƕȹ���ر�Ԥ��
float debug_kf_delay = 100;//80;  //Ԥ����ʱ����

float debug_kf_yaw_speed = 0.2;  //�ٶȹ��͹ر�Ԥ��
float debug_kf_yaw_speed_h = 5;//6.5;//�ٶȹ��߹ر�Ԥ��
float debug_kf_yaw_angle;
float debug_kf_pitch_speed = 0.08;
float debug_kf_pitch_speed_h = 3;
float debug_kf_pitch_angle;
float debug_kf_yaw_angcon = 250;//yawԤ�����޷�
float debug_kf_pitch_angcon = 80;//pitchԤ�����޷�
float debug_yaw_speed, debug_pitch_speed;

float timeaaa = 80;
float yaw_angle_raw;//�������˲��ǶȲ���ֵ
float pitch_angle_raw;
float yaw_angle_ref;//��¼Ŀ��Ƕ�
float pitch_angle_ref;//��¼Ŀ��Ƕ�
void GIMBAL_AUTO_Mode_Ctrl(void)
{
	GIMBAL_KalmanParameter_Init();
	
	Mobility_Prediction_Yaw = FALSE; //Ĭ�ϱ��Ԥ��û����
	Mobi_Pre_Yaw_Fire       = FALSE; //Ĭ�ϱ��Ԥ��û��λ����ֹ����
	
	if(VisionRecvData.identify_target == TRUE)//ʶ����Ŀ��
	{
		Auto_KF_Delay++;//�˲���ʱ����
		
		/*Ԥ�⿪������*/
		if( fabs(Auto_Error_Yaw[NOW]) < debug_auto_yaw_err 
				&& Auto_KF_Delay > debug_kf_delay 
					&& fabs(yaw_kf_result[1]) > debug_kf_yaw_speed 
						&& fabs(yaw_kf_result[1]) < debug_kf_yaw_speed_h && VisionRecvData.In_Your_Face == 0)
			{
//				debug_kf_yaw_angcon = 71.282f * abs(debug_yaw_speed) + 3;
//				debug_kf_yaw_angcon = constrain( debug_kf_yaw_angcon, -330, 330 );
				
				if( yaw_kf_result[1] < 0 )//�ٶ����ң�Ϊ������Ԥ����Ϊ�������ң�
				{
					debug_kf_yaw_angle = debug_yaw_sk * (yaw_kf_result[1] - debug_kf_yaw_speed);
					debug_kf_yaw_angle = constrain(debug_kf_yaw_angle, -debug_kf_yaw_angcon, debug_kf_yaw_angcon);
				}
				else if( yaw_kf_result[1] > 0 )//�ٶ�����Ϊ����Ԥ����Ϊ�ӣ�����
				{
					debug_kf_yaw_angle = debug_yaw_sk * (yaw_kf_result[1] + debug_kf_yaw_speed);
					debug_kf_yaw_angle = constrain(debug_kf_yaw_angle, -debug_kf_yaw_angcon, debug_kf_yaw_angcon);
				}
				
				aimAngle[Auto][YAW] = yaw_kf_result[0] + debug_kf_yaw_angle;
				
				/*��������������������������������������������Ԥ�⵽λ�жϡ�������������������������������������������������������������*/
				/*yaw_kf_result[1]�����������Ƹ���debug��������*/
				/*���Զ��򵯷�Χ����С���*/
				if( (yaw_kf_result[1]>0) //Ŀ�������������ֵ��ʾ˵Ŀ�����ұߣ���˵��Ԥ�⵽λ�ã��ɴ�
					&& (Auto_Error_Yaw[NOW] < 0) )
					{
						mobpre_yaw_right_delay = 0;//����Ԥ�⿪����ʱ����

						mobpre_yaw_left_delay++;
						if(mobpre_yaw_left_delay > 80)//�ȶ�240ms
							{
								Mobi_Pre_Yaw_Fire = TRUE;//Ԥ�⵽λ���ɿ���
							}
						else
							{
								Mobi_Pre_Yaw_Fire = FALSE;//Ԥ��û��λ�����ɿ���
							}	
					}
				else if( (yaw_kf_result[1] < 0) //Ŀ�������������ֵ��ʾ˵Ŀ������ߣ���˵��Ԥ�⵽λ�ã��ɴ�
						&& ( Auto_Error_Yaw[NOW] > 0) )
					{
						mobpre_yaw_left_delay = 0;//����Ԥ�⿪����ʱ����
				
						mobpre_yaw_right_delay++;
						if(mobpre_yaw_right_delay > 80)//�ȶ�240ms
							{
								Mobi_Pre_Yaw_Fire = TRUE;//Ԥ�⵽λ���ɿ���
							}
						else
							{
								Mobi_Pre_Yaw_Fire = FALSE;//Ԥ��û��λ�����ɿ���
							}
					}
				else
					{
						Mobi_Pre_Yaw_Fire = FALSE;//���Ԥ��û��λ����ֹ����
				
						mobpre_yaw_left_delay = 0;//����Ԥ�⿪����ʱ����
						mobpre_yaw_right_delay = 0;//����Ԥ�⿪����ʱ����
					}
					/*������������������������������������������������Ԥ�⵽λ�жϡ���������������������������������������������������������*/
				Mobility_Prediction_Yaw = TRUE;//���Ԥ���ѿ���

				mobpre_yaw_stop_delay = 0;//���þ�ֹʱ�Ŀ����ӳ�
			}
		else			/*Ԥ������û�ﵽ���ر�Ԥ��*/
			{
				aimAngle[Auto][YAW] = yaw_angle_ref;  //��б������ʱ��������⣬�����ߵ���Ŀ��ֵ����ֱ�Ӽ�
				Mobility_Prediction_Yaw = FALSE;//���Ԥ��û����
				mobpre_yaw_left_delay  = 0;//������Ԥ��Ŀ����ӳ�
				mobpre_yaw_right_delay = 0;//������Ԥ��Ŀ����ӳ�	
			
				mobpre_yaw_stop_delay++;
				if(mobpre_yaw_stop_delay > 50)//ֹͣ�ȶ�150ms
					{
						Mobi_Pre_Yaw_Fire = TRUE;//��ʱ�����Ӿ������־λ������жϣ��ǵ�һ��Ҫ��TRUE
					}
				else
					{
						Mobi_Pre_Yaw_Fire = FALSE;//���û�ظ���λ����ֹ����
					}
			}
			
		if(( fabs(Auto_Error_Pitch[NOW]) < debug_auto_pitch_err && Auto_KF_Delay > debug_kf_delay //pitch�Ὺ��Ԥ������
					&& fabs(pitch_kf_result[1]) > debug_kf_pitch_speed && fabs(pitch_kf_result[1]) < debug_kf_pitch_speed_h ))
			{
				if( pitch_kf_result[1] < 0 )//�ٶ����ϣ�Ϊ����Ԥ����Ϊ�������ϣ�
				{
					debug_kf_pitch_angle = debug_pitch_sk * (pitch_kf_result[1] - debug_kf_pitch_speed);
					debug_kf_pitch_angle = constrain(debug_kf_pitch_angle, -debug_kf_pitch_angcon, debug_kf_pitch_angcon);
				}
				else if( pitch_kf_result[1] > 0 )//�ٶ����£�Ϊ����Ԥ����Ϊ�ӣ����£�
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
	else//δʶ��Ŀ��,�����������̨
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

		mobpre_yaw_left_delay  = 0;//������Ԥ��Ŀ����ӳ�
		mobpre_yaw_right_delay = 0;//������Ԥ��Ŀ����ӳ�	
		mobpre_yaw_stop_delay = 0;//ֹͣԤ�⿪����ʱ����		
	}
}

/**
  * @brief  �Ƿ�������
  * @param  void
  * @retval TRUE����  FALSE�ر�
  * @attention 
  */
bool GIMBAL_IfAutoHit(void)
{
  if(actGimbal == GIMBAL_AUTO)//����Ҽ�����
	{
		return TRUE;
	}
	else
	{
		return FALSE;
	}
}

float speed_threshold = 5.0f;//�ٶȹ���
float debug_speed;//�����Ҹ�,һ�㶼��1����,debug��
float Target_Speed_Calc(speed_calc_data_t *S, uint32_t time, float position)
{
	S->delay_cnt++;

	if (time != S->last_time)
	{
		S->speed = (position - S->last_position) / (time - S->last_time) * 2;//�����ٶ�

//		if ((S->speed - S->processed_speed) < -speed_threshold)
//		{
//			S->processed_speed = S->processed_speed - speed_threshold;//�ٶ�б�±仯
//		}
//		else if ((S->speed - S->processed_speed) > speed_threshold)
//		{
//			S->processed_speed = S->processed_speed + speed_threshold;//�ٶ�б�±仯
//		}

		S->processed_speed = S->speed;

		S->last_time = time;
		S->last_position = position;
		S->last_speed = S->speed;
		S->delay_cnt = 0;
	}

	if(S->delay_cnt > 100) // delay 200ms speed = 0
	{
		S->processed_speed = 0;//ʱ���������Ϊ�ٶȲ���
	}
	debug_speed = S->processed_speed;
	return S->processed_speed;//��������ٶ�
}

void GIMBAL_KalmanParameter_Init(void)
{
	if(Shoot_State == BIGBOMB_MODE)
	{
		if( Motor_angel[PITCH] > P_M_MID + 300 ) //̧ͷһ���Ƕȿ����飬����Ϊ���ڴ��ڱ�
		{
			debug_pitch_sk = 25;
		}
		else // �����ڱ�ʱ
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
	
	if( IF_KEY_PRESSED_V || (VisionRecvData.In_Your_Face == 1 && sentry_flag == 0) ) //���鰴Vʱ��������������СԤ��
	{
		debug_yaw_sk = 30;
	}
}

void KlamanCalc(void)
{
	if(VisionRecvData.identify_target == TRUE)//ʶ����Ŀ��
		{
			/*�����������������������������������������������ݸ��¡�������������������������������������������������������������*/
			if(Vision_If_Update() == TRUE)//�Ӿ����ݸ�����
				{		
					//��ȡ�Ƕ�ƫ����,ŷ��������,�����������Ӿ��ľ�׼��
					Vision_Error_Angle_Yaw(&Auto_Error_Yaw[NOW]);
					Vision_Error_Angle_Pitch(&Auto_Error_Pitch[NOW]);
					Vision_Get_Distance(&Auto_Distance);
				
					Auto_Distance = KalmanFilter(&Vision_Distance_Kalman, Auto_Distance);
				
					//����Ŀ��Ƕ�//��¼��ǰʱ�̵�Ŀ��λ��,Ϊ��������׼��
					yaw_angle_ref   = Calc_yaw + Auto_Error_Yaw[NOW] * error_yaw_k;
					pitch_angle_ref = Motor_angel[PITCH] + Auto_Error_Pitch[NOW] * error_pitch_k;
				
					Vision_Clean_Update_Flag();//һ��Ҫ�ǵ�����,�����һֱִ��
					
					if(Auto_Error_Yaw[NOW] != 0 && Auto_Error_Pitch[NOW] !=0)
						{
							Vision_Time[NOW] = micros();//��ȡ�����ݵ���ʱ��ʱ��
						}
				}
			/*���������������������������������������������������ݸ��¡���������������������������������������������������������*/	
				
			/*�����������������������������������������������׿����������������������������������������������������������������������*/
			if(Vision_Time[NOW] != Vision_Time[LAST])//���������ݵ�����ʱ��
				{
					vision_time_js = Vision_Time[NOW] - Vision_Time[LAST];//�����Ӿ��ӳ�
					yaw_angle_raw  = yaw_angle_ref;//���¶��׿������˲�����ֵ
					pitch_angle_raw = pitch_angle_ref;
					Vision_Time[LAST] = Vision_Time[NOW];
				}
							
			//Ŀ���ٶȽ���
			Vision_Angle_Speed_Yaw = Target_Speed_Calc(&Vision_Yaw_speed_Struct, Vision_Time[NOW], yaw_angle_raw);
			Vision_Angle_Speed_Pitch = Target_Speed_Calc(&Vision_Pitch_speed_Struct, Vision_Time[NOW], pitch_angle_raw);
			
			if( GIMBAL_IfAutoHit() == FALSE )	//�ԽǶȺ��ٶȽ��ж��׿������˲��ں�,0λ��,1�ٶ�
				{
					yaw_kf_result = kalman_filter_calc(&yaw_kalman_filter, yaw_angle_raw, 0);
					pitch_kf_result = kalman_filter_calc(&pitch_kalman_filter, pitch_angle_raw, 0);
				}
			else
				{
					yaw_kf_result = kalman_filter_calc(&yaw_kalman_filter, yaw_angle_raw, Vision_Angle_Speed_Yaw);
					pitch_kf_result = kalman_filter_calc(&pitch_kalman_filter, pitch_angle_raw, Vision_Angle_Speed_Pitch);
				}
			/*���������������������������������������������������׿������������������������������������������������������������������*/
			debug_yaw_speed = yaw_kf_result[1];
			debug_pitch_speed = pitch_kf_result[1];
		}
}


/**
  * @brief 	������̨�Ķ���ģʽ
  * @param  void
  * @retval float
  */
float GIMBAL_ActionMode(void)
{
	return actGimbal;
}


/**
  * @brief 	��������ʶ��Ŀ��
  * @param  void
  * @retval int16_t sentry_flag
	* @attention ̧ͷ�Ǵ���ˮƽ200��е��ʱ����Ϊ���������ڱ������͸��Ӿ�
  */
int16_t Get_Attack_attention_Mode(void)
{
	if( Motor_angel[PITCH] > P_M_MID + 200 )  //�ڱ�̧ͷ����
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
  * @brief  ��ͷ����ģʽ
  * @param  void
  * @retval void
  * @attention ���ص㣬ֻ��yaw������pitch���̶���е�Ǽӵ���
  *            ��Ϊ̧ͷ���Ӿ�ʶ�𲻵����������������ң���̧ͷ
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
			
			Vision_Clean_Update_Flag();  //����
			Vision_Time[NOW] = micros(); //��ȡ�����ݵ�����ʱ��
		}
/*   ̧ͷ�󿴵õ�����������		
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
  * @brief 	�ٽ�ֵ�ṹ���ʼ��
  * @param  critical:�ٽ�ֵ�ṹ��ָ��
  *    			get:��ǰ��ȡ���ĽǶȣ������ǻ��е�Ƕȣ�
  * @retval void
  */
void Critical_Handle_Init(Critical_t *critical, float get)
{
	critical->AngleSum = 0;
	critical->CurAngle = get;
	critical->LastAngle = get;
}


/**
  * @brief  ���ؾ����ٽ紦����ʵ�ʽǶȣ����������õ�ǰ�Ƕȼ�ȥ��һ�ζ�ȡ�ĵ���������ǽǶ�
  * 	 			�ó��µķ���ֵ����ȥPID����
  * @param  critical:�ٽ�ֵ�ṹ��ָ��
  *	     		get:��ǰ��ȡ���ĽǶȣ������ǵĻ��ǷŴ��ĽǶȣ�
  * @retval float�������ǲο�����ֵ
  */
float Critical_Handle(Critical_t *critical, float get)
{
	if( modeGimbal == GYRO ) //������ģʽ
	{
		critical->CurAngle = get - critical->LastAngle;
		critical->LastAngle = get;
		
		if(critical->CurAngle < -3600)		//�˴���������180�Ŵ�20�����ٽ�ֵ
				critical->CurAngle += 7200;
		if(critical->CurAngle > 3600)
				critical->CurAngle -= 7200;
		critical->AngleSum += critical->CurAngle;
	}
	
	return critical->AngleSum;
}

/*--------------------------------------------------------------------------------------------*/
/*----------------------------------------��̨����PID-----------------------------------------*/
void YAW_Mach_PID(void)   //yaw���еģʽPID
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
		
	aimAngle[G][YAW] = 20 * yaw; //��ŷ���Ǹ�ֵ��������ģʽ
	Critical_Gyro_yaw.CurAngle = Critical_Gyro_yaw.AngleSum = Critical_Gyro_yaw.LastAngle = yaw * 20;
}


void YAW_Gyro_PID(void)  //yaw��������ģʽPID
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
	
	AngleError[G][YAW] = KalmanFilter(&Gimbal_Yaw_Gyro_Error_Kalman, AngleError[G][YAW]); //һ�׿������˲�
	 
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
		
	aimAngle[M][YAW] = Y_M_MID;  //����еģʽ��yaw�����м�
}

void YAW_Auto_PID(void)  //����yaw��PID
{
	AngleError[Auto][YAW] = aimAngle[Auto][YAW] -  Calc_yaw;
	
	AngleError[Auto][YAW] = KalmanFilter(&Gimbal_Yaw_Auto_Error_Kalman, AngleError[Auto][YAW]); //һ�׿������˲�

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
		
	aimAngle[M][YAW] = Y_M_MID;  //����еģʽ��yaw�����м�
	aimAngle[G][YAW] = aimAngle[Auto][YAW]; 
}

int32_t debug_pitch_Pterm, debug_pitch_Iterm, debug_pitch_PIDterm;
void PITCH_Mach_PID(void)   //PITCH���еģʽPID
{	
	aimAngle[M][PITCH] = constrain( aimAngle[M][PITCH], P_M_MIN, P_M_MAX );
	 
	AngleError[M][PITCH] = aimAngle[M][PITCH] - Motor_angel[PITCH];
	 
	AngleError[M][PITCH] = KalmanFilter(&Gimbal_Pitch_Mach_Error_Kalman, AngleError[M][PITCH]); //һ�׿������˲�
	
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

void PITCH_Auto_PID(void) //����Pitch��PID
{
	aimAngle[Auto][PITCH] = constrain( aimAngle[Auto][PITCH], P_M_MIN, P_M_MAX );
	 
	AngleError[Auto][PITCH] = aimAngle[Auto][PITCH] - Motor_angel[PITCH];
	
	AngleError[Auto][PITCH] = KalmanFilter(&Gimbal_Pitch_Auto_Error_Kalman, AngleError[Auto][PITCH]); //һ�׿������˲�
	
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
/*---------------------------б�º���--------------------------------*/
float slope_Control(float final,float aimAngle,int tilt) //б�º���
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


/* ��̨������ʼ�� */
void GIMBAL_InitArgument(void)
{
	//�������˲�����ʼ��
	/*PID�Ƕ�������,һ��*/
	KalmanCreate(&Gimbal_Pitch_Mach_Error_Kalman, 1, 40);
	KalmanCreate(&Gimbal_Pitch_Auto_Error_Kalman, 1, 40);
	
	KalmanCreate(&Gimbal_Yaw_Mach_Error_Kalman, 1, 40);
	KalmanCreate(&Gimbal_Yaw_Gyro_Error_Kalman, 1, 50);
	KalmanCreate(&Gimbal_Yaw_Auto_Error_Kalman, 1, 60);
	
	KalmanCreate(&Vision_Distance_Kalman, 1, 2000);
	
	/*���鿨�����˲�,����*/
	mat_init(&yaw_kalman_filter.Q,2,2, yaw_kalman_filter_para.Q_data);
	mat_init(&yaw_kalman_filter.R,2,2, yaw_kalman_filter_para.R_data);
	kalman_filter_init(&yaw_kalman_filter, &yaw_kalman_filter_para);

	mat_init(&pitch_kalman_filter.Q,2,2, pitch_kalman_filter_para.Q_data);
	mat_init(&pitch_kalman_filter.R,2,2, pitch_kalman_filter_para.R_data);
	kalman_filter_init(&pitch_kalman_filter, &pitch_kalman_filter_para);
	
	Critical_Handle_Init( &Critical_Gyro_yaw, 20 * yaw );
}


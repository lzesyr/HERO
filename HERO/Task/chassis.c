#include "chassis.h"
#include "filter.h"
#include "kalman_filter.h"
#include "vision.h"

/* ���ְ뾶��mm��*/
#define RADIUS     76
/* �����ܳ�(mm)  */
#define PERIMETER  478

/* �����־�(mm)  */
#define WHEELTRACK 423.76
/* ǰ�����(mm)  */
#define WHEELBASE  469.62

/* ��̨��Ե������ĵ�xƫ��(mm) */
#define GIMBAL_X_OFFSET 0
/* ��̨��Ե������ĵ�yƫ��(mm) */
#define GIMBAL_Y_OFFSET 71.8     //����Ϊ������ͷ

/* ��������� */
#define CHASSIS_DECELE_RATIO (1.0f/19.0f)
/* radian coefficient */
#define RADIAN_COEF 57.3f
/*�������ֵת���ɽǶȵı�ֵ*/
#define ENCODER_ANGLE_RATIO    (8192.0f/360.0f)

bool chassis_Follow_flag;
typedef enum
{
	ACTIVE  = 0, //��еģʽ��  ��̨�������
	FOLLOW  = 1, //������ģʽ�����̸�����̨
	UPSLOPE = 2, //����ģʽ
	DODGE   = 3, //Ť��ģʽ	
	GETBOMB = 4, //ȡ��ģʽ
}eChaCtrlMode;
eChaCtrlMode  modeChassis;

extKalman_t RC_FOLLOW_Error_Kalman;
extKalman_t KEY_FOLLOW_Error_Kalman;
extKalman_t Gimbal_SpeedZ_Kalman;

extern int16_t Motor_chassis[4][2],Motor_angel[2];

/* ����PID�м�ֵ */
float Chassis_P=10, Chassis_I = 1, Chassis_D;//PIDϵ��
float Cha_Pterm[4],Cha_Iterm[4],Cha_Dterm[4],Chassis_PIDterm[4],speedmax=8000,Itermmax=2000; 

 /* ����ң�ر��� */
float mul=10, key_ws_num=4, key_ad_num=4, StopSpeedNum=20, speed_num[2], CommutationTime = 20, Commutation;       
bool Forward_flag, Back_off_flag;

/* �����ٶȷ��� */
float speed_dx, speed_dy, speed_x, speed_y, speed_z;

/* �������ݲ��� */
uint32_t remainspeedtime = 0;
uint8_t just_a_flag = 0, yunsuflag = 0;
float motor_speed[4], Speed_target_all, Last_target_all;

 /* ���̹��ʲ��� */
float TotalOut, OutLimit;

/* ����Ť������ */
float debug_P = 15, debug_D = 0.06;//ҡ�ڲ���
uint32_t twist_count;  //Ť���ļ�ʱ
int16_t  position_ref;//���̷�λ(����Ť��)

/* ȡ��ģʽ */
float Get_Bomb_Speed = 25;  //���˵��ٶ�
int16_t YanshiTime = 100, yanshiCount = 0, Allow_Back_flag = 0;

void Chassis_control(void)
{
	if( Get_SystemState() == SYSTEM_STARTING )
		{
			speed_x = 0;
			speed_y = 0;
			speed_z = 0;
		}
	else
		{
			if( Get_remoteMode() == RC )
				{
					Cha_RC_control();
				}
			else
				{
					Cha_KEY_control();
				}
		}
		
	Cha_Motor_speed(speed_x, speed_y, speed_z);
		
	SuperCap_Charge_Control();
		
	Cha_PowerLimit();
		
	CAN1_Chassis_Send();
		
	if( modeChassis == FOLLOW )
		chassis_Follow_flag = 1;
	else
		chassis_Follow_flag = 0;
}

float Error_Z, Last_Error_Z, Error_ZPterm, Error_ZDterm;
void Cha_RC_control()
{
	speed_x = RC_Ctl.rc.ch2 - RC_CH_VALUE_OFFSET;
	speed_y = RC_Ctl.rc.ch3 - RC_CH_VALUE_OFFSET;
	
	if( speed_y > 0 )
		Forward_flag = 1;
	else if( speed_y < 0 )
		Back_off_flag = 1;
	
	if( RC_Ctl.rc.s2 == RC_SW_MID )
		{ modeChassis = FOLLOW; }
	else
		{ modeChassis = ACTIVE; }
	
	if(modeChassis == ACTIVE)
		{
			speed_z = (RC_Ctl.rc.ch0 - RC_CH_VALUE_OFFSET) * 0.9f;
		}
	if(modeChassis == FOLLOW)
		{
			Error_Z = Motor_angel[YAW] - Y_M_MID;
			Error_Z = KalmanFilter(&RC_FOLLOW_Error_Kalman, Error_Z);
			Error_ZPterm = Error_Z * 1.2f;
			if( abs(Error_Z) < 250 )
			Error_ZDterm = (Error_Z - Last_Error_Z) * 0.40f / 0.01f;
			else
			Error_ZDterm = (Error_Z - Last_Error_Z) * 0.8f / 0.01f;		
			speed_z = Error_ZPterm + Error_ZDterm;
			Last_Error_Z = Error_Z;
		}
}

float debugXishu;
float MouseSpeed_Kalman;
bool change_F_flag = 0, change_B_flag = 0, Protect_Angle_flag = 0;
void Cha_KEY_control()
{
	switch(modeChassis)  //ģʽ�ж�
	{
		case FOLLOW:
			if( !IF_KEY_PRESSED_F )
			{ change_F_flag = 0; }
			if( !IF_KEY_PRESSED_B )
			{ change_B_flag = 0; }
		
			if( IF_KEY_PRESSED_CTRL || Base_Yaw_ready_flag == 1 || GIMBAL_ActionMode() == 5 )  //��Ctrl�����еģʽ
				{
					modeChassis = ACTIVE;
				}
			else if( IF_KEY_PRESSED_R )//��סR����������ģʽ
				{
					modeChassis = UPSLOPE;
				}
			else if( IF_KEY_PRESSED_F && change_F_flag == 0 )//��F������ҡ��ģʽ
				{
					modeChassis = DODGE;
				}
			else if( IF_KEY_PRESSED_B && change_B_flag == 0 )//��B������ȡ��ģʽ
				{
					modeChassis = GETBOMB;
				}
		break;
		
		case ACTIVE:
			if( !IF_KEY_PRESSED_CTRL && GIMBAL_ActionMode() != 3 && GIMBAL_ActionMode() != 5 ) //�ſ�Ctrl���Ҳ��ǵ���ģʽ�ص�������ģʽ
				{
					modeChassis = FOLLOW;
				}
		break;
		
		case UPSLOPE:
			if( !IF_KEY_PRESSED_R )   //�ſ�R���ص�������ģʽ
				{
					modeChassis = FOLLOW;
				}
		break;
		
		case DODGE:
			if( !IF_KEY_PRESSED_F )
			{ change_F_flag = 1; }
			if( !IF_KEY_PRESSED_B )
			{ change_B_flag = 0; }
		
			if( IF_KEY_PRESSED_F && change_F_flag == 1 ) //�ٴΰ���F���˳�ҡ��ģʽ
				{
					modeChassis = FOLLOW;
				}
			if( IF_KEY_PRESSED_CTRL )
				{
					modeChassis = ACTIVE;
					change_F_flag = 0;
				}
			else if( IF_KEY_PRESSED_B && change_B_flag == 0 && !IF_KEY_PRESSED_SHIFT )//��B������ȡ��ģʽ
				{
					modeChassis = GETBOMB;
				}
		break;
				
		case GETBOMB:
			if( !IF_KEY_PRESSED_B )
			{ change_B_flag = 1; }
			if( !IF_KEY_PRESSED_F )
			{ change_F_flag = 0; }
			
			if( (IF_KEY_PRESSED_B && change_B_flag == 1 && !IF_KEY_PRESSED_SHIFT) || IF_KEY_PRESSED_W ) //��B�˳�����֮ǰ��Wǰ���˳�ȡ��ģʽ
				{
					modeChassis = FOLLOW;
					yanshiCount = 0;
					Allow_Back_flag = 0;
				}
			else if( IF_KEY_PRESSED_F && change_F_flag == 0)//��F������ҡ��ģʽ
				{
					modeChassis = DODGE;
					yanshiCount = 0;
					Allow_Back_flag = 0;
				}
		break;
	}
		
	if( modeChassis == UPSLOPE )  //����ģʽʱ���ٶ�Сһ��
		key_ws_num = 1;
	else
		key_ws_num = 3;
	
	
	if( IF_KEY_PRESSED_W )
		{
			speed_num[0] += key_ws_num;
			Forward_flag = 1;
		}
	else if( IF_KEY_PRESSED_S )
		{
			speed_num[0] -= key_ws_num;
			Back_off_flag = 1;
		}
	else
		{
			if( speed_num[0] > 0 )
				{
					speed_num[0] -= StopSpeedNum;
					if( speed_num[0] < 0 )
						{speed_num[0] = 0;}
				}
			if( speed_num[0] < 0 )
				{
					speed_num[0] += StopSpeedNum;
					if(speed_num[0] > 0)
						{speed_num[0] = 0;}
				}
		}
	if( IF_KEY_PRESSED_D )
		{
			speed_num[1] += key_ad_num;
		}
	else if( IF_KEY_PRESSED_A )
		{
			speed_num[1] -= key_ad_num;
		}
	else
		{
			if( speed_num[1] > 0 )
				{
					speed_num[1] -= StopSpeedNum;
					if( speed_num[1] < 0 )
						{speed_num[1] = 0;}
				}
			if( speed_num[1] < 0 )
				{
					speed_num[1] += StopSpeedNum;
					if(speed_num[1] > 0)
						{speed_num[1] = 0;}
				}
		}
		
	speed_num[0] = constrain( speed_num[0], -660, 660);
	speed_num[1] = constrain( speed_num[1], -330, 330);//���˴򻬣����ҷ���û����ʱ��ƽ�ƿ�
			
	if( modeChassis == GETBOMB )  //ȡ��ģʽ����������
		{
			yanshiCount++;
			
			if( yanshiCount >= YanshiTime && Allow_Back_flag == 0 )  //��ʱ200ms��ʼ���ˣ�Ϊ���Ӿ�����ͷ�����ܹ��ȶ���ͼ
			{ Allow_Back_flag = 1; }
			
			if( VisionRecvData.centre_lock == 1 && Allow_Back_flag == 1 )
			{ speed_num[0] = -Get_Bomb_Speed; }
			else if( VisionRecvData.centre_lock == 2 )
			{
				speed_num[0] = 0;  //���˵�λ��ͣ��
				Allow_Back_flag = 2;
			} 
			
			speed_num[1] = constrain( speed_num[1], -Get_Bomb_Speed, Get_Bomb_Speed);
		}
	
	speed_dx = speed_num[1];
	speed_dy = speed_num[0];
		
	if(modeChassis == FOLLOW  || modeChassis == UPSLOPE)
		{
			if( modeChassis == FOLLOW )
			{
				if( abs(Motor_angel[YAW] - Y_M_MID) < 30 && Protect_Angle_flag == 0 && IF_Hurt == 1 && Speed_target_all < 100
						 && !IF_KEY_PRESSED_W && !IF_KEY_PRESSED_A && !IF_KEY_PRESSED_S && !IF_KEY_PRESSED_D ) //������ֹʱ�ܵ��˺���ת45��
				{
					Protect_Angle_flag = 1;
					IF_Hurt = 0;
				}
				if( Protect_Angle_flag == 1 )
				{
					Error_Z = Motor_angel[YAW] - DODGE_ANGLE;  //�ܵ��˺�������ת45��
					
					if( (abs(Motor_angel[YAW] - DODGE_ANGLE) < 30 && IF_Hurt == 1) //�ٴ��ܵ��˺��󣬻��߰�����������̻ص�����
								|| IF_KEY_PRESSED_W || IF_KEY_PRESSED_A || IF_KEY_PRESSED_S || IF_KEY_PRESSED_D )
						{
							Protect_Angle_flag = 0;
							IF_Hurt = 0;
						}
				}
				else
				{
					Error_Z = Motor_angel[YAW] - Y_M_MID;
				}
			}
		if( modeChassis == UPSLOPE )
			{ Error_Z = Motor_angel[YAW] - Y_M_MID; }
			
			Error_Z = KalmanFilter(&KEY_FOLLOW_Error_Kalman, Error_Z);
			Error_ZPterm = Error_Z * 1.2f;
			
			if( abs(Error_Z) < 250 )
			Error_ZDterm = (Error_Z - Last_Error_Z) * 0.40f / 0.01f;
			else
			Error_ZDterm = (Error_Z - Last_Error_Z) * 0.8f / 0.01f;	
			
			speed_z = Error_ZPterm + Error_ZDterm;
			Last_Error_Z = Error_Z;
		}
	else if( modeChassis == ACTIVE || modeChassis == GETBOMB )
		{
			if( modeChassis == GETBOMB || Base_Yaw_ready_flag == 1 || GIMBAL_ActionMode() == 5 ) //ȡ��ģʽʱ�������������
				{
					speed_z = constrain( 3 * AverageFiler(MOUSE_X_MOVE_SPEED), -6500, 6500);
				}
			else
				{
					speed_z = constrain( 15 * AverageFiler(MOUSE_X_MOVE_SPEED), -6500, 6500);
				}
		}
		
	if( modeChassis == DODGE )
		{
			static int16_t twist_period = 500;
			static int16_t twist_angle  = 30;

			twist_count++; 
				
			position_ref = 	twist_angle * sin(2 * PI / twist_period * twist_count);
				
			speed_y =   speed_dx * sin( (Y_M_MID - Motor_angel[YAW]) / ENCODER_ANGLE_RATIO / RADIAN_COEF )   //����45���
	              + speed_dy * cos( (Y_M_MID - Motor_angel[YAW]) / ENCODER_ANGLE_RATIO / RADIAN_COEF );
			
			speed_x =   speed_dx * cos( (Y_M_MID - Motor_angel[YAW]) / ENCODER_ANGLE_RATIO / RADIAN_COEF )
	              - speed_dy * sin( (Y_M_MID - Motor_angel[YAW]) / ENCODER_ANGLE_RATIO / RADIAN_COEF );
			
			Error_Z = ((Motor_angel[YAW] - DODGE_ANGLE) / ENCODER_ANGLE_RATIO) - position_ref;
			Error_ZPterm = Error_Z * debug_P;
			Error_ZDterm = (Error_Z - Last_Error_Z) * debug_D / 0.01f;
			speed_z = Error_ZPterm + Error_ZDterm;
			Last_Error_Z = Error_Z;
		}
	else
		{
			speed_x = speed_dx;
			speed_y = speed_dy;
		}
}

int rotation_center_gimbal = 1;//��̨���ڵ�������
void Cha_Motor_speed(float vx, float vy, float vw)
{
	static float rotate_ratio_fr;
  static float rotate_ratio_fl;
  static float rotate_ratio_bl;
  static float rotate_ratio_br;
  static float wheel_rpm_ratio;
	
	if( modeChassis == UPSLOPE )
		mul = 6;
	else
		mul = 10;
	
	vx = vx * mul;
	vy = vy * mul;
	
	/* ��ת���ٶ�Խ��ʱ������x��y���ٶȣ���ֹ����������ת�䲻����*/
	float rate = 0.0f;
	if( abs(vw) <= 600 )
	rate = 1 - abs(vw) / 1200;
	else
	rate = 0.5f - abs(vw) / 3600;

	vx = vx * rate;
	vy = vy * rate;
	/*----------*/
	
	if (rotation_center_gimbal)
		{
			rotate_ratio_fr = ((WHEELBASE + WHEELTRACK) / 2.0f - GIMBAL_Y_OFFSET + GIMBAL_X_OFFSET)/RADIAN_COEF;
			rotate_ratio_fl = ((WHEELBASE + WHEELTRACK) / 2.0f - GIMBAL_Y_OFFSET - GIMBAL_X_OFFSET)/RADIAN_COEF;
			rotate_ratio_bl = ((WHEELBASE + WHEELTRACK) / 2.0f + GIMBAL_Y_OFFSET - GIMBAL_X_OFFSET)/RADIAN_COEF;
			rotate_ratio_br = ((WHEELBASE + WHEELTRACK) / 2.0f + GIMBAL_Y_OFFSET + GIMBAL_X_OFFSET)/RADIAN_COEF;
		}
  else
		{
			rotate_ratio_fr = ((WHEELBASE + WHEELTRACK) / 2.0f) / RADIAN_COEF;
			rotate_ratio_fl = rotate_ratio_fr;
			rotate_ratio_bl = rotate_ratio_fr;
			rotate_ratio_br = rotate_ratio_fr;
		}
  wheel_rpm_ratio = 60.0f/(PERIMETER * CHASSIS_DECELE_RATIO);             //	60/�ܳ�*���ٱ�
	
	motor_speed[0] =  (+vx + vy + vw * rotate_ratio_fl) * wheel_rpm_ratio;		//0��ǰ	
	motor_speed[1] = -(-vx + vy - vw * rotate_ratio_fr) * wheel_rpm_ratio;	  //1��ǰ
	motor_speed[2] =  (-vx + vy + vw * rotate_ratio_bl) * wheel_rpm_ratio;	  //2���
	motor_speed[3] = -(+vx + vy - vw * rotate_ratio_br) * wheel_rpm_ratio;		//3�Һ�	
	
	motor_speed[0] = constrain(motor_speed[0], -speedmax, speedmax);
	motor_speed[1] = constrain(motor_speed[1], -speedmax, speedmax);
	motor_speed[2] = constrain(motor_speed[2], -speedmax, speedmax);
	motor_speed[3] = constrain(motor_speed[3], -speedmax, speedmax);
	
	Speed_target_all = motor_speed[0] + motor_speed[1] + motor_speed[2] + motor_speed[3];   //������ֵ
	Last_target_all = Speed_target_all;
	
	/******  �������ݵ������ж�  ******/
	if( Last_target_all == Speed_target_all && Speed_target_all != 0 && just_a_flag == 0)
		{
			remainspeedtime = micros();
			just_a_flag = 1;
		}
	else if( Last_target_all == Speed_target_all && Speed_target_all != 0 && just_a_flag == 1)
		{
			if( micros() - remainspeedtime > 1000000)//���ٱ���һ��
			 {
				 just_a_flag = 2;
			 }
		}
	else if( Last_target_all == Speed_target_all && Speed_target_all!=0 && just_a_flag == 2 )
		 {
			 yunsuflag = 1;
		 }
		 else 
		 {
			 just_a_flag = 0;
			 yunsuflag = 0;
		 }
 /*****************************************/
	
	chassis_PID(0, Motor_chassis[0][1], motor_speed[0]);
	chassis_PID(1, Motor_chassis[1][1], motor_speed[1]);
	chassis_PID(2, Motor_chassis[2][1], motor_speed[2]);
	chassis_PID(3, Motor_chassis[3][1], motor_speed[3]);
}

void chassis_PID(int num, float actspeed, float aimspeed)
{
	static float error[4], LastError[4];
	
	error[num] = aimspeed - actspeed;
	
	Cha_Pterm[num] = error[num] * Chassis_P;
	
if( cap_output_flag == 1 ) //�������ݷŵ�ʱ�������ٶ��������
		{
			speedmax = 10000;
		}	
	else
		{
			if( modeChassis == UPSLOPE )
				{
					if(num == 0 || num == 1)   //�޶�ǰ�ֵ��ٶȣ������ʷ��������
						{
							speedmax = 4200;
						}
					else if(num == 2 || num == 3 )
						{
							speedmax = 8000;
						}
				}
			else
				{
					if( (Forward_flag == 1 && IF_KEY_PRESSED_S) || (Forward_flag == 1 && RC_Ctl.rc.ch3 - RC_CH_VALUE_OFFSET < 0) )  //ǰ��ʱ�������޶������ٶȣ������ʷ����ǰ��
						{
							if(num == 2 || num == 3 )
								{
									speedmax = 2000;
								}
								
							Commutation ++;
								
							if( Commutation >= CommutationTime )
								{ 
									Forward_flag = 0; 
									Commutation = 0;
								}	
						}
					else if( (Back_off_flag == 1 && IF_KEY_PRESSED_W) || (Back_off_flag == 1 && RC_Ctl.rc.ch3 - RC_CH_VALUE_OFFSET > 0) )  //����ʱ��ǰ�����޶�ǰ���ٶȣ������ʷ��������
						{
							if(num == 0 || num == 1 )
								{
									speedmax = 2000;
								}
								
							Commutation ++;
								
							if( Commutation >= CommutationTime )
								{ 
									Back_off_flag = 0;
									Commutation = 0;
								}	
						}
					else
						{
							speedmax = 8000;
						}
					
					if( abs(speed_z) <= 50 || aimspeed == 0 ) //����û�˶�ʱ��I��С
						Itermmax = 200;
					else
						Itermmax = 2000;
				}
		}
	
	Cha_Iterm[num] += error[num] * Chassis_I * 0.001f;//�˲���ʱ��
	Cha_Iterm[num] = constrain(Cha_Iterm[num], -Itermmax, Itermmax);
	
	Cha_Dterm[num] = Chassis_D * ( error[num] - LastError[num] ) / 0.01f;
	LastError[num] = error[num];
		
	Chassis_PIDterm[num] = Cha_Pterm[num] + Cha_Iterm[num] + Cha_Dterm[num];
	Chassis_PIDterm[num] = constrain (Chassis_PIDterm[num], -speedmax, speedmax);
	
}

void Cha_PowerLimit(void)
{
	TotalOut = abs(Chassis_PIDterm[0]) + abs(Chassis_PIDterm[1])   //���������
	          +abs(Chassis_PIDterm[2]) + abs(Chassis_PIDterm[3]);
	
	if( ! PowerProtect.judgemengt_connecting )
		{
			OutLimit = 10000;
		}
	else
		{
			PowerProtect.connecting_num++;
			if( PowerProtect.connecting_num > 500 )
				{
					PowerProtect.judgemengt_connecting = 0;
				}
			if( PowerProtect.power_remain < 50 )
				{
					OutLimit = ( (float)(PowerProtect.power_remain / 60.0f) 
						* (float)(PowerProtect.power_remain / 60.0f) ) * 36000;
				}
			else
				{
					OutLimit = 36000;
				}
		}
	if( TotalOut > OutLimit )
		{
			Chassis_PIDterm[0] = (Chassis_PIDterm[0]) / (TotalOut) * OutLimit;
			Chassis_PIDterm[1] = (Chassis_PIDterm[1]) / (TotalOut) * OutLimit;
			Chassis_PIDterm[2] = (Chassis_PIDterm[2]) / (TotalOut) * OutLimit;
			Chassis_PIDterm[3] = (Chassis_PIDterm[3]) / (TotalOut) * OutLimit;
		}
}

void Chassis_InitArgument(void)
{
	KalmanCreate(&RC_FOLLOW_Error_Kalman, 1, 220);
	KalmanCreate(&KEY_FOLLOW_Error_Kalman, 1, 220);
	
	KalmanCreate(&Gimbal_SpeedZ_Kalman, 1, 50);
}

float Get_Chassis_Mode(void)
{
	return modeChassis;
}

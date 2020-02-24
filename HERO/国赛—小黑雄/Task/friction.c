#include "friction.h"

/* 700  480 */
/*
****19  ���ⵯ  ����16.0����  PWM35
***  ��������PWM  33  ����***

****18  ���ⵯ  ����16.0����  PWM34
***  ��������PWM  33  ***

�ӵ�ѵ��  PWM 32/31  ����15.5����
*/
float    RC_FRC42_SPEED  = 37; //���ⵯ43
float    RC_FRC17_SPEED  = 440;
float    KEY_FRC42_SPEED = 37; //���ⵯ43
float    KEY_FRC17_SPEED = 580, KEY_FRC17_XIAOBOMB_SPEED = 640
	;
int16_t  aimFrcSpeed[2], RealFrcSpeed[2];
bool     Fric17_ON_flag = 0, Fric42_ON_flag = 0, Fric17_ready = 0, Fric42_ready = 0;
int      Fric_flag;
float    FRC42_offset = 0, FRC17_offset = 0;

bool Xianwei_Ready_Flag = 0;
void FRIC_Control(void)
{
	if( Xianwei_Ready_Flag == 0 && abs(FRC_MACH_PIT - Motor_angel[PITCH]) < 80 )
	{	
		XIANWEI_Angle( CLOSE_ANGLE );
		Xianwei_Ready_Flag = 1;
	}
	
	if( Get_remoteMode() == RC )
		{
			Fric_RC_Control();
		}
	else
		{
			Fric_KEY_Control();
		}
		
	if( RealFrcSpeed[FRICTION_42] >= 0  )
		{
			if( RealFrcSpeed[FRICTION_42] < aimFrcSpeed[FRICTION_42] )
				{
					RealFrcSpeed[FRICTION_42] += 1;
				}
			else if( RealFrcSpeed[FRICTION_17] > aimFrcSpeed[FRICTION_42] )
				RealFrcSpeed[FRICTION_42] -= 1;
			if( RealFrcSpeed[FRICTION_42] < 0 )
				RealFrcSpeed[FRICTION_42] = 0;				
		}
		
	if( RealFrcSpeed[FRICTION_17] >= 0 )
		{
			if( RealFrcSpeed[FRICTION_17] < aimFrcSpeed[FRICTION_17] )
				RealFrcSpeed[FRICTION_17] += 1;
			else if( RealFrcSpeed[FRICTION_17] > aimFrcSpeed[FRICTION_17] )
				RealFrcSpeed[FRICTION_17] -= 0.5f;
			if( RealFrcSpeed[FRICTION_17] < 0 )
				RealFrcSpeed[FRICTION_17] = 0;				
		}
		
	friction42_PWM( RealFrcSpeed[FRICTION_42], RealFrcSpeed[FRICTION_42] + FRC42_offset);
	friction17_PWM( RealFrcSpeed[FRICTION_17], RealFrcSpeed[FRICTION_17] + FRC17_offset);
}

void Fric_RC_Control(void)
{
	if( RC_Ctl.rc.s2 == RC_SW_DOWN )
		{
			if( RC_Ctl.rc.s1 == RC_SW_UP )
				{
					if(Fric_flag == 1)
						{Fric_flag = 2;}
					else if(Fric_flag == 2)
						{Fric_flag = 0;}								
				}
			else
				{Fric_flag = 1;}
		}
	else
		{
			Fric_flag = 0;
		}	
	if( Fric_flag == 2 )
		{
			if( aimFrcSpeed[FRICTION_42] > 0 )
				{
					aimFrcSpeed[FRICTION_42] = 0;
					Fric42_ON_flag = 0;
				}
			else
				{aimFrcSpeed[FRICTION_42] = RC_FRC42_SPEED;}
				
			if( aimFrcSpeed[FRICTION_17] > 0 )
				{ 
					aimFrcSpeed[FRICTION_17] = 0; 
					Fric17_ON_flag = 0;
				}
			else
				{ aimFrcSpeed[FRICTION_17] = RC_FRC17_SPEED;}
		}						
}

bool jiangsuflag = 0, shengsuflag = 0;
void Fric_KEY_Control(void)
{
	if( ((IF_MOUSE_PRESSED_LEFT) || (aimFrcSpeed[FRICTION_42] > 0)) && Fric17_ON_flag == 0 )//С������Ҫ�ı�Ͳ����������ж���
		{
			aimFrcSpeed[FRICTION_17] = KEY_FRC17_SPEED;
			
			Fric17_ON_flag = 1;
			Fric42_ON_flag = 1;
		}
	if( Fric42_ON_flag == 1 && Xianwei_Ready_Flag == 1 )
		{
			aimFrcSpeed[FRICTION_42] = KEY_FRC42_SPEED;
		}
	
  /********* С�����ٵ��� ****************/	
	if( Shoot_State == BIGBOMB_MODE && Fric17_ON_flag && GIMBAL_ActionMode() != 3 )
		{
			aimFrcSpeed[FRICTION_17] = KEY_FRC17_SPEED;  //��ģʽʱ�����ٵ�һ�㣬��ϴ�
		}
	else if( Shoot_State == XIAOBOMB_MODE && Fric17_ON_flag && GIMBAL_ActionMode() != 3 )
		{
			aimFrcSpeed[FRICTION_17] = KEY_FRC17_XIAOBOMB_SPEED;  //С��ģʽʱ�����ٸ�һ��
		}
	/***************************************************************************************************/	

	/********* ������PWM���� *************/		
	if( !IF_KEY_PRESSED_CTRL || !IF_KEY_PRESSED_Z )	
		{ jiangsuflag = 0; }
	if( !IF_KEY_PRESSED_CTRL || !IF_KEY_PRESSED_X )
		{ shengsuflag = 0; }
	if( IF_KEY_PRESSED_CTRL && IF_KEY_PRESSED_Z && jiangsuflag == 0) //��PWM��һ���ڱ���һֱ����ʱ��
		{
			KEY_FRC42_SPEED -= 2;
			jiangsuflag = 1;
		}
	if( IF_KEY_PRESSED_CTRL && IF_KEY_PRESSED_X && shengsuflag == 0 ) //��PWM�����ٵ�ʱ������
		{
			KEY_FRC42_SPEED += 2;
			shengsuflag = 1;
		}
	/****************************************************************************************************/		
}


void IF_Fric_Ready(void)
{
	static uint32_t fric17Time = 0;
	static uint32_t fric42Time = 0;
	uint32_t Fric17Current = 0;
	uint32_t Fric42Current = 0;
	Fric17Current = micros();
	Fric42Current = micros();
	if( RealFrcSpeed[FRICTION_17] > 0 )
		{
			if( Fric17Current >= fric17Time + 1500000 )  //Ħ���ֿ���1.5s����ܿ�������
				{
					Fric17_ready = 1;
				}
		}
	else
		{
			fric17Time = Fric17Current;		
			Fric17_ready = 0;	
		}
		
	if( RealFrcSpeed[FRICTION_42] > 0 )
		{
			if( Fric42Current >= fric42Time + 1500000 )
				{
					Fric42_ready = 1;
				}
		}
	else
		{
			fric42Time = Fric42Current;
			Fric42_ready = 0;
		}
}

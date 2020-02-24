#include "weapon.h"
#include "vision.h"

typedef enum
{
	SHOOT_NORMAL  = 0, //正常模式
	SHOOT_AUTO    = 1, //自瞄模式
}eactRevolver;
eactRevolver actRevolver;

void Weapon_Control(void);
void Weapon_RC_Control(void);
void Weapon_KEY_Control(void);
void XiaoBomb_Control(void);
void Motianlun_Control(void);
void M2006_RecordBombNum_Iint(void);
void M2006_Speed_PID(int num);

bool RC_shoot_flag = 0, shoot42_Frequency_flag = 0, shoot17_Frequency_flag = 0, ContinueTime_flag =0;  //继电器变量
float Open_Time, continued_Time = 13, XianWei_Time, XianWeiContinue_Time = 40; 

/* M2006位置环PID */
float A_Error[3], A_Pterm[3], A_Iterm[3], R_Target[3], R_Error[3], R_Pterm[3], R_Iterm[3];   //位置环变量
float P[2][2] = {0.4, 0, 6, 0}, I[2][2];
float R_Speed = 1.5;

u8   bigbopan_Num_Init, Yuzhi_TwoBomb_Iint, Bigbopan_stop_Init;    //摩天轮变量
int16_t  bigbopan_flag = 0;
float Yuzhi_TwoBomb_Count = 0, Shot_Num=0;
bool Yuzhi_TwoBomb_flag = 0;
int16_t Got_BombNum = 0;
int16_t  S_ReversalNum[2], S_LockNum[2], shootFrequencyTime42, shootFrequencyTime17;
float aimSpeed[2], S_Pterm[2], S_Iterm[2], S_Error[2], SP[2]={6,-7}, SI[2]={2,-8};

uint32_t HEAT_LIMIT_42mm, HEAT_LIMIT_17mm;      //热量限制
int bomb_42_allowNum, bomb_17_allowNum;         //允许射击数量
float SpeedMax_42 = 100, SpeedMax_17 = 35;      //打一发子弹，增加100热量
float Record_Heat42, Bullet_Speed[2][2];        //记录枪口热量，用作计算
uint32_t Record_Time_42, Record_Time_17;        //记录热量跟新时间
int Bomb_Shoot_42_Num, Bomb_Shoot_17_Num;       //子弹发射数量 
bool Bomb42_Shoot_Allow = 1, Bomb17_Shoot_Allow = 1;//允许发弹标志位

float debug_Ouput;
int16_t Shoot_State = XIAOBOMB_MODE; //上电默认为小弹模式
bool Big_Bomb_Flag = 0;
void Weapon_Control(void)
{
	IF_Fric_Ready();
	
	if( Get_SystemState() == SYSTEM_STARTING )
		{
			OFF_FIRE;

			M06_Angle.Sum[XIAOBOPAN] = M06_Angle.Target[XIAOBOPAN] = M06_Angle.Total[XIAOBOPAN];//给目标值赋一个初始值，才不会一上电就随便动
		}
	
	ShotBomb_Count();	 //大弹计数
		
	Muzzle_Heat_Limit();	//枪口热量限制	
		
	if( Get_remoteMode() == RC )
		{
			Weapon_RC_Control();
		}
	else if( Get_remoteMode() == KEY )
		{
			Weapon_KEY_Control();
		}
		
	Motianlun_Control();
	
	CAN2_Motor_Send();
	
	debug_Ouput = GPIO_ReadOutputDataBit( GPIOB, GPIO_Pin_1 );
}

float openagle = 53;
void Weapon_RC_Control(void)
{
	/*   大弹发射   */
	if( (RC_Ctl.rc.s1 == RC_SW_DOWN) && (RC_Ctl.rc.s2 == RC_SW_DOWN) && !ContinueTime_flag && Fric42_ready && RC_shoot_flag && Bomb42_Shoot_Allow)
		{
			XIANWEI_Angle( OPEN_ANGLE );
			ContinueTime_flag = 1;
			RC_shoot_flag = 0;
		}
	
	/*   小弹发射   */
	XiaoBomb_Control();
	
	if( RC_Ctl.rc.s1 == RC_SW_MID )
		RC_shoot_flag = 1;
}

void Weapon_KEY_Control(void)
{
	if( (IF_KEY_PRESSED_SHIFT && IF_KEY_PRESSED_X) )  //按Shift + X切换成小弹模式
	{
		Shoot_State = XIAOBOMB_MODE;
		Big_Bomb_Flag = 0;
	}
	if( (Big_Bomb_Flag == 1) || (IF_KEY_PRESSED_SHIFT && IF_KEY_PRESSED_Z) ) //按Shift + Z或者检测到有大弹切换成大弹模式
	{
		Shoot_State = BIGBOMB_MODE;
	}
	
	switch(actRevolver)
	{
		case SHOOT_NORMAL:
		
		if( IF_MOUSE_PRESSED_RIGH )
			{
				actRevolver = SHOOT_AUTO;
			}
		else
			{
				if( IF_MOUSE_PRESSED_LEFT && (ContinueTime_flag == 0) && (shoot42_Frequency_flag == 0)
					&& (Fric42_ready == 1) && Bomb42_Shoot_Allow && Shoot_State == BIGBOMB_MODE)
					{
						XIANWEI_Angle( OPEN_ANGLE );
						ContinueTime_flag = 1;
						shoot42_Frequency_flag = 1;
					}
				
				/*   小弹发射   */
				XiaoBomb_Control();
			}
		break;
			
		case SHOOT_AUTO:
			if( !IF_MOUSE_PRESSED_RIGH )
				{
					actRevolver = SHOOT_NORMAL;
				}
			else
				{
					if( (IF_MOUSE_PRESSED_LEFT || VisionRecvData.centre_lock == 1) && (ContinueTime_flag == 0) && (shoot42_Frequency_flag == 0) && (Fric42_ready == 1)
						&& Bomb42_Shoot_Allow && Shoot_State == BIGBOMB_MODE)
						{
							XIANWEI_Angle( OPEN_ANGLE );
							ContinueTime_flag = 1;
							
							
							shoot42_Frequency_flag = 1;
						}
						
					/*   小弹发射   */
					XiaoBomb_Control();
				}
		break;
	}
}

void XiaoBomb_Control(void)
{
	static unsigned char IfXiaoStop = FALSE;
	
	if( IfXiaoStop == TRUE )   //小拨盘卡弹，速度环反转
		{
			aimSpeed[XIAOBOPAN] = -2000;
			M2006_Speed_PID(XIAOBOPAN);
			S_ReversalNum[XIAOBOPAN]++;
			if( S_ReversalNum[XIAOBOPAN] > 150 )//反转300ms
				{
					S_ReversalNum[XIAOBOPAN] = 0;
					IfXiaoStop = FALSE;
				}
		}
	else
		{
			if( (abs(S_PIDTerm[XIAOBOPAN]) > 5500) && (abs(M2006_actSpeed[XIAOBOPAN]) < 100) )
				{
					S_LockNum[XIAOBOPAN]++;
				}
			else
				{
					S_LockNum[XIAOBOPAN] = 0;
				}
			if( S_LockNum[XIAOBOPAN] > 100 ) //维持200ms，则堵转
				{
					S_LockNum[XIAOBOPAN] = 0;
					IfXiaoStop = TRUE;
				}
			else
				{
					if( Get_remoteMode() == RC )
						{
							if( (RC_Ctl.rc.s1 == RC_SW_DOWN) && (RC_Ctl.rc.s2 == RC_SW_MID) && RC_shoot_flag 
								&& Fric17_ready && Bomb17_Shoot_Allow && (Motor_angel[PITCH] > CANNOT_XIAOBOMB))
								{
									if( (abs(S_PIDTerm[XIAOBOPAN]) < 5500) || (abs(M2006_actSpeed[XIAOBOPAN]) > 100) )
										{
											M06_Angle.Sum[XIAOBOPAN] += 110578.5f;  //拨一下三连发
										}
									RC_shoot_flag = 0;
								}
						}
					else
						{
							if( actRevolver == SHOOT_NORMAL )
								{
									if( IF_MOUSE_PRESSED_LEFT && Fric17_ready && Bomb17_Shoot_Allow 
										&& (shoot17_Frequency_flag == 0) && (Motor_angel[PITCH] > CANNOT_XIAOBOMB) 
											&& GIMBAL_ActionMode() != 3 )
										{
											if( (abs(S_PIDTerm[XIAOBOPAN]) < 5500) || (abs(M2006_actSpeed[XIAOBOPAN]) > 100) )
												{
													M06_Angle.Sum[XIAOBOPAN] += 110578.5f;  //拨一下三连发
													shoot17_Frequency_flag = 1;
												}
										}
								}
							else //自瞄模式
								{
									if( (IF_MOUSE_PRESSED_LEFT || VisionRecvData.centre_lock == 1) && Fric17_ready && Bomb17_Shoot_Allow && (shoot17_Frequency_flag == 0)
										 && (Motor_angel[PITCH] > CANNOT_XIAOBOMB))
										{
											if( (abs(S_PIDTerm[XIAOBOPAN]) < 5500) || (abs(M2006_actSpeed[XIAOBOPAN]) > 100) )
												{
													M06_Angle.Sum[XIAOBOPAN] += 110578.5f;  //拨一下三连发
													shoot17_Frequency_flag = 1;
												}
										}
								}								
						}
				}
							
			M2006_Position_PID(XIAOBOPAN);
		}
}

float speed_dubug = 400;
void Motianlun_Control(void) //摩天轮供弹控制
{
	static bool Get_Bomb_flag = 0;
	static unsigned char IfM06Stop = FALSE;
	
	bigbopan_Num_Init = GPIO_ReadInputDataBit( GPIOC, GPIO_Pin_6 ); 
	Bigbopan_stop_Init = GPIO_ReadInputDataBit( GPIOC, GPIO_Pin_7 );
	Yuzhi_TwoBomb_Iint = GPIO_ReadInputDataBit( GPIOB, GPIO_Pin_9 );
	
	if( (bigbopan_flag == 0) && (bigbopan_Num_Init == 0) )
		{
			bigbopan_flag = 1;
		}
  else if( (bigbopan_flag == 1) && (bigbopan_Num_Init == 1) )//一个完整的检测过程后，弹丸数加一
		{
			Got_BombNum++ ;
			bigbopan_flag = 0;
			Big_Bomb_Flag = 1; //检测到大弹时，标志位置1,转为大弹模式
		}
		
	if( Yuzhi_TwoBomb_Iint == 0 )	
		{ Yuzhi_TwoBomb_Count++; }
	else
		{ Yuzhi_TwoBomb_Count = 0; }
	if( Yuzhi_TwoBomb_Count >= 50 )//枪管的光电检测到遮挡500ms则认为有弹
		{ Yuzhi_TwoBomb_flag = 1; }
	else
		{ Yuzhi_TwoBomb_flag = 0; }
			
	if( IF_KEY_PRESSED_SHIFT && IF_KEY_PRESSED_CTRL && Get_Bomb_flag == 0)	
		{
			Got_BombNum -= 2;
			Get_Bomb_flag = 1;
		}
	if( !IF_KEY_PRESSED_SHIFT || !IF_KEY_PRESSED_CTRL)
		{
			Get_Bomb_flag = 0;
		}
		
//	if( (Got_BombNum - Shot_42_Num >= 2 && Get_remoteMode() == RC)  //拨盘里剩余2颗子弹时停下，防止软管里子弹数量过多
//		  || (Got_BombNum - Shot_42_Num >= 0 && Get_remoteMode() == KEY) //键盘模式时不主动供弹防止死后弹链堆积太多子弹
//				|| Yuzhi_TwoBomb_flag == 1 )
//		{
//			aimSpeed[MOTIANLUN] = 0;
//		}
	if( Yuzhi_TwoBomb_flag == 1 )
		{
			aimSpeed[MOTIANLUN] = 0;
		}
	else
		{
			if( (abs(S_PIDTerm[MOTIANLUN]) >= 7000) && (M2006_actSpeed[MOTIANLUN] <= 50) )
				{
					S_LockNum[MOTIANLUN]++;
				}
			if( S_LockNum[MOTIANLUN] >= 750 )
				{
					IfM06Stop = !IfM06Stop;   //卡住就换一个方向转
					S_LockNum[MOTIANLUN] = 0;
				}

			if( Fric42_ready == 1 )    //大拨盘的摩擦轮准备好就开始转动
				{
					if( IfM06Stop == TRUE )
						{	
							aimSpeed[MOTIANLUN] = speed_dubug;
						}
					else
						{
							aimSpeed[MOTIANLUN] = -speed_dubug;
						}
				}
			else
				{
					aimSpeed[MOTIANLUN] = 0;
				}
		}
	M2006_Speed_PID(MOTIANLUN);
}

void M2006_RecordBombNum_Iint(void) //摩天轮的光电开关初始化
{
	GPIO_InitTypeDef gpio;
	RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOC|RCC_AHB1Periph_GPIOB|RCC_AHB1Periph_GPIOE, ENABLE );

	gpio.GPIO_Mode  = GPIO_Mode_IN;
	gpio.GPIO_PuPd  = GPIO_PuPd_UP;
	gpio.GPIO_Speed = GPIO_Speed_50MHz;
	gpio.GPIO_Pin   = GPIO_Pin_6;
	GPIO_Init( GPIOC, &gpio );
	
	gpio.GPIO_Pin   = GPIO_Pin_9;
	GPIO_Init( GPIOB, &gpio );
	
	gpio.GPIO_Pin   = GPIO_Pin_9;
	GPIO_Init( GPIOE, &gpio );
}

void relay_init(void)  //继电器初始化
{
	  GPIO_InitTypeDef GPIO_InitStructure;
	  
	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);
	
	  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_1;
	  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
	  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;  //自制是pp，蓝色继电器是OD
	  GPIO_Init(GPIOB,&GPIO_InitStructure); 
}

/*------------------------------------------------------------*/
/*------------------------枪口热量限制------------------------*/
void ShotBomb_Count(void)
{
	if( Shoot_Data_42_Update == 1 )  //大弹发射计数
		{
			Shoot_Data_42_Update = 0;
			
			Bullet_Speed[BOMB_42][NOW] = ShootData.bullet_speed;

			if( Bullet_Speed[BOMB_42][NOW] != Bullet_Speed[BOMB_42][LAST] )	
				{
					Shot_42_Num++;   //热量数据更新一次，并且射速不一样，证明发射一颗弹丸
					Bomb_Shoot_42_Num++;
				}
			Bullet_Speed[BOMB_42][LAST] = Bullet_Speed[BOMB_42][NOW];
			 
			Record_Time_42 = millis();
		}
		
	if( Shoot_Data_17_Update == 1 )
		{
			Shoot_Data_17_Update = 0;
			
			Bullet_Speed[BOMB_17][NOW] = ShootData.bullet_speed;
			
			if( Bullet_Speed[BOMB_17][NOW] != Bullet_Speed[BOMB_17][LAST] )	
				{
					Shot_17_Num++;   //热量数据更新一次，并且射速不一样，证明发射一颗弹丸
					Bomb_Shoot_17_Num++;
				}
			Bullet_Speed[BOMB_17][LAST] = Bullet_Speed[BOMB_17][NOW];
			 
			Record_Time_17 = millis();
		}
}

void Muzzle_Heat_Limit(void)
{
	if( Get_remoteMode() == RC )
		{
			HEAT_LIMIT_42mm = 200;
			HEAT_LIMIT_17mm = 240;
		}
	else
		{
			switch(GameRobotState.robot_level)
				{
					case 1: HEAT_LIMIT_42mm = 200;
									HEAT_LIMIT_17mm = 240;
							break;
					case 2: HEAT_LIMIT_42mm = 300;
									HEAT_LIMIT_17mm = 360;
							break;
					case 3: HEAT_LIMIT_42mm = 400;
									HEAT_LIMIT_17mm = 480;
							break;
					default:HEAT_LIMIT_42mm = 200;     //没有等级时，以一级限制为准
									HEAT_LIMIT_17mm = 240;
							break;
				}
		}

	bomb_42_allowNum = ( HEAT_LIMIT_42mm - PowerHeatData.shooter_heat1 ) / SpeedMax_42;
	bomb_17_allowNum = ( HEAT_LIMIT_17mm - PowerHeatData.shooter_heat0 ) / SpeedMax_17;
		
  if( debug_Ouput == 0 && Shoot_Data_42_Update == 0 && (millis() - Record_Time_42) > 100 )
		{
			Bomb_Shoot_42_Num = 0;
		}
		
	if (Bomb_Shoot_42_Num > bomb_42_allowNum || PowerHeatData.shooter_heat1 + 110 >= HEAT_LIMIT_42mm || bomb_42_allowNum == 0)
		{
			Bomb42_Shoot_Allow = 0;
		}
	else
		{
			Bomb42_Shoot_Allow = 1;
		}
		

  if( M2006_actSpeed[XIAOBOPAN] < 10 && Shoot_Data_17_Update == 0 && (millis() - Record_Time_17) > 100 )
		{
			Bomb_Shoot_17_Num = 0;
		}
		
	if (Bomb_Shoot_17_Num > bomb_17_allowNum || PowerHeatData.shooter_heat0 + 100 >= HEAT_LIMIT_17mm || bomb_17_allowNum <= 4)
		{
			Bomb17_Shoot_Allow = 0;
		}
	else
		{
			Bomb17_Shoot_Allow = 1;
		}
}


/*-----M2006速度环------*/
void M2006_Speed_PID(int num)
{
	S_Error[num] = aimSpeed[num] - M2006_actSpeed[num];
	
	S_Pterm[num] = SP[num] * S_Error[num];
	
	S_Iterm[num] += SI[num] * S_Error[num] * 0.001f;
	S_Iterm[num] = constrain( S_Iterm[num], -6000, 6000 );
	
	if( aimSpeed[num] == 0 )   //电机速度期望为零时，立刻停下
		{
			S_Iterm[num] = 0;
		}
	if( num == 0)	
	S_PIDTerm[num] = constrain( S_Pterm[num]+S_Iterm[num], -8000, 8000 );
	else if ( num == 1 )
	S_PIDTerm[num] = constrain( S_Pterm[num]+S_Iterm[num], -10000, 10000 );
}

/*----M2006位置环-----*/
void M2006_Position_PID(int num)
{
	if( (M06_Angle.Target[num] < M06_Angle.Sum[num]) && (M06_Angle.Target[num] - M06_Angle.Total[num]) <= 100000 )
		{
			M06_Angle.Target[num] += 1000;
			if( M06_Angle.Target[num] > M06_Angle.Sum[num] )
				{
					M06_Angle.Target[num] = M06_Angle.Sum[num];
				}
		}
	else if( (M06_Angle.Target[num] > M06_Angle.Sum[num]) && (M06_Angle.Target[num] - M06_Angle.Total[num]) >= -100000 )
		{
			M06_Angle.Target[num] -= 1000;
			if( M06_Angle.Target[num] < M06_Angle.Sum[num] )
				{
					M06_Angle.Target[num] = M06_Angle.Sum[num];
				}
		}
		
	A_Error[num] = M06_Angle.Target[num] - M06_Angle.Total[num];
		
	A_Pterm[num] = P[OUT][num] * A_Error[num];
		
	A_Iterm[num] += I[OUT][num] * A_Error[num] * 0.001f;
	A_Iterm[num] = constrain( A_Iterm[num], -2000, 2000 );
		
	R_Target[num] = constrain( A_Pterm[num] + A_Iterm[num], -8000, 8000 );
	
	R_Error[num] = R_Target[num] - M2006_actSpeed[num] * R_Speed;
		
	R_Pterm[num] = R_Error[num] * P[IN][num];
	
	R_Iterm[num] += R_Error[num] * I[IN][num] * 0.001f;
	R_Iterm[num] = constrain(R_Iterm[num], -4000, 4000);
	if ( R_Error[num] == 0 || R_Pterm[num] * R_Iterm[num] < 0)  //强制让误差经过零时不再反向增长
		{
		  R_Iterm[num] = 0;
		}
		
	S_PIDTerm[num] = R_Pterm[num] + R_Iterm[num];
	S_PIDTerm[num] = constrain(S_PIDTerm[num], -8000, 8000);
}


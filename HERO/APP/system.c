#include "system.h"
#include "vision.h"
#include "filter.h"
extern int SystemMode;
static volatile uint32_t usTicks = 0;
extern int16_t Angle_Target;

extern float Gimbal_PIDterm[2],Chassis_PIDterm[4];

uint32_t currentTime = 0;
uint32_t loopTime_1ms=0;
uint32_t previousTime = 0;
uint16_t cycleTime = 0; 

PID_t PID;

short gyrox,gyroy,gyroz;	//陀螺仪原始数据
float pitch,roll,yaw,yaw_10;		//欧拉角
	
//限幅
float constrain(float amt, float low, float high)
{
	if (amt < low)
		return low;
	else if (amt > high)
		return high;
	else
		return amt;
}

int32_t constrain_int32(int32_t amt, int32_t low, int32_t high)
{
	if (amt < low)
		return low;
	else if (amt > high)
		return high;
	else
		return amt;
}

int16_t constrain_int16(int16_t amt, int16_t low, int16_t high)
{
	if (amt < low)
		return low;
	else if (amt > high)
		return high;
	else
		return amt;
}

int constrain_int(int amt,int low,int high)
{
	if (amt < low)
		return low;
	else if (amt > high)
		return high;
	else
		return amt;
}

//计数器初始化
static void cycleCounterInit(void)
{
	RCC_ClocksTypeDef clocks;
	RCC_GetClocksFreq(&clocks);
	usTicks = clocks.SYSCLK_Frequency / 1000000; 
}

//以微秒为单位返回系统时间
uint32_t micros(void)
{
	register uint32_t ms, cycle_cnt;
	do {
			ms = sysTickUptime;
			cycle_cnt = SysTick->VAL;
	} while (ms != sysTickUptime);
	return (ms * 1000) + (usTicks * 1000 - cycle_cnt
		) / usTicks;
}

//微秒级延时
void delay_us(uint32_t us)
{
	uint32_t now = micros();
	while (micros() - now < us);
}

//毫秒级延时
void delay_ms(uint32_t ms)
{
	while (ms--)
	delay_us(1000);
}
//以毫秒为单位返回系统时间
uint32_t millis(void)
{
	return sysTickUptime;
}

/*控制模式*/
eRemoteMode remoteMode = RC;

/*系统状态*/
SystemState systemState = SYSTEM_STARTING;

/*更新系统状态*/
void SYSTEM_UpdateSystemState()
{
	static  uint32_t  systime  =  0;
	
	if (systemState == SYSTEM_STARTING)
		{
			systime++;
			if (systime > 3000)
				{
					systime = 0;
					systemState = SYSTEM_RUNNING;
				}
		}
}

/*获取系统状态*/
SystemState Get_SystemState()
{
	return systemState;
}

void SYSTEM_Reset( void )
{
	systemState = SYSTEM_STARTING;
}
	
/*控制模式*/
void SystemRemote(void)
{
	if( RC_Ctl.rc.s2 == RC_SW_UP )
		{
			remoteMode = KEY;
		}
	else
		{
			remoteMode = RC;
		}
}

/*获取控制模式状态*/
eRemoteMode  Get_remoteMode(void)
{
	return remoteMode;
}


//系统初始化
void systemInit(void)
{
	cycleCounterInit();
	SysTick_Config(SystemCoreClock / 1000);	//滴答定时器配置，1ms
}

int SystemMonitor=Normal_Mode;
float openAngle;
extern float KEY_FRC42_SPEED;
void Stop()
{
	SYSTEM_Reset();
	
	Chassis_PIDterm[0] = 0;   //底盘电机
	Chassis_PIDterm[1] = 0;
	Chassis_PIDterm[2] = 0;
	Chassis_PIDterm[3] = 0;
	CAN1_Chassis_Send();
	
	Gimbal_PIDterm[YAW] = 0;  //云台
	Gimbal_PIDterm[PITCH] = 0;
	CAN1_Gimbal_Send();
	
	aimFrcSpeed[FRICTION_17] = 0;          //摩擦轮17
	RealFrcSpeed[FRICTION_17] = 0;
	friction17_PWM( RealFrcSpeed[FRICTION_17], RealFrcSpeed[FRICTION_17] );
	Fric17_ON_flag = 0;
	Fric42_ON_flag = 0;
	KEY_FRC42_SPEED = 37;
		
	aimFrcSpeed[FRICTION_42] = 0;          //摩擦轮42
	RealFrcSpeed[FRICTION_42] = 0;
	friction42_PWM( RealFrcSpeed[FRICTION_42], RealFrcSpeed[FRICTION_42] );
	
	S_PIDTerm[MOTIANLUN] = 0;
	S_PIDTerm[XIAOBOPAN] = 0;
	CAN2_Motor_Send();
	R_Iterm[MOTIANLUN] = 0;
	
	delay_ms(1000);

	XIANWEI_Angle( OPEN_ANGLE ); //限位

	Xianwei_Ready_Flag = 0; 
}

void Parameter_Init(void)
{
	GIMBAL_InitArgument();
	Chassis_InitArgument();
}

int pass_num;
bool pass_flag=1;
void System_Init(void)
{	
	static uint32_t loopTime_mpu6050 = 0;
	CRC_init();	
	cycleCounterInit();
	SysTick_Config(SystemCoreClock / 1000);
	
	Parameter_Init();
	M2006_RecordBombNum_Iint();
	usart2_Init();
	UART4_Init();
	usart5_Init();
	Magazine_Servo_Init();
	TIM3_Init();
	TIM4_Init();
	SuperCap_ADC_Init();
	SuperCap_DAC_Init();
	SuperCap_IO_Init();
	CAN1_Init();
	CAN2_Init();
	Led_Init();
	Laser_Init();
	relay_init();
	MPU_Init();
	while(mpu_dmp_init())//注意自检函数
	{
		currentTime = micros();//获取当前系统时间	
		if((int32_t)(currentTime - loopTime_mpu6050) >= 100000)  
		{	
			loopTime_mpu6050 = currentTime + 100000;			//100ms
			pass_num++;
			if(pass_num>=3)//若超时 则屏蔽自检函数
			{
				pass_flag=0;
				pass_num=10;
			}
		}
	}
}
float angleaaaa = 5;
uint16_t io_status;
float Duoji_yanshiTime = 35000, relay_opentime, xianweitime = 160000;
//主循环
void Loop(void)
{
	static uint32_t currentTime   = 0;
	static uint32_t loopTime_1ms  = 0;
	static uint32_t loopTime_2ms  = 0;
	static uint32_t FricTime_2ms  = 0;
	static uint32_t GimbalTime_3ms  = 0;
	static uint32_t loopTime_10ms = 0;
	static uint32_t loopTime_100ms = 0;
	static uint32_t loopTime_500ms = 0;
	static uint32_t loopTime_Shoot = 0;
	static uint32_t shootFrequencyTime42 = 0;
	static uint32_t shootFrequencyTime17 = 0;
	static uint32_t ProtectAngleTime_500ms = 0;
	
	currentTime = micros();	//获取当前系统时间
	
	if((int32_t)(currentTime - loopTime_1ms) >= 0)  
		{	
			loopTime_1ms = currentTime + 1000;	      //1ms		
			MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);	//读取角速度
			mpu_dmp_get_data(&roll,&pitch,&yaw);		  //读取欧拉角
			SYSTEM_UpdateSystemState();  	            //更新系统状态
			SystemRemote();                           //跟新系统控制模式	
		}
	
	if((int32_t)(currentTime - FricTime_2ms) >= 0)//武器系统控制
		{
			FricTime_2ms = currentTime + 2000;
			if(SystemMonitor == Normal_Mode)
				{
					FRIC_Control();
					Weapon_Control();
				}
		}
		
	if((int32_t)(currentTime - loopTime_2ms) >= 0)//底盘控制
		{
			loopTime_2ms = currentTime + 2000;
			if(SystemMonitor == Normal_Mode)
				{
					Chassis_control();
					ADC_VOLT = Get_Realvoltage();
					io_status = GPIO_ReadOutputDataBit(GPIOA,GPIO_Pin_5);
				}
		}
		
	if((int32_t)(currentTime - GimbalTime_3ms) >= 0)//云台控制
		{
			GimbalTime_3ms = currentTime + 3000;
			Temperature_Value = MPU_Get_Temperature();
			if(SystemMonitor == Normal_Mode)
				{
					Gimbal_Control();
					
					gimbaler_send_sentry();
				}
		}
		
	if((int32_t)(currentTime - loopTime_100ms) >= 0)
		{
			loopTime_100ms = currentTime + 100000;	
			//发送自定义数据到客户端
			Vision_Ctrl();           //发送视觉指令
			
			determine_ID();
			
			Send_Data_To_Judgement();//发送给客户端
		}
		
		
	if((int32_t)(currentTime - loopTime_500ms) >= 0)
		{
			loopTime_500ms = currentTime + 500000;
//			Tempeture_PID();
			
			Seng_Data_To_TeamMate(); //发送给队友数据
		}
		
		
	/*   大弹时间配合    */	
	if( ContinueTime_flag == 1 )
		{
			if( (int32_t)(currentTime - loopTime_Shoot) >= Duoji_yanshiTime ) //延时35ms,为了给舵机运动时间
				{
					OPEN_FIRE;
				}
			if( Get_remoteMode() == RC ) 
				{ relay_opentime = 18000; }//小摩擦轮高转速时，电磁铁也更有力，所以通电时间可以短点
			else
				{ 
					relay_opentime = 20000;  //低转速时，电磁铁也无力，通电时间长点，因为接的同个转压模块出来的12V(垃圾电磁铁不稳)
				}
				
			if((int32_t)(currentTime - loopTime_Shoot) >= Duoji_yanshiTime + relay_opentime)     
				{
					OFF_FIRE;
				}
			
			if( Motor_angel[PITCH] <= P_M_MID - 200 )//低头时
			{ xianweitime = 70000; }
			else if( Motor_angel[PITCH] >= P_M_MID + 300 )//抬头时
			{ xianweitime = 120000; }
			else 
			{ xianweitime = 90000; }
				
		  if((int32_t)(currentTime - loopTime_Shoot) >= xianweitime) //70ms，舵机抬起后停70ms
			{
				XIANWEI_Angle( CLOSE_ANGLE );
				ContinueTime_flag = 0;
				loopTime_Shoot = currentTime + xianweitime;
			}
		}
	else
		{
			loopTime_Shoot = currentTime;
		}
		
	if( shoot42_Frequency_flag == 1 )  //大弹射频限制
		{
			if( (int32_t)(currentTime - shootFrequencyTime42) >= 600000 ) //延时600ms
			{
				shoot42_Frequency_flag = 0;
				shootFrequencyTime42 = currentTime + 600000;
			}
		}
	else
		{
			shootFrequencyTime42 = currentTime;
		}
		
	if( shoot17_Frequency_flag == 1 ) //小弹射频限制
		{
			if( (int32_t)(currentTime - shootFrequencyTime17) >= 200000 ) //延时200ms
			{
				shoot17_Frequency_flag = 0;
				shootFrequencyTime17 = currentTime + 200000;
			}
		}
	else
		{
			shootFrequencyTime17 = currentTime;
		}
		
	if( Protect_Angle_flag == 1 ) //受到伤害时的自动闪避
		{
			if( (int32_t)(currentTime - ProtectAngleTime_500ms) >= 500000 ) //延时500ms
			{
				Protect_Angle_flag = 0;
				ProtectAngleTime_500ms = currentTime + 500000;
			}
		}
	else
		{
			ProtectAngleTime_500ms = currentTime;
		}
		
}











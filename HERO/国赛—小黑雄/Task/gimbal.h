#ifndef _GIMBAL__H
#define _GIMBAL__H
#include "system.h" 


/*云台角度值*/
#define  Y_M_MIN   (2050)
#define  Y_M_MID   (4102)
#define  Y_M_MAX   (6100)
#define  DODGE_ANGLE   ( (Y_M_MAX + Y_M_MID) * 1 / 2  )
	
#define  P_M_MIN   (3550)
#define  P_M_MID   (4082)
#define  P_M_MAX   (4940)  //抬最高
#define  CANNOT_XIAOBOMB   (P_M_MIN + 100)
	
	
/*摩擦轮抬头*/
#define    FRC_MACH_PIT    (P_M_MAX - 300)

	
#define  PITCH     0  
#define  YAW       1
	
#define  OUT       0
#define  IN        1
	
#define  M         0
#define  G         1
#define  Auto      2
	
#define  NOW       0
#define  LAST      1

/*       临界值处理结构体       */
typedef struct 
{

	float LastAngle;//上一次读取到的数据
	float CurAngle;	//当前读取到的角度
	float AngleSum;	//角度累加值
	
}Critical_t;



void Gimbal_Control(void);
void Gimbal_InitControl(void);
void GIMBAL_InitArgument(void);
void RC_Gimbal_control(void);
void KEY_Gimbal_control(void);
	
void YAW_Mach_PID(void);
void YAW_Gyro_PID(void);
void YAW_Auto_PID(void);
void PITCH_Mach_PID(void);
void PITCH_Auto_PID(void);

int16_t Gimbal_GetAngleError(void);
bool GIMBAL_IfAutoHit(void);
void GIMBAL_AUTO_Mode_Ctrl(void);
void GIMBAL_BASE_Mode_Ctrl(void);
void GIMBAL_KalmanParameter_Init(void);
void KlamanCalc(void);

float slope_Control(float final,float aimAngle,int tilt);
float GIMBAL_ActionMode(void);
int16_t Get_Attack_attention_Mode(void);

void Critical_Handle_Init(Critical_t *critical, float get);
float Critical_Handle(Critical_t *critical, float get);
	

extern bool Mobi_Pre_Yaw_Fire;
extern float debug_yaw_speed, debug_pitch_speed;
extern float debug_kf_yaw_angle;
extern bool Mobility_Prediction_Yaw;
extern bool Base_Yaw_ready_flag;

#endif


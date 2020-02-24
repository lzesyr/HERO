#ifndef __CHASSIS_H
#define __CHASSIS_H
#include"system.h"


void Chassis_control(void);
void Cha_RC_control(void);
void Cha_KEY_control(void);
void Cha_Motor_speed(float vx, float vy, float vw);
void chassis_PID(int num,float actspeed,float aimspeed);
void Cha_PowerLimit(void);
void Chassis_InitArgument(void);
float Get_Chassis_Mode(void);

extern float Speed_target_all;
extern uint8_t yunsuflag;
extern bool change_F_flag, change_B_flag, Protect_Angle_flag, chassis_Follow_flag;

#endif


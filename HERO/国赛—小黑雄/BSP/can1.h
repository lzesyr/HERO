#ifndef __CAN1_H
#define __CAN1_H

#include "system.h"

extern float Gimbal_PIDterm[2];   //电机输出值
extern int16_t Motor_angel[2], Motor_speed[2];    //电机实际角度值,速度
extern bool Cha_Normal_flag1, Cha_Normal_flag2, Cha_Normal_flag3, Cha_Normal_flag4;
extern bool Gim_Yaw_Normal_flag, Gim_Pitch_Normal_flag;
extern int Cha_Normal_Num1, Cha_Normal_Num2, Cha_Normal_Num3, Cha_Normal_Num4;
extern int Gim_Yaw_Normal_Num, Gim_Pitch_Normal_Num;

void CAN1_Init(void);
void CAN1_Chassis_Send(void);	
void CAN1_Gimbal_Send(void);

#endif


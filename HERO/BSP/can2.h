#ifndef __CAN2_H__
#define __CAN2_H__

#include "system.h"

typedef struct
{
		int16_t Begin[2];
		int16_t Current[2];
    int16_t last[2];
		int32_t Total[2];
	  float Sum[2];
	  float Target[2];
		float Count[2];
}RM06_Angle_t;

extern RM06_Angle_t M06_Angle;
extern float S_PIDTerm[2];
extern int16_t M2006_actAngle[2], M2006_actSpeed[2];
extern int  Motianlun_Roll_Num;

void CAN2_Init(void);
void CAN2_Motor_Send(void);
void circle_count(int num);

#endif 

#ifndef _FRICTION__H
#define _FRICTION__H

#include "system.h" 

#define  FRICTION_17  0
#define  FRICTION_42  1

void FRIC_Control(void);
void Fric_RC_Control(void);
void Fric_KEY_Control(void);
void IF_Fric_Ready(void);

extern bool Fric17_ready, Fric42_ready, Fric17_ON_flag, Fric42_ON_flag;
extern int16_t aimFrcSpeed[2], RealFrcSpeed[2]; 
extern bool Xianwei_Ready_Flag;

#endif



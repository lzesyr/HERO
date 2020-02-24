#ifndef _WEAPON__H
#define _WEAPON__H

#include "system.h"

#define   XIAOBOPAN    0
#define   MOTIANLUN    1

#define   BOMB_42      0
#define   BOMB_17      1

#define   BIGBOMB_MODE   1    //大弹模式
#define   XIAOBOMB_MODE  0    //小弹模式

#define   OPEN_ANGLE   70
#define   CLOSE_ANGLE  102

#define   OPEN_FIRE    GPIO_SetBits(GPIOB,GPIO_Pin_1)     
#define   OFF_FIRE     GPIO_ResetBits(GPIOB, GPIO_Pin_1) 

#define   OPEN_XIANWEI GPIO_SetBits(GPIOB, GPIO_Pin_9) 
#define   OFF_XIANWEI  GPIO_ResetBits(GPIOB, GPIO_Pin_9)

void Weapon_Control(void);
void Weapon_RC_Control(void);
void Weapon_KEY_Control(void);

void Motianlun_Control(void);
void XiaoBomb_Control(void);
void relay_init(void);
void M2006_RecordBombNum_Iint(void);
void M2006_Speed_PID(int num);
void M2006_Position_PID(int num);

void ShotBomb_Count(void);
void Muzzle_Heat_Limit(void);

extern float R_Iterm[3];
extern bool ContinueTime_flag;
extern int16_t Shoot_State;   //射击模式（包含大弹模式、小弹模式）
extern bool shoot42_Frequency_flag, shoot17_Frequency_flag, Motianlun_Roll_flag, Yuzhi_TwoBomb_flag;

#endif


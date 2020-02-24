#ifndef __PWM_H__
#define __PWM_H__

#include "system.h"
#include "stm32f4xx.h"

void TIM1_Init(void);
void TIM3_Init(void);
void TIM4_Init(void);

#define PWM1_42  TIM4->CCR1
#define PWM2_42  TIM4->CCR2

#define PWM1_17  TIM3->CCR1
#define PWM2_17  TIM3->CCR2

void friction42_PWM( int16_t pwm1, int16_t pwm2 );
void friction17_PWM( int16_t pwm1, int16_t pwm2 );
void XIANWEI_Angle( int16_t pwm );

void Magazine_Servo_Init(void);

#endif

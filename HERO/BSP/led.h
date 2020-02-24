#ifndef __LED_H
#define __LED_H

#include "system.h"

#define Blue_On  	 GPIO_ResetBits(GPIOC,GPIO_Pin_13)
#define Blue_Off  	 GPIO_SetBits(GPIOC,GPIO_Pin_13)

#define Orange_On			 GPIO_ResetBits(GPIOC,GPIO_Pin_14)
#define Orange_Off	     GPIO_SetBits(GPIOC,GPIO_Pin_14)

#define Visual_On		 GPIO_SetBits(GPIOA,GPIO_Pin_8)
#define Visual_Off	 GPIO_ResetBits(GPIOA,GPIO_Pin_8)


#define Laser_On GPIO_SetBits(GPIOD,GPIO_Pin_9)
#define Laser_Off GPIO_ResetBits(GPIOD,GPIO_Pin_9)

void Laser_Init(void);
void Led_Init(void);
#endif


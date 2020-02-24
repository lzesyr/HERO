#include "led.h"

void Led_Init()
{
		GPIO_InitTypeDef gpio;	
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA|RCC_AHB1Periph_GPIOC,ENABLE);

		gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
		gpio.GPIO_Speed = GPIO_Speed_50MHz;	
		gpio.GPIO_Mode = GPIO_Mode_OUT;										
		gpio.GPIO_Pin = GPIO_Pin_13|GPIO_Pin_14;  		
		GPIO_Init(GPIOC,&gpio);	

		gpio.GPIO_Pin = GPIO_Pin_8;		
		GPIO_Init(GPIOA,&gpio);	

		Visual_Off;
		Blue_Off;
		Orange_Off;
}

void Laser_Init()
{
		GPIO_InitTypeDef  GPIO_InitStructure;
	
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);//使能GPIOC时钟

		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;//上拉
		GPIO_Init(GPIOD, &GPIO_InitStructure);//初始化
	
		Laser_On;
}



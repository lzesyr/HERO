#include "pwm.h"

void TIM4_Init(void)	
{
	GPIO_InitTypeDef          gpio;
	TIM_TimeBaseInitTypeDef   tim;
	TIM_OCInitTypeDef         oc;
		
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);	

	gpio.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13;
	gpio.GPIO_Mode = GPIO_Mode_AF;
	gpio.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOD, &gpio);
		
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource12, GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource13, GPIO_AF_TIM4); 
		
	tim.TIM_Prescaler = 560;
	tim.TIM_CounterMode = TIM_CounterMode_Up;		//向上计数
	tim.TIM_Period = 2499;                      //25ms	计数周期  PWM频率=84M/84/2500
	tim.TIM_ClockDivision = TIM_CKD_DIV1;		    //设置始终分割
	TIM_TimeBaseInit(TIM4,&tim);
		
	oc.TIM_OCMode = TIM_OCMode_PWM2;		          //选择定时器模式
	oc.TIM_OutputState = TIM_OutputState_Enable;	//选择输出比较状态
	oc.TIM_OutputNState = TIM_OutputState_Disable;//选择互补输出比较状态
	oc.TIM_Pulse = 0;		                          //设置待装入捕获比较器的脉冲值
	oc.TIM_OCPolarity = TIM_OCPolarity_Low;		    //设置输出极性
	oc.TIM_OCNPolarity = TIM_OCPolarity_High;		  //设置互补输出极性
	oc.TIM_OCIdleState = TIM_OCIdleState_Reset;		//选择空闲状态下的非工作状态
	oc.TIM_OCNIdleState = TIM_OCIdleState_Set;		//选择互补空闲状态下的非工作状态
	TIM_OC1Init(TIM4,&oc);		//通道1
	TIM_OC1PreloadConfig(TIM4,TIM_OCPreload_Enable);
		
	TIM_OC2Init(TIM4,&oc);		//通道2
	TIM_OC2PreloadConfig(TIM4,TIM_OCPreload_Enable);
					 
	TIM_ARRPreloadConfig(TIM4,ENABLE);
		
	TIM_CtrlPWMOutputs(TIM4,ENABLE);
		
	TIM_Cmd(TIM4,ENABLE);
}

uint16_t PWM_DEBUG = 165;
void friction42_PWM( int16_t pwm1, int16_t pwm2 )
{
	PWM1_42 = pwm1 + PWM_DEBUG;
	PWM2_42 = pwm2 + PWM_DEBUG;
}


void TIM3_Init(void)
{
	GPIO_InitTypeDef           gpio;
	TIM_TimeBaseInitTypeDef    tim;
	TIM_OCInitTypeDef          oc;
		
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);	

	gpio.GPIO_Pin   = GPIO_Pin_6 | GPIO_Pin_7;
	gpio.GPIO_Mode  = GPIO_Mode_AF;
	gpio.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOA, &gpio);
		
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_TIM3);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_TIM3); 
		
	tim.TIM_Prescaler = 84-1;
	tim.TIM_CounterMode = TIM_CounterMode_Up;		  //向上计数
	tim.TIM_Period = 2499;                        //25ms	计数周期  PWM频率=84M/84/2500
	tim.TIM_ClockDivision = TIM_CKD_DIV1;		      //设置始终分割
	TIM_TimeBaseInit(TIM3,&tim);
		
	oc.TIM_OCMode       = TIM_OCMode_PWM2;		    //选择定时器模式
	oc.TIM_OutputState  = TIM_OutputState_Enable;	//选择输出比较状态
	oc.TIM_OutputNState = TIM_OutputState_Disable;//选择互补输出比较状态
	oc.TIM_Pulse        = 0;		                  //设置待装入捕获比较器的脉冲值
	oc.TIM_OCPolarity   = TIM_OCPolarity_Low;		  //设置输出极性
	oc.TIM_OCNPolarity  = TIM_OCPolarity_High;		//设置互补输出极性
	oc.TIM_OCIdleState  = TIM_OCIdleState_Reset;  //选择空闲状态下的非工作状态
	oc.TIM_OCNIdleState = TIM_OCIdleState_Set;		//选择互补空闲状态下的非工作状态
	TIM_OC1Init(TIM3,&oc);		//通道1
	TIM_OC1PreloadConfig(TIM3,TIM_OCPreload_Enable);
		
	TIM_OC2Init(TIM3,&oc);		//通道2
	TIM_OC2PreloadConfig(TIM3,TIM_OCPreload_Enable);
					 
	TIM_ARRPreloadConfig(TIM3,ENABLE);
		
	TIM_CtrlPWMOutputs(TIM3,ENABLE);
		
	TIM_Cmd(TIM3,ENABLE);
	PWM1_17 = 1000;
	PWM2_17 = 1000;
}

void friction17_PWM( int16_t pwm1, int16_t pwm2 )
{
	PWM1_17 = pwm1 + 1000;
	PWM2_17 = pwm2 + 1000;
}


/**
  * @brief  ???????   TIM1 CH2
  * @param  void   
  * @retval void
  */
void Magazine_Servo_Init(void)
{
	GPIO_InitTypeDef          gpio;
	TIM_TimeBaseInitTypeDef   tim;
	TIM_OCInitTypeDef         oc;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);		

	gpio.GPIO_Pin = GPIO_Pin_11;
	gpio.GPIO_Mode = GPIO_Mode_AF;
	gpio.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOE,&gpio);
	
	GPIO_PinAFConfig(GPIOE,GPIO_PinSource11, GPIO_AF_TIM1);     
	
	tim.TIM_Prescaler     = 3360-1;
	tim.TIM_CounterMode   = TIM_CounterMode_Up;	 //????
	tim.TIM_Period        = 999;                 //20ms	????
	tim.TIM_ClockDivision = TIM_CKD_DIV1;		     //????,??1?*2
	TIM_TimeBaseInit(TIM1,&tim);
	
	oc.TIM_OCMode = TIM_OCMode_PWM2;		//PWM2??
	oc.TIM_OutputState = TIM_OutputState_Enable;		//??????
	oc.TIM_OutputNState = TIM_OutputState_Disable;	//????????
	oc.TIM_Pulse = 0;		//?????
	oc.TIM_OCPolarity = TIM_OCPolarity_Low;		//?????
	oc.TIM_OCNPolarity = TIM_OCPolarity_High;		//???????
	oc.TIM_OCIdleState = TIM_OCIdleState_Reset;		
	oc.TIM_OCNIdleState = TIM_OCIdleState_Set;		
	
	TIM_OC2Init(TIM1,&oc);		//??2
	TIM_OC2PreloadConfig(TIM1,TIM_OCPreload_Enable);
				 
	TIM_ARRPreloadConfig(TIM1,ENABLE);
	
	TIM_CtrlPWMOutputs(TIM1,ENABLE);
	
	TIM_Cmd(TIM1,ENABLE);
	
	TIM1->CCR2 = CLOSE_ANGLE;
}

void XIANWEI_Angle( int16_t pwm )
{
	pwm = abs(pwm);
	TIM1->CCR2 = pwm;
}









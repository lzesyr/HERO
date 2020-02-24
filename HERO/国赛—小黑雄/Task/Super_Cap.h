#ifndef _SuperCap__H
#define _SuperCap__H
#include "system.h"

void  SuperCap_ADC_Init(void);		 //ADC通道初始化
float Get_Realvoltage(void);
u16 Get_Adc_Average(u8 times);

void SuperCap_DAC_Init(void);      //DAC通道初始化
void Dac1_Set_Vol(u16 vol);

void SuperCap_Charge_Control(void);     //超级电容控制
void SuperCap_Power_PID(void);
void SuperCap_Charge_on(void);
void SuperCap_Output(void);
void SuperCap_IO_Init(void);

#define Charge_On		 GPIO_SetBits(GPIOA,GPIO_Pin_2)
#define Charge_Off	 GPIO_ResetBits(GPIOA,GPIO_Pin_2)

#define CAP_OUT_On	 GPIO_SetBits(GPIOA,GPIO_Pin_5)
#define CAP_OUT_Off	 GPIO_ResetBits(GPIOA,GPIO_Pin_5)

extern int8_t cap_output_flag;
extern float ADC_VOLT;

#endif


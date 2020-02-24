#include "Super_Cap.h"

float realtimePower, realPowerError, realtimePower_P=12, realtimePower_I=8, realtimePower_D=0.0001, charge_xishu = 1;;
float realtimePterm, realtimeIterm, realtimeDterm, cap_radio, Chass_Realtime_DAC_SetCurrent, ADC_VOLT;
int8_t cap_output_flag = 0;

void SuperCap_Charge_Control(void)
{
	if( Get_SystemState() == SYSTEM_STARTING )
		{
			Charge_Off;   //起始状态下停止充放电
			CAP_OUT_Off;
		}
	else
		{
			if( IF_KEY_PRESSED_SHIFT )
				{
					SuperCap_Output();
				}
			else
				{
					CAP_OUT_Off;
					cap_output_flag = 0;
				}	
			
			SuperCap_Charge_on();
		}
}

void SuperCap_Output(void)
{	
//	Charge_Off; //关闭充电,目前做不到边放边充
	
	if( ADC_VOLT > 12 )  //当电容电压大于12V时，放电
		{
			CAP_OUT_On;   //开启放电
			cap_output_flag = 1;
		}
	else
		{
			CAP_OUT_Off;  //电压小于12V时停止放电
			cap_output_flag = 0;
		}
}

void SuperCap_Charge_on(void)
{
//	CAP_OUT_Off; //禁止放电
	Charge_On; //开启充电
		
	SuperCap_Power_PID();
	Dac1_Set_Vol(Chass_Realtime_DAC_SetCurrent);//电压输出
}

void SuperCap_Power_PID(void)
{
	static float realtimelastError = 0;
	
		//如果静止的话
	if(Speed_target_all == 0)
	{
		cap_radio = 1;
		realtimePower_P = 12;
		realtimePower_I = 8;
	}
	//运动的时候
	if( IF_KEY_PRESSED_SHIFT )
	{
		cap_radio = 1;
		realtimePower_P = 12;
		realtimePower_I = 8;
	}
	else
	{
		if(yunsuflag == 0 && Speed_target_all!=0 )//非匀速
			{
				cap_radio=7900.f / abs((float)(Speed_target_all + 1));
				cap_radio=constrain(cap_radio,0,1);
				realtimePower_P = 15;
				realtimePower_I = 10;
			}
		else if(yunsuflag == 1 && Speed_target_all!=0 )//匀速运动
			{
				cap_radio=7900.f / abs((float)(Speed_target_all + 1)) * 2;
				cap_radio=constrain(cap_radio,0,1);
				realtimePower_P = 15;
				realtimePower_I = 9;
			}
	}
	
	realPowerError =  75 - realtimePower;  //计算剩余可用功率
	
	realtimePterm  =  realPowerError * realtimePower_P;
	
	realtimeIterm  += realPowerError * realtimePower_I * 0.02f;
	realtimeIterm  =  constrain_int32( realtimeIterm, -2000, 2000 );
	
	realtimeDterm  = ( realPowerError - realtimelastError ) * realtimePower_D / 0.02f;
	
	realtimelastError = realPowerError;
	
	if( PowerProtect.power_remain < 50 || ADC_VOLT >= 23.5f )  //功率缓存小于50时强制使电容充电电压为0,不抢底盘功率
		{
			Chass_Realtime_DAC_SetCurrent = 0;
			Charge_Off;
		}	
	Chass_Realtime_DAC_SetCurrent = ( realtimePterm + realtimeIterm + realtimeDterm ) * cap_radio;
	Chass_Realtime_DAC_SetCurrent = constrain_int32(Chass_Realtime_DAC_SetCurrent/3.f,0,1000); 
	Chass_Realtime_DAC_SetCurrent = 1270 + Chass_Realtime_DAC_SetCurrent / 2;
	Chass_Realtime_DAC_SetCurrent = constrain_int32( Chass_Realtime_DAC_SetCurrent, 1270, 1600 );//8A
}


/***********************  ADC  **************************/
void  SuperCap_ADC_Init(void)
{
	GPIO_InitTypeDef  GPIO_ADCInitStructure;
	ADC_CommonInitTypeDef ADC_CommonInitStructure;
	ADC_InitTypeDef       ADC_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);//使能GPIOA时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE); //使能ADC1时钟

	//先初始化ADC1通道10 IO口
	GPIO_ADCInitStructure.GPIO_Pin = GPIO_Pin_0;//PC0 通道10
	GPIO_ADCInitStructure.GPIO_Mode = GPIO_Mode_AN;//模拟输入
	GPIO_ADCInitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;//不带上下拉
	GPIO_Init(GPIOC, &GPIO_ADCInitStructure);//初始化  

	RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1,ENABLE);	  //ADC1复位
	RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1,DISABLE);	//复位结束	 

	ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;//独立模式
	ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;//两个采样阶段之间的延迟5个时钟
	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled; //DMA失能
	ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div4;//预分频4分频。ADCCLK=PCLK2/4=84/4=21Mhz,ADC时钟最好不要超过36Mhz 
	ADC_CommonInit(&ADC_CommonInitStructure);//初始化

	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;//12位模式
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;//非扫描模式	
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;//关闭连续转换
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;//禁止触发检测，使用软件触发
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;//右对齐	
	ADC_InitStructure.ADC_NbrOfConversion = 1;//1个转换在规则序列中 也就是只转换规则序列1 
	ADC_Init(ADC1, &ADC_InitStructure);//ADC初始化

	ADC_Cmd(ADC1, ENABLE);//开启AD转换器
}

//获取电压值
float value = 0;
float Get_Realvoltage()
{
	value = Get_Adc_Average(10);		//取10次平均值
	value = (float)value*(3.3/4096);
	return (value*11);
}

//获得ADC值
//返回值:转换结果
u16 Get_Adc()   
{
	//设置指定ADC的规则组通道，一个序列，采样时间
	ADC_RegularChannelConfig(ADC1, 10, 1, ADC_SampleTime_480Cycles );	//ADC1,ADC通道10,480个周期,提高采样时间可以提高精确度			    
  
	ADC_SoftwareStartConv(ADC1);		//使能指定的ADC1的软件转换启动功能	
	 
	return ADC_GetConversionValue(ADC1);	//返回最近一次ADC1规则组的转换结果

}

//获取通道ch的转换值，取times次,然后平均 
//times:获取次数
//返回值:通道ch的times次转换结果平均值
u16 Get_Adc_Average(u8 times)
{
	u32 temp_val=0;
	u8 t;
	for(t=0;t<times;t++)
	{
		temp_val+=Get_Adc();
	}
	return temp_val/times;
} 

/****************************  IO口初始化  ********************************/
void SuperCap_IO_Init(void)
{
	GPIO_InitTypeDef gpioa;	
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
	
	gpioa.GPIO_PuPd  = GPIO_PuPd_DOWN;
	gpioa.GPIO_Speed = GPIO_Speed_50MHz;	
	gpioa.GPIO_Mode  = GPIO_Mode_OUT;		
	gpioa.GPIO_Pin   = GPIO_Pin_5|GPIO_Pin_2;		
	GPIO_Init(GPIOA,&gpioa);	
	
	Charge_Off;
	CAP_OUT_Off;
}
/*******************************  DAC  ************************************/
void SuperCap_DAC_Init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	DAC_InitTypeDef   DAC_InitType;
	
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//使能GPIOA时钟
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE);//使能DAC时钟

	
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;//模拟
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
  GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化

	DAC_InitType.DAC_Trigger=DAC_Trigger_None;	//不使用触发功能 TEN1=0
	DAC_InitType.DAC_WaveGeneration=DAC_WaveGeneration_None;//不使用波形发生
	DAC_InitType.DAC_LFSRUnmask_TriangleAmplitude=DAC_LFSRUnmask_Bit0;//屏蔽、幅值设置
	DAC_InitType.DAC_OutputBuffer=DAC_OutputBuffer_Disable ;	//DAC1输出缓存关闭 BOFF1=1
  DAC_Init(DAC_Channel_1,&DAC_InitType);	 //初始化DAC通道1

	DAC_Cmd(DAC_Channel_1, ENABLE);  //使能DAC通道1
  
  DAC_SetChannel1Data(DAC_Align_12b_R, 0);  //12位右对齐数据格式设置DAC值
}

//设置通道1输出电压
//vol:0~3300,代表0~3.3V
void Dac1_Set_Vol(u16 vol)
{
	double temp=vol;
	temp/=1000;
	temp=temp*4096/3.3;
	DAC_SetChannel1Data(DAC_Align_12b_R,temp);//12位右对齐数据格式设置DAC值
}


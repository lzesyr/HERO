#include "Super_Cap.h"

float realtimePower, realPowerError, realtimePower_P=12, realtimePower_I=8, realtimePower_D=0.0001, charge_xishu = 1;;
float realtimePterm, realtimeIterm, realtimeDterm, cap_radio, Chass_Realtime_DAC_SetCurrent, ADC_VOLT;
int8_t cap_output_flag = 0;

void SuperCap_Charge_Control(void)
{
	if( Get_SystemState() == SYSTEM_STARTING )
		{
			Charge_Off;   //��ʼ״̬��ֹͣ��ŵ�
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
//	Charge_Off; //�رճ��,Ŀǰ�������߷ű߳�
	
	if( ADC_VOLT > 12 )  //�����ݵ�ѹ����12Vʱ���ŵ�
		{
			CAP_OUT_On;   //�����ŵ�
			cap_output_flag = 1;
		}
	else
		{
			CAP_OUT_Off;  //��ѹС��12Vʱֹͣ�ŵ�
			cap_output_flag = 0;
		}
}

void SuperCap_Charge_on(void)
{
//	CAP_OUT_Off; //��ֹ�ŵ�
	Charge_On; //�������
		
	SuperCap_Power_PID();
	Dac1_Set_Vol(Chass_Realtime_DAC_SetCurrent);//��ѹ���
}

void SuperCap_Power_PID(void)
{
	static float realtimelastError = 0;
	
		//�����ֹ�Ļ�
	if(Speed_target_all == 0)
	{
		cap_radio = 1;
		realtimePower_P = 12;
		realtimePower_I = 8;
	}
	//�˶���ʱ��
	if( IF_KEY_PRESSED_SHIFT )
	{
		cap_radio = 1;
		realtimePower_P = 12;
		realtimePower_I = 8;
	}
	else
	{
		if(yunsuflag == 0 && Speed_target_all!=0 )//������
			{
				cap_radio=7900.f / abs((float)(Speed_target_all + 1));
				cap_radio=constrain(cap_radio,0,1);
				realtimePower_P = 15;
				realtimePower_I = 10;
			}
		else if(yunsuflag == 1 && Speed_target_all!=0 )//�����˶�
			{
				cap_radio=7900.f / abs((float)(Speed_target_all + 1)) * 2;
				cap_radio=constrain(cap_radio,0,1);
				realtimePower_P = 15;
				realtimePower_I = 9;
			}
	}
	
	realPowerError =  75 - realtimePower;  //����ʣ����ù���
	
	realtimePterm  =  realPowerError * realtimePower_P;
	
	realtimeIterm  += realPowerError * realtimePower_I * 0.02f;
	realtimeIterm  =  constrain_int32( realtimeIterm, -2000, 2000 );
	
	realtimeDterm  = ( realPowerError - realtimelastError ) * realtimePower_D / 0.02f;
	
	realtimelastError = realPowerError;
	
	if( PowerProtect.power_remain < 50 || ADC_VOLT >= 23.5f )  //���ʻ���С��50ʱǿ��ʹ���ݳ���ѹΪ0,�������̹���
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

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);//ʹ��GPIOAʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE); //ʹ��ADC1ʱ��

	//�ȳ�ʼ��ADC1ͨ��10 IO��
	GPIO_ADCInitStructure.GPIO_Pin = GPIO_Pin_0;//PC0 ͨ��10
	GPIO_ADCInitStructure.GPIO_Mode = GPIO_Mode_AN;//ģ������
	GPIO_ADCInitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;//����������
	GPIO_Init(GPIOC, &GPIO_ADCInitStructure);//��ʼ��  

	RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1,ENABLE);	  //ADC1��λ
	RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1,DISABLE);	//��λ����	 

	ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;//����ģʽ
	ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;//���������׶�֮����ӳ�5��ʱ��
	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled; //DMAʧ��
	ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div4;//Ԥ��Ƶ4��Ƶ��ADCCLK=PCLK2/4=84/4=21Mhz,ADCʱ����ò�Ҫ����36Mhz 
	ADC_CommonInit(&ADC_CommonInitStructure);//��ʼ��

	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;//12λģʽ
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;//��ɨ��ģʽ	
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;//�ر�����ת��
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;//��ֹ������⣬ʹ���������
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;//�Ҷ���	
	ADC_InitStructure.ADC_NbrOfConversion = 1;//1��ת���ڹ��������� Ҳ����ֻת����������1 
	ADC_Init(ADC1, &ADC_InitStructure);//ADC��ʼ��

	ADC_Cmd(ADC1, ENABLE);//����ADת����
}

//��ȡ��ѹֵ
float value = 0;
float Get_Realvoltage()
{
	value = Get_Adc_Average(10);		//ȡ10��ƽ��ֵ
	value = (float)value*(3.3/4096);
	return (value*11);
}

//���ADCֵ
//����ֵ:ת�����
u16 Get_Adc()   
{
	//����ָ��ADC�Ĺ�����ͨ����һ�����У�����ʱ��
	ADC_RegularChannelConfig(ADC1, 10, 1, ADC_SampleTime_480Cycles );	//ADC1,ADCͨ��10,480������,��߲���ʱ�������߾�ȷ��			    
  
	ADC_SoftwareStartConv(ADC1);		//ʹ��ָ����ADC1�����ת����������	
	 
	return ADC_GetConversionValue(ADC1);	//�������һ��ADC1�������ת�����

}

//��ȡͨ��ch��ת��ֵ��ȡtimes��,Ȼ��ƽ�� 
//times:��ȡ����
//����ֵ:ͨ��ch��times��ת�����ƽ��ֵ
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

/****************************  IO�ڳ�ʼ��  ********************************/
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
	
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//ʹ��GPIOAʱ��
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE);//ʹ��DACʱ��

	
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;//ģ��
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
  GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��

	DAC_InitType.DAC_Trigger=DAC_Trigger_None;	//��ʹ�ô������� TEN1=0
	DAC_InitType.DAC_WaveGeneration=DAC_WaveGeneration_None;//��ʹ�ò��η���
	DAC_InitType.DAC_LFSRUnmask_TriangleAmplitude=DAC_LFSRUnmask_Bit0;//���Ρ���ֵ����
	DAC_InitType.DAC_OutputBuffer=DAC_OutputBuffer_Disable ;	//DAC1�������ر� BOFF1=1
  DAC_Init(DAC_Channel_1,&DAC_InitType);	 //��ʼ��DACͨ��1

	DAC_Cmd(DAC_Channel_1, ENABLE);  //ʹ��DACͨ��1
  
  DAC_SetChannel1Data(DAC_Align_12b_R, 0);  //12λ�Ҷ������ݸ�ʽ����DACֵ
}

//����ͨ��1�����ѹ
//vol:0~3300,����0~3.3V
void Dac1_Set_Vol(u16 vol)
{
	double temp=vol;
	temp/=1000;
	temp=temp*4096/3.3;
	DAC_SetChannel1Data(DAC_Align_12b_R,temp);//12λ�Ҷ������ݸ�ʽ����DACֵ
}


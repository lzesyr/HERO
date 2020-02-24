#include "can1.h"

 float Gimbal_PIDterm[2];
int16_t Motor_angel[2], Motor_speed[2], Motor_chassis[4][2];
extern float Chassis_PIDterm[4];

void CAN1_Init()
{
	GPIO_InitTypeDef gpio;
	NVIC_InitTypeDef nvic;
	CAN_InitTypeDef can;
	CAN_FilterInitTypeDef filter;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1,ENABLE);

	gpio.GPIO_Mode=GPIO_Mode_AF;
	gpio.GPIO_Pin=GPIO_Pin_12|GPIO_Pin_11;
	GPIO_Init(GPIOA,&gpio);

	GPIO_PinAFConfig(GPIOA,GPIO_PinSource11,GPIO_AF_CAN1);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource12,GPIO_AF_CAN1);

//	nvic.NVIC_IRQChannel=CAN1_TX_IRQn;
//	nvic.NVIC_IRQChannelPreemptionPriority=1;
//	nvic.NVIC_IRQChannelSubPriority=1;
//	nvic.NVIC_IRQChannelCmd=ENABLE;
//	NVIC_Init(&nvic);

	nvic.NVIC_IRQChannel = CAN1_RX0_IRQn;
  nvic.NVIC_IRQChannelPreemptionPriority = 0;
  nvic.NVIC_IRQChannelSubPriority = 1;
  nvic.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&nvic);
	
	can.CAN_TTCM = DISABLE;		//��ʱ�䴥��ͨ��ģʽ
  can.CAN_ABOM = DISABLE;		//����Զ����߹���
  can.CAN_AWUM = DISABLE;		//˯��ģʽͨ���������(���CAN->MCR��SLEEPλ)
  can.CAN_NART = DISABLE;		//��ֹ�����Զ����� ���߸������һ��CAN ��Ӱ�췢�� ��ʱ�ɸ�ΪENABLE
  can.CAN_RFLM = DISABLE;		//���Ĳ��������µĸ��Ǿɵ�
  can.CAN_TXFP = ENABLE;		//���ȼ��ɱ��ı�ʶ������
	can.CAN_BS1=CAN_BS1_9tq;
	can.CAN_BS2=CAN_BS2_4tq;
	can.CAN_Mode=CAN_Mode_Normal;
	can.CAN_Prescaler=3;
	can.CAN_SJW=CAN_SJW_1tq;
	CAN_Init(CAN1,&can);
	
	filter.CAN_FilterNumber=0;  							 			//������0
	filter.CAN_FilterMode=CAN_FilterMode_IdMask;   	//����ģʽ
	filter.CAN_FilterScale=CAN_FilterScale_32bit;   //32λ��
	filter.CAN_FilterFIFOAssignment=0;              //������0������FIFO0
	filter.CAN_FilterActivation=ENABLE;   				  //���������
	filter.CAN_FilterIdHigh=0x0000;                 //32λID
	filter.CAN_FilterIdLow=0x0000;
	filter.CAN_FilterMaskIdHigh=0x0000;             //32λMask
	filter.CAN_FilterMaskIdLow=0x0000;
	CAN_FilterInit(&filter);
	
	CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);    ////FIFO0��Ϣ�Һ��ж�����
//	CAN_ITConfig(CAN1,CAN_IT_TME,ENABLE);
}

void CAN1_Chassis_Send()		//���̵��
{		
	CanTxMsg TxMessage;
	
	TxMessage.StdId = 0x200;					 //ʹ�õ���չID�����820R��ʶ��0X200
	TxMessage.IDE = CAN_ID_STD;				 //��׼ģʽ
	TxMessage.RTR = CAN_RTR_DATA;			 //����֡RTR=0��Զ��֡RTR=1
	TxMessage.DLC = 8;							 	 //���ݳ���Ϊ8�ֽ�

	TxMessage.Data[0] = (unsigned char)((int16_t)Chassis_PIDterm[0]>>8); 
	TxMessage.Data[1] = (unsigned char)((int16_t)Chassis_PIDterm[0]); 
	TxMessage.Data[2] = (unsigned char)((int16_t)Chassis_PIDterm[1]>>8);  
	TxMessage.Data[3] = (unsigned char)((int16_t)Chassis_PIDterm[1]); 
	TxMessage.Data[4] = (unsigned char)((int16_t)Chassis_PIDterm[2]>>8);  
	TxMessage.Data[5] =	(unsigned char)((int16_t)Chassis_PIDterm[2]); 
	TxMessage.Data[6] = (unsigned char)((int16_t)Chassis_PIDterm[3]>>8);  
	TxMessage.Data[7] =	(unsigned char)((int16_t)Chassis_PIDterm[3]); 

	CAN_Transmit(CAN1, &TxMessage);	//��������
}


void CAN1_Gimbal_Send()
{
	CanTxMsg TxMessage;
	
	TxMessage.StdId = 0x1FF;					 
	TxMessage.IDE = CAN_ID_STD;				 //��׼ģʽ
	TxMessage.RTR = CAN_RTR_DATA;			 //����֡RTR=0��Զ��֡RTR=1
	TxMessage.DLC = 8;	
	
	TxMessage.Data[0]=(unsigned char)((int16_t)Gimbal_PIDterm[YAW]>>8);    //Yaw
	TxMessage.Data[1]=(unsigned char)((int16_t)Gimbal_PIDterm[YAW]);
	TxMessage.Data[2]=(unsigned char)((int16_t)Gimbal_PIDterm[PITCH]>>8);  //Pitch                     
	TxMessage.Data[3]=(unsigned char)((int16_t)Gimbal_PIDterm[PITCH]);
	TxMessage.Data[4]=0;
	TxMessage.Data[5]=0;
	TxMessage.Data[6]=0;
	TxMessage.Data[7]=0;	
  
	CAN_Transmit(CAN1,&TxMessage);			
}


bool Cha_Normal_flag1, Cha_Normal_flag2, Cha_Normal_flag3, Cha_Normal_flag4;
bool Gim_Yaw_Normal_flag, Gim_Pitch_Normal_flag;
int Cha_Normal_Num1, Cha_Normal_Num2, Cha_Normal_Num3, Cha_Normal_Num4;
int Gim_Yaw_Normal_Num, Gim_Pitch_Normal_Num;
void CAN1_RX0_IRQHandler()
{
	CanRxMsg RxMessage;
	if(CAN_GetITStatus!=RESET)
		{
			CAN_ClearITPendingBit(CAN1,CAN_IT_FMP0);		//����жϹ���
			CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);		//����can����
		}
		
	if(RxMessage.StdId == 0x201)//��ǰ		
		{
			Motor_chassis[0][0]=((int16_t)RxMessage.Data[0]<<8 | RxMessage.Data[1]);
			Motor_chassis[0][1]=((int16_t)RxMessage.Data[2]<<8 | RxMessage.Data[3]);
			
			Cha_Normal_flag1 = 1;
			Cha_Normal_Num1 = 0;
		}
		
	if(RxMessage.StdId == 0x202)//��ǰ		
		{    
			Motor_chassis[1][0]=((int16_t)RxMessage.Data[0]<<8 | RxMessage.Data[1]);
			Motor_chassis[1][1]=((int16_t)RxMessage.Data[2]<<8 | RxMessage.Data[3]);
			
			Cha_Normal_flag2 = 1;
			Cha_Normal_Num2 = 0;
		}
		
	if(RxMessage.StdId == 0x203)//���		
		{    
			Motor_chassis[2][0]=((int16_t)RxMessage.Data[0]<<8 | RxMessage.Data[1]);
			Motor_chassis[2][1]=((int16_t)RxMessage.Data[2]<<8 | RxMessage.Data[3]);
			
			Cha_Normal_flag3 = 1;
			Cha_Normal_Num3 = 0;
		}
		
	if(RxMessage.StdId == 0x204)//�Һ�		
		{

			Motor_chassis[3][0]=((int16_t)RxMessage.Data[0]<<8 | RxMessage.Data[1]);
			Motor_chassis[3][1]=((int16_t)RxMessage.Data[2]<<8 | RxMessage.Data[3]);
			
			Cha_Normal_flag4 = 1;
			Cha_Normal_Num4 = 0;
		}
		
	if(RxMessage.StdId == 0x205)//YAW��
		{
			Motor_angel[YAW]=((int16_t)RxMessage.Data[0]<<8 | RxMessage.Data[1]);//��е�Ƕ�0~8191��0x1FFF��
			Motor_speed[YAW]=((int16_t)RxMessage.Data[2]<<8 | RxMessage.Data[3]);//ת�٣�rpm��
			Gim_Yaw_Normal_flag = 1;
			Gim_Yaw_Normal_Num = 0;
		}
		
	if(RxMessage.StdId == 0x206)//PITCH��
		{
			Motor_angel[PITCH]=((int16_t)RxMessage.Data[0]<<8 | RxMessage.Data[1]);//��е�Ƕ�0~8191��0x1FFF��
			Motor_speed[PITCH]=((int16_t)RxMessage.Data[2]<<8 | RxMessage.Data[3]);//ת�٣�rpm��
			Gim_Pitch_Normal_flag = 1;
			Gim_Pitch_Normal_Num = 0;
		}
}

//void CAN1_TX_IRQHandler()
//{
//	if(CAN_GetITStatus!=RESET)
//		{
//			CAN_ClearITPendingBit(CAN1,CAN_IT_TME);
//		}
//}



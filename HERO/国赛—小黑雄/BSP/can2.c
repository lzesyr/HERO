#include "can2.h"

/*----CAN2_TX-----PB13----*/
/*----CAN2_RX-----PB12----*/

RM06_Angle_t M06_Angle;
float S_PIDTerm[2];
int16_t M2006_actAngle[2], M2006_actSpeed[2];


void CAN2_Init(void)
{
    CAN_InitTypeDef        can;
    CAN_FilterInitTypeDef  can_filter;
    GPIO_InitTypeDef       gpio;
    NVIC_InitTypeDef       nvic;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN2, ENABLE);

    GPIO_PinAFConfig(GPIOB, GPIO_PinSource12, GPIO_AF_CAN2);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_CAN2); 

    gpio.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 ;
    gpio.GPIO_Mode = GPIO_Mode_AF;
    GPIO_Init(GPIOB, &gpio);

    nvic.NVIC_IRQChannel = CAN2_RX0_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 1;
    nvic.NVIC_IRQChannelSubPriority = 0;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
		
		nvic.NVIC_IRQChannel = CAN2_TX_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 1;
    nvic.NVIC_IRQChannelSubPriority = 1;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic); 

    CAN_DeInit(CAN2);
    CAN_StructInit(&can);

    can.CAN_TTCM = DISABLE;
    can.CAN_ABOM = DISABLE;    
    can.CAN_AWUM = DISABLE;    
    can.CAN_NART = DISABLE;    
    can.CAN_RFLM = DISABLE;    
    can.CAN_TXFP = ENABLE;     
    can.CAN_Mode = CAN_Mode_Normal; 
    can.CAN_SJW  = CAN_SJW_1tq;
    can.CAN_BS1 = CAN_BS1_9tq;
    can.CAN_BS2 = CAN_BS2_4tq;
    can.CAN_Prescaler = 3;   //CAN BaudRate 42/(1+9+4)/3=1Mbps
    CAN_Init(CAN2, &can);
    
    can_filter.CAN_FilterNumber=14;
    can_filter.CAN_FilterMode=CAN_FilterMode_IdMask;
    can_filter.CAN_FilterScale=CAN_FilterScale_32bit;
    can_filter.CAN_FilterIdHigh=0x0000;
    can_filter.CAN_FilterIdLow=0x0000;
    can_filter.CAN_FilterMaskIdHigh=0x0000;
    can_filter.CAN_FilterMaskIdLow=0x0000;
    can_filter.CAN_FilterFIFOAssignment=0;//the message which pass the filter save in fifo0 CAN_Filter_FIFO0
    can_filter.CAN_FilterActivation=ENABLE;
    CAN_FilterInit(&can_filter);
    
    CAN_ITConfig(CAN2,CAN_IT_FMP0,ENABLE);    ////FIFO0消息挂号中断允许
		CAN_ITConfig(CAN2,CAN_IT_TME,ENABLE);
} 

void CAN2_Motor_Send(void)
{				
		CanTxMsg	tx_message;
		tx_message.StdId = 0x200;					   //使用的标准ID
		tx_message.IDE = CAN_ID_STD;				 //标准模式
		tx_message.RTR = CAN_RTR_DATA;			 //发送的是数据
		tx_message.DLC = 0x08;							 //数据长度为8字节
		tx_message.Data[0] = (unsigned char)((int16_t)S_PIDTerm[XIAOBOPAN]>>8);    //大拨盘电机
		tx_message.Data[1] = (unsigned char)((int16_t)S_PIDTerm[XIAOBOPAN]);
		tx_message.Data[2] = (unsigned char)((int16_t)S_PIDTerm[MOTIANLUN]>>8);   //摩天轮电机
		tx_message.Data[3] = (unsigned char)((int16_t)S_PIDTerm[MOTIANLUN]);
		tx_message.Data[4] = 0;
		tx_message.Data[5] = 0;
		tx_message.Data[6] = 0;
		tx_message.Data[7] = 0;
		CAN_Transmit(CAN2, &tx_message);
}


bool Motianlun_Roll_flag;
int  Motianlun_Roll_Num;
void CAN2_RX0_IRQHandler(void)
{
		CanRxMsg RxMessage;
		if (CAN_GetITStatus(CAN2,CAN_IT_FMP0)!= RESET) 
    {				 
			CAN_ClearITPendingBit(CAN2, CAN_IT_FMP0);
			CAN_Receive(CAN2, CAN_FIFO0, &RxMessage);

			if(RxMessage.StdId == 0x201)		   //小拨盘电机
				{
					M2006_actAngle[XIAOBOPAN] = ((int16_t)RxMessage.Data[0]<<8 | RxMessage.Data[1]);  //机械角度
					M2006_actSpeed[XIAOBOPAN] = ((int16_t)RxMessage.Data[2]<<8 | RxMessage.Data[3]);  //实际速度
					
					circle_count(XIAOBOPAN);
				}		
				
			if(RxMessage.StdId == 0x202)		   //摩天轮电机
				{
					M2006_actAngle[MOTIANLUN] = ((int16_t)RxMessage.Data[0]<<8 | RxMessage.Data[1]);  //机械角度
					M2006_actSpeed[MOTIANLUN] = ((int16_t)RxMessage.Data[2]<<8 | RxMessage.Data[3]);  //实际速度
					
					Motianlun_Roll_flag = 1;
					Motianlun_Roll_Num = 0;
				}	
		}
}

void CAN2_TX_IRQHandler(void)
{
	if (CAN_GetITStatus(CAN2,CAN_IT_TME)!= RESET) 
	{
		CAN_ClearITPendingBit(CAN2,CAN_IT_TME);
	}
}


/*------------------------位置环，角度累加计算-----------------------*/
bool record_flag = 1;

void circle_count(int num)
{
	if( record_flag == 1 )
		{
			M06_Angle.Begin[num] = M2006_actAngle[num];
			record_flag = 0;
		}
				
	M06_Angle.last[num] = M06_Angle.Current[num];
			
	M06_Angle.Current[num] = M2006_actAngle[num];
				
	if( M06_Angle.Current[num] - M06_Angle.last[num] > 4096 )
		{
			M06_Angle.Count[num]--;
		}
	else if( M06_Angle.Current[num] - M06_Angle.last[num] < -4096 )
		{
			M06_Angle.Count[num]++;
		}
				
	M06_Angle.Total[num] = M06_Angle.Count[num] * 8192 + M06_Angle.Current[num] - M06_Angle.Begin[num];
}



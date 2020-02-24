#include "usart5.h"  
#include<string.h>

extern float realtimePower;

unsigned char RxBuffer[200] = {0};
unsigned char CliendTxBuffer[200] = {0};
unsigned char TeamMateTxBuffer[200] = {0};
int judgementDataLength;
uint16_t FrameLength;

uint16_t kehuHero,kehuEngineer,kehuInfantry3,kehuInfantry4,kehuInfantry5,kehuPlane;
uint16_t Hero,Engineer,Infantry3,Infantry4,Infantry5,Plane,Sentry;

//��������
extFrameHeader   									FrameHeader;
ext_game_state_t                  GameState;
ext_game_result_t                 GameResult;
ext_game_robot_survivors_t        GameRobotSurvivors;
ext_event_data_t                  EventData;
ext_supply_projectile_action_t    SupplyProjectileAction;
ext_supply_projectile_booking_t   SupplyProjectileBooking;
ext_game_robot_state_t            GameRobotState;
ext_power_heat_data_t             PowerHeatData;
ext_game_robot_pos_t              GameRobotPos;
ext_buff_musk_t                   BuffMusk;
aerial_robot_energy_t             AerialRobotEnergy;
ext_robot_hurt_t                  RobotHurt;
ext_shoot_data_t                  ShootData;
ext_teammate_data_t               TeamMateData;

//��������
exSendClientData_t      SendClient;
exCommunatianData_t     CommuData;



PowerProtect_t     PowerProtect;

void usart5_Init(void)
{
	GPIO_InitTypeDef   gpio;
	USART_InitTypeDef  usart5;
	DMA_InitTypeDef    dma1;
	NVIC_InitTypeDef   nvic;
	
	RCC_APB1PeriphClockCmd( RCC_APB1Periph_UART5, ENABLE );
	RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOD, ENABLE );
	RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_DMA1, ENABLE );
	
	GPIO_PinAFConfig( GPIOC, GPIO_PinSource12, GPIO_AF_UART5);
	GPIO_PinAFConfig( GPIOD, GPIO_PinSource2, GPIO_AF_UART5);
	
	gpio.GPIO_Pin    =  GPIO_Pin_12;
	gpio.GPIO_Mode   =  GPIO_Mode_AF;
	gpio.GPIO_OType  =  GPIO_OType_PP;
	gpio.GPIO_PuPd   =  GPIO_PuPd_UP;
	gpio.GPIO_Speed  =  GPIO_Speed_50MHz;
	GPIO_Init( GPIOC, &gpio );
	gpio.GPIO_Pin    =  GPIO_Pin_2;
	GPIO_Init( GPIOD, &gpio );
	
	USART_DeInit( UART5 );
	usart5.USART_BaudRate            = 115200;
	usart5.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
	usart5.USART_Parity              = USART_Parity_No;
	usart5.USART_StopBits            = USART_StopBits_1;
	usart5.USART_WordLength          = USART_WordLength_8b;
	usart5.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_Init( UART5, &usart5);
	USART_Cmd( UART5, ENABLE );
	USART_ITConfig( UART5, USART_IT_IDLE, ENABLE );
	USART_DMACmd( UART5, USART_DMAReq_Rx, ENABLE );
	USART_DMACmd( UART5, USART_DMAReq_Tx, ENABLE );
	
	nvic.NVIC_IRQChannel = UART5_IRQn;
	nvic.NVIC_IRQChannelPreemptionPriority = 0;
	nvic.NVIC_IRQChannelSubPriority = 0;
	nvic.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic);
	
	DMA_DeInit( DMA1_Stream0 );
	dma1.DMA_Channel            = DMA_Channel_4;
	dma1.DMA_DIR                = DMA_DIR_PeripheralToMemory;
	dma1.DMA_Mode               = DMA_Mode_Circular;
	dma1.DMA_FIFOMode           = DMA_FIFOMode_Disable;
	dma1.DMA_FIFOThreshold      = DMA_FIFOThreshold_1QuarterFull;
	dma1.DMA_BufferSize         = 200;
	dma1.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;
	dma1.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	dma1.DMA_Memory0BaseAddr    = (uint32_t)RxBuffer;
	dma1.DMA_PeripheralBaseAddr = (uint32_t)&(UART5->DR);
	dma1.DMA_MemoryInc          = DMA_MemoryInc_Enable;
	dma1.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
	dma1.DMA_Priority           = DMA_Priority_VeryHigh;
	dma1.DMA_PeripheralBurst    = DMA_PeripheralBurst_Single;
	dma1.DMA_MemoryBurst        = DMA_MemoryBurst_Single;
	DMA_Init( DMA1_Stream0, &dma1 );
	DMA_Cmd( DMA1_Stream0, ENABLE );
}

bool Shoot_Data_42_Update, Shoot_Data_17_Update; //ǹ�������Ƿ����
int32_t Shot_42_Num, Shot_17_Num;
float current_heat42, current_heat17;
float Chassis_Volt, Chassis_Current;
uint32_t lostpack;
bool IF_Hurt = 0;
void Re_Date_From_judgement(unsigned char *judgementData)
{
	int CmdID;
	lostpack++;
	memcpy(&FrameHeader, judgementData, sizeof(extFrameHeader));
	
	if((*judgementData == 0xA5)  
	&& (Verify_CRC8_Check_Sum(judgementData, sizeof(extFrameHeader)) == 1) 
	&& (Verify_CRC16_Check_Sum(judgementData, sizeof(extFrameHeader) + CMD_ID_SIZE + FrameHeader.DataLength + FRAME_TAIL_SIZE) == 1))
	{
		lostpack--;
		CmdID = ( judgementData[6] << 8 | judgementData[5] );  //������ID
		switch (CmdID)
		{
			case game_state:     //����״̬����
			{
				memcpy( &GameState, judgementData + 7, sizeof(ext_game_state_t) );
				break;
			}

			case game_result:    //�����������
			{
				memcpy( &GameResult, judgementData + 7, sizeof(ext_game_result_t) );
				break;
			}
						
			case game_robot_survivors: //���������˴������
			{
				memcpy( &GameRobotSurvivors, judgementData + 7, sizeof(ext_game_robot_survivors_t) );
				break;
			}
						
			case event_data:    //�����¼�����
			{
				memcpy( &EventData, judgementData + 7, sizeof(ext_event_data_t) );
				break;
			}
						
			case supply_projectile_action:  //���ز���վ������ʶ����
			{
				memcpy( &SupplyProjectileAction, judgementData + 7, sizeof(ext_supply_projectile_action_t) );
				break;
			}
						
			case supply_projectile_booking: //���ز���վԤԼ�ӵ�����
			{
				memcpy( &SupplyProjectileBooking, judgementData + 7, sizeof(ext_supply_projectile_booking_t) );
				break;
			}
						
			case game_robot_state: //����������״̬
			{
				memcpy( &GameRobotState, judgementData + 7, sizeof(ext_game_robot_state_t) );
				break;
			}
						
			case power_heat_data:  //ʵʱ���ʺ�����
			{
				memcpy( &PowerHeatData, judgementData + 7, sizeof(ext_power_heat_data_t) );
				PowerProtect.power_remain = PowerHeatData.chassis_power_buffer;
				realtimePower   = PowerHeatData.chassis_power;
				current_heat42  = PowerHeatData.shooter_heat1;
				Chassis_Volt    = PowerHeatData.chassis_volt;
				Chassis_Current = PowerHeatData.chassis_current;
				if( Chassis_Current * Chassis_Volt != 0 )
					{
						PowerProtect.judgemengt_connecting = 1;
						PowerProtect.connecting_num = 0;
					}
				break;
			}
						
			case game_robot_pos:  //������λ�ú�ǹ�ڳ�����Ϣ
			{
				memcpy( &GameRobotPos, judgementData + 7, sizeof(ext_game_robot_pos_t) );
				break;
			}
						
			case buff_musk:     //��ȡ��Buff
			{
				memcpy( &BuffMusk, judgementData + 7, sizeof(ext_buff_musk_t) );
				break;
			}
						
			case aerial_robot_energy:  //���л���������״̬����
			{
				memcpy( &AerialRobotEnergy, judgementData + 7, sizeof(aerial_robot_energy_t) );
				break;
			}
						
			case robot_hurt:  //�˺�״̬����
			{
				memcpy( &RobotHurt, judgementData + 7, sizeof(ext_robot_hurt_t) );
				if( RobotHurt.hurt_type == 0 && chassis_Follow_flag == 1 && !IF_KEY_PRESSED_A && !IF_KEY_PRESSED_W && !IF_KEY_PRESSED_S && !IF_KEY_PRESSED_D)
					{
						IF_Hurt = 1;
					}
				break;
			}
						
			case shoot_data:      //ʵʱ�������
			{
				memcpy( &ShootData, judgementData + 7, sizeof(ext_shoot_data_t) );
				if( ShootData.bullet_type == 2 )
					{
						Shoot_Data_42_Update = 1;
					}
				if( ShootData.bullet_type == 1 )
					{
						Shoot_Data_17_Update = 1;
					}
				break;
			}
			
			case TeamMate_data:  //������Ϣ
			{
				memcpy( &TeamMateData, judgementData + 7, sizeof(ext_teammate_data_t) );
				break;
			}
		}
				
		if(*(judgementData + sizeof(extFrameHeader) + CMD_ID_SIZE + FrameHeader.DataLength + FRAME_TAIL_SIZE) == 0xA5)
		{
			Re_Date_From_judgement(judgementData + sizeof(extFrameHeader) + CMD_ID_SIZE + FrameHeader.DataLength + FRAME_TAIL_SIZE);
		}	
	}
}

int term;
void UART5_IRQHandler(void)
{
	if( USART_GetITStatus( UART5, USART_IT_IDLE ) != RESET )
	{
		term = UART5->SR;
		term = UART5->DR;

		DMA_Cmd( DMA1_Stream0, DISABLE );
		
		judgementDataLength = 200 - DMA_GetCurrDataCounter( DMA1_Stream0 );
		Re_Date_From_judgement( RxBuffer );
		
		memset( RxBuffer, 0, 200 ); //û�ӻᵼ�����ݽ��ղ���ȫ
		
		DMA_ClearFlag(DMA1_Stream0, DMA_IT_TCIF0);
		DMA1_Stream0->NDTR = 200;	
		
		DMA_Cmd( DMA1_Stream0, ENABLE );
	}
}


uint16_t debug_data;
float Have_BigBomb_Num = 0, Add_BigBomb_Num = 0;
bool AddBomb_flag, Chassis_Normal_Flag, Gimbal_Normal_Flag, Two_Light1_flag, Two_Light2_flag;
void Send_Data_To_Judgement(void)
{
	static u8 i = 0,Data_Len = 0;
	
	SendClient.txFrameHeader.SQF = 0xA5;  //����֡��ʼ
	SendClient.txFrameHeader.DataLength = sizeof(ext_student_interactive_header_data_t) + sizeof(client_custom_data_t);  //����֡��DATA�ĳ���
	SendClient.txFrameHeader.Seq = 0;
	memcpy(CliendTxBuffer, &SendClient.txFrameHeader, sizeof(extFrameHeader));
	Append_CRC8_Check_Sum(CliendTxBuffer, sizeof(extFrameHeader));	//֡ͷCRC8У��
	
	SendClient.CmdID = 0x0301;														//����ID
	
	SendClient.dataFrameHeader.data_cmd_id = 0xD180;
	SendClient.dataFrameHeader.send_ID = GameRobotState.robot_id;
	SendClient.dataFrameHeader.receiver_ID = kehuHero;
	
	if( TeamMateData.interactData.data[0] != 0 )
		{
			Add_BigBomb_Num = TeamMateData.interactData.data[0];  //���̳����ж��ٵ�
			AddBomb_flag = 1;
		}
	if( TeamMateData.interactData.data[0] == 0 && AddBomb_flag == 1)
		{
			Have_BigBomb_Num = Have_BigBomb_Num + Add_BigBomb_Num;  //������Ӣ�ۺ󣬷��������ݻ����㣬��ʱҪ�ۼ��ϵ�����
			AddBomb_flag = 0;
		}
			
		if( Cha_Normal_flag1 == 1 )
		{ Cha_Normal_Num1++;
			if( Cha_Normal_Num1 > 4 )
			{ Cha_Normal_flag1 = 0; }
		}
		if( Cha_Normal_flag2 == 1 )
		{ Cha_Normal_Num2++;
			if( Cha_Normal_Num2 > 4 )
			{ Cha_Normal_flag2 = 0; }
		}
		if( Cha_Normal_flag3 == 1 )
		{ Cha_Normal_Num3++;
			if( Cha_Normal_Num3 > 4 )
			{ Cha_Normal_flag3 = 0; }
		}
		if( Cha_Normal_flag4 == 1 )
		{ Cha_Normal_Num4++;
			if( Cha_Normal_Num4 > 4 )
			{ Cha_Normal_flag4 = 0; }
		}
		if( Gim_Pitch_Normal_flag == 1 )
		{ Gim_Pitch_Normal_Num++;
			if( Gim_Pitch_Normal_Num > 4 )
			{ Gim_Pitch_Normal_flag = 0; }
		}
		if( Gim_Yaw_Normal_flag == 1 )
		{ Gim_Yaw_Normal_Num++;
			if( Gim_Yaw_Normal_Num > 4 )
			{ Gim_Yaw_Normal_flag = 0; }
		}
		if( Motianlun_Roll_flag == 1 )
		{ Motianlun_Roll_Num++;
			if( Motianlun_Roll_Num > 4 )
			{ Motianlun_Roll_flag = 0; }
		}
		
		
	if( Cha_Normal_flag1 && Cha_Normal_flag2 && Cha_Normal_flag3 && Cha_Normal_flag4 )
		{
			Chassis_Normal_Flag = 1;
		}
	if( Gim_Pitch_Normal_flag && Gim_Yaw_Normal_flag )
		{
			Gimbal_Normal_Flag = 1;
		}
		
	if( Chassis_Normal_Flag && Gimbal_Normal_Flag && Motianlun_Roll_flag )	//������ʱ����յ��
		{
			Two_Light1_flag = 1;
			Two_Light2_flag = 1;
		}
	else if( !Chassis_Normal_Flag && Gimbal_Normal_Flag && Motianlun_Roll_flag)  //���̻�����������
		{
			Two_Light1_flag = 0;
			Two_Light2_flag = 1;
		}
	else if( Chassis_Normal_Flag && !Gimbal_Normal_Flag && Motianlun_Roll_flag ) //��̨�����������
		{
			Two_Light1_flag = 1;
			Two_Light2_flag = 0;
		}
	else if( Chassis_Normal_Flag && Gimbal_Normal_Flag && !Motianlun_Roll_flag )  //Ħ���ֻ����������
		{
			Two_Light1_flag = 0;
			Two_Light1_flag = 0;
		}
		
	//��һ��ָʾ��Ϊ��С��ģʽ����Ϊ�󣬺�ΪС
	//�ڶ���ָʾ��Ϊ�Ƿ�ҡ�ڣ���Ϊҡ�ڣ�����
	//������ָʾ��Ϊ�Ƿ�б���棬��Ϊ�ǣ���Ϊ��
	//���ĸ�ָʾ��Ϊǹ�����Ƿ��е�����Ϊ�У���Ϊ��
	if (SystemMonitor == Normal_Mode)
	{
		SendClient.clientData.data1 = (int)((ADC_VOLT - 12) / 12 * 100);		//����1�����������ݴ洢����,����ѹת����100%
		SendClient.clientData.data2 = RealFrcSpeed[FRICTION_42];//(int)Have_BigBomb_Num - Shot_42_Num;  //ʣ�൯��
		SendClient.clientData.data3 = (int16_t)Motor_angel[PITCH];          //Pitch�Ƕ�ֵ
		SendClient.clientData.masks = (uint16_t)(Shoot_State | change_F_flag<<1 | change_B_flag<<2 | Yuzhi_TwoBomb_flag<<3 | Two_Light2_flag<<4 | Two_Light1_flag<<5 );
	}
	else
	{
		SendClient.clientData.data1 = (float)666;					
		SendClient.clientData.data2 = (float)666;					
		SendClient.clientData.data3 = (float)666;		
	}
	memcpy(CliendTxBuffer + 5, (uint8_t*)&SendClient.CmdID, (sizeof(SendClient.CmdID)+ sizeof(SendClient.dataFrameHeader)+ sizeof(SendClient.clientData)));			
	Append_CRC16_Check_Sum(CliendTxBuffer, sizeof(SendClient));	
	
	Data_Len = sizeof(SendClient); 
	
	for(i = 0;i < Data_Len;i++)
	{
		USART_SendData(UART5,(uint16_t)CliendTxBuffer[i]);
		
		while(USART_GetFlagStatus(UART5,USART_FLAG_TC)==RESET);
	}
}

uint32_t record_292_start_time;
uint16_t to_send_id;
uint8_t to_send_flag;
void gimbaler_send_sentry(void)
{
   if(IF_KEY_PRESSED_Z && IF_KEY_PRESSED_X)
	{
		to_send_id = 0x0292;
		record_292_start_time = millis();
	}
	else if( millis() - record_292_start_time > 10000 )
	{
		to_send_id = 0;
		to_send_flag = 0;
	}
}



void Seng_Data_To_TeamMate(void)
{
	static u8 datalength, i;
	
	memset(TeamMateTxBuffer, 0, 200);
	
	CommuData.txFrameHeader.SQF = 0xA5;
	CommuData.txFrameHeader.DataLength = sizeof( ext_student_interactive_header_data_t ) + sizeof( robot_interactive_data_t );
	CommuData.txFrameHeader.Seq = 0;
	memcpy( TeamMateTxBuffer, &CommuData.txFrameHeader, sizeof(extFrameHeader) );
	Append_CRC8_Check_Sum(TeamMateTxBuffer, sizeof(extFrameHeader));
	
	CommuData.CmdID = 0x0301;
	CommuData.dataFrameHeader.data_cmd_id = to_send_id;
	CommuData.dataFrameHeader.send_ID = GameRobotState.robot_id;//�����ߵ�ID
	CommuData.dataFrameHeader.receiver_ID = Sentry;
	
	CommuData.interactData.data[0] = to_send_flag;
	  
	
	memcpy(TeamMateTxBuffer+5,(uint8_t *)&CommuData.CmdID,(sizeof(CommuData.CmdID)+sizeof(CommuData.dataFrameHeader)+sizeof(CommuData.interactData)));		
	Append_CRC16_Check_Sum(TeamMateTxBuffer,sizeof(CommuData));
	
	datalength = sizeof(CommuData); 
	for(i = 0;i < datalength;i++)
	{
		USART_SendData(UART5,(uint16_t)TeamMateTxBuffer[i]);
		while(USART_GetFlagStatus(UART5,USART_FLAG_TC)==RESET);
	}	 
}




//�ж��Լ����Ա�
bool is_red_or_blue(void)
{
	if(GameRobotState.robot_id > 10)
	{
		return BlUE;
	}
	else 
	{
		return RED;
	}
}

int Color;
void determine_ID(void)
{
	Color = is_red_or_blue();
	if(Color == BlUE)
	{
		kehuHero = 0x0111;
		kehuEngineer = 0x0112;
		kehuInfantry3 = 0x0113;
		kehuInfantry4 = 0x0114;
		kehuInfantry5 = 0x0115;
		kehuPlane = 0x0116;
		
		
		Hero = 11;
		Engineer = 12;
		Infantry3 = 13;
		Infantry4 = 14;
		Infantry5 = 15;
		Plane = 16;
		Sentry = 17;
	}
	else if(Color == RED)
	{
		kehuHero = 0x0101;
		kehuEngineer = 0x0102;
		kehuInfantry3 = 0x0103;
		kehuInfantry4 = 0x0104;
		kehuInfantry5 = 0x0105;
		kehuPlane = 0x0106;
		
		
		Hero = 1;
		Engineer = 2;
		Infantry3 = 3;
		Infantry4 = 4;
		Infantry5 = 5;
		Plane = 6;
		Sentry = 7;
	}
}

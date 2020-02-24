#include "vision.h"
#include "math.h"
#include "filter.h"


//�Ҽ�����

/*-------�Ӿ��ֱ���Ԥ����--------*/
#define	VISION_1280P	0
#define	VISION_640P		1

#define VISION_DPI		VISION_1280P

#if	VISION_DPI == VISION_1280P
	#define VISION_MID_YAW		640
	#define VISION_MID_PITCH	360
	
#elif VISION_DPI == VISION_640P
	#define VISION_MID_YAW		320
	#define VISION_MID_PITCH	240
#endif


#define INFANTRY_DEBUG_ID  DEBUG_ID_ZERO

#if		INFANTRY_DEBUG_ID == DEBUG_ID_ZERO
	#define	COMPENSATION_YAW	0        //��������ƫ����������ƫ
	#define	COMPENSATION_PITCH 0.77      //��������ƫ����������ƫ
	
	//�Ƕȳ�ʼ������
	float Vision_Comps_Yaw   = COMPENSATION_YAW;
	float Vision_Comps_Pitch = COMPENSATION_PITCH;
	
#elif	INFANTRY_DEBUG_ID == DEBUG_ID_ONE
	#define	COMPENSATION_YAW	(0f)
	#define	COMPENSATION_PITCH	(0f)
	//�Ƕȳ�ʼ������
	float Vision_Comps_Yaw   = COMPENSATION_YAW;
	float Vision_Comps_Pitch = COMPENSATION_PITCH;
#endif
/*----------------------------------------------------------------*/

extVisionSendHeader_t    VisionSendHeader;  //ͷ
extVisionRecvData_t      VisionRecvData;    //�Ӿ����սṹ��
extVisionSendData_t      VisionSendData;    //�Ӿ����ͽṹ��

#define ATTACK_NONE    0	//��ʶ��
#define ATTACK_RED     1	//ʶ��췽
#define ATTACK_BLUE    2	//ʶ������
uint8_t Attack_Color_Choose = ATTACK_NONE;//Ĭ�ϲ�ʶ��

//�ǶȲ���,���͸��Ӿ�
float Vision_Comps_Yaw_Send   = COMPENSATION_YAW;
float Vision_Comps_Pitch_Send = COMPENSATION_PITCH;

//�Ӿ��Ƿ���������,FALSEû��,TRUE�����µ�
uint8_t Vision_Get_New_Data = FALSE;


/************************************************************************************/
/************************************************************************************/
int yaw_raw_js;
int yaw_kf_js;
/**
  * @brief  ��ȡ�Ӿ���Ϣ
  * @param  usart4��������
  * @retval void
  * @attention  IRQִ��
  */
void Vision_Read_Data(uint8_t *ReadFromUsart)
{
	//�ж�֡ͷ�����Ƿ�Ϊ0xA5
	if(ReadFromUsart[0] == VISION_SOF)
	{
		//֡ͷCRC8У��
		if(Verify_CRC8_Check_Sum( ReadFromUsart, VISION_LEN_HEADER ) == TRUE)
		{
			//֡βCRC16У��
			if(Verify_CRC16_Check_Sum( ReadFromUsart, VISION_LEN_PACKED ) == TRUE)
			{
				//�������ݿ���
				memcpy( &VisionRecvData, ReadFromUsart, VISION_LEN_PACKED);	
				Vision_Get_New_Data = TRUE;//����Ӿ����ݸ�����
			}
		}
	}
	yaw_raw_js = VisionRecvData.distance*1000;
	yaw_kf_js = VisionRecvData.yaw_angle*1000;
}

/**
  * @brief  �����Ӿ�ָ��
  * @param  CmdID
  * @retval void
  * @attention  ��Э���������ݷ���
  */
uint8_t vision_send_pack[50] = {0};//����22����
void Vision_Send_Data( uint8_t CmdID )
{
//	uint8_t vision_send_pack[50] = {0};//����22����
	int i;    //ѭ�����ʹ���

	VisionSendHeader.SOF = VISION_SOF;
	VisionSendHeader.CmdID = CmdID;//���Ӿ���˵����Ҫ������
	
	//д��֡ͷ
	memcpy( vision_send_pack, &VisionSendHeader, VISION_LEN_HEADER );
	
	//֡ͷCRC8У��Э��
	Append_CRC8_Check_Sum( vision_send_pack, VISION_LEN_HEADER );
	
	if( Get_Attack_attention_Mode() == 1 )  //�����ڱ�ʱ�������Ӿ�1������0
		{	VisionSendData.attack_sentry = 1; }
	else
		{	VisionSendData.attack_sentry = 0; }	
	
	if( abs(debug_yaw_speed) < 0.3f )
		{ VisionSendData.kf_yaw_angle = 0; }
	else
		{ VisionSendData.kf_yaw_angle = debug_kf_yaw_angle / 8191 * 360; }  //��Ԥ��Ƕȷ����Ӿ��������ж�
		
	if( Mobility_Prediction_Yaw == TRUE )
		{	VisionSendData.Pre_Yaw_Open = 1; }
	else
		{ VisionSendData.Pre_Yaw_Open = 0; }
		
	memcpy( vision_send_pack + VISION_LEN_HEADER, &VisionSendData, VISION_LEN_DATA);
	
	//֡βCRC16У��Э��
	Append_CRC16_Check_Sum( vision_send_pack, VISION_LEN_PACKED );
	
	//������õ�����ͨ��������λ���͵�����ϵͳ
	for (i = 0; i < VISION_LEN_PACKED; i++)
	{
		UART4_SendChar( vision_send_pack[i] );
	}
}

/**********************************�Ӿ�����*****************************************/
/**
  * @brief  �Ӿ��ܿ���,ָ�����
  * @param  void
  * @retval void
  * @attention  8λ,ֻ�м���ģʽ���Ӿ�
  */
void Vision_Ctrl(void)
{
	if(Get_remoteMode() == KEY)//����ģʽ
	{
		Vision_Auto_Attack_Ctrl();
	}
	else
	{
		Vision_Auto_Attack_Off();
	}
}

/**
  * @brief  �������
  * @param  void
  * @retval void
  * @attention  
  */
void Vision_Auto_Attack_Ctrl(void)
{
	/* ȷ���з���ɫ */
	//s1��ʶ���,��ʶ����,�м䲻ʶ��
	if(RC_Ctl.rc.s1 == RC_SW_UP)
	{
		Attack_Color_Choose = ATTACK_RED;
	}
	else if(RC_Ctl.rc.s1 == RC_SW_DOWN)
	{
		Attack_Color_Choose = ATTACK_BLUE;
	}
	else
	{
		Attack_Color_Choose = ATTACK_NONE;
	}
	
	
	//��С���Է�����ɫʶ��ָ��
	if(Attack_Color_Choose == ATTACK_BLUE)
	{
		if( Get_Chassis_Mode() == 4 ) //ȡ��ģʽ
		{
			Vision_Send_Data( VISION_RED_GETBOMB ); //��ɫ��������ɫ�෴
		}
		else if( GIMBAL_ActionMode() == 3 ) //����ģʽ
		{
			Vision_Send_Data( VISION_BLUE_BASE );
		}
		else
		{
			if( Shoot_State == BIGBOMB_MODE )
			{
				Vision_Send_Data( VISION_BLUE_BIG );
			}
			else if( Shoot_State == XIAOBOMB_MODE )
			{
				Vision_Send_Data( VISION_BLUE_XIAO );
			}
		}
	}
	else if(Attack_Color_Choose == ATTACK_RED)
	{
		
		if( Get_Chassis_Mode() == 4 ) //ȡ��ģʽ
		{
			Vision_Send_Data( VISION_BLUE_GETBOMB );
		}
		else if( GIMBAL_ActionMode() == 3 ) //����ģʽ
		{
			Vision_Send_Data( VISION_RED_BASE );
		}
		else
		{
			if( Shoot_State == BIGBOMB_MODE )
			{
				Vision_Send_Data( VISION_RED_BIG );
			}
			else if( Shoot_State == XIAOBOMB_MODE )
			{
				Vision_Send_Data( VISION_RED_XIAO );
			}
		}
	}
	else if(Attack_Color_Choose == ATTACK_NONE)
	{
		Vision_Auto_Attack_Off();
	}
}

/**
  * @brief  �ر�����
  * @param  void
  * @retval void
  * @attention  
  */
void Vision_Auto_Attack_Off(void)
{
	Vision_Send_Data( VISION_OFF );
}

/*******************************�Ӿ�����ȡ*************************************/
/**
  * @brief  ��ȡyaw�������(x��)
  * @param  ���ָ��
  * @retval void
  * @attention  ���Ͻ�Ϊ0,������ʾĿ�����е����,������ʾ���ұ�
  */
void Vision_Error_Yaw(float *error)
{
	//���Ϊ��ʱ��̨����,Ϊ��ʱ����
	if( VisionRecvData.yaw_angle != 0 )
		{
			*error = -(VisionRecvData.yaw_angle - VISION_MID_YAW);
		}
	else
		{
			*error = 0;
		}
}

/**
  * @brief  ��ȡpitch���Ƕ�(y��)
  * @param  ���ָ��
  * @retval void
  * @attention  ���Ͻ�Ϊ0,������ʾĿ�����е��Ϸ�,������ʾ���·�
  */
void Vision_Error_Pitch(float *error)
{	
	*error = VisionRecvData.pitch_angle - VISION_MID_PITCH;
}

/**
  * @brief  ��ȡyaw���Ƕ�
  * @param  ���ָ��
  * @retval void
  * @attention  ������
  */
void Vision_Error_Angle_Yaw(float *error)
{
	//�Ӿ�������,�������̨���ƽǶ�ѡ������(����Ҽ�)
	*error = (-VisionRecvData.yaw_angle + Vision_Comps_Yaw * VisionRecvData.distance/100) 
				* 8192.0f / 360.0f / 20.0f;//������Լ���ŷ���ǵķŴ������˶�Ӧ����
	if(VisionRecvData.yaw_angle == 0)//����
	{
		*error = 0;
	}
}

/**
  * @brief  ��ȡpitch���Ƕ�
  * @param  ���ָ��
  * @retval void
  * @attention  �Ӿ��ϸ�����,ע����̨������̧ͷ���ǵ�ͷ
  */
float XiaoBombCenter = 1.1;
void Vision_Error_Angle_Pitch(float *error)
{	
	if( Shoot_State == BIGBOMB_MODE )
	{
		if( Motor_angel[PITCH] > P_M_MID + 200 )  //�ڱ�̧ͷ����
		{
			Vision_Comps_Pitch = 1.3f;
		}
		else
		{
			Vision_Comps_Pitch = 1.5f;
		}
	}		
	else if( Shoot_State == XIAOBOMB_MODE )//С��ģʽʱ̧ͷ������С��
	{
		Vision_Comps_Pitch = 0;
	}
	
	//�Ӿ��ϸ�����,ע����̨������̧ͷ���ǵ�ͷ(�ϼ��¼���
	*error = (-VisionRecvData.pitch_angle + Vision_Comps_Pitch * VisionRecvData.distance/100)
				* 8192.0f / 360.0f / 20.0f;//��Ϊpitch�ǻ�еģʽ,���԰�ŷ����ת���ɻ�е��
	if(VisionRecvData.pitch_angle == 0)
	{
		*error = 0;
	}
}

/**
  * @brief  ��ȡ����
  * @param  void
  * @retval void
  * @attention  
  */
void Vision_Get_Distance(float *distance)
{
	*distance = VisionRecvData.distance;
	if(VisionRecvData.distance < 0)
	{
		*distance = 0;
	}
}

/*******************�Ӿ���������*********************/

/**
  * @brief  ������ɫѡ��
  * @param  void
  * @retval TRUE��ɫ    FALSE��ɫ
  * @attention  ����ϵͳ���ҵ�һ���ƺ�Ϊʶ���,��Ϊʶ����
  */
bool VISION_IfAutoRed(void)
{
	if (Attack_Color_Choose == ATTACK_RED)
	{
		return TRUE;
	}
	else
	{
		return FALSE;
	}
}

/**
  * @brief  �жϷ��͵�ָ�����Ӿ����յ���ָ���Ƿ���ͬ
  * @param  void
  * @retval TRUEָ��һ��    FALSEָ�һ��
  * @attention  �Ӿ��յ�ʲôָ��,�ͷ�ͬ����ָ�����
  */
bool VISION_IfCmdID_Identical(void)
{
	if (VisionRecvData.CmdID == VisionSendHeader.CmdID)
	{
		return TRUE;
	}
	else
	{
		return FALSE;
	}
}

/**
  * @brief  �ж��Ӿ����ݸ�������
  * @param  void
  * @retval TRUE������   FALSEû����
  * @attention  Ϊ������׼��,���ڿ����ж�ÿ����һ����ͨ��У��,��Vision_Get_New_Data��TRUE
  */
bool Vision_If_Update(void)
{
	return Vision_Get_New_Data;
}

/**
  * @brief  �Ӿ����ݸ��±�־λ�ֶ���0(false)
  * @param  void
  * @retval void
  * @attention  �ǵ�Ҫ����,���������Լ�ѡ,���������������
  */
void Vision_Clean_Update_Flag(void)
{
	Vision_Get_New_Data = FALSE;
}


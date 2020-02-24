#include "vision.h"
#include "math.h"
#include "filter.h"


//右键自瞄

/*-------视觉分辨率预编译--------*/
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
	#define	COMPENSATION_YAW	0        //正是往左偏，负是往右偏
	#define	COMPENSATION_PITCH 0.77      //正是往下偏，负是往上偏
	
	//角度初始化补偿
	float Vision_Comps_Yaw   = COMPENSATION_YAW;
	float Vision_Comps_Pitch = COMPENSATION_PITCH;
	
#elif	INFANTRY_DEBUG_ID == DEBUG_ID_ONE
	#define	COMPENSATION_YAW	(0f)
	#define	COMPENSATION_PITCH	(0f)
	//角度初始化补偿
	float Vision_Comps_Yaw   = COMPENSATION_YAW;
	float Vision_Comps_Pitch = COMPENSATION_PITCH;
#endif
/*----------------------------------------------------------------*/

extVisionSendHeader_t    VisionSendHeader;  //头
extVisionRecvData_t      VisionRecvData;    //视觉接收结构体
extVisionSendData_t      VisionSendData;    //视觉发送结构体

#define ATTACK_NONE    0	//不识别
#define ATTACK_RED     1	//识别红方
#define ATTACK_BLUE    2	//识别蓝方
uint8_t Attack_Color_Choose = ATTACK_NONE;//默认不识别

//角度补偿,发送给视觉
float Vision_Comps_Yaw_Send   = COMPENSATION_YAW;
float Vision_Comps_Pitch_Send = COMPENSATION_PITCH;

//视觉是否发了新数据,FALSE没有,TRUE发了新的
uint8_t Vision_Get_New_Data = FALSE;


/************************************************************************************/
/************************************************************************************/
int yaw_raw_js;
int yaw_kf_js;
/**
  * @brief  读取视觉信息
  * @param  usart4缓存数据
  * @retval void
  * @attention  IRQ执行
  */
void Vision_Read_Data(uint8_t *ReadFromUsart)
{
	//判断帧头数据是否为0xA5
	if(ReadFromUsart[0] == VISION_SOF)
	{
		//帧头CRC8校验
		if(Verify_CRC8_Check_Sum( ReadFromUsart, VISION_LEN_HEADER ) == TRUE)
		{
			//帧尾CRC16校验
			if(Verify_CRC16_Check_Sum( ReadFromUsart, VISION_LEN_PACKED ) == TRUE)
			{
				//接收数据拷贝
				memcpy( &VisionRecvData, ReadFromUsart, VISION_LEN_PACKED);	
				Vision_Get_New_Data = TRUE;//标记视觉数据更新了
			}
		}
	}
	yaw_raw_js = VisionRecvData.distance*1000;
	yaw_kf_js = VisionRecvData.yaw_angle*1000;
}

/**
  * @brief  发送视觉指令
  * @param  CmdID
  * @retval void
  * @attention  按协议打包好数据发送
  */
uint8_t vision_send_pack[50] = {0};//大于22就行
void Vision_Send_Data( uint8_t CmdID )
{
//	uint8_t vision_send_pack[50] = {0};//大于22就行
	int i;    //循环发送次数

	VisionSendHeader.SOF = VISION_SOF;
	VisionSendHeader.CmdID = CmdID;//对视觉来说最重要的数据
	
	//写入帧头
	memcpy( vision_send_pack, &VisionSendHeader, VISION_LEN_HEADER );
	
	//帧头CRC8校验协议
	Append_CRC8_Check_Sum( vision_send_pack, VISION_LEN_HEADER );
	
	if( Get_Attack_attention_Mode() == 1 )  //击打哨兵时，发给视觉1，否则0
		{	VisionSendData.attack_sentry = 1; }
	else
		{	VisionSendData.attack_sentry = 0; }	
	
	if( abs(debug_yaw_speed) < 0.3f )
		{ VisionSendData.kf_yaw_angle = 0; }
	else
		{ VisionSendData.kf_yaw_angle = debug_kf_yaw_angle / 8191 * 360; }  //将预测角度发给视觉，做打弹判断
		
	if( Mobility_Prediction_Yaw == TRUE )
		{	VisionSendData.Pre_Yaw_Open = 1; }
	else
		{ VisionSendData.Pre_Yaw_Open = 0; }
		
	memcpy( vision_send_pack + VISION_LEN_HEADER, &VisionSendData, VISION_LEN_DATA);
	
	//帧尾CRC16校验协议
	Append_CRC16_Check_Sum( vision_send_pack, VISION_LEN_PACKED );
	
	//将打包好的数据通过串口移位发送到裁判系统
	for (i = 0; i < VISION_LEN_PACKED; i++)
	{
		UART4_SendChar( vision_send_pack[i] );
	}
}

/**********************************视觉控制*****************************************/
/**
  * @brief  视觉总控制,指令更新
  * @param  void
  * @retval void
  * @attention  8位,只有键盘模式有视觉
  */
void Vision_Ctrl(void)
{
	if(Get_remoteMode() == KEY)//键盘模式
	{
		Vision_Auto_Attack_Ctrl();
	}
	else
	{
		Vision_Auto_Attack_Off();
	}
}

/**
  * @brief  自瞄控制
  * @param  void
  * @retval void
  * @attention  
  */
void Vision_Auto_Attack_Ctrl(void)
{
	/* 确定敌方颜色 */
	//s1上识别红,下识别蓝,中间不识别
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
	
	
	//向小电脑发送颜色识别指令
	if(Attack_Color_Choose == ATTACK_BLUE)
	{
		if( Get_Chassis_Mode() == 4 ) //取弹模式
		{
			Vision_Send_Data( VISION_RED_GETBOMB ); //颜色与自瞄颜色相反
		}
		else if( GIMBAL_ActionMode() == 3 ) //吊射模式
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
		
		if( Get_Chassis_Mode() == 4 ) //取弹模式
		{
			Vision_Send_Data( VISION_BLUE_GETBOMB );
		}
		else if( GIMBAL_ActionMode() == 3 ) //吊射模式
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
  * @brief  关闭自瞄
  * @param  void
  * @retval void
  * @attention  
  */
void Vision_Auto_Attack_Off(void)
{
	Vision_Send_Data( VISION_OFF );
}

/*******************************视觉误差获取*************************************/
/**
  * @brief  获取yaw误差像素(x轴)
  * @param  误差指针
  * @retval void
  * @attention  左上角为0,负数表示目标在中点左边,正数表示在右边
  */
void Vision_Error_Yaw(float *error)
{
	//输出为负时云台右移,为正时左移
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
  * @brief  获取pitch误差角度(y轴)
  * @param  误差指针
  * @retval void
  * @attention  左上角为0,负数表示目标在中点上方,正数表示在下方
  */
void Vision_Error_Pitch(float *error)
{	
	*error = VisionRecvData.pitch_angle - VISION_MID_PITCH;
}

/**
  * @brief  获取yaw误差角度
  * @param  误差指针
  * @retval void
  * @attention  左负右正
  */
void Vision_Error_Angle_Yaw(float *error)
{
	//视觉左负右正,请根据云台控制角度选择正负(左加右减)
	*error = (-VisionRecvData.yaw_angle + Vision_Comps_Yaw * VisionRecvData.distance/100) 
				* 8192.0f / 360.0f / 20.0f;//请根据自己对欧拉角的放大倍数来乘对应倍数
	if(VisionRecvData.yaw_angle == 0)//发零
	{
		*error = 0;
	}
}

/**
  * @brief  获取pitch误差角度
  * @param  误差指针
  * @retval void
  * @attention  视觉上负下正,注意云台正负是抬头还是低头
  */
float XiaoBombCenter = 1.1;
void Vision_Error_Angle_Pitch(float *error)
{	
	if( Shoot_State == BIGBOMB_MODE )
	{
		if( Motor_angel[PITCH] > P_M_MID + 200 )  //哨兵抬头补偿
		{
			Vision_Comps_Pitch = 1.3f;
		}
		else
		{
			Vision_Comps_Pitch = 1.5f;
		}
	}		
	else if( Shoot_State == XIAOBOMB_MODE )//小弹模式时抬头补偿减小点
	{
		Vision_Comps_Pitch = 0;
	}
	
	//视觉上负下正,注意云台正负是抬头还是低头(上加下减）
	*error = (-VisionRecvData.pitch_angle + Vision_Comps_Pitch * VisionRecvData.distance/100)
				* 8192.0f / 360.0f / 20.0f;//因为pitch是机械模式,所以把欧拉角转换成机械角
	if(VisionRecvData.pitch_angle == 0)
	{
		*error = 0;
	}
}

/**
  * @brief  获取距离
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

/*******************视觉辅助函数*********************/

/**
  * @brief  自瞄颜色选择
  * @param  void
  * @retval TRUE红色    FALSE蓝色
  * @attention  裁判系统左到右第一个灯红为识别红,绿为识别蓝
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
  * @brief  判断发送的指令与视觉接收到的指令是否相同
  * @param  void
  * @retval TRUE指令一样    FALSE指令不一样
  * @attention  视觉收到什么指令,就发同样的指令回来
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
  * @brief  判断视觉数据更新了吗
  * @param  void
  * @retval TRUE更新了   FALSE没更新
  * @attention  为自瞄做准备,串口空闲中断每触发一次且通过校验,则Vision_Get_New_Data置TRUE
  */
bool Vision_If_Update(void)
{
	return Vision_Get_New_Data;
}

/**
  * @brief  视觉数据更新标志位手动置0(false)
  * @param  void
  * @retval void
  * @attention  记得要清零,在哪清零自己选,调用这个函数就行
  */
void Vision_Clean_Update_Flag(void)
{
	Vision_Get_New_Data = FALSE;
}


#ifndef _VISION_H
#define _VISION_H

#include "system.h"


/*--------------------------------暂定协议-------------------------------------*/

#define    VISION_LENGTH        22     		 //暂定22字节,头3字节,数据17字节,尾2字节

//起始字节,协议固定为0xA5
#define    VISION_SOF         (0xA5)

//长度根据协议定义,数据段长度为n需要根据帧头第二字节来获取
#define    VISION_LEN_HEADER    3         //帧头长
#define    VISION_LEN_DATA      17        //数据段长度,可自定义
#define    VISION_LEN_TAIL      2	      //帧尾CRC16
#define    VISION_LEN_PACKED    22        //数据包长度,可自定义

#define    VISION_OFF           (0x00)
#define    VISION_RED_BIG       (0x01)
#define    VISION_RED_XIAO      (0x02)
#define    VISION_BLUE_BIG      (0x03)
#define    VISION_BLUE_XIAO     (0x04)
#define    VISION_RED_GETBOMB   (0x05)
#define    VISION_BLUE_GETBOMB  (0x06)
#define    VISION_RED_BASE      (0x07)
#define    VISION_BLUE_BASE     (0x08)

//可利用收和发的指令码进行比较,当收和发的指令码相同时,可判定为数据可用

//帧头加CRC8校验,保证发送的指令是正确的

//PC收发与STM32收发成镜像关系,以下结构体适用于STM32,PC需稍作修改

typedef __packed struct
{
	/* 头 */
	uint8_t   SOF;			//帧头起始位,暂定0xA5
	uint8_t   CmdID;		//指令
	uint8_t   CRC8;			//帧头CRC校验,保证发送的指令是正确的
	
}extVisionSendHeader_t;


//STM32接收,直接将串口接收到的数据拷贝进结构体
typedef __packed struct
{
	/* 头 */
	uint8_t   SOF;			//帧头起始位,暂定0xA5
	uint8_t   CmdID;		//指令
	uint8_t   CRC8;			//帧头CRC校验,保证发送的指令是正确的
	
	/* 数据 */
	float     pitch_angle;
	float     yaw_angle;
	float     distance;			//距离
	uint8_t   centre_lock;		//是否瞄准到了中间  0没有  1瞄准到了 &&  在补弹模式时，该标志位(0相机无法打开，1可以倒车，2代表停车)
	uint8_t	  identify_target;	//视野内是否有目标/是否识别到了目标   0否  1是
	
	uint8_t   blank_a;			//预留
	uint8_t	  blank_b;
	uint8_t	  In_Your_Face;  //贴脸标志
	
	
	/* 尾 */
	uint16_t  CRC16;       
	
}extVisionRecvData_t;


//STM32发送,直接将打包好的数据一个字节一个字节地发送出去
typedef __packed struct
{
//	/* 头 */
//	uint8_t   SOF;			//帧头起始位,暂定0xA5
//	uint8_t   CmdID;		//指令
//	uint8_t   CRC8;			//帧头CRC校验,保证发送的指令是正确的
	
	/* 数据 */  
	uint8_t    attack_sentry;		 //是否击打哨兵
	float      kf_yaw_angle;     //预测偏移角度
	uint8_t    Pre_Yaw_Open; 
	
	/* 尾 */
	uint16_t  CRC16;
	
}extVisionSendData_t;



//关于如何写入CRC校验值
//我们可以直接利用官方给的CRC代码

//注意,CRC8和CRC16所占字节不一样,8为一个字节,16为2个字节

//写入    CRC8 调用    Append_CRC8_Check_Sum( param1, param2)
//其中 param1代表写好了帧头数据的数组(帧头后的数据还没写没有关系),
//     param2代表CRC8写入后数据长度,我们定义的是头的最后一位,也就是3

//写入    CRC16 调用   Append_CRC16_Check_Sum( param3, param4)
//其中 param3代表写好了   帧头 + 数据  的数组(跟上面是同一个数组)
//     param4代表CRC16写入后数据长度,我们定义的整个数据长度是22,所以是22

/*----------------------------------------------------------*/



extern extVisionRecvData_t    VisionRecvData;//视觉接收结构体

void Vision_Read_Data(uint8_t *ReadFormUsart);
void Vision_Send_Data( uint8_t CmdID );


/****视觉控制*****/
void Vision_Ctrl(void);

void Vision_Auto_Attack_Ctrl(void);
void Vision_Auto_Attack_Off(void);

/*****视觉偏差获取******/
void Vision_Error_Yaw(float *error);
void Vision_Error_Pitch(float *error);
void Vision_Error_Angle_Yaw(float *error);
void Vision_Error_Angle_Pitch(float *error);
void Vision_Get_Distance(float *distance);
void Vision_Compensation(float *compe_yaw, float *compe_pitch);

/********视觉辅助函数*********/
bool VISION_IfAutoRed(void);
bool VISION_IfCmdID_Identical(void);
bool Vision_If_Update(void);
void Vision_Clean_Update_Flag(void);

#endif

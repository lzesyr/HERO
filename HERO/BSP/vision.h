#ifndef _VISION_H
#define _VISION_H

#include "system.h"


/*--------------------------------�ݶ�Э��-------------------------------------*/

#define    VISION_LENGTH        22     		 //�ݶ�22�ֽ�,ͷ3�ֽ�,����17�ֽ�,β2�ֽ�

//��ʼ�ֽ�,Э��̶�Ϊ0xA5
#define    VISION_SOF         (0xA5)

//���ȸ���Э�鶨��,���ݶγ���Ϊn��Ҫ����֡ͷ�ڶ��ֽ�����ȡ
#define    VISION_LEN_HEADER    3         //֡ͷ��
#define    VISION_LEN_DATA      17        //���ݶγ���,���Զ���
#define    VISION_LEN_TAIL      2	      //֡βCRC16
#define    VISION_LEN_PACKED    22        //���ݰ�����,���Զ���

#define    VISION_OFF           (0x00)
#define    VISION_RED_BIG       (0x01)
#define    VISION_RED_XIAO      (0x02)
#define    VISION_BLUE_BIG      (0x03)
#define    VISION_BLUE_XIAO     (0x04)
#define    VISION_RED_GETBOMB   (0x05)
#define    VISION_BLUE_GETBOMB  (0x06)
#define    VISION_RED_BASE      (0x07)
#define    VISION_BLUE_BASE     (0x08)

//�������պͷ���ָ������бȽ�,���պͷ���ָ������ͬʱ,���ж�Ϊ���ݿ���

//֡ͷ��CRC8У��,��֤���͵�ָ������ȷ��

//PC�շ���STM32�շ��ɾ����ϵ,���½ṹ��������STM32,PC�������޸�

typedef __packed struct
{
	/* ͷ */
	uint8_t   SOF;			//֡ͷ��ʼλ,�ݶ�0xA5
	uint8_t   CmdID;		//ָ��
	uint8_t   CRC8;			//֡ͷCRCУ��,��֤���͵�ָ������ȷ��
	
}extVisionSendHeader_t;


//STM32����,ֱ�ӽ����ڽ��յ������ݿ������ṹ��
typedef __packed struct
{
	/* ͷ */
	uint8_t   SOF;			//֡ͷ��ʼλ,�ݶ�0xA5
	uint8_t   CmdID;		//ָ��
	uint8_t   CRC8;			//֡ͷCRCУ��,��֤���͵�ָ������ȷ��
	
	/* ���� */
	float     pitch_angle;
	float     yaw_angle;
	float     distance;			//����
	uint8_t   centre_lock;		//�Ƿ���׼�����м�  0û��  1��׼���� &&  �ڲ���ģʽʱ���ñ�־λ(0����޷��򿪣�1���Ե�����2����ͣ��)
	uint8_t	  identify_target;	//��Ұ���Ƿ���Ŀ��/�Ƿ�ʶ����Ŀ��   0��  1��
	
	uint8_t   blank_a;			//Ԥ��
	uint8_t	  blank_b;
	uint8_t	  In_Your_Face;  //������־
	
	
	/* β */
	uint16_t  CRC16;       
	
}extVisionRecvData_t;


//STM32����,ֱ�ӽ�����õ�����һ���ֽ�һ���ֽڵط��ͳ�ȥ
typedef __packed struct
{
//	/* ͷ */
//	uint8_t   SOF;			//֡ͷ��ʼλ,�ݶ�0xA5
//	uint8_t   CmdID;		//ָ��
//	uint8_t   CRC8;			//֡ͷCRCУ��,��֤���͵�ָ������ȷ��
	
	/* ���� */  
	uint8_t    attack_sentry;		 //�Ƿ�����ڱ�
	float      kf_yaw_angle;     //Ԥ��ƫ�ƽǶ�
	uint8_t    Pre_Yaw_Open; 
	
	/* β */
	uint16_t  CRC16;
	
}extVisionSendData_t;



//�������д��CRCУ��ֵ
//���ǿ���ֱ�����ùٷ�����CRC����

//ע��,CRC8��CRC16��ռ�ֽڲ�һ��,8Ϊһ���ֽ�,16Ϊ2���ֽ�

//д��    CRC8 ����    Append_CRC8_Check_Sum( param1, param2)
//���� param1����д����֡ͷ���ݵ�����(֡ͷ������ݻ�ûдû�й�ϵ),
//     param2����CRC8д������ݳ���,���Ƕ������ͷ�����һλ,Ҳ����3

//д��    CRC16 ����   Append_CRC16_Check_Sum( param3, param4)
//���� param3����д����   ֡ͷ + ����  ������(��������ͬһ������)
//     param4����CRC16д������ݳ���,���Ƕ�����������ݳ�����22,������22

/*----------------------------------------------------------*/



extern extVisionRecvData_t    VisionRecvData;//�Ӿ����սṹ��

void Vision_Read_Data(uint8_t *ReadFormUsart);
void Vision_Send_Data( uint8_t CmdID );


/****�Ӿ�����*****/
void Vision_Ctrl(void);

void Vision_Auto_Attack_Ctrl(void);
void Vision_Auto_Attack_Off(void);

/*****�Ӿ�ƫ���ȡ******/
void Vision_Error_Yaw(float *error);
void Vision_Error_Pitch(float *error);
void Vision_Error_Angle_Yaw(float *error);
void Vision_Error_Angle_Pitch(float *error);
void Vision_Get_Distance(float *distance);
void Vision_Compensation(float *compe_yaw, float *compe_pitch);

/********�Ӿ���������*********/
bool VISION_IfAutoRed(void);
bool VISION_IfCmdID_Identical(void);
bool Vision_If_Update(void);
void Vision_Clean_Update_Flag(void);

#endif

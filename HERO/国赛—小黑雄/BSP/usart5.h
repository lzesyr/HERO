#ifndef __USART5_H
#define __USART5_H
#include "system.h"

void usart5_Init(void);
void Re_Date_From_judgement(unsigned char *judgementDate);
void Send_Data_To_Judgement(void);
void Seng_Data_To_TeamMate(void);
void gimbaler_send_sentry(void);
bool is_red_or_blue(void);
void determine_ID(void);

#define CMD_ID_SIZE 	 		 2
#define FRAME_TAIL_SIZE 	 2

#define  RED   1
#define  BlUE  0


typedef struct 
{
	bool  judgemengt_connecting;
	float power_remain;
	int   connecting_num;
}PowerProtect_t;


#define    HEADER_LENGTH    5
#define    CMDID_LENGTH     2 
#define    TAIL_LENGTH      2

typedef enum 
{
	game_state                = 0x0001,   //����״̬���ݣ�1Hz���ڷ���
	game_result               = 0x0002,   //����������ݣ�������������
	game_robot_survivors      = 0x0003,   //���������˴�����ݣ�1Hz����
	event_data                = 0x0101,   //�����¼����ݣ��¼��ı����
	supply_projectile_action  = 0x0102,   //���ز���վ������ʶ���ݣ������ı����
	supply_projectile_booking = 0x0103,   //���ز���վԤԼ�ӵ����ݣ��ɲ����ӷ��ͣ�����10Hz
	game_robot_state          = 0x0201,   //����������״̬��10Hz���ڷ���
	power_heat_data           = 0x0202,   //ʵʱ�����������ݣ�50Hz���ڷ���
	game_robot_pos            = 0x0203,   //������λ�����ݣ�10Hz����
	buff_musk                 = 0x0204,   //�������������ݣ�����״̬�ı����
	aerial_robot_energy				= 0x0205,   //���л���������״̬���ݣ�10Hz���ڷ��ͣ�ֻ�п��л��������ط���
	robot_hurt								= 0x0206,		//�˺�״̬���ݣ��˺���������
	shoot_data							  = 0x0207,		//ʵʱ������ݣ��ӵ��������
	TeamMate_data             = 0x0301,   //���ն�����Ϣ
}Cmd_ID;

typedef __packed struct       //0x0001������״̬����
{
 uint8_t game_type : 4;       //��������
 uint8_t game_progress : 4;		//��ǰ�����׶�
 uint16_t stage_remain_time;	//��ǰ�׶�ʣ��ʱ��
} ext_game_state_t;

typedef __packed struct      	//0x0002�������������
{
 uint8_t winner;							//0ƽ�֣�1��ʤ��2��ʤ
} ext_game_result_t;

typedef __packed struct       //0x0003,�����˴������
{
 uint16_t robot_legion;				//��Ӧ��λ1�����0����
} ext_game_robot_survivors_t;

typedef __packed struct       //0x0101�������¼�����
{
 uint32_t event_type;
} ext_event_data_t;


typedef __packed struct       //0x0102������վ������ʶ
{
 uint8_t supply_projectile_id;    //����վ��ID
 uint8_t supply_robot_id; 				//ԤԼ������ID
 uint8_t supply_projectile_step;  //�ӵ��ڿ���״̬��0�أ�1׼���У�2����
} ext_supply_projectile_action_t;

typedef __packed struct       //0x0103������վԤԼ�ӵ�
{
 uint8_t supply_projectile_id;//ԤԼ����վ��ID
 uint8_t supply_num;					//ԤԼ�ӵ���Ŀ
} ext_supply_projectile_booking_t;

typedef __packed struct       //0x0201������������״̬
{
 uint8_t robot_id;						//������ID
 uint8_t robot_level;					//�����˵ȼ���1����1��
 uint16_t remain_HP;					//������ʣ��Ѫ��
 uint16_t max_HP;							//��������Ѫ��
 uint16_t shooter_heat0_cooling_rate;    //������17mm�ӵ�������ȴ�ٶ�
 uint16_t shooter_heat0_cooling_limit;   //������17mm�ӵ���������
 uint16_t shooter_heat1_cooling_rate;    //������42mm�ӵ�������ȴ�ٶ�
 uint16_t shooter_heat1_cooling_limit;   //������42mm�ӵ���������
 uint8_t mains_power_gimbal_output : 1;  //��̨���������0�ޣ�1��24V
 uint8_t mains_power_chassis_output : 1; //�����������
 uint8_t mains_power_shooter_output : 1; //ǹ���������
} ext_game_robot_state_t;

typedef __packed struct       //0x0202��ʵʱ������������
{
 uint16_t chassis_volt;       //���������ѹ
 uint16_t chassis_current;    //�����������
 float chassis_power;         //�����������
 uint16_t chassis_power_buffer; //���̹��ʻ���
 uint16_t shooter_heat0; 			//17mmǹ������
 uint16_t shooter_heat1; 			//42mmǹ������
} ext_power_heat_data_t;			

typedef __packed struct       //0x0203��������λ��
{
 float x;											//λ��x����
 float y;											//λ��y����
 float z;											//λ��z����
 float yaw;										//λ��ǹ��
} ext_game_robot_pos_t;

typedef __packed struct       //0x0204,����������
{
 uint8_t power_rune_buff;			//bit0��Ѫ״̬��1ǹ����ȴ����
}ext_buff_musk_t;             //bit2�����˷����ӳɣ�3�����ӳ�

typedef __packed struct       //0x0205�����л���������״̬
{
 uint8_t energy_point;        //���۵�������
 uint8_t attack_time;					//�ɹ���ʱ��
} aerial_robot_energy_t;

typedef __packed struct       //0x0206���˺�״̬
{
 uint8_t armor_id : 4;        //bit0-3��װ�а��˺���0-4����5��װ�װ�
 uint8_t hurt_type : 4;       //0x0װ���˺���Ѫ��1ģ����ߣ�2ǹ��������3���̳�����
} ext_robot_hurt_t;

typedef __packed struct       //0x0207��ʵʱ�����Ϣ
{
 uint8_t bullet_type;         //�ӵ����ͣ�1��17mm  2��42mm
 uint8_t bullet_freq;					//�ӵ���Ƶ
 float bullet_speed;					//�ӵ�����
} ext_shoot_data_t;

/*  ---------  �����˼佻������  ---------  */
/* ������ID��1Ӣ�ۣ��죩�� 2���̣��죩��   3/4/5�������죩�� 6���У��죩�� 7�ڱ����죩
            11Ӣ�ۣ�������12���̣�������13/14/15������������16���У��죩��17�ڱ�������

	 �ͻ���ID��0x0101ΪӢ�۲����ֿͻ��ˣ��죩��0x0102���̣��죩���Դ�����
  
	 ����ID        ���ȣ�ͷ�ṹ����+�������ݶγ��ȣ�        ����˵��
   0xD180               6 + 13                          �ͻ����Զ�������
   0x0200-0x02ff        6 + n                           ���������˼�ͨ��   

*/
typedef __packed struct       //0x0301���������ݽ�����Ϣ
{
 uint16_t data_cmd_id;				//���ݶε�����ID
 uint16_t send_ID;						//�����ߵ�ID
 uint16_t receiver_ID;        //�����ߵ�ID
}ext_student_interactive_header_data_t;

typedef __packed struct       //�ͻ����Զ������ݣ�cmd_id:0x0301  ����ID��0xD180
{
float data1;
float data2;
float data3;
uint8_t masks;
} client_custom_data_t;


typedef __packed struct       //�����˼�ͨ�ţ�cmd_id0x0301, ����ID��0x0200-0x02ff
{
	uint8_t data[10];
} robot_interactive_data_t;



typedef __packed struct
{
	uint8_t 	SQF; 			 	//����֡��ʼ�ֽڣ��̶�ֵΪ0xA5
	uint16_t 	DataLength;	//����֡��Data����
	uint8_t 	Seq;			  //�����
	uint8_t 	CRC8;			  //֡ͷCRCУ��
}extFrameHeader;

//֡ͷ  ������   ���ݶ�ͷ�ṹ  ���ݶ�   ֡β
//�ϴ��ͻ���
typedef __packed struct
{
	extFrameHeader   txFrameHeader;//֡ͷ
	uint16_t		 CmdID;//������
	ext_student_interactive_header_data_t   dataFrameHeader;//���ݶ�ͷ�ṹ
	client_custom_data_t  	 clientData;//���ݶ�
	uint16_t		 FrameTail;//֡β
}exSendClientData_t;


//�����˽�����Ϣ
typedef __packed struct
{
	extFrameHeader   txFrameHeader;//֡ͷ
	uint16_t		 CmdID;//������
	ext_student_interactive_header_data_t   dataFrameHeader;//���ݶ�ͷ�ṹ
	robot_interactive_data_t  	 interactData;//���ݶ�
	uint16_t		 FrameTail;//֡β
}exCommunatianData_t;

//���ն�����Ϣ
typedef __packed struct
{
	ext_student_interactive_header_data_t   dataFrameHeader;//���ݶ�ͷ�ṹ
	robot_interactive_data_t  	 interactData;//���ݶ�
}ext_teammate_data_t;


extern ext_game_state_t                  GameState;
extern ext_game_result_t                 GameResult;
extern ext_game_robot_survivors_t        GameRobotSurvivors ;
extern ext_event_data_t                  EventData;
extern ext_supply_projectile_action_t    SupplyProjectileAction;
extern ext_supply_projectile_booking_t   SupplyProjectileBooking;
extern ext_game_robot_state_t            GameRobotState;
extern ext_power_heat_data_t             PowerHeatData;
extern ext_game_robot_pos_t              GameRobotPos;
extern ext_buff_musk_t                   BuffMusk;
extern aerial_robot_energy_t             AerialRobotEnergy;
extern ext_robot_hurt_t                  RobotHurt;
extern ext_shoot_data_t                  ShootData;
extern ext_teammate_data_t               TeamMateData;


extern PowerProtect_t  PowerProtect;
extern float current_heat42, current_heat17;   //ǹ������
extern bool Shoot_Data_42_Update, Shoot_Data_17_Update;   //ǹ�����ݸ���
extern int32_t Shot_42_Num, Shot_17_Num;
extern bool IF_Hurt;

#endif


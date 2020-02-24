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
	game_state                = 0x0001,   //比赛状态数据，1Hz周期发送
	game_result               = 0x0002,   //比赛结果数据，比赛结束后发送
	game_robot_survivors      = 0x0003,   //比赛机器人存活数据，1Hz发送
	event_data                = 0x0101,   //场地事件数据，事件改变后发送
	supply_projectile_action  = 0x0102,   //场地补给站动作标识数据，动作改变后发送
	supply_projectile_booking = 0x0103,   //场地补给站预约子弹数据，由参赛队发送，上限10Hz
	game_robot_state          = 0x0201,   //机器人数据状态，10Hz周期发送
	power_heat_data           = 0x0202,   //实时功率热量数据，50Hz周期发送
	game_robot_pos            = 0x0203,   //机器人位置数据，10Hz发送
	buff_musk                 = 0x0204,   //机器人增益数据，增益状态改变后发送
	aerial_robot_energy				= 0x0205,   //空中机器人能量状态数据，10Hz周期发送，只有空中机器人主控发送
	robot_hurt								= 0x0206,		//伤害状态数据，伤害发生后发送
	shoot_data							  = 0x0207,		//实时射击数据，子弹发射后发送
	TeamMate_data             = 0x0301,   //接收队友信息
}Cmd_ID;

typedef __packed struct       //0x0001，比赛状态数据
{
 uint8_t game_type : 4;       //比赛类型
 uint8_t game_progress : 4;		//当前比赛阶段
 uint16_t stage_remain_time;	//当前阶段剩余时间
} ext_game_state_t;

typedef __packed struct      	//0x0002，比赛结果数据
{
 uint8_t winner;							//0平局，1红胜，2蓝胜
} ext_game_result_t;

typedef __packed struct       //0x0003,机器人存活数据
{
 uint16_t robot_legion;				//对应的位1代表存活，0阵亡
} ext_game_robot_survivors_t;

typedef __packed struct       //0x0101，场地事件数据
{
 uint32_t event_type;
} ext_event_data_t;


typedef __packed struct       //0x0102，补给站动作标识
{
 uint8_t supply_projectile_id;    //补给站口ID
 uint8_t supply_robot_id; 				//预约机器人ID
 uint8_t supply_projectile_step;  //子弹口开闭状态，0关，1准备中，2下落
} ext_supply_projectile_action_t;

typedef __packed struct       //0x0103，补给站预约子弹
{
 uint8_t supply_projectile_id;//预约补给站口ID
 uint8_t supply_num;					//预约子弹数目
} ext_supply_projectile_booking_t;

typedef __packed struct       //0x0201，比赛机器人状态
{
 uint8_t robot_id;						//机器人ID
 uint8_t robot_level;					//机器人等级，1代表1级
 uint16_t remain_HP;					//机器人剩余血量
 uint16_t max_HP;							//机器人满血量
 uint16_t shooter_heat0_cooling_rate;    //机器人17mm子弹热量冷却速度
 uint16_t shooter_heat0_cooling_limit;   //机器人17mm子弹热量上限
 uint16_t shooter_heat1_cooling_rate;    //机器人42mm子弹热量冷却速度
 uint16_t shooter_heat1_cooling_limit;   //机器人42mm子弹热量上限
 uint8_t mains_power_gimbal_output : 1;  //云台有无输出，0无，1有24V
 uint8_t mains_power_chassis_output : 1; //底盘有无输出
 uint8_t mains_power_shooter_output : 1; //枪管有无输出
} ext_game_robot_state_t;

typedef __packed struct       //0x0202，实时功率热量数据
{
 uint16_t chassis_volt;       //底盘输出电压
 uint16_t chassis_current;    //底盘输出电流
 float chassis_power;         //底盘输出功率
 uint16_t chassis_power_buffer; //底盘功率缓冲
 uint16_t shooter_heat0; 			//17mm枪口热量
 uint16_t shooter_heat1; 			//42mm枪口热量
} ext_power_heat_data_t;			

typedef __packed struct       //0x0203，机器人位置
{
 float x;											//位置x坐标
 float y;											//位置y坐标
 float z;											//位置z坐标
 float yaw;										//位置枪口
} ext_game_robot_pos_t;

typedef __packed struct       //0x0204,机器人增益
{
 uint8_t power_rune_buff;			//bit0补血状态，1枪口冷却加速
}ext_buff_musk_t;             //bit2机器人防御加成，3攻击加成

typedef __packed struct       //0x0205，空中机器人能量状态
{
 uint8_t energy_point;        //积累的能量点
 uint8_t attack_time;					//可攻击时间
} aerial_robot_energy_t;

typedef __packed struct       //0x0206，伤害状态
{
 uint8_t armor_id : 4;        //bit0-3，装夹板伤害，0-4代表5个装甲板
 uint8_t hurt_type : 4;       //0x0装夹伤害扣血，1模块掉线，2枪口热量，3底盘超功率
} ext_robot_hurt_t;

typedef __packed struct       //0x0207，实时射击信息
{
 uint8_t bullet_type;         //子弹类型：1：17mm  2：42mm
 uint8_t bullet_freq;					//子弹射频
 float bullet_speed;					//子弹射速
} ext_shoot_data_t;

/*  ---------  机器人间交互数据  ---------  */
/* 机器人ID：1英雄（红）， 2工程（红），   3/4/5步兵（红）， 6空中（红）， 7哨兵（红）
            11英雄（蓝），12工程（蓝），13/14/15步兵（蓝），16空中（红），17哨兵（蓝）

	 客户端ID：0x0101为英雄操作手客户端（红），0x0102工程（红），以此类推
  
	 内容ID        长度（头结构长度+内容数据段长度）        功能说明
   0xD180               6 + 13                          客户端自定义数据
   0x0200-0x02ff        6 + n                           己方机器人间通信   

*/
typedef __packed struct       //0x0301，交互数据接收信息
{
 uint16_t data_cmd_id;				//数据段的内容ID
 uint16_t send_ID;						//发送者的ID
 uint16_t receiver_ID;        //接受者的ID
}ext_student_interactive_header_data_t;

typedef __packed struct       //客户端自定义数据：cmd_id:0x0301  内容ID：0xD180
{
float data1;
float data2;
float data3;
uint8_t masks;
} client_custom_data_t;


typedef __packed struct       //机器人间通信：cmd_id0x0301, 内容ID：0x0200-0x02ff
{
	uint8_t data[10];
} robot_interactive_data_t;



typedef __packed struct
{
	uint8_t 	SQF; 			 	//数据帧起始字节，固定值为0xA5
	uint16_t 	DataLength;	//数据帧内Data长度
	uint8_t 	Seq;			  //包序号
	uint8_t 	CRC8;			  //帧头CRC校验
}extFrameHeader;

//帧头  命令码   数据段头结构  数据段   帧尾
//上传客户端
typedef __packed struct
{
	extFrameHeader   txFrameHeader;//帧头
	uint16_t		 CmdID;//命令码
	ext_student_interactive_header_data_t   dataFrameHeader;//数据段头结构
	client_custom_data_t  	 clientData;//数据段
	uint16_t		 FrameTail;//帧尾
}exSendClientData_t;


//机器人交互信息
typedef __packed struct
{
	extFrameHeader   txFrameHeader;//帧头
	uint16_t		 CmdID;//命令码
	ext_student_interactive_header_data_t   dataFrameHeader;//数据段头结构
	robot_interactive_data_t  	 interactData;//数据段
	uint16_t		 FrameTail;//帧尾
}exCommunatianData_t;

//接收队友信息
typedef __packed struct
{
	ext_student_interactive_header_data_t   dataFrameHeader;//数据段头结构
	robot_interactive_data_t  	 interactData;//数据段
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
extern float current_heat42, current_heat17;   //枪口热量
extern bool Shoot_Data_42_Update, Shoot_Data_17_Update;   //枪口数据更新
extern int32_t Shot_42_Num, Shot_17_Num;
extern bool IF_Hurt;

#endif


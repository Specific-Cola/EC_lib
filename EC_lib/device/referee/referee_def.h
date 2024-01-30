#ifndef REFEREE_INFO_H__
#define REFEREE_INFO_H__

#include "struct_typedef.h"
#include "main.h"

/*****************************1.6.1裁判系统数据**********************/
#define ROBOT_INTERACTION_DATA_LENGTH      113u
#define CUSTOM_ROBOT_DATA_LENGTH            30u
#define HEADER_SOF                         0xA5

typedef __PACKED_STRUCT
{
    uint8_t SOF;          // 固定枕头    0xA5
    uint16_t data_length; // 数据帧中data的长度
    uint8_t seq;          // 包序号
    uint8_t CRC8;         // 枕头CRC8校验
}
frame_header_t;


/****************************cmd_id命令码说明****************************/

/* 命令码ID,用来判断接收的是什么数据 */
typedef enum
{
    ID_game_state = 0x0001,                // 比赛状态数据
    ID_game_result = 0x0002,               // 比赛结果数据
    ID_game_robot_survivors = 0x0003,      // 比赛机器人血量数据
    ID_event_data = 0x0101,                // 场地事件数据
    ID_supply_projectile_action = 0x0102,  // 场地补给站动作标识数据
    ID_supply_projectile_booking = 0x0103, // 场地补给站预约子弹数据
    ID_judge_warning = 0x0104,             // 裁判警告数据
    ID_dart_data = 0x0105,                 // 飞镖发射相关数据
    ID_game_robot_state = 0x0201,          // 机器人状态数据
    ID_power_heat_data = 0x0202,           // 实时功率热量数据
    ID_game_robot_pos = 0x0203,            // 机器人位置数据
    ID_buff_musk = 0x0204,                 // 机器人增益数据
    ID_aerial_robot_energy = 0x0205,       // 空中机器人能量状态数据
    ID_robot_hurt = 0x0206,                // 伤害状态数据
    ID_shoot_data = 0x0207,                // 实时射击数据
    ID_bullets_limit = 0x0208,             // 允许发单量
    ID_rfid_data = 0x0209,                 // RFID模块状态
    ID_dart_command = 0x020A,              // 飞镖选手指令数据
    ID_all_robot_pos = 0x020B,             // 发给哨兵的机器人位置数据
    ID_radar_marking = 0x020C,             // 雷达标记进度
    ID_sentry_message_sync = 0x202D,       // 哨兵自主决策信息同步
    ID_radar_message_sync = 0x020E,        // 雷达自主决策信息同步
    ID_student_interactive = 0x0301,       // 机器人间交互数据
    //  后续更新

} CmdID_e;

/* 命令码数据段长,根据官方协议来定义长度，还有自定义数据长度 */
typedef enum
{
    LEN_game_state = 11,                                  // 0x0001
    LEN_game_result = 1,                                  // 0x0002
    LEN_game_robot_HP = 32,                               // 0x0003
    LEN_event_data = 4,                                   // 0x0101
    LEN_supply_projectile_action = 4,                     // 0x0102
    LEN_judge_warning = 3,                                // 0x0104
    LEN_dart_data = 3,                                    // 0x0105
    LEN_game_robot_state = 13,                            // 0x0201
    LEN_power_heat_data = 16,                             // 0x0202
    LEN_game_robot_pos = 16,                              // 0x0203
    LEN_buff_musk = 6,                                    // 0x0204
    LEN_aerial_robot_energy = 2,                          // 0x0205
    LEN_robot_hurt = 1,                                   // 0x0206
    LEN_shoot_data = 7,                                   // 0x0207
    LEN_bullets_limit = 6,                                // 0x0208
    LEN_rfid_data = 4,                                    // 0x0209,
    LEN_dart_command = 6,                                 // 0x020A,
    LEN_all_robot_pos = 40,                                // 0x020B,
    LEN_radar_marking = 6,                                 // 0x020C,
    LEN_sentry_message_sync = 4,                           // 0x202D,
    LEN_radar_message_sync = 1,                            // 0x020E,
    LEN_receive_data = 6 + ROBOT_INTERACTION_DATA_LENGTH, // 0x0301

} RefereeDataLength_e;

/******************具体各种命令码内容***************/

typedef __PACKED_STRUCT
{ // 0x0001
    uint8_t game_type : 4;
    uint8_t game_progress : 4;
    uint16_t stage_remain_time;
    uint64_t SyncTimeStamp;
}
game_status_t;

typedef __PACKED_STRUCT
{ // 0x0002
    uint8_t winner;
}
game_result_t;

typedef __PACKED_STRUCT
{ // 0x0003
    uint16_t red_1_robot_HP;
    uint16_t red_2_robot_HP;
    uint16_t red_3_robot_HP;
    uint16_t red_4_robot_HP;
    uint16_t red_5_robot_HP;
    uint16_t red_7_robot_HP;
    uint16_t red_outpost_HP;
    uint16_t red_base_HP;
    uint16_t blue_1_robot_HP;
    uint16_t blue_2_robot_HP;
    uint16_t blue_3_robot_HP;
    uint16_t blue_4_robot_HP;
    uint16_t blue_5_robot_HP;
    uint16_t blue_7_robot_HP;
    uint16_t blue_outpost_HP;
    uint16_t blue_base_HP;
}
game_robot_HP_t;

typedef __PACKED_STRUCT
{ // 0x0101
    uint32_t event_data;
}
event_data_t;

typedef __PACKED_STRUCT // 0x0102
{
    uint8_t reserved;
    uint8_t supply_robot_id;
    uint8_t supply_projectile_step;
    uint8_t supply_projectile_num;
}
ext_supply_projectile_action_t;

typedef __PACKED_STRUCT // 0x0104
{
    uint8_t level;
    uint8_t offending_robot_id;
    uint8_t count;
}
referee_warning_t;

typedef __PACKED_STRUCT // 0x0105
{
    uint8_t dart_remaining_time;
    uint16_t dart_info;
}
dart_info_t;

typedef __PACKED_STRUCT // 0x0201
{
    uint8_t robot_id;
    uint8_t robot_level;
    uint16_t current_HP;
    uint16_t maximum_HP;
    uint16_t shooter_barrel_cooling_value;
    uint16_t shooter_barrel_heat_limit;
    uint16_t chassis_power_limit;
    uint8_t power_management_gimbal_output : 1;
    uint8_t power_management_chassis_output : 1;
    uint8_t power_management_shooter_output : 1;
}
robot_status_t;

typedef __PACKED_STRUCT // 0202
{
    uint16_t chassis_voltage;
    uint16_t chassis_current;
    float chassis_power;
    uint16_t buffer_energy;
    uint16_t shooter_17mm_1_barrel_heat;
    uint16_t shooter_17mm_2_barrel_heat;
    uint16_t shooter_42mm_barrel_heat;
}
power_heat_data_t;

typedef __PACKED_STRUCT // 0x0203
{
    float x;
    float y;
    float angle;
}
robot_pos_t;

typedef __PACKED_STRUCT // 0204
{
    uint8_t recovery_buff;
    uint8_t cooling_buff;
    uint8_t defence_buff;
    uint8_t vulnerability_buff;
    uint16_t attack_buff;
}
buff_t;

typedef __PACKED_STRUCT // 0205
{
    uint8_t airforce_status;
    uint8_t time_remain;
}
air_support_data_t;

typedef __PACKED_STRUCT // 0206
{
    uint8_t armor_id : 4;
    uint8_t HP_deduction_reason : 4;
}
hurt_data_t;

typedef __PACKED_STRUCT // 0207
{
    uint8_t bullet_type;
    uint8_t shooter_number;
    uint8_t launching_frequency;
    float initial_speed;
}
shoot_data_t;

typedef __PACKED_STRUCT // 0208
{
    uint16_t projectile_allowance_17mm;
    uint16_t projectile_allowance_42mm;
    uint16_t remaining_gold_coin;
}
projectile_allowance_t;

typedef __PACKED_STRUCT // 0209
{
    uint32_t rfid_status;
}
rfid_status_t;

typedef __PACKED_STRUCT // 020A
{
    uint8_t dart_launch_opening_status;
    uint8_t reserved;
    uint16_t target_change_time;
    uint16_t latest_launch_cmd_time;
}
dart_client_cmd_t;

typedef __PACKED_STRUCT // 020B
{
    float hero_x;
    float hero_y;
    float engineer_x;
    float engineer_y;
    float standard_3_x;
    float standard_3_y;
    float standard_4_x;
    float standard_4_y;
    float standard_5_x;
    float standard_5_y;
}
ground_robot_position_t;

typedef __PACKED_STRUCT // 020C
{

    uint8_t mark_hero_progress;
    uint8_t mark_engineer_progress;
    uint8_t mark_standard_3_progress;
    uint8_t mark_standard_4_progress;
    uint8_t mark_standard_5_progress;
    uint8_t mark_sentry_progress;
}
radar_mark_data_t;

typedef __PACKED_STRUCT // 020D
{
    uint32_t sentry_info;
}
sentry_info_t;

typedef __PACKED_STRUCT // 020E
{
    uint8_t radar_info;
}
radar_info_t;

typedef __PACKED_STRUCT // 0301
{
    uint16_t data_cmd_id;
    uint16_t sender_id;
    uint16_t receiver_id;
    uint8_t user_data[ROBOT_INTERACTION_DATA_LENGTH];
}
robot_interaction_data_t;

typedef __PACKED_STRUCT // 0100
{
    uint8_t delete_type;
    uint8_t layer;
}
interaction_layer_delete_t;

typedef __PACKED_STRUCT // 0101
{
    uint8_t figure_name[3];
    uint32_t operate_tpye : 3;
    uint32_t figure_tpye : 3;
    uint32_t layer : 4;
    uint32_t color : 4;
    uint32_t details_a : 9;
    uint32_t details_b : 9;
    uint32_t width : 10;
    uint32_t start_x : 11;
    uint32_t start_y : 11;
    uint32_t details_c : 10;
    uint32_t details_d : 11;
    uint32_t details_e : 11;
}
interaction_figure_t;

typedef __PACKED_STRUCT // 0102
{
    interaction_figure_t interaction_figure[2];
}
interaction_figure_2_t;

typedef __PACKED_STRUCT // 0103
{
    interaction_figure_t interaction_figure[5];
}
interaction_figure_3_t;

typedef __PACKED_STRUCT // 0104
{
    interaction_figure_t interaction_figure[7];
}
interaction_figure_4_t;

//typedef __PACKED_STRUCT // 0110
//{
//    graphic_data_struct_t grapic_data_struct;
//    uint8_t data[30];
//}
//ext_client_custom_character_t;

typedef __PACKED_STRUCT // 0120
{
    uint32_t sentry_cmd;
}
sentry_cmd_t;

typedef __PACKED_STRUCT // 0121
{
    uint8_t radar_cmd;
}
radar_cmd_t;

typedef __PACKED_STRUCT // 0303
{
    float target_position_x;
    float target_position_y;
    uint8_t cmd_keyboard;
    uint8_t target_robot_id;
    uint8_t cmd_source;
}
map_command_t;

typedef __PACKED_STRUCT // 0305
{
    uint16_t target_robot_id;
    float target_position_x;
    float target_position_y;
}
map_robot_data_t;

typedef __PACKED_STRUCT // 0307
{
    uint8_t intention;
    uint16_t start_position_x;
    uint16_t start_position_y;
    int8_t delta_x[49];
    int8_t delta_y[49];
    uint16_t sender_id;
}
map_data_t;

typedef __PACKED_STRUCT // 0308
{
    uint16_t sender_id;
    uint16_t receiver_id;
    uint8_t user_data[30];
}
custom_info_t;

typedef __PACKED_STRUCT // 0302
{
    uint8_t data[CUSTOM_ROBOT_DATA_LENGTH];
}
custom_robot_data_t;

typedef __PACKED_STRUCT // 0304
{
    int16_t mouse_x;
    int16_t mouse_y;
    int16_t mouse_z;
    int8_t left_button_down;
    int8_t right_button_down;
    uint16_t keyboard_value;
    uint16_t reserved;
}
remote_control_t;

typedef __PACKED_STRUCT // 0x0306
{
    uint16_t key_value;
    uint16_t x_position : 12;
    uint16_t mouse_left : 4;
    uint16_t y_position : 12;
    uint16_t mouse_right : 4;
    uint16_t reserved;
}
custom_client_data_t;

#endif // !REFEREE_INFO_H__

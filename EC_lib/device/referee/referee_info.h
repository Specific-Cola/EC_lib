#ifndef REFEREE_INFO_H__
#define REFEREE_INFO_H__

#include "struct_typedef.h"
#include "main.h"

/*****************************1.6.1裁判系统数据**********************/
#define ROBOT_INTERACTION_DATA_LENGTH      113u
#define CUSTOM_ROBOT_DATA_LENGTH            30u

typedef __PACKED_STRUCT
{
    uint8_t SOF;          // 固定枕头    0xA5
    uint16_t data_length; // 数据帧中data的长度
    uint8_t seq;          // 包序号
    uint8_t CRC8;         // 枕头CRC8校验
}
frame_header_t;

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

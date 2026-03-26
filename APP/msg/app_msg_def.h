//
// Created by 3545 on 25-11-30.
//

#ifndef APP_MSG_DEF_H
#define APP_MSG_DEF_H
#include <cstdint>

struct app_msg_gimbal_to_chassis {
    //底盘运动数据
    float s_yaw_pos_equally;
    float vx, vy, rotate;
    float b_yaw_pos;
    float chassis_power_limit;
    float k_rotate;

} __attribute__((packed));

struct app_msg_chassis_to_gimbal {
    uint16_t robot_hp;
    uint8_t game_state;
    uint16_t shooter_heat;
    uint16_t shooter_heat_limit;
    uint16_t shooter_heat_ps;
    uint16_t robot_allow_armor;
    uint8_t chassis_all_motor_ready_flag;

} __attribute__((packed));

#endif //APP_MSG_DEF_H

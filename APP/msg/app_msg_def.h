//
// Created by 3545 on 25-11-30.
//

#ifndef APP_MSG_DEF_H
#define APP_MSG_DEF_H
#include <cstdint>

struct app_msg_gimbal_to_chassis {
    //底盘运动数据
    float ins_yaw;
    float vx, vy, rotate;
    int b_yaw_cnt;

} __attribute__((packed));

struct app_msg_chassis_to_gimbal {
    uint8_t robot_id, robot_level;

} __attribute__((packed));

#endif //APP_MSG_DEF_H

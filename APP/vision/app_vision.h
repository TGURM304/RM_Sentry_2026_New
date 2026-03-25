//
// Created by fish on 2024/12/18.
//

#pragma once

#include <cstdint>

namespace vision {
struct SendPacket {
    uint8_t head[2]    = { 'T', 'G' };
    uint8_t mode       = 0;              // 0: 空闲, 1: 自瞄, 2: 小符, 3: 大符
    float q[4]         = { 0, 0, 0, 0 }; // wxyz顺序
    float yaw          = 0;
    float yaw_vel      = 0;
    float pitch        = 0;
    float pitch_vel    = 0;
    float bullet_speed = 0;
    // uint16_t bullet_count = 0;
    uint8_t is_start = 0;
    uint16_t hp      = 0;
    uint16_t crc16;
} __attribute__((packed));
static_assert(sizeof(SendPacket) <= 64);

struct RecvPacket {
    uint8_t head[2] = { 'T', 'G' };
    uint8_t mode; // 0: 不控制, 1: 控制云台但不开火，2: 控制云台且开火
    float yaw;
    float yaw_vel;
    float yaw_acc;
    float pitch;
    float pitch_vel;
    float pitch_acc;
    float nav_x;
    float nav_y;
    float nav_z;
    uint16_t crc16;
} __attribute__((packed));
static_assert(sizeof(RecvPacket) <= 64);

void init();
RecvPacket *recv();
void send(const float *q,
          float yaw,
          float yaw_vel,
          float pitch,
          float pitch_vel,
          float bullet_speed,
          uint8_t is_start,
          uint8_t hp);
uint32_t last_update_time();
} // namespace vision
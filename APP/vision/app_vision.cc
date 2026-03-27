//
// Created by fish on 2024/12/18.
//

#include "app_vision.h"

#include <algorithm>
#include <cstring>

#include "app_ins.h"
#include "bsp_uart.h"
#include "alg_crc.h"
#include "bsp_def.h"
#include "bsp_time.h"

using namespace vision;

static RecvPacket rx_packet;

// 新增：最近一次收到视觉数据的本地时间戳（ms）
static uint32_t last_update_time_ms = 0;

void uart_rx_callback(bsp_uart_e e, uint8_t *s, uint16_t l) {
    if(l < sizeof rx_packet) return;
    if(CRC16::verify(s, sizeof rx_packet)) {
        std::copy_n(s, sizeof rx_packet, reinterpret_cast <uint8_t *> (&rx_packet));
        last_update_time_ms = bsp_time_get_ms();
    }
}

void vision::init() {
    bsp_uart_init(E_UART_VISION, &huart1);
    bsp_uart_set_callback(E_UART_VISION, uart_rx_callback);
}

RecvPacket *vision::recv() {
    return &rx_packet;
}

uint32_t vision::last_update_time() {
    return last_update_time_ms;
}

static auto ins = app_ins_data();

void vision::send(const float *q, float yaw, float yaw_vel, float pitch, float pitch_vel, float bullet_speed, uint8_t is_start, uint16_t hp) {
    SendPacket pkg = {};

    pkg.mode = 1;
    for (size_t i = 0; i < 4; i++)
        pkg.q[i] = q[i];
    pkg.yaw = yaw;
    pkg.yaw_vel = yaw_vel;
    pkg.pitch = pitch;
    pkg.pitch_vel = pitch_vel;
    pkg.bullet_speed = bullet_speed;
    pkg.is_start = is_start;
    pkg.hp = hp;
    // pkg.bullet_count = bullet_count;
    CRC16::append(pkg);
    bsp_uart_send(E_UART_VISION, reinterpret_cast <uint8_t *> (&pkg), sizeof pkg);
}
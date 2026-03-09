//
// Created by fish on 2026/2/3.
//

#include "app_referee.h"
#include "app_referee_def.h"
#include <cstring>

#include "bsp_time.h"
#include "alg_crc.h"


using namespace robomaster;

namespace robomaster::basic {
    data_t data_;
    frame_header_t header;
    void callback(bsp_uart_e device, uint8_t *data, uint16_t size);
}

const basic::data_t* basic::data() {
    return &data_;
}

void basic::init(bsp_uart_e uart) {
    //常规链路 波特率115200,
    bsp_uart_init(E_UART_REFEREE, &huart7);
    bsp_uart_set_callback(E_UART_REFEREE, callback);
}

void basic::callback(bsp_uart_e device, uint8_t* data, uint16_t size) {
    if (size < sizeof(frame_header_t)) return;

    size_t p = 0;
    while (p + sizeof(frame_header_t) < size) {
        memcpy(&header, data + p, sizeof(frame_header_t));
        if (header.sof != 0xa5 or !CRC8::verify(header)) { p ++; continue; }
        if (CRC16::calc(data + p, sizeof(frame_header_t) + 2 + header.data_length) !=
            *(uint16_t *)(data + p + sizeof(frame_header_t) + 2 + header.data_length))
        {
            p ++;
            continue;
        }

        uint16_t cmd_id = *(uint16_t *)(data + p + sizeof(frame_header_t));
        p += sizeof(frame_header_t) + 2;

#define upd(x) if(header.data_length == sizeof(data_.x)) memcpy(&data_.x, data + p, sizeof(data_.x)), data_.timestamps.x = bsp_time_get_ms()
        switch(cmd_id) {
        case 0x0001: upd(game_status); break;
        case 0x0002: upd(game_result); break;
        case 0x0003: upd(game_robot_hp); break;
        case 0x0101: upd(event_data); break;
        case 0x0104: upd(referee_warning); break;
        case 0x0105: upd(dart_info); break;
        case 0x0201: upd(robot_status); break;
        case 0x0202: upd(power_heat_data); break;
        case 0x0203: upd(robot_pos); break;
        case 0x0204: upd(buff); break;
        case 0x0206: upd(hurt_data); break;
        case 0x0207: upd(shoot_data); break;
        case 0x0208: upd(projectile_allowance); break;
        case 0x0209: upd(rfid_status); break;
        case 0x020a: upd(dart_client_cmd); break;
        case 0x020b: upd(ground_robot_position); break;
        case 0x020c: upd(radar_mark_data); break;
        case 0x020d: upd(sentry_info); break;
        case 0x020e: upd(radar_info); break;
        default:
            break;
        }

#undef upd

        p += header.data_length + 2; // data + crc16
    }
}

namespace robomaster::image {
    namespace rc {
        raw_frame_t raw;
        data_t rc_data_;
    }
    void callback(bsp_uart_e device, uint8_t *data, uint16_t size);
}

const image::rc::data_t *image::rc::data() {
    return &rc_data_;
}

void image::init(bsp_uart_e uart) {
    static_assert(sizeof(rc::raw_frame_t) == 21);
    //图传链路 波特率921600(但目前是115200，需单独调用函数更改）
    bsp_uart_init(E_UART_REFEREE, &huart7);
    bsp_uart_set_callback(E_UART_REFEREE, callback);
}

void image::callback(bsp_uart_e device, uint8_t* data, uint16_t size) {
    if (size == sizeof(rc::raw_frame_t) and data[0] == 0xa9 and data[1] == 0x53) {
        memcpy(&rc::raw, data, sizeof(rc::raw_frame_t));
        if (!CRC16::verify(rc::raw)) return;

        rc::rc_data_.l[0] = static_cast<int16_t>(static_cast<int16_t>(rc::raw.ch3) - 1024);
        rc::rc_data_.l[1] = static_cast<int16_t>(static_cast<int16_t>(rc::raw.ch2) - 1024);
        rc::rc_data_.r[0] = static_cast<int16_t>(static_cast<int16_t>(rc::raw.ch0) - 1024);
        rc::rc_data_.r[1] = static_cast<int16_t>(static_cast<int16_t>(rc::raw.ch1) - 1024);
        rc::rc_data_.dial = static_cast<int16_t>(static_cast<int16_t>(rc::raw.dial) - 1024);
        rc::rc_data_.sw = static_cast<int8_t>(rc::raw.sw - 1);

        rc::rc_data_.key_suspend = rc::raw.key_suspend;
        rc::rc_data_.key_l = rc::raw.key_l;
        rc::rc_data_.key_r = rc::raw.key_r;
        rc::rc_data_.key_shoot = rc::raw.key_shoot;

        rc::rc_data_.mouse_x = rc::raw.mouse_x;
        rc::rc_data_.mouse_y = rc::raw.mouse_y;
        rc::rc_data_.mouse_z = rc::raw.mouse_z;
        rc::rc_data_.mouse_l = rc::raw.mouse_l;
        rc::rc_data_.mouse_r = rc::raw.mouse_r;
        rc::rc_data_.mouse_m = rc::raw.mouse_m;
        rc::rc_data_.keyboard = rc::raw.keyboard;

        rc::rc_data_.timestamp = bsp_time_get_ms();

        return;
    }

}
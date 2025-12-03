//
// Created by fish on 2024/12/18.
//

#include "app_chassis.h"

#include <cstring>
#include "app_gimbal.h"
#include "app_ins.h"

#include "app_motor.h"
#include "app_msg.h"
#include "app_sys.h"
#include "bsp_rc.h"
#include "bsp_time.h"
#include "ctrl_motor_base_pid.h"
#include "ctrl_low_pass_filter.h"
#include "dev_cap.h"
#include "dev_motor_dji.h"
#include "sys_task.h"
#include "bsp_rc.h"
#include "app_msg_def.h"

#ifdef COMPILE_CHASSIS

/////////////////////////////////////////////////////////////////////////////
///舵轮底盘示意图
//                  减震铁杆                减震铁杆
//                   ---                   ---
//    id:2,角度:4400   o--------电池----------o  id:3,角度:5119
//     (s2)           |                     |  (s3)
//                    |                     |
//                    |                     |
//                  分电板       中心        喵板
//                    |                     |
//                    |                     |
//                    |                     |
//    id:1,角度:340    o--------达妙----------o  id:4,角度:5765
//     (s1)          ---                   ---  (s4)
//                  减震铁杆               减震铁杆
////////////////////////////////////////////////////////////////////////////

using namespace Motor;
using namespace Controller;

MotorController w_1(std::make_unique <DJIMotor> (
    "w_1",
    DJIMotor::M3508,
    (DJIMotor::Param) { 0x01, E_CAN1, DJIMotor::CURRENT }
));
MotorController w_2(std::make_unique <DJIMotor> (
    "w_2",
    DJIMotor::M3508,
    (DJIMotor::Param) { 0x02, E_CAN1, DJIMotor::CURRENT }
));
MotorController w_3(std::make_unique <DJIMotor> (
    "w_3",
    DJIMotor::M3508,
    (DJIMotor::Param) { 0x03, E_CAN1, DJIMotor::CURRENT }
));
MotorController w_4(std::make_unique <DJIMotor> (
    "w_4",
    DJIMotor::M3508,
    (DJIMotor::Param) { 0x04, E_CAN1, DJIMotor::CURRENT }
));
// 舵
MotorController s_1(std::make_unique <DJIMotor> (
    "s_1",
    DJIMotor::GM6020,
    (DJIMotor::Param) { 0x01, E_CAN2, DJIMotor::CURRENT }
));
MotorController s_2(std::make_unique <DJIMotor> (
    "s_2",
    DJIMotor::GM6020,
    (DJIMotor::Param) { 0x02, E_CAN2, DJIMotor::CURRENT }
));
MotorController s_3(std::make_unique <DJIMotor> (
    "s_3",
    DJIMotor::GM6020,
    (DJIMotor::Param) { 0x03, E_CAN2, DJIMotor::CURRENT }
));
MotorController s_4(std::make_unique <DJIMotor> (
    "s_4",
    DJIMotor::GM6020,
    (DJIMotor::Param) { 0x04, E_CAN2, DJIMotor::CURRENT }
));

struct SW {
    MotorController *servo = nullptr, *wheel = nullptr;
    bool reversed = false;
    double vx_ = 0, vy_ = 0, angle_ = 0;
    void update(double vx, double vy) {
        vx *= -1;
        auto angle = std::atan2(vy, vx);
        if(std::min(std::abs(angle - angle_), 2 * M_PI - std::abs(angle - angle_)) > M_PI / 2) {
            reversed ^= 1;
        }
        auto cur_angle = static_cast <float> (angle < 0 ? 2 * M_PI + angle : angle), cur_speed = static_cast <float> (std::sqrt(vx * vx + vy * vy));
        cur_angle = static_cast <float> (cur_angle / M_PI * 180 + 270);

        if(reversed) {
            cur_angle += 180;
            cur_speed *= -1;
        }

        while(cur_angle >= 360) cur_angle -= 360;
        while(cur_angle < 0) cur_angle += 360;

        servo->update(cur_angle), wheel->update(cur_speed);
        vx_ = vx, vy_ = vy, angle_ = angle;
    }
};

SW sw_1 = { &s_1, &w_1 }, sw_2 = { &s_2, &w_2 }, sw_3 = { &s_3, &w_3 }, sw_4 = { &s_4, &w_4 };

float target = 0;
bool dir = false;
const auto rc = bsp_rc_data();
const auto ins = app_ins_data();

void set_target(bsp_uart_e e, uint8_t *s, uint16_t l) {
    sscanf((char *) s, "%f", &target);
}

void motor_update(double vx, double vy, double rotate) {
    // Servo 1, rotate v = ( 1,  1)
    auto x1 =  rotate / M_SQRT2 + vx, y1 =  rotate / M_SQRT2 + vy;
    // Servo 2, rotate v = ( 1, -1)
    auto x2 =  rotate / M_SQRT2 + vx, y2 = -rotate / M_SQRT2 + vy;
    // Servo 3, rotate v = (-1, -1)
    auto x3 = -rotate / M_SQRT2 + vx, y3 = -rotate / M_SQRT2 + vy;
    // Servo 4, rotate v = (-1,  1)
    auto x4 = -rotate / M_SQRT2 + vx, y4 =  rotate / M_SQRT2 + vy;

    sw_1.update(x1, y1); sw_2.update(x2, y2); sw_3.update(x3, y3); sw_4.update(x4, y4);
}

int8_t send_count = 0;

float encoder_to_angle_360(int32_t enc, int32_t zero_offset)
{
    // 1. 规整到 0~65535
    enc %= 65536;
    if (enc < 0) enc += 65536;

    zero_offset %= 65536;
    if (zero_offset < 0) zero_offset += 65536;

    // 2. 计算相对零点的偏移（允许跨圈），结果先在 -65535~65535
    int32_t diff = enc - zero_offset;

    // 3. 折叠到一圈内：用 16384（一圈脉冲数）取模，得到 0~16383 的相对位置
    // 先移动到正数再取模，保证 C 下取模为正
    int32_t pos_in_turn = diff % 16384;
    if (pos_in_turn < 0) pos_in_turn += 16384;

    // 现在约定：零点位置对应 pos_in_turn = 0
    // 我们希望：
    //  pos_in_turn = 0       -> -180°
    //  pos_in_turn = 8192    ->   0°
    //  pos_in_turn = 16384   -> +180° ≡ -180°
    //
    // 线性映射公式：angle = (pos_in_turn / 16384.0) * 360.0 - 180.0
    float angle = (float)pos_in_turn * 360.0f / 16384.0f - 180.0f;

    // 为避免在 16384 精度误差导致得到 +180.0f，可做个小处理：
    if (angle >= 180.0f) {
        angle -= 360.0f; // 保证范围为 [-180, 180)
    }

    return angle;
}


//双板通信
//收
app_msg_can_receiver <app_msg_gimbal_to_chassis> gimbal(E_CAN3, 0x036);
//发
void send_msg_to_gimbal() {
    app_msg_chassis_to_gimbal pkg = {
        .robot_id = 1,
        .robot_level = 110
    };
    app_msg_can_send(E_CAN3, 0x044, pkg);
}


// 静态任务，在 CubeMX 中配置
void app_chassis_task(void *args) {
	// Wait for system init.
	while(!app_sys_ready()) OS::Task::SleepMilliseconds(10);

    int zero_count = 0;
    bsp_uart_set_callback(E_UART_DEBUG, set_target);
    gimbal.init();

    double vx = 0, vy = 0, rotate = 0;
    const double eps = 1e-7;

	while(true) {


        //双板通信
	    if(++ send_count == 10) {
	        send_count = 0;
	        send_msg_to_gimbal();
	    }


	    if (bsp_time_get_ms() - gimbal.timestamp<100)
	    {
	        vx = -gimbal()->vx * 1.0;
	        vy = -gimbal()->vy * 1.0;
	        rotate = gimbal()->rotate * 1.0;
	    }else {
	        vx=vy=rotate = 0;
	    }



	    auto theta = std::atan2(vy, vx), r = std::sqrt((vx * vx) + (vy * vy));
	    // theta -= M_PI / 4096;//无解算git
	    theta -= encoder_to_angle_360(gimbal()->b_yaw_cnt,3600) * M_PI / 180.0;//大yaw正方向解算
	    vx = r * std::cos(theta), vy = r * std::sin(theta);

	    //防滑控制
	    // if(rotate == 0) rotate = eps;
	    // if(vx == 0 and vy == 0) {
	    //     if(rotate > eps or zero_count == 500) {
	    //         sw_1.update( rotate / M_SQRT2,  rotate / M_SQRT2);
	    //         sw_2.update( rotate / M_SQRT2, -rotate / M_SQRT2);
	    //         sw_3.update(-rotate / M_SQRT2, -rotate / M_SQRT2);
	    //         sw_4.update(-rotate / M_SQRT2,  rotate / M_SQRT2);
	    //     } else {
	    //         zero_count ++;
	    //         motor_update(vx, vy, rotate);
	    //     }
	    // } else {
	    //     zero_count = 0;
	    //     motor_update(vx, vy, rotate);
	    // }

	    //正常控制
	    if(vx == 0 and vy == 0 and rotate ==0) {
            //舵不更新，轮速为0
	        s_1.relax(), s_2.relax(), s_3.relax(), s_4.relax();
	        w_1.update(0);
	        w_2.update(0);
	        w_3.update(0);
	        w_4.update(0);
	    }else {
	        s_1.activate(), s_2.activate(), s_3.activate(), s_4.activate();
	        motor_update(vx, vy, rotate);
	    }





	    app_msg_vofa_send(E_UART_DEBUG,
	                                    // s_1.device()->angle,  //340
	                                    // s_2.device()->angle,  //4400
	                                    // s_3.device()->angle,  //5119
	                                    // s_4.device()->angle,  //5765
	                                    gimbal()->ins_yaw,
	                                    gimbal()->b_yaw_cnt,
	                                    encoder_to_angle_360(gimbal()->b_yaw_cnt,4096),
	                                    bsp_time_get_ms(),
	                                    gimbal.timestamp,
	                                    encoder_to_angle_360(gimbal()->b_yaw_cnt,3600) * M_PI / 180.0
	                                    // encoder_to_angle_360(gimbal()->b_yaw_cnt,5041)
	                                    // gimbal()->vx,
	                                    // gimbal()->vy,
	                                    // gimbal()->rotate,
	                                    // w_1.current,
	                                    // s_1.current,
	                                    // w_2.current,
	                                    // s_2.current
	                                    // send_count





	                                    // w_2.device()->speed,
	                                    // w_2.device()->current,
	                                    // w_2.output,
	                                    // w_2.target()
                                        // rc->rc_l[0],
                                        // rc->rc_l[1]
	                                    );

		OS::Task::SleepMilliseconds(1);
	}
}

void app_chassis_init() {
    w_1.init(); w_2.init(); w_3.init(); w_4.init();
	w_1.add_controller(std::make_unique <MotorBasePID> (
		MotorBasePID::PID_SPEED,
		std::make_unique <PID> (15, 0.5, 0.0, 16384, 1000),
		nullptr
	));
	w_2.add_controller(std::make_unique <MotorBasePID> (
		MotorBasePID::PID_SPEED,
		std::make_unique <PID> (15, 0.5, 0.0, 16384, 1000),
		nullptr
	));
	w_3.add_controller(std::make_unique <MotorBasePID> (
		MotorBasePID::PID_SPEED,
		std::make_unique <PID> (15, 0.5, 0.0, 16384, 1000),
		nullptr
	));
	w_4.add_controller(std::make_unique <MotorBasePID> (
		MotorBasePID::PID_SPEED,
		std::make_unique <PID> (15, 0.5, 0.0, 16384, 1000),
		nullptr
	));
	s_1.init();s_2.init(); s_3.init(); s_4.init();
	s_1.add_controller(std::make_unique <MotorBasePID> (
		MotorBasePID::PID_SPEED | MotorBasePID::PID_ANGLE,
		std::make_unique <PID> (35, 0.2, 0.0, 16384, 10000),
		std::make_unique <PID> (5, 0, 0, 180, 0),
		true
	));
	s_2.add_controller(std::make_unique <MotorBasePID> (
		MotorBasePID::PID_SPEED | MotorBasePID::PID_ANGLE,
		std::make_unique <PID> (35, 0.2, 0.0, 16384, 10000),
		std::make_unique <PID> (5, 0, 0, 180, 0),
		true
	));
	s_3.add_controller(std::make_unique <MotorBasePID> (
		MotorBasePID::PID_SPEED | MotorBasePID::PID_ANGLE,
		std::make_unique <PID> (35, 0.2, 0.0, 16384, 10000),
		std::make_unique <PID> (5, 0, 0, 180, 0),
		true
	));
    s_4.add_controller(std::make_unique <MotorBasePID> (
        MotorBasePID::PID_SPEED | MotorBasePID::PID_ANGLE,
        std::make_unique <PID> (35, 0.2, 0.0, 16384, 10000),
        std::make_unique <PID> (5, 0, 0, 180, 0),
        true
    ));
	s_1.use_degree_angle = true; s_1.encoder_zero = 2388;
	s_2.use_degree_angle = true; s_2.encoder_zero = 6448;
	s_3.use_degree_angle = true; s_3.encoder_zero = 7167;
	s_4.use_degree_angle = true; s_4.encoder_zero = 7813;

    // w_1.relax(); w_2.relax(); w_3.relax(); w_4.relax();
    // s_1.relax(); s_2.relax(); s_3.relax(); s_4.relax();


}

#endif
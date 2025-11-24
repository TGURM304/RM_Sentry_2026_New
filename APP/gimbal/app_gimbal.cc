//
// Created by fish on 2024/11/17.
//

#include "app_gimbal.h"

#include "app_sys.h"
#include "sys_task.h"
#include "alg_filter.h"
#include "app_sys.h"
#include "sys_task.h"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include "app_ins.h"
#include "app_motor.h"
#include "app_msg.h"
#include "app_referee.h"
#include "app_sys.h"
#include "bsp_can.h"
#include "bsp_rc.h"
#include "bsp_time.h"
#include "ctrl_pid.h"
#include "dev_motor_dji.h"
#include "dev_motor_dm.h"
#include "sys_task.h"

#ifdef COMPILE_GIMBAL

#define B_YAW_ZERO_SPEED 2047

using namespace Motor;
using namespace Controller;

DJIMotor trigger("trigger", DJIMotor::M2006, { 0x01, E_CAN3, DJIMotor::CURRENT });

DJIMotor s_yaw("gimbal_yaw_small", DJIMotor::GM6020, { 0x04, E_CAN2, DJIMotor::CURRENT });
PID s_yaw_speed(60, 0.8, 0, 16384, 10000);
PID s_yaw_angle(15, 0, 0, 360, 0);

DJIMotor pit("gimbal_pit", DJIMotor::GM6020, { 0x03, E_CAN2, DJIMotor::CURRENT });
PID pit_speed(80, 0.5, 0, 16384, 10000);
PID pit_angle(16, 0, 0, 210, 0);

DMMotor b_yaw("gimbal_yaw_big", DMMotor::J4310, (DMMotor::Param) {
    .slave_id = 0x06, .master_id = 0x05, .port = E_CAN3, .mode = DMMotor::MIT,
    .p_max = 12.5, .v_max = 30, .t_max = 10, .kp_max = 500, .kd_max = 5
});
PID b_yaw_speed(0.018, 0.0002, 0, 10, 6);
PID b_yaw_angle(10, 0, 0, 720, 540);

// 将编码器值(0~8191, 顺时针减小)转换为 [0,360) 角度，逆时针为正
static inline double encoder_to_deg_ccw(int16_t enc, int16_t counts_per_rev)
{
    // 保护
    if (counts_per_rev <= 0) return 0.0;

    // 由于顺时针减少，这里先按绝对值映射到 [0,counts_per_rev)
    int32_t e = enc % counts_per_rev;
    if (e < 0) e += counts_per_rev;

    // 以编码器数值递增为参考是顺时针，因此角度要取负号：
    //   enc 增加 -> 实际顺时针 -> 角度减小
    double deg_cw = (double)e * 360.0 / (double)counts_per_rev;
    double deg_ccw = -deg_cw;  // 逆时针为正

    // 归一化到 [0,360)
    while (deg_ccw < 0.0)   deg_ccw += 360.0;
    while (deg_ccw >= 360.) deg_ccw -= 360.0;

    return deg_ccw;
}

// 把当前编码器值, 按给定 "中点编码器值" 作为 0°，输出 [-180,180) 角度
// enc_now, enc_mid: int16_t 编码器读数(0~8191 或带符号溢出的原始值)
// counts_per_rev: 8192
double encoder_to_deg_mid_zero(int16_t enc_now,
                               int16_t enc_mid,
                               int16_t counts_per_rev)
{
    // 1. 先把当前值和中点值都转成物理角度(逆时针为正, [0,360))
    double deg_now = encoder_to_deg_ccw(enc_now, counts_per_rev);
    double deg_mid = encoder_to_deg_ccw(enc_mid, counts_per_rev);

    // 2. 以中点为零：当前角度 - 中点角度
    double deg = deg_now - deg_mid;

    // 3. 归一化到 [-180,180)
    while (deg < -180.0) deg += 360.0;
    while (deg >= 180.0) deg -= 360.0;

    return deg;   // 左转(逆时针)为正，右转(顺时针)为负
}

// 连续角度解包：输入当前 yaw(-180~+180)，输出连续角度
double unwrap_yaw_deg(double yaw_now_deg)
{
    // 静态变量保存上一次的 yaw 和累计角度
    static bool   initialized = false;
    static double last_yaw_deg = 0.0;   // 上一次原始 yaw(-180~180)
    static double total_angle_deg = 0.0; // 累计角度

    if (!initialized) {
        // 第一次调用时初始化
        last_yaw_deg = yaw_now_deg;
        total_angle_deg = yaw_now_deg;  // 也可以初始化为 0，看你需要
        initialized = true;
        return total_angle_deg;
    }

    double delta = yaw_now_deg - last_yaw_deg;

    // 处理跨越 -180/180 边界的情况
    if (delta > 180.0) {
        // 例：last = +179, now = -179 -> delta = -358 (实际 -2)
        delta -= 360.0;
    } else if (delta < -180.0) {
        // 例：last = -179, now = +179 -> delta = +358 (实际 +2)
        delta += 360.0;
    }

    total_angle_deg += delta;
    last_yaw_deg = yaw_now_deg;

    return total_angle_deg;
}
double yaw_unwrapped = 0.0;

//串口发变量
float target = 0;
void set_target(bsp_uart_e e, uint8_t *s, uint16_t l) {
    sscanf((char *) s, "%f", &target);
}

double s_yaw_target;
double s_yaw_current;
double s_yaw_output;


double pit_target;
double pit_current;
double pit_output;


double s_yaw_enc_deg;
double b_yaw_real_speed;
double b_yaw_target;
double b_yaw_current;
double b_yaw_output;

const auto ins = app_ins_data();
const auto rc = bsp_rc_data();

// 静态任务，在 CubeMX 中配置
void app_gimbal_task(void *args) {
    // Wait for system init.
    while(!app_sys_ready())
        OS::Task::SleepMilliseconds(10);

    bsp_uart_set_callback(E_UART_DEBUG, set_target);

    b_yaw.reset();
    OS::Task::SleepMilliseconds(100);
    b_yaw.enable();


    while(true) {



        //pit轴控制量设置
        // pit_target = static_cast <float> (rc->rc_r[1]) / 25.0f;//测试用
        pit_target -= static_cast <float> (rc->rc_r[1]) * 0.0002f;
        if (pit_target < -3) pit_target = -3;//pitch限位
        if (pit_target > 25) pit_target = 25;
        pit_current = ins->roll;
        //pit轴控制
        pit_output = pit_target;
        pit_output = pit_angle.update(static_cast<float>(pit_current), static_cast<float>(pit_output));
        pit_output = pit_speed.update(static_cast <float> (ins->raw.gyro[0] * 180.0 / M_PI), static_cast<float>(pit_output));
        pit.update(static_cast<float>(pit_output));


        //小yaw轴状态量设置
        yaw_unwrapped = unwrap_yaw_deg(ins->yaw);
        //小yaw轴控制量设置
        // s_yaw_target = static_cast <float> (rc->rc_r[0]) / 25.0f;//测试用
        s_yaw_target -= static_cast <float> (rc->rc_r[0]) * 0.0006f;
        s_yaw_current = yaw_unwrapped;
        //小yaw轴控制
        s_yaw_output = s_yaw_target;
        s_yaw_output = s_yaw_angle.update(static_cast<float>(s_yaw_current), static_cast<float>(s_yaw_output));
        s_yaw_output = s_yaw_speed.update(-static_cast <float> (ins->raw.gyro[2] * 180.0 / M_PI), static_cast<float>(s_yaw_output));
        s_yaw.update(static_cast<float>(s_yaw_output));



        //重新使能达妙
        if(b_yaw.status.err == 0 || b_yaw.status.err == 0xD) {
            b_yaw.reset();
            b_yaw.enable();
        }
        //大yaw状态量设置
        s_yaw_enc_deg = encoder_to_deg_mid_zero(s_yaw.feedback_.angle,700,8192);//小yaw编码器范围:中心点700，从左限位到到右限位1816,1815.....2,1,0,8192,8191,...,7951,7950
        b_yaw_real_speed = b_yaw.feedback_.vel - B_YAW_ZERO_SPEED;//正对应往右转（顺时针
        //大yaw控制量设置
        // b_yaw_target = static_cast <float> (rc->rc_r[0]) ;//速度环测试用
        b_yaw_target = 0;
        b_yaw_current = s_yaw_enc_deg;
        //大yaw轴控制
        b_yaw_output = b_yaw_target;
        b_yaw_output = b_yaw_angle.update(static_cast<float>(-b_yaw_current), static_cast<float>(b_yaw_target));
        b_yaw_output = b_yaw_speed.update(static_cast<float>(b_yaw_real_speed), static_cast<float>(b_yaw_output));
        //大yaw控制
        b_yaw.control(0,0,0,0,static_cast<float>(b_yaw_output));//正对应往右转（顺时针


        app_msg_vofa_send(E_UART_DEBUG,
            // pit.status.angle,
            // pit.status.speed,
            // ins->roll,
            // ins->raw.gyro[0],
            // rc->rc_r[1],
            // pit_target,
            // pit_current

            // s_yaw.status.speed,
            // ins->yaw,
            // ins->raw.gyro[2],
            // rc->rc_r[0],
            // s_yaw_target,
            // s_yaw_current,
            // s_yaw_output

            s_yaw_enc_deg,
            b_yaw_real_speed,
            b_yaw_output,
            static_cast <float> (rc->rc_r[0]),
            ins->yaw,
            s_yaw.feedback_.angle,
            yaw_unwrapped


            // trigger.status.angle,
            // trigger.status.speed,



        );

        OS::Task::SleepMilliseconds(1);
    }
}

void app_gimbal_init() {
    s_yaw.init();
    b_yaw.init();
    pit.init();
    trigger.init();

}

#endif

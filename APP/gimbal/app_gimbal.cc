//
// Created by fish on 2024/11/17.
//

#include "app_gimbal.h"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <algorithm>
#include "alg_filter.h"
#include "sys_task.h"
#include "bsp_can.h"
#include "bsp_rc.h"
#include "bsp_time.h"
#include "app_sys.h"
#include "app_ins.h"
#include "app_motor.h"
#include "app_msg.h"
#include "app_referee.h"

#include "ctrl_pid.h"
#include "dev_motor_dji.h"
#include "dev_motor_dm.h"
#include "app_vision.h"
#include "ctrl_motor_base_pid.h"
#include "app_msg_def.h"
#include "bsp_buzzer.h"
#include "app_music.h"

#ifdef COMPILE_GIMBAL

#define B_YAW_ZERO_SPEED 2047

using namespace Motor;
using namespace Controller;

DJIMotor s_yaw("gimbal_yaw_small", DJIMotor::GM6020, { 0x04, E_CAN2, DJIMotor::CURRENT });
PID s_yaw_speed(60, 0.9, 0, 16384, 10000);
PID s_yaw_angle(15, 0, 0, 720, 0);

DJIMotor pit("gimbal_pit", DJIMotor::GM6020, { 0x03, E_CAN2, DJIMotor::CURRENT });
PID pit_speed(80, 0.5, 0, 16384, 10000);
PID pit_angle(16, 0, 0, 210, 0);

DMMotor b_yaw("gimbal_yaw_big", DMMotor::J4310, (DMMotor::Param) {
    .slave_id = 0x06, .master_id = 0x05, .port = E_CAN3, .mode = DMMotor::MIT,
    .p_max = 12.5, .v_max = 30, .t_max = 10, .kp_max = 500, .kd_max = 5
});
PID b_yaw_speed(0.015, 0.0002, 0, 10, 6);
PID b_yaw_angle(15, 0.008, 0, 720, 540);

MotorController m_trigger(std::make_unique <DJIMotor>(
    "trigger",
    DJIMotor::M2006,
    (DJIMotor::Param) {.id = 0x01,.port = E_CAN3,.mode = DJIMotor::CURRENT}
));

MotorController m_left_shoot(std::make_unique <DJIMotor>(
    "left_shoot",
    DJIMotor::M3508,
    (DJIMotor::Param) {.id = 0x01,.port = E_CAN1,.mode = DJIMotor::CURRENT}
));

MotorController m_right_shoot(std::make_unique <DJIMotor>(
    "right_shoot",
    DJIMotor::M3508,
    (DJIMotor::Param) {.id = 0x02,.port = E_CAN1,.mode = DJIMotor::CURRENT}
));

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

float s_yaw_target;
float s_yaw_current;
float s_yaw_output;

float pit_target;
float pit_current;
float pit_output;

float s_yaw_enc_deg;
float b_yaw_real_speed;
float b_yaw_target;
float b_yaw_current;
float b_yaw_output;

double trigger_speed;
double left_shoot_speed;
double right_shoot_speed;

float chassis_vx;
float chassis_vy;
float chassis_rotate;

const auto ins = app_ins_data();
const auto rc = bsp_rc_data();

vision::RecvPacket* vd = vision::recv();

//双板通信
//收
app_msg_can_receiver <app_msg_chassis_to_gimbal> chassis(E_CAN3, 0x044);
//发
void send_msg_to_chassis() {
    app_msg_gimbal_to_chassis pkg = {
        .ins_yaw = ins->yaw,
        .vx = chassis_vx,
        .vy = chassis_vy,
        .rotate = chassis_rotate,
        .b_yaw_cnt = b_yaw.feedback_.pos,

    };
    app_msg_can_send(E_CAN3, 0x036, pkg);
}
int8_t send_count = 0;

// 静态任务，在 CubeMX 中配置
void app_gimbal_task(void *args) {
    // Wait for system init.
    while(!app_sys_ready())
        OS::Task::SleepMilliseconds(10);

    bsp_uart_set_callback(E_UART_DEBUG, set_target);
    chassis.init();

    b_yaw.reset();
    OS::Task::SleepMilliseconds(100);
    b_yaw.enable();

    int pc_send = 0;
    int8_t last_s_r = 0x7f;

    while(true) {
        // 上位机通信，10ms一次
        if(++pc_send == 10) {
            pc_send = 0;
            uint16_t bullet_count = 0;
            float bullet_speed = 20;
            float roll = ins->roll / 180 * M_PI;
            float pitch = ins->pitch / 180 * M_PI;
            float pitch_vel = ins->raw.gyro[0];
            float yaw = ins->yaw / 180 * M_PI;
            float yaw_vel = ins->raw.gyro[2];
            vision::send(roll, yaw, yaw_vel, pitch, pitch_vel, bullet_speed, bullet_count);
            // app_msg_vofa_send(E_UART_VISION, roll ,yaw, yaw_vel, pitch, pitch_vel, bullet_speed, bullet_count);
        }

        //双板通信
        if(++ send_count == 10) {
            send_count = 0;
            send_msg_to_chassis();
        }

        //控制逻辑区，遥控器作为安全控制器和控制源选择器
        if(bsp_time_get_ms() - rc->timestamp <= 50) {
            //切换模式提示音
            // 1\. 检测模式切换（只要 s_r 变化就触发一次）
            if (rc->s_r != last_s_r) {
                last_s_r = rc->s_r;

                // 2\. 根据新模式播放不同提示音
                switch (rc->s_r) {
                case -1:
                    // 失能/备用模式提示音
                        app_sys_music_play(E_MUSIC_MODE1);
                    break;
                case 0:
                    // 遥控器控制模式提示音
                        app_sys_music_play(E_MUSIC_MODE2);
                    break;
                case 1:
                    // 小电脑控制模式提示音
                        app_sys_music_play(E_MUSIC_MODE3);
                    break;
                default:
                    break;
                }
            }

            //选择控制源
            if(rc->s_r == -1) {
                //备用,目前当作失能模式,情况同控制器离线:Yyp位置不控制(维持原位)，xy速度和shooter速度置为0,
                trigger_speed = 0;
                left_shoot_speed = 0;
                right_shoot_speed = 0;
                chassis_vx = 0;
                chassis_vy = 0;
                chassis_rotate = 0;

            }else if(rc->s_r == 0) {
                //遥控器控制区
                //云台控制
                pit_target -= static_cast <float> (rc->rc_r[1]) * 0.0002f;
                s_yaw_target -= static_cast <float> (rc->rc_r[0]) * 0.0006f;
                //底盘控制
                chassis_vx = static_cast<float>(3.0*rc->rc_l[0]);
                chassis_vy = static_cast<float>(3.0*rc->rc_l[1]);
                chassis_rotate = static_cast<float>(3.0*rc->reserved);
                //射击控制
                if(rc->s_l == 0) {
                    //不射击
                    trigger_speed = 0;
                    left_shoot_speed = 0;
                    right_shoot_speed = 0;
                }else if(rc->s_l == 1) {
                    //射击
                    trigger_speed = -2250;
                    left_shoot_speed = 7500;
                    right_shoot_speed = -7500;
                }else if(rc->s_l == -1) {
                    //退弹(因为拨弹盘有机械限位，所以只有摩擦轮反转
                    trigger_speed = 0;
                    left_shoot_speed = -7500;
                    right_shoot_speed = 7500;
                }

            }else if(rc->s_r == 1) {
                //小电脑控制区
                bool control_by_pc = true;//占位符，此处填写哨兵控制代码
                //云台控制
                pit_target -= 0;
                s_yaw_target -= 0;
                //底盘
                chassis_vx = 0;
                chassis_vy = 0;
                chassis_rotate = 0;
                //拨弹盘控制
                trigger_speed = 0;
                left_shoot_speed = 0;
                right_shoot_speed = 0;
            }
        }else {
            //安全控制器离线:失能,Yyp位置不控制(维持原位)，xy速度和shooter速度置为0,
            trigger_speed = 0;
            left_shoot_speed = 0;
            right_shoot_speed = 0;
            chassis_vx = 0;
            chassis_vy = 0;
            chassis_rotate = 0;
        }


        //电机控制区，分为pit轴，小yaw轴，大yaw轴控制、VxVy控制发送、摩擦轮和拨弹盘控制
        //pit轴控制量设置
        if (pit_target < -5) pit_target = -5;//pitch限位
        if (pit_target > 25) pit_target = 25;
        pit_current = ins->roll;
        //pit轴控制
        pit_output = pit_target;
        pit_output = pit_angle.update((pit_current), (pit_output));
        pit_output = pit_speed.update(static_cast <float> (ins->raw.gyro[0] * 180.0 / M_PI), (pit_output));
        pit.update((pit_output));

        //小yaw轴状态量设置
        yaw_unwrapped = unwrap_yaw_deg(ins->yaw);
        //小yaw轴控制量设置
        s_yaw_current = static_cast<float>(yaw_unwrapped);
        //小yaw轴控制
        s_yaw_output = s_yaw_target;
        s_yaw_output = s_yaw_angle.update((s_yaw_current), (s_yaw_output));
        s_yaw_output = s_yaw_speed.update(-static_cast <float> (ins->raw.gyro[2] * 180.0 / M_PI), (s_yaw_output));
        s_yaw.update((s_yaw_output));

        //大yaw状态量设置
        s_yaw_enc_deg = static_cast<float>(encoder_to_deg_mid_zero(s_yaw.feedback_.angle, 3423, 8192));//小yaw编码器范围:中心点700，从左限位到到右限位1816,1815.....2,1,0,8192,8191,...,7951,7950
        b_yaw_real_speed = static_cast<float>(b_yaw.feedback_.vel) - B_YAW_ZERO_SPEED;//正对应往右转（顺时针
        //大yaw控制量设置
        b_yaw_target = 0;
        b_yaw_current = s_yaw_enc_deg;
        //大yaw轴控制
        b_yaw_output = b_yaw_target;
        b_yaw_output = b_yaw_angle.update((-b_yaw_current), (b_yaw_target));
        b_yaw_output = b_yaw_speed.update((b_yaw_real_speed), (b_yaw_output));
        //重新使能并控制
        if(b_yaw.status.err == 0 || b_yaw.status.err == 0xD) {
            b_yaw.reset();
            b_yaw.enable();
        }
        b_yaw.control(0,0,0,0,(b_yaw_output));//控制，正对应顺时针转
        // b_yaw.control(0,0,0,0,0);//测试，发空包,为了得到反馈数据


        //发射机构控制
        m_trigger.update(trigger_speed);
        m_left_shoot.update(left_shoot_speed);
        m_right_shoot.update(right_shoot_speed);



        //调试区
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

            // m_trigger.device()->angle,
            // m_left_shoot.device()->speed,
            // m_right_shoot.device()->speed

            // chassis()->robot_id,
            // chassis()->robot_level
            // static_cast<float>(rc->rc_l[0]),
            // static_cast<float>(rc->rc_l[1]),
            // static_cast<float>(rc->reserved),
            // chassis.timestamp,
            // b_yaw.feedback_.pos
        );

        OS::Task::SleepMilliseconds(1);
    }
}

void app_gimbal_init() {
    s_yaw.init();
    b_yaw.init();
    pit.init();
    vision::init();

    m_trigger.add_controller(std::make_unique <Controller::MotorBasePID> (
        Controller::MotorBasePID::PID_SPEED,
        std::make_unique <Controller::PID> (10.0, 1.0, 0.0, 16384, 10000),
        nullptr
    ));
    m_trigger.init();

    m_left_shoot.add_controller(std::make_unique <Controller::MotorBasePID> (
        Controller::MotorBasePID::PID_SPEED,
        std::make_unique <Controller::PID> (15.0, 1.0, 0.0, 16384, 10000),
        nullptr
    ));
    m_left_shoot.init();

    m_right_shoot.add_controller(std::make_unique <Controller::MotorBasePID> (
        Controller::MotorBasePID::PID_SPEED,
        std::make_unique <Controller::PID> (15.0, 1.0, 0.0, 16384, 10000),
        nullptr
    ));
    m_right_shoot.init();

}

#endif

/**
 *******************************************************************************
 * @file      :chassis.cpp
 * @brief     :
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      yyyy-mm-dd      <author>        1. <note>
 *******************************************************************************
 * @attention :
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */

/* ============================================================================
 * 【新人任务】实现 chassis.hpp 中声明的底盘运动学。
 *
 * 你要做：
 *   1. 逆解：输入 (vx, vy, w)，输出每个舵的目标转角与每个轮的轮速 / 电流指令。
 *   2. 舵向轮需处理舵角连续性：当目标舵角与当前舵角相差超过 π/2 时，选择等效的
 *      另一组解（舵角反向、轮速取反），避免舵机大角度回摆。
 *   3. 对解算结果做限幅，避免超速。
 *
 * 约束 / 提示：
 *   - 解算结果会经电机组件的 encode() 下发，注意输入量的类型（电流 / 转矩）。
 * ==========================================================================*/

#include "chassis.hpp"
#include <cmath>
#include "assert.hpp"
#include "base.hpp"
#include "motor_base.hpp"

namespace {

// 轮组安装位置（车体系，x 前 / y 左）
// 0 左前 / 1 左后 / 2 右后 / 3 右前
constexpr float kWheelX[4] = {0.167f, -0.167f, -0.167f, 0.167f};
constexpr float kWheelY[4] = {0.167f, 0.167f, -0.167f, -0.167f};

// 单个轮组的线速度上限 (m/s) 超出后整体等比缩放
constexpr float kWheelSpeedMax = 3.0f;

// 逆解死区：轮心目标线速度低于该值时不转舵 (m/s)
constexpr float kSolveSpeedDeadband = 0.02f;

// 舵角回中：中位容差（rad）与输出限幅（raw）
constexpr float kSteerCenterTol = 0.03f;
constexpr float kSteerCenterOutMax = 3000.0f;
// 舵电机为电流控制模式：raw 指令换算成安培（PID 满量程 9000 raw 对应 3 A）
constexpr float kSteerCurrPerRaw = 3.0f / 9000.0f;

// 跟随模式 δ 角度环参数
constexpr float kFollowKp = 4.0f;
constexpr float kFollowKi = 0.5f;
constexpr float kFollowKd = 0.4f;
constexpr float kFollowIntegMax = 2.0f;

}  // namespace

robot::Chassis::Chassis(): pid_follow_(kFollowKp, kFollowKi, 
                   kFollowKd, kMaxFollowYawRate, kFollowIntegMax) {}

// motor/pid注册
void robot::Chassis::registerMotorWheel(int idx, hello_world::motor::Motor* motor){
    HW_ASSERT(0 <= idx && idx < 4, "Error wheel idx: %d", idx);
    motor_wheel_[idx] = motor;
}
void robot::Chassis::registerMotorSteer(int idx, hello_world::motor::Motor* motor){
    HW_ASSERT(0 <= idx && idx < 4, "Error steer idx: %d", idx);
    motor_steer_[idx] = motor;
}
void robot::Chassis::registerPidWheel(int idx, hello_world::pid::Pid* pid){
    HW_ASSERT(0 <= idx && idx < 4, "Error wheel pid idx: %d", idx);
    pid_wheel_[idx] = pid;
}
void robot::Chassis::registerPidSteer(int idx, hello_world::pid::Pid* pid){
    HW_ASSERT(0 <= idx && idx < 4, "Error steer pid idx: %d", idx);
    pid_steer_[idx] = pid;
}

// 读取模式，输入Motion，完成旋转，完成逆解，pid calc，电机setinput
void robot::Chassis::update(const ChassisMotion& motion, float w_manual, float dt){
    // 根据工作模式，得到底盘 w 
    // 底盘的 w 由工作模式决定：
    // 分离->0；跟随->由 δ 角环给出；小陀螺->w_manual
    yaw_rate_ = calcYawRate(w_manual, dt);

    // 旋转：云台系 -> 车系
    const float cos_delta = cosf(delta_);
    const float sin_delta = sinf(delta_);
    const float vx = motion.vx * cos_delta - motion.vy * sin_delta;
    const float vy = motion.vx * sin_delta + motion.vy * cos_delta;

    // 四轮逆解 (初步分解)
    float vix[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    float viy[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    float speed_max = 0.0f;
    for (int i = 0; i < 4u; i++) {
        vix[i] = vx - yaw_rate_ * kWheelY[i];
        viy[i] = vy + yaw_rate_ * kWheelX[i];
        const float speed = sqrtf(vix[i] * vix[i] + viy[i] * viy[i]);
        if (speed > speed_max) {  speed_max = speed;  }
    }

    // 四轮限速（等比缩放）并写入Chassis类
    float scale = 1.0f;
    if (speed_max > kWheelSpeedMax) {
        scale = kWheelSpeedMax / speed_max;
    }
    motion_chassis_ =      // 最终目标速度
        ChassisMotion(vx * scale, vy * scale, yaw_rate_ * scale);
    yaw_rate_ *= scale;

    // 逐轮解算并pid输出
    for (int i = 0; i < 4u; i++) {
        hello_world::motor::Motor* steer = motor_steer_[i];
        hello_world::motor::Motor* wheel = motor_wheel_[i];
        hello_world::pid::Pid* steer_pid = pid_steer_[i];
        hello_world::pid::Pid* wheel_pid = pid_wheel_[i];

        if (steer == nullptr || wheel == nullptr || steer_pid == nullptr ||
            wheel_pid == nullptr) {
            solution_[i] = WheelSolution();
            continue;
        }

        const float cur_angle = steer->angle();
        float speed = sqrtf(vix[i] * vix[i] + viy[i] * viy[i]) * scale;  //速度
        float angle = atan2f(viy[i], vix[i]);                            //方向

        // 舵角多解：与当前舵角相差超过 π/2 时舵角 +π、轮速取反
        if (fabsf(hello_world::AngleNormRad(angle - cur_angle)) > PI / 2) {
            angle += PI;
            speed = -speed;
        }

        // 轮速过小时不转舵
        if (fabsf(speed) < kSolveSpeedDeadband) {
            speed = 0.0f;
            angle = cur_angle;
        }

        solution_[i].steer_angle = angle;
        solution_[i].wheel_speed = speed / kWheelRadius;

        // 舵向角环：目标角平移到合理等效值再进 PID，防止跨零时突变
        const float ref = cur_angle + hello_world::AngleNormRad(angle - cur_angle);
        steer->setInput(steer_pid->calc(ref, cur_angle, dt) * kSteerCurrPerRaw);

        // 驱动轮速度环：目标为输出端角速度，输出为输出端力矩
        wheel->setInput(wheel_pid->calc(solution_[i].wheel_speed, wheel->vel(), dt));
    }
}

void robot::Chassis::recoverSteer(float dt)
{
    for (int i = 0; i < 4u; i++) {
        if (motor_wheel_[i] != nullptr) {
            motor_wheel_[i]->setInput(0.0f);
        }     // 驱动轮不出力
        solution_[i] = WheelSolution();   // solution 置零

        if (motor_steer_[i] == nullptr || pid_steer_[i] == nullptr) {
            continue;
        }

        // 舵角回中：目标 0，输出限幅避免自检时舵机猛甩
        const float cur_angle = motor_steer_[i]->angle();
        const float ref = cur_angle + hello_world::AngleNormRad(-cur_angle);
        const float out = pid_steer_[i]->calc(ref, cur_angle, dt);
        motor_steer_[i]->setInput(hello_world::Bound(out, kSteerCenterOutMax, -kSteerCenterOutMax) * kSteerCurrPerRaw);
    }
}

bool robot::Chassis::isSteerCentered() const
{
    for (int i = 0; i < 4u; i++) {
        if (motor_steer_[i] == nullptr) {
            return false;
        }
        if (fabsf(hello_world::AngleNormRad(motor_steer_[i]->angle())) >
            kSteerCenterTol) {
            return false;
        }
    }
    return true;
}

void robot::Chassis::resetController()
{
    for (int i = 0; i < 4u; i++) {
        if (pid_wheel_[i] != nullptr) {
            pid_wheel_[i]->reset();
        }
        if (pid_steer_[i] != nullptr) {
            pid_steer_[i]->reset();
        }
    }
    pid_follow_.reset();
    yaw_rate_ = 0.0f;
}

void robot::Chassis::stop()
{
    for (int i = 0; i < 4u; i++) {
        if (motor_wheel_[i] != nullptr) {
            motor_wheel_[i]->setInput(0.0f);
        }
        if (motor_steer_[i] != nullptr) {
            motor_steer_[i]->setInput(0.0f);
        }
        solution_[i] = WheelSolution();
    }
    resetController();
    motion_chassis_.reset();
}

const robot::Chassis::WheelSolution& robot::Chassis::solution(int idx) const{
    HW_ASSERT(0 <= idx && idx < 4, "Error wheel solution idx: %d", idx);
    return solution_[idx];
}

float robot::Chassis::calcYawRate(float w_manual, float dt){
    float w = 0.0f;
    switch (mode_) {
        case ChassisMode::kFollow:      
            w = pid_follow_.calc(delta_ + hello_world::AngleNormRad(delta_), delta_, dt);
            break;
        case ChassisMode::kSpin:
            w = hello_world::Bound(w_manual, kMaxSpinRate, -kMaxSpinRate);
            break;
        case ChassisMode::kSeperate:
            w = 0.0f;
            break;
        default:   // 非法模式，保守停机
            w = 0.0f;
            break;
    }
    return w;
}



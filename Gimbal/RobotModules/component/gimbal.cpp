/**
 *******************************************************************************
 * @file      :gimbal.cpp
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
 * 【新人任务】实现 gimbal.hpp 中声明的云台位置控制。
 *
 * 你要做：
 *   1. yaw 环：目标角与反馈角在同一角度域做差，再经位置环 PID 输出转矩 / 电流。
 *   2. pitch 环：加软件限位，超出范围时钳位指令，防止顶到机械限位堵转。
 *   3. 角度回绕：yaw 的差值要归一到 [-π, π] 区间，避免 359°→1° 时反向大转。
 *
 * 约束 / 提示：
 *   - 位置环输出是转矩 / 电流，注意与电机 setInput 的输入类型匹配。
 * ==========================================================================*/

#include "gimbal.hpp"
#include "base.hpp"  
#include "motor.hpp"  
#include "pid.hpp"    

namespace robot {

namespace {
    constexpr float kDefaultDt = 0.001f;
}  // namespace

void Gimbal::registerMotor(hello_world::motor::Motor* yaw,
                           hello_world::motor::Motor* pitch){
    motor_yaw_ = yaw;
    motor_pitch_ = pitch;
}

void Gimbal::registerPid(hello_world::pid::Pid* yaw,
                         hello_world::pid::Pid* pitch){
    pid_yaw_ = yaw;
    pid_pitch_ = pitch;
}

void Gimbal::addTargetDelta(const GimbalMotion& delta, float dt){

    if (dt <= 0.0f) { return; }

    // 限速率，限制单周期目标增量大小，防止目标跳变
    const float yaw_step = hello_world::Bound(
        delta.yaw, -kYawTargetRateLimit * dt, kYawTargetRateLimit * dt);
    const float pitch_step = hello_world::Bound(
        delta.pitch, -kPitchTargetRateLimit * dt, kPitchTargetRateLimit * dt);

    // AngleNormRad以防止目标值随时间无限增大
    target_.yaw = hello_world::AngleNormRad(target_.yaw + yaw_step);
    
    const float pitch_next = target_.pitch + pitch_step;
    // 软件限位
    target_.pitch = hello_world::Bound(pitch_next, kPitchLowerLimit, kPitchUpperLimit);
    // 记下这个状态，update() 里据此停积分（防积分饱和）
    pitch_limited_ = (pitch_next > kPitchUpperLimit) || (pitch_next < kPitchLowerLimit);
}

void Gimbal::syncTargetToActual(){
    target_ = actual_;
    pitch_limited_ = false;
}

void Gimbal::stop(){
    yaw_torque_ = 0.0f;
    pitch_torque_ = 0.0f;

    if (motor_yaw_ != nullptr) { motor_yaw_->setInput(0.0f); }
    if (motor_pitch_ != nullptr) { motor_pitch_->setInput(0.0f); }

    // 复位控制器
    if (pid_yaw_ != nullptr) { pid_yaw_->reset(); }
    if (pid_pitch_ != nullptr) { pid_pitch_->reset(); }

    // 目标角跟随当前关节角：再次使能时从当前位置起步
    syncTargetToActual();
}

void Gimbal::update(float dt, bool allow_output, float yaw_fb){
    if (dt <= 0.0f) { dt = kDefaultDt; }

    // 关节反馈角
    if (motor_yaw_ != nullptr) { actual_.yaw = yaw_fb; }
    if (motor_pitch_ != nullptr) { actual_.pitch = motor_pitch_->angle(); }

    // 不允许出力（非 Working / 双板离线 / 急停）：零力矩 + 复位
    if (!allow_output || motor_yaw_ == nullptr || motor_pitch_ == nullptr ||
        pid_yaw_ == nullptr || pid_pitch_ == nullptr) {
        stop();
        return;
    }

    const float yaw_err = hello_world::AngleNormRad(target_.yaw - actual_.yaw);
    const float yaw_ref = actual_.yaw + yaw_err;    // 跨零处理
    yaw_torque_ = pid_yaw_->calc(yaw_ref, actual_.yaw, dt);

    // pitch目标角限幅再次兜底（外部可能直接改目标）
    if (target_.pitch > kPitchUpperLimit) {
        target_.pitch = kPitchUpperLimit;
    } else if (target_.pitch < kPitchLowerLimit) {
        target_.pitch = kPitchLowerLimit;
    }

    // 到限位就停积分：目标一直被限位夹住时清积分
    if (pitch_limited_) { pid_pitch_->reset(); }
    
    pitch_torque_ = pid_pitch_->calc(target_.pitch, actual_.pitch, dt);

    // 力矩限幅
    yaw_torque_ = hello_world::Bound(yaw_torque_, -kTorqueLimit, kTorqueLimit);
    pitch_torque_ = hello_world::Bound(pitch_torque_, -kTorqueLimit, kTorqueLimit);

    // 指令下发
    motor_yaw_->setInput(yaw_torque_);
    motor_pitch_->setInput(pitch_torque_);
}

}  // namespace robot



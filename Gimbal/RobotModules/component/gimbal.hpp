/**
 *******************************************************************************
 * @file      :gimbal.hpp
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

#ifndef GIMBAL_ROBOT_MODULES_GIMBAL_HPP_
#define GIMBAL_ROBOT_MODULES_GIMBAL_HPP_

/* ============================================================================
 * 【新人任务】声明云台运动模块类 Gimbal（namespace robot）。
 *
 * 你要做：
 *   1. 设计 Gimbal 类，实现 yaw / pitch 两个关节的关节空间位置控制（位置环）。
 *   2. 读入云台目标角度（来自遥控器 / 键鼠 / 底盘经双板通讯传来的期望值），输出
 *      yaw / pitch 电机的控制量。
 *   3. 处理 yaw 轴 360° 连续旋转的角度回绕，pitch 轴用软件限位防堵转。
 *
 * 约束 / 提示：
 *   - yaw / pitch 均为达妙 DM-J4310 电机，反馈角度 / 指令接口见 inc/motor_DaMiao.hpp。
 *   - 位置环 PID 由你自己实现（PID 类），本模块只负责调用。
 * ==========================================================================*/

#include "module_state.hpp"

namespace hello_world {
namespace motor {
class Motor;
}  // namespace motor
namespace pid {
class Pid;
}  // namespace pid
}  // namespace hello_world

namespace robot {

//   yaw  : 误差先归一化到 [-pi, pi) 再闭环
//   pitch: 软件限位 + 到限位停积分
class Gimbal {
 public:
    static constexpr float kTorqueLimit = 2.5f;           // 关节力矩上限(Nm)

    static constexpr float kPitchLowerLimit = -0.35f;
    static constexpr float kPitchUpperLimit = 0.55f;      // pitch 软件限位（按实车改）(rad)

    static constexpr float kYawTargetRateLimit = 8.0f;
    static constexpr float kPitchTargetRateLimit = 4.0f;   // 关节目标角速率上限(rad/s)

    Gimbal() = default;
    ~Gimbal() = default;

    // 注册
    void registerMotor(hello_world::motor::Motor* yaw,
                       hello_world::motor::Motor* pitch);
    void registerPid(hello_world::pid::Pid* yaw, 
                     hello_world::pid::Pid* pitch);

    // 积分得目标角
    void addTargetDelta(const GimbalMotion& delta, float dt);
    // 重新使能 / 状态切换时让目标角跟随当前关节角，防止突变
    void syncTargetToActual();
    // 输出零力矩并复位 PID，同时让目标角跟随当前关节角
    void stop();

    // 周期任务
    // allow_output = false 时等价于 stop()
    void update(float dt, bool allow_output, float yaw_fb);

    // 数据访问
    const GimbalMotion& target() const { return target_; }
    const GimbalMotion& actual() const { return actual_; }
    float yaw_torque() const { return yaw_torque_; }
    float pitch_torque() const { return pitch_torque_; }
    bool pitch_limited() const { return pitch_limited_; }

 private:
    hello_world::motor::Motor* motor_yaw_ = nullptr;
    hello_world::motor::Motor* motor_pitch_ = nullptr;
    hello_world::pid::Pid* pid_yaw_ = nullptr;
    hello_world::pid::Pid* pid_pitch_ = nullptr;

    GimbalMotion target_;        // 关节目标角 (rad)
    GimbalMotion actual_;        // 关节反馈角 (rad)
    float yaw_torque_ = 0.0f;    // 本周期 yaw 力矩指令 (Nm)
    float pitch_torque_ = 0.0f;  // 本周期 pitch 力矩指令 (Nm)
    bool pitch_limited_ = false; // 目标角是否已被 pitch 软件限位夹住
};

}  // namespace robot


#endif /* GIMBAL_ROBOT_MODULES_GIMBAL_HPP_ */

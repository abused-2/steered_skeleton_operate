/**
 *******************************************************************************
 * @file      :chassis.hpp
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

#ifndef CHASSIS_ROBOT_MODULES_CHASSIS_HPP_
#define CHASSIS_ROBOT_MODULES_CHASSIS_HPP_

/* ============================================================================
 * 【新人任务】声明底盘运动模块类 Chassis（namespace robot）。
 *
 * 你要做：
 *   1. 设计 Chassis 类，负责把底盘速度指令 (vx, vy, w) 逆解为每个轮 / 舵的控制量：
 *      舵向轮运动学逆解（4 个 GM6020 舵电机 + 4 个 M3508 轮电机），输出每个舵的
 *      目标转角与每个轮的轮速（或电流 / 转矩指令）。
 *   2. 提供接口让 Robot 下发目标速度、读取解算结果。
 *
 * 约束 / 提示：
 *   - 几何参数（轮半径、轮距、轮心距、轮位坐标）见考核说明文档。
 *   - 逆解公式需自己推导（舵向轮注意舵角多解与轮速符号翻转），网上「舵向轮 / 麦克纳姆轮逆运动学」资料可参考。
 * ==========================================================================*/

#include "module_state.hpp"
#include "pid.hpp"

namespace hello_world {
namespace motor {
class Motor;
}}  // namespace

namespace robot{

class Chassis {
 public:
    static constexpr float kWheelRadius = 0.066f;     // 轮半径 (m)
    static constexpr float kMaxLinearSpeed = 2.0f;    // 平移速度上限 (m/s)
    static constexpr float kMaxSpinRate = 8.0f;       // 小陀螺自转速度上限
    static constexpr float kMaxFollowYawRate = 6.0f;  // 跟随：δ 角环输出上限

    // 单个轮组的解算结果
    struct WheelSolution {
        float steer_angle = 0.0f;  // 舵角目标 (rad)（允许超出 [-π, π)）
        float wheel_speed = 0.0f;  // 轮速目标 (rad/s)
    };

    Chassis();

    // 电机与pid注册（MainTaskInit 中调用一次）
    void registerMotorWheel(int idx, hello_world::motor::Motor* motor);
    void registerMotorSteer(int idx, hello_world::motor::Motor* motor);
    void registerPidWheel(int idx, hello_world::pid::Pid* pid);
    void registerPidSteer(int idx, hello_world::pid::Pid* pid);

    // 工作模式
    void setMode(ChassisMode mode) { mode_ = mode; }  // 写入
    ChassisMode mode() const { return mode_; }        // 读取

    //云台偏向角 delta
    void setDelta(float delta) { delta_ = delta; }
    float delta() const { return delta_; }

    // 周期任务
    // motion 为RC指令 (vx, vy)（w 分量只适用于云台，直接忽略）
    // w_manual 为拨轮映射出的小陀螺自转指令 (rad/s)
    void update(const ChassisMotion& motion, float w_manual, float dt);

    // 舵角回中（Resurrection 自检用）：只动舵电机且输出限幅，驱动轮不出力
    void recoverSteer(float dt);

    // 四舵是否都在中位容差内
    bool isSteerCentered() const;

    // 复位所有控制器（状态切换时调用）
    void resetController();

    // 停机：所有电机输入清零 + 复位控制器 + 清空解算结果
    void stop();

    // 结果读取
    const WheelSolution& solution(int idx) const;
    const ChassisMotion& motion_chassis() const { return motion_chassis_; }
    float yaw_rate() const { return yaw_rate_; }

 private:
    // 控制器/执行器对象
    hello_world::motor::Motor* motor_wheel_[4] = {nullptr, nullptr, nullptr, nullptr};
    hello_world::motor::Motor* motor_steer_[4] = {nullptr, nullptr, nullptr, nullptr};
    hello_world::pid::Pid* pid_wheel_[4] = {nullptr, nullptr, nullptr, nullptr};
    hello_world::pid::Pid* pid_steer_[4] = {nullptr, nullptr, nullptr, nullptr};

    float delta_ = 0.0f;              // 相对偏航角 (rad)
    ChassisMode mode_ = ChassisMode::kSeperate;

    // 按工作模式计算偏航角速度
    float calcYawRate(float w_manual, float dt);

    hello_world::pid::Pid pid_follow_; // 跟随模式的 δ 角度环

    //各级解算结果
    float yaw_rate_ = 0.0f;           // 本周期实际下发的w (rad/s)    （车级）
    ChassisMotion motion_chassis_{};  // 旋转并限速后的车体系速度指令   (车级)
    WheelSolution solution_[4];       // 各轮组解算结果                (轮级)
};

}  // namespace robot

#endif /* CHASSIS_ROBOT_MODULES_CHASSIS_HPP_ */

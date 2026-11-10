/**
 *******************************************************************************
 * @file      :robot.hpp
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

#ifndef GIMBAL_ROBOT_MODULES_ROBOT_HPP_
#define GIMBAL_ROBOT_MODULES_ROBOT_HPP_

/* ============================================================================
 * 【新人任务】声明云台板顶层状态机类 Robot（namespace robot）。
 *
 * 你要做：
 *   1. 设计 Robot 类，作为本板每个控制周期的入口：汇总云台目标角度（来自遥控器 /
 *      键鼠 / 底盘经双板通讯传来的期望值）、IMU 姿态、电机反馈，生成 yaw / pitch
 *      控制指令。
 *   2. 实现「生命状态」三态切换：Dead → Resurrection → Working，并定义各状态
 *      进入 / 退出时的复位动作。
 *   3. 提供注册各模块指针的接口（Gimbal、Imu、yaw / pitch 电机、GimbalChassisComm），
 *      并在每个周期按正确顺序更新数据。
 *
 * 约束 / 提示：
 *   - 与底盘板之间只能通过 GimbalChassisComm 的
 *     CAN 帧交换信息，不能直接互相调用函数或共享变量。
 *   - 控制周期入口由 Task/component/main_task.cpp 的定时中断调用，本类自己不要开中断、
 *     不要死等延时。
 *   - 急停：遥控器拨到急停位时，必须保证所有电机指令归零（硬性验收项）。
 *   - 状态机的基类 / 迁移机制要自己设计，本骨架不提供（裁判系统 / 超电 / 蜂鸣器等组件本骨架也没有，可忽略）。
 * ==========================================================================*/

#include <cstdint>
#include "module_state.hpp"
#include "offline_checker.hpp"

namespace hello_world {
namespace imu {
class Imu;
}  // namespace imu
namespace motor {
class Motor;
}  // namespace motor
namespace remote_control {
class DT7;
}  // namespace remote_control
}  // namespace hello_world

namespace robot {

class Gimbal;
class GimbalChassisComm;

class Robot {
 public:
    Robot() = default;
    ~Robot() = default;

    // 注册
    void registerImu(hello_world::imu::Imu* imu);
    void registerRemoteControl(hello_world::remote_control::DT7* remote_control);
    void registerGimbal(Gimbal* gimbal);
    void registerGimbalChassisComm(GimbalChassisComm* comm);
    // 关节电机：idx 0 = yaw，1 = pitch
    void registerMotor(int idx, hello_world::motor::Motor* motor);

    // 1 ms 控制周期入口：读数据 -> 迁移状态 -> 生成指令 -> 下发
    void update();

    // 遥控器帧到达通知，用于离线检测
    void notifyRemoteControlRx();
    // 遥控器是否离线
    bool isRemoteControlOffline();

    // 状态查询
    LifeState life_state() const { return life_state_; }
    ChassisMode chassis_mode() const { return chassis_mode_; }
    

 private:
    // 刷新输入与健康度判定
    void readInput();
    // 生命状态迁移
    void updateLifeState();
    // 三种状态下的出力
    void outputDead();
    void outputResurrection();
    void outputWorking();
    // 状态切换（含所有 PID 复位）
    void switchState(LifeState next);
    //  G2C 帧写入
    void updateCommData();
    // 所有电机输入清零(绕过Gimbal层，最外层置零)
    void stopAll();
    // 摇杆死区处理：死区内输出 0，死区外线性拉伸回 [-1, 1]
    static float applyDeadzone(float value, float deadzone);
    // 自检项：两台达妙电机是否已反馈使能
    bool motorsEnabled() const;

    hello_world::imu::Imu* imu_ = nullptr;
    hello_world::remote_control::DT7* remote_control_ = nullptr;
    Gimbal* gimbal_ = nullptr;
    GimbalChassisComm* comm_ = nullptr;
    hello_world::motor::Motor* motor_[2] = {nullptr, nullptr};
    hello_world::OfflineChecker rc_checker_{100};  // 遥控器离线判定，阈值 100 ms

    LifeState life_state_ = LifeState::kDead;            // 当前生命状态
    ChassisMode chassis_mode_ = ChassisMode::kSeperate;  // 底盘模式
    bool e_stop_ = true;              // 本周期急停（遥控缺失 / 离线 / 右拨杆 Down）
    bool comm_online_ = false;        // 双板帧是否在线
    bool rc_switch_cleared_ = false;  // 上电联锁：右拨杆曾回到过 Mid / Down
    uint32_t resurrection_cnt_ = 0;   // Resurrection 已驻留的周期数
};

}  // namespace robot

#endif /* GIMBAL_ROBOT_MODULES_ROBOT_HPP_ */

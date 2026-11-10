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

#ifndef CHASSIS_ROBOT_MODULES_ROBOT_HPP_
#define CHASSIS_ROBOT_MODULES_ROBOT_HPP_

/* ============================================================================
 * 【新人任务】声明底盘板顶层状态机类 Robot（namespace robot）。
 *
 * 你要做：
 *   1. 设计 Robot 类，作为本板每个控制周期（1ms）的入口：汇总各模块数据，
 *      生成控制指令并下发给执行部件。
 *   2. 实现「生命状态」三态切换：Dead（上电未就绪 / 离线）→ Resurrection（恢复
 *      自检）→ Working（正常控制），并定义各状态进入 / 退出时的复位动作。
 *   3. 在 Working 态下按手动控制源与工作模式（分离 / 随动 / 小陀螺）生成底盘运动指令。
 *   4. 提供注册各模块指针的接口（Chassis、Imu、4 轮电机、4 舵电机、GimbalChassisComm），并在每个周期按正确顺序更新数据。
 *
 * 约束 / 提示：
 *   - 底盘板与云台板之间只能通过 GimbalChassisComm 的
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
}  // namespace hello_world

namespace robot{

class Chassis;
class GimbalChassisComm;

class Robot {
 public:
    Robot() = default;

    // 模块注册（MainTaskInit 中调用一次）
    void registerImu(hello_world::imu::Imu* imu);
    void registerChassis(Chassis* chassis);
    void registerGimbalChassisComm(GimbalChassisComm* comm);
    void registerMotorWheel(int idx, hello_world::motor::Motor* motor);
    void registerMotorSteer(int idx, hello_world::motor::Motor* motor);

    // 1ms周期内：读数据 -> 迁移状态 -> 生成指令 -> 下发
    void update();

    // 状态查询
    LifeState life_state() const { return life_state_; }
    ChassisMode chassis_mode() const { return mode_; }

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
    // 把本板信息写进 C2G 帧
    void updateCommData();
    // 所有电机输入清零(绕过Chassis层，最外层置零)
    void stopAll();
    // 摇杆死区处理：死区内输出 0，死区外线性拉伸回 [-1, 1]
    static float applyDeadzone(float value, float deadzone);

    hello_world::imu::Imu* imu_ = nullptr;
    Chassis* chassis_ = nullptr;
    GimbalChassisComm* comm_ = nullptr;

    hello_world::motor::Motor* motor_wheel_[4] = {nullptr,nullptr,nullptr,nullptr};
    hello_world::motor::Motor* motor_steer_[4] = {nullptr,nullptr,nullptr,nullptr};

    LifeState life_state_ = LifeState::kDead;    // 当前生命状态
    ChassisMode mode_ = ChassisMode::kSeperate;  // 当前底盘工作模式

    bool e_stop_ = true;              // 本周期急停（帧离线 / 右拨杆 Down）
    bool comm_online_ = false;        // 双板帧是否在线
    bool rc_switch_cleared_ = false;  // 上电联锁：右拨杆曾回到过 Mid / Down
    uint32_t resurrection_cnt_ = 0;   // Resurrection 已驻留的周期数
};

}  // namespace robot


#endif /* CHASSIS_ROBOT_MODULES_ROBOT_HPP_ */

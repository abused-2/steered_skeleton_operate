/**
 *******************************************************************************
 * @file      :robot.cpp
 * @brief     :底盘板顶层状态机实现
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-11-03      <author>        1. 遥控信息改由 G2C 帧获取，状态机独立判断
 *******************************************************************************
 * @attention :
 *  1. 实车上只有云台板接遥控器接收机，因此本板所有遥控器输入（两个摇杆、拨轮、
 *     左右拨杆）都来自双板帧 G2C，见 gimbal_chassis_comm.hpp 的帧格式说明。
 *  2. 由此带来一条硬性约束：**帧离线等价于急停**。readInput() 里默认 e_stop_ = true，
 *     只有确认帧在线、且右拨杆不在急停位时才允许清除。
 *  3. 急停判断放在最外层（update() 的第一步 readInput），任何控制分支之前
 *     都已经把电机指令清零（Dead 状态），不做降级运行。
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */

#include "robot.hpp"
#include "DT7.hpp"
#include "assert.hpp"
#include "chassis.hpp"
#include "gimbal_chassis_comm.hpp"
#include "imu.hpp"
#include "motor_base.hpp"

namespace {
constexpr float kCtrlDt = 0.001f;               // 控制周期 1 ms
constexpr float kRcDeadzone = 0.05f;            // 摇杆 / 拨轮死区
constexpr uint32_t kResurrectionMinTick = 100;  // 自检驻留 100 ms
}  // namespace

void robot::Robot::registerImu(hello_world::imu::Imu* imu) { imu_ = imu; }
void robot::Robot::registerChassis(Chassis* chassis) { chassis_ = chassis; }
void robot::Robot::registerGimbalChassisComm(GimbalChassisComm* comm) {
    comm_ = comm;
}
void robot::Robot::registerMotorWheel(int idx, hello_world::motor::Motor* motor) {
    HW_ASSERT(0 <= idx && idx < 4, "Error wheel motor idx: %d", idx);
    motor_wheel_[idx] = motor;
}
void robot::Robot::registerMotorSteer(int idx, hello_world::motor::Motor* motor) {
    HW_ASSERT(0 <= idx && idx < 4, "Error steer motor idx: %d", idx);
    motor_steer_[idx] = motor;
}

void robot::Robot::update() {
    // 1. IMU 姿态更新（1 ms 一次）
    if (imu_ != nullptr) {
        imu_->update();
    }

    // 2. 输入与健康度判定（急停在最外层）
    readInput();

    // 3. 生命状态迁移
    updateLifeState();

    // 4. 按当前状态出力
    switch (life_state_) {
        case LifeState::kDead:
            outputDead();
            break;
        case LifeState::kResurrection:
            outputResurrection();
            break;
        case LifeState::kWorking:
            outputWorking();
            break;
        default:
            stopAll();
            break;
    }

    // 5. 把本板状态写进 C2G 帧
    updateCommData();
}

void robot::Robot::readInput() {
    // 双板帧健康度：双板帧是否在线
    comm_online_ = (comm_ != nullptr) && !comm_->isOffline();

    // 遥控器健康度：遥控器是否在线，右拨杆是否在急停位
    e_stop_ = true;
    if (comm_online_ && comm_->gimbal_data().rc_valid) {
        e_stop_ = (comm_->gimbal_data().rc_r_switch == hello_world::remote_control::kSwitchStateDown);
    }

    // 上电联锁
    if (comm_online_ && comm_->gimbal_data().rc_valid &&
        comm_->gimbal_data().rc_r_switch !=
            hello_world::remote_control::kSwitchStateUp) {
        rc_switch_cleared_ = true;
    }

    // 工作模式：左拨杆 Up 分离 / Mid 跟随 / Down 小陀螺
    mode_ = ChassisMode::kSeperate;
    if (comm_online_) {
        switch (comm_->gimbal_data().rc_l_switch) {
            case hello_world::remote_control::kSwitchStateMid:
                mode_ = ChassisMode::kFollow;
                break;
            case hello_world::remote_control::kSwitchStateDown:
                mode_ = ChassisMode::kSpin;
                break;
            case hello_world::remote_control::kSwitchStateUp:
            default:
                // 拨杆值非法，保守处理
                mode_ = ChassisMode::kSeperate;
                break;
        }
    }

    if (chassis_ != nullptr) {
        chassis_->setMode(mode_);
        // delta 有效性校验：只接受落在 [-π, π] 的值（NaN 比较恒假，同样被挡下）；
        // 非法时沿用上一拍的合法值，避免方向突然跳回「对齐」位置
        float delta = chassis_->delta();
        if (comm_online_) {
            const float d = comm_->gimbal_data().delta_joint;
            if (d >= -PI && d <= PI) { delta = d; }
        }
        chassis_->setDelta(delta);
    }
}

void robot::Robot::updateLifeState() {
    const bool imu_ready = (imu_ != nullptr) && imu_->isReady();

    switch (life_state_) {
        case LifeState::kDead:
            // 遥控在线且无急停且双板帧在线，才允许开始恢复自检
            if (!e_stop_ && comm_online_) {
                switchState(LifeState::kResurrection);
            }
            break;

        case LifeState::kResurrection: {
            if (e_stop_ || !comm_online_) {
                switchState(LifeState::kDead);
                break;
            }

            ++resurrection_cnt_;

            // 自检项：IMU 就绪 + 舵角回中 + 驻留时间足够
            const bool self_check_ok =
                imu_ready && (chassis_ != nullptr) &&
                chassis_->isSteerCentered() &&
                (resurrection_cnt_ >= kResurrectionMinTick);
            const bool run_switch =
                comm_online_ &&
                (comm_->gimbal_data().rc_r_switch ==
                 hello_world::remote_control::kSwitchStateUp);

            if (self_check_ok && rc_switch_cleared_ && run_switch) {
                switchState(LifeState::kWorking);
            }
            break;
        }

        case LifeState::kWorking:
            // 任何异常都不降级运行：先归零，再回 Dead 重新走自检
            if (e_stop_ || !comm_online_ || !imu_ready) {
                switchState(LifeState::kDead);
            }
            break;

        default:
            switchState(LifeState::kDead);
            break;
    }
}

void robot::Robot::outputDead() { 
    stopAll(); 
}

void robot::Robot::outputResurrection() {
    // 自检阶段只做舵角回中（限速），驱动轮不出力
    if (chassis_ != nullptr) {
        chassis_->recoverSteer(kCtrlDt);
    } else {
        stopAll();
    }
}

void robot::Robot::outputWorking() {
    if (chassis_ == nullptr || comm_ == nullptr) {
        stopAll();
        return;
    }

    // 待命位（右拨杆 Mid）：保持 Working 但不出力
    if (comm_->gimbal_data().rc_r_switch !=
        hello_world::remote_control::kSwitchStateUp) {
        chassis_->stop();
        return;
    }

    const float vx = applyDeadzone(comm_->gimbal_data().rc_lv, kRcDeadzone) *
                     Chassis::kMaxLinearSpeed;
    // 车体系 x 前 / y 左，而 rc_lh 右推为正，故取负
    const float vy = -applyDeadzone(comm_->gimbal_data().rc_lh, kRcDeadzone) *
                     Chassis::kMaxLinearSpeed;
    const float wheel = applyDeadzone(comm_->gimbal_data().rc_wheel, kRcDeadzone);
    // 拨轮右推（正）对应顺时针（右转），故取负
    const float w_manual = (mode_ == ChassisMode::kSpin) ? -wheel * Chassis::kMaxSpinRate : 0.0f;

    // 调用 update 完成逆解 + PID 下发
    chassis_->update(ChassisMotion(vx, vy, 0.0f), w_manual, kCtrlDt);
}

void robot::Robot::switchState(LifeState next) {
    if (next == life_state_) {
        return;
    }

    life_state_ = next;

    // 状态切换一律复位所有控制器，避免把上一状态的积分 / 微分历史带进来
    if (chassis_ != nullptr) {
        chassis_->resetController();
    }

    if (life_state_ == LifeState::kDead) {
        rc_switch_cleared_ = false;  // 重新上锁，恢复后必须再拨一次右拨杆
        stopAll();
    } else if (life_state_ == LifeState::kResurrection) {
        resurrection_cnt_ = 0;
    }
}

void robot::Robot::updateCommData() {
    if (comm_ == nullptr) {
        return;
    }

    comm_->chassis_data().ready = (life_state_ == LifeState::kWorking);
    comm_->chassis_data().life_state = life_state_;
    comm_->chassis_data().mode = mode_;
    if (imu_ != nullptr && imu_->isReady()) {
        comm_->chassis_data().yaw = imu_->yaw();
        comm_->chassis_data().w = imu_->gyro()[2];
    } else {
        comm_->chassis_data().yaw = 0.0f;
        comm_->chassis_data().w = 0.0f;
    }
}

void robot::Robot::stopAll() {
    for (int i = 0; i < 4u; i++) {
        if (motor_wheel_[i] != nullptr) {
            motor_wheel_[i]->setInput(0.0f);
        }
        if (motor_steer_[i] != nullptr) {
            motor_steer_[i]->setInput(0.0f);
        }
    }
    if (chassis_ != nullptr) {
        chassis_->stop();
    }
}

float robot::Robot::applyDeadzone(float value, float deadzone) {
    if (value > deadzone) {
        return (value - deadzone) / (1.0f - deadzone);
    }
    if (value < -deadzone) {
        return (value + deadzone) / (1.0f - deadzone);
    }
    return 0.0f;
}

/**
 *******************************************************************************
 * @file      :robot.cpp
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
 * 【新人任务】实现 robot.hpp 中声明的状态机。
 *
 * 你要做：
 *   1. 周期入口：按「更新数据 → 状态机迁移 → 生成指令 → 下发」的顺序组织 update()。
 *   2. 三态实现：Dead 时输出全零并等待就绪条件；Resurrection 时做自检与复位（如等待
 *      电机使能、IMU 零漂收敛）；Working 时执行正常控制逻辑。
 *   3. 各工作模式（分离 / 随动 / 小陀螺）的指令生成分支。
 *
 * 约束 / 提示：
 *   - 状态切换时记得复位上一状态残留的指令，避免电机突变。
 *   - 所有下发指令统一经过本类，急停判断放在最外层，确保任何分支都先过急停。
 * ==========================================================================*/

#include "robot.hpp"
#include "DT7.hpp"
#include "assert.hpp"
#include "base.hpp"
#include "gimbal.hpp"
#include "gimbal_chassis_comm.hpp"
#include "imu.hpp"
#include "motor_DaMiao.hpp"

// ---------- 调试镜像变量（仅供 Ozone 观测，不参与任何控制逻辑）----------
volatile uint8_t g_dbg_life_state  = 0;   // life_state_：0=Dead 1=Resur 2=Working
volatile uint8_t g_dbg_e_stop      = 0;   // 1=急停
volatile uint8_t g_dbg_comm_online = 0;   // 1=收到底盘帧
volatile uint8_t g_dbg_rc_cleared  = 0;   // 1=上电联锁已解除
volatile float   g_dbg_yaw_target  = 0.0f;
volatile float   g_dbg_yaw_actual  = 0.0f;
volatile float g_dbg_yaw_torque   = 0.0f;
volatile float g_dbg_pitch_torque = 0.0f;
volatile uint8_t g_dbg_pitch_limited = 0;
volatile uint8_t g_dbg_imu_ready = 0;
volatile uint8_t g_dbg_motorsEnabled = 0;
volatile uint8_t g_dbg_yaw_enabled   = 0;
volatile uint8_t g_dbg_pitch_enabled = 0;
volatile float   g_dbg_yaw_angle     = 0.0f;
volatile float   g_dbg_pitch_angle   = 0.0f;

namespace {
constexpr float kCtrlDt = 0.001f;               // 控制周期（s）
constexpr float kRcDeadzone = 0.02f;            // 摇杆死区（瞄准要细，比底盘小）
constexpr uint32_t kResurrectionMinTick = 100;  // 自检最短驻留 100 ms

constexpr float kYawStickMaxRate = 6.0f;
constexpr float kPitchStickMaxRate = 3.0f;      //最大指令速度
}  // namespace


void robot::Robot::registerImu(hello_world::imu::Imu* imu){
    imu_ = imu;
}
void robot::Robot::registerRemoteControl(hello_world::remote_control::DT7* remote_control){
    remote_control_ = remote_control;
}
void robot::Robot::registerGimbal(Gimbal* gimbal){
    gimbal_ = gimbal;
}
void robot::Robot::registerGimbalChassisComm(GimbalChassisComm* comm){
    comm_ = comm;
}
void robot::Robot::registerMotor(int idx, hello_world::motor::Motor* motor){
    HW_ASSERT(0 <= idx && idx < 2, "Error joint motor idx: %d", idx);
    motor_[idx] = motor;
}

void robot::Robot::update(){
    // 1. IMU 姿态更新（1 ms 一次）
    if (imu_ != nullptr) {
        imu_->update();
    }

    // 2. 输入与健康度判定
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

    // 5. 把本板状态写进 G2C 帧
    updateCommData();

    // 调试镜像（放在最后，保证反映本周期最终状态）
    g_dbg_life_state  = static_cast<uint8_t>(life_state_);
    g_dbg_e_stop      = e_stop_ ? 1u : 0u;
    g_dbg_comm_online = comm_online_ ? 1u : 0u;
    g_dbg_rc_cleared  = rc_switch_cleared_ ? 1u : 0u;
    if (gimbal_ != nullptr) {
        g_dbg_yaw_target = gimbal_->target().yaw;
        g_dbg_yaw_actual = gimbal_->actual().yaw;
        g_dbg_yaw_torque    = gimbal_->yaw_torque();
        g_dbg_pitch_torque  = gimbal_->pitch_torque();
        g_dbg_pitch_limited = gimbal_->pitch_limited() ? 1u : 0u;
    }
    
}

void robot::Robot::notifyRemoteControlRx(){
    rc_checker_.update();
}
bool robot::Robot::isRemoteControlOffline(){
    return rc_checker_.isOffline();
}

void robot::Robot::readInput()
{
    // 遥控器健康度：未注册 / 掉线 / 右拨杆打到急停位，一律视为急停
    e_stop_ = true;
    if (remote_control_ != nullptr && !rc_checker_.isOffline()) {
        e_stop_ = (remote_control_->rc_r_switch() ==
                   hello_world::remote_control::kSwitchStateDown);
    }

    // 双板帧健康度：底盘掉线后云台禁止出力
    comm_online_ = (comm_ != nullptr) && !comm_->isOffline();

    // 上电联锁
    if (remote_control_ != nullptr && remote_control_->rc_r_switch() !=
            hello_world::remote_control::kSwitchStateUp) {
        rc_switch_cleared_ = true;
    }

    // 记录左拨杆状态(底盘模式)，传输给底盘板
    chassis_mode_ = ChassisMode::kSeperate;
    if (remote_control_ != nullptr) {
        switch (remote_control_->rc_l_switch()) {
            case hello_world::remote_control::kSwitchStateMid:
                chassis_mode_ = ChassisMode::kFollow;
                break;
            case hello_world::remote_control::kSwitchStateDown:
                chassis_mode_ = ChassisMode::kSpin;
                break;
            case hello_world::remote_control::kSwitchStateUp:
            default:
                chassis_mode_ = ChassisMode::kSeperate;
                break;
        }
    }
}

void robot::Robot::updateLifeState()
{
    const bool imu_ready = (imu_ != nullptr) && imu_->isReady();

    switch (life_state_) {
        case LifeState::kDead:
            // 遥控在线、双板在线且无急停，才允许开始恢复自检
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

            // 自检项：IMU 就绪 + 两台关节电机已反馈使能 + 驻留时间足够；
            // 另外还要求上电联锁已满足、右拨杆在运行位，才允许进 Working
            const bool self_check_ok =
                imu_ready && motorsEnabled() &&
                (resurrection_cnt_ >= kResurrectionMinTick);
            const bool run_switch =
                (remote_control_ != nullptr) &&
                (remote_control_->rc_r_switch() ==
                 hello_world::remote_control::kSwitchStateUp);
            // 调试用
            if (imu_ready) g_dbg_imu_ready = 1;
            if (motorsEnabled()) g_dbg_motorsEnabled = 1;

            if (self_check_ok && rc_switch_cleared_ && run_switch) {
                switchState(LifeState::kWorking);
            }
            break;
        }

        case LifeState::kWorking:
            // 任何异常都不降级运行：先归零，再回 Dead 重新走自检
            if (e_stop_ || !comm_online_ || !imu_ready || !motorsEnabled()) {
                switchState(LifeState::kDead);
            }
            break;

        default:
            switchState(LifeState::kDead);
            break;
    }
}

void robot::Robot::outputDead(){
    stopAll();
}

void robot::Robot::outputResurrection()
{
    // 自检阶段不出力，但刷新一次关节角反馈供上报
    if (gimbal_ != nullptr) {
        gimbal_->update(kCtrlDt, false, (imu_ != nullptr) ? imu_->yaw() : 0.0f);
    } else {
        stopAll();
    }
}

void robot::Robot::outputWorking()
{
    if (gimbal_ == nullptr || remote_control_ == nullptr) {
        stopAll();
        return;
    }

    // 待命位（右拨杆 Mid）：与底盘板一致，保持 Working 但不出力
    if (remote_control_->rc_r_switch() != hello_world::remote_control::kSwitchStateUp) {
        gimbal_->stop();
        return;
    }

    // 读取计算目标角增量(rad)
    const float yaw_delta =
        -applyDeadzone(remote_control_->rc_rh(), kRcDeadzone) * kYawStickMaxRate * kCtrlDt;
    const float pitch_delta =
        applyDeadzone(remote_control_->rc_rv(), kRcDeadzone) * kPitchStickMaxRate * kCtrlDt;

    gimbal_->addTargetDelta(GimbalMotion(yaw_delta, pitch_delta), kCtrlDt);

    gimbal_->update(kCtrlDt, true, (imu_ != nullptr) ? imu_->yaw() : 0.0f);   

}

void robot::Robot::switchState(LifeState next)
{
    if (next == life_state_) { return; }

    life_state_ = next;

    // 状态切换一律复位所有控制器
    if (gimbal_ != nullptr) { gimbal_->stop(); }

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

    GimbalChassisComm::GimbalData& data = comm_->gimbal_data();

    const bool self_check_passed = (imu_ != nullptr) && imu_->isReady();
    data.ready = self_check_passed && (life_state_ != LifeState::kDead);
    data.life_state = life_state_;

    // 关节角
    if (gimbal_ != nullptr) {
        // 临时试验：+π 补偿 delta 零点偏移
        data.delta_joint = (motor_[0] != nullptr)
            ? hello_world::AngleNormRad(motor_[0]->angle() + PI) : 0.0f;
        data.pitch_joint = gimbal_->actual().pitch;
    } else {
        data.delta_joint = 0.0f;
        data.pitch_joint = 0.0f;
    }

    // 遥控数据是否可信
    data.rc_valid = (remote_control_ != nullptr) && !rc_checker_.isOffline();

    // 遥控器全部信息经本帧下发给底盘
    if (remote_control_ != nullptr && !rc_checker_.isOffline()) {
        data.rc_lv = remote_control_->rc_lv();
        data.rc_lh = remote_control_->rc_lh();
        data.rc_wheel = remote_control_->rc_wheel();
        data.rc_l_switch = remote_control_->rc_l_switch();
        data.rc_r_switch = remote_control_->rc_r_switch();
    } else {         //错误即取保守值，防止底盘取上一帧陈旧值导致误判
        data.rc_lv = 0.0f;
        data.rc_lh = 0.0f;
        data.rc_wheel = 0.0f;
        data.rc_l_switch = hello_world::remote_control::kSwitchStateUp;
        data.rc_r_switch = hello_world::remote_control::kSwitchStateDown;
    }
}

void robot::Robot::stopAll(){
    for (int i = 0; i < 2; i++) {
        if (motor_[i] != nullptr) {
            motor_[i]->setInput(0.0f);
        }
    }
    if (gimbal_ != nullptr) {
        gimbal_->stop();
    }    // 两层都下置零指令
}

float robot::Robot::applyDeadzone(float value, float deadzone){
    if (value > deadzone) {
        return (value - deadzone) / (1.0f - deadzone);
    }
    if (value < -deadzone) {
        return (value + deadzone) / (1.0f - deadzone);
    }
    return 0.0f;
}

bool robot::Robot::motorsEnabled() const{
    if (motor_[0] == nullptr || motor_[1] == nullptr) {
        return false;
    }
    const hello_world::motor::DaMiao* yaw =
        static_cast<const hello_world::motor::DaMiao*>(motor_[0]);

    //调试用
    g_dbg_yaw_enabled   = (motor_[0] && static_cast<const hello_world::motor::DaMiao*>(motor_[0])->is_enabled()) ? 1u : 0u;
    g_dbg_pitch_enabled = (motor_[1] && static_cast<const hello_world::motor::DaMiao*>(motor_[1])->is_enabled()) ? 1u : 0u;
    g_dbg_yaw_angle     = motor_[0] ? motor_[0]->angle() : 0.0f;
    g_dbg_pitch_angle   = motor_[1] ? motor_[1]->angle() : 0.0f;

    // 临时：pitch 电机无反馈，先只要求 yaw 使能，便于单轴调试
    return yaw->is_enabled();
}


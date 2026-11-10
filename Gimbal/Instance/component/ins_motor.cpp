/**
 *******************************************************************************
 * @file      :ins_motor.cpp
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

#include "ins_motor.hpp"
#include "motor_DaMiao.hpp"
#include "motor.hpp"

namespace {
constexpr uint8_t kMotorYawId = 1;     // CAN ID = 1（反馈 ID = 0x11）
constexpr uint8_t kMotorPitchId = 2;   // CAN ID = 2（反馈 ID = 0x12）

// 关节零点偏移，单位 rad
constexpr float kMotorYawAngleOffset = 1.60879f;
constexpr float kMotorPitchAngleOffset = 2.91772f;

void InitJointMotor(hello_world::motor::DM_J4310& motor, uint8_t id,
                    hello_world::motor::Dir dir, float angle_offset,
                    hello_world::motor::AngleRange angle_range)
{
    hello_world::motor::OptionalParams opt;
    opt.input_type = hello_world::motor::InputType::kTorq;
    opt.angle_range = angle_range;
    opt.dir = dir;
    opt.angle_offset = angle_offset;

    // auto_enable = true：上电后持续发使能帧，直到电机反馈使能成功
    // enable_mode_switch = false：只用 MIT 模式
    motor.init(id, opt, true, false);
}
}  // namespace

hello_world::motor::Motor *GetMotorYaw() {
  /* TODO【新人任务】:
   * 返回云台 yaw 关节电机单例（DM-J4310）。
   *
   * 你要做：
   *   1. 静态定义 yaw 电机对象（CreateMotor(kDM_J4310, id, opt) 或直接构造 DM_J4310）。
   *   2. 配置 id / 方向 / 减速比 / 角度偏移，并确保使能（auto_enable / enable()），
   *      达妙电机失能状态下不会反馈数据。
   *   3. 返回其地址。
   *
   * 约束 / 提示：
   *   - 设备类见 inc/motor_DaMiao.hpp（DM_J4310），工厂 MotorType::kDM_J4310。
   *   - yaw 关节注意 360° 连续旋转的角度过零处理（MIT 位置控制内部不做多圈累计，
   *     见 motor_DaMiao.hpp 注释）。
   */

  // 首次调用时构造并初始化，之后只返回地址
  static hello_world::motor::DM_J4310 motor;
  static bool is_inited = false;
  if (!is_inited) {
    // yaw 连续旋转：dir 与关节正方向同向，角度不做周期归一化以便多圈累计
    InitJointMotor(motor, kMotorYawId, hello_world::motor::kDirFwd,
                  kMotorYawAngleOffset,
                  hello_world::motor::AngleRange::kNegInfToPosInf);
    is_inited = true;
  }
  return &motor;
}

hello_world::motor::Motor *GetMotorPitch() {
  /* TODO【新人任务】:
   * 返回云台 pitch 关节电机单例（DM-J4310），做法同 GetMotorYaw()。
   * pitch 关节需用反馈角度做限位，防止堵转 / 撞结构。
   */

  static hello_world::motor::DM_J4310 motor;
  static bool is_inited = false;
  if (!is_inited) {
    // pitch 反向安装：dir 取反；角度归一到 [-pi, pi)，便于软件限位判断
    InitJointMotor(motor, kMotorPitchId, hello_world::motor::kDirRev,
                  kMotorPitchAngleOffset,
                  hello_world::motor::AngleRange::kNegPiToPosPi);
    is_inited = true;
  }
  return &motor;
}

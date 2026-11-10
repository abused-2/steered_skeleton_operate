/*
 * @Author: Qian Yicheng abused-2@users.noreply.github.com
 * @Date: 2026-10-27 21:48:23
 * @LastEditors: Qian Yicheng abused-2@users.noreply.github.com
 * @LastEditTime: 2026-11-01 16:50:55
 * @FilePath: \steered_skeleton_operate\Chassis\Instance\component\ins_motor.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
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
#include "assert.hpp"
#include "motor_GM6020.hpp"
#include "motor_M3508.hpp"

namespace {

// 各轮ID: 0 左前 / 1 左后 / 2 右后 / 3 右前
constexpr uint8_t kWheelId[4] = {1, 2, 3, 4};
constexpr uint8_t kSteerId[4] = {1, 2, 3, 4};

// 舵电机零点标定值
constexpr float kSteerAngleOffset[4] = {2.61346f, -2.06844f, 0.00729f,
                                                1.58518f};

// 舵电机电压输入上限（raw）
constexpr float kSteerRawLimit = 12000.0f;

}  //namespace

hello_world::motor::Motor *GetMotorWheel(int idx) {
  /* TODO【新人任务】:
   * 返回第 idx 号轮电机（M3508）单例。
   *
   * 你要做：
   *   1. 静态定义 4 个轮电机对象并存入数组（可用 hello_world::motor::CreateMotor
   *      (hello_world::motor::MotorType::kM3508, id, opt) 工厂，或直接构造 M3508）。
   *   2. 校验 idx ∈ [0,4)，越界时用 HW_ASSERT 或直接断言。
   *   3. 返回对应元素的地址。
   *   4. 每个电机的 CAN id、方向、减速比、角度偏移按实机接线配置（见考核说明文档）。
   *
   * 约束 / 提示：
   *   - 电机设备类已提供：inc/motor_M3508.hpp（M3508）、inc/motor.hpp（CreateMotor 工厂、
   *     OptionalParams 配置项）。
   *   - 电机按「左前 / 左后 / 右后 / 右前」命名，id 与方向见实机接线图。
   */

  HW_ASSERT(0 <= idx && idx < 4u, "Error wheel motor idx: %d", idx);

  // 4 台 M3508 驱动轮：力矩输入、正方向与规定一致、无零点偏移
  static hello_world::motor::M3508 motors[4];
  static bool is_inited = false;
  if (!is_inited) {
      for (int i = 0; i < 4u; i++) {
          hello_world::motor::OptionalParams opt;
          opt.dir = hello_world::motor::kDirFwd;
          opt.input_type = hello_world::motor::InputType::kTorq;
          opt.angle_range = hello_world::motor::AngleRange::kNegInfToPosInf;
          opt.angle_offset = 0.0f;
          motors[i].init(kWheelId[i], opt);
      }
      is_inited = true;
  }

  return &motors[idx];
}

hello_world::motor::Motor *GetMotorSteer(int idx) {
  /* TODO【新人任务】:
   * 返回第 idx 号舵电机（GM6020）单例，做法同 GetMotorWheel()。
   * 舵电机控制的是轮子的转向角（舵向轮底盘），与轮电机（驱动轮速）配合完成运动；
   * 设备类见 inc/motor_GM6020.hpp，工厂 MotorType::kGM6020。
   */
  HW_ASSERT(0 <= idx && idx < 4u, "Error steer motor idx: %d", idx);

  // 4 台 GM6020 舵向电机：电压输入、反向安装、舵角限定在 [-π, π)
  static hello_world::motor::GM6020 motors[4];
  static bool is_inited = false;
  if (!is_inited) {
      for (int i = 0; i < 4u; i++) {
          hello_world::motor::OptionalParams opt;
          opt.dir = hello_world::motor::kDirRev;
          opt.input_type = hello_world::motor::InputType::kCurr;
          opt.angle_range = hello_world::motor::AngleRange::kNegPiToPosPi;
          opt.angle_offset = kSteerAngleOffset[i];
          opt.max_raw_input_lim = kSteerRawLimit;
          motors[i].init(kSteerId[i], opt);
      }
      is_inited = true;
  }

  return &motors[idx];
}


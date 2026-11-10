/**
 *******************************************************************************
 * @file      :ins_pid.cpp
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

#include "ins_pid.hpp"
#include "assert.hpp"
#include "pid.hpp"

// PID参数
namespace {
  constexpr float kWheelKp = 0.35f;
  constexpr float kWheelKi = 0.60f;
  constexpr float kWheelKd = 0.0f;
  constexpr float kWheelOutMax = 2.5f;
  constexpr float kWheelIntegMax = 0.8f;

  constexpr float kSteerKp = 8000.0f;
  constexpr float kSteerKi = 80.0f;
  constexpr float kSteerKd = 400.0f;
  constexpr float kSteerOutMax = 9000.0f;
  constexpr float kSteerIntegMax = 2500.0f;
}  // namespace

hello_world::pid::Pid *GetPidMotorWheel(int idx) {
  /* TODO【新人任务】:
   * 返回对应的 PID 单例。
   *
   * 你要做：
   *   1. 先自己实现一个 PID 类（位置式 / 增量式，含 Kp/Ki/Kd、积分限幅、输出限幅），
   *      建议单独新建文件，类名放到 hello_world::pid 命名空间并命名为 Pid（或改这里
   *      的返回类型以匹配你的类名）。
   *   2. 静态定义各控制回路的 PID 对象并整定参数（考核会检查稳 / 快 / 无超调）。
   */
  
  HW_ASSERT(0 <= idx && idx < 4u, "Error wheel pid idx: %d", idx);

  static hello_world::pid::Pid pids[4];
  static bool is_inited = false;

  if(!is_inited){
    for(uint8_t i = 0; i < 4u; i++){
      pids[i].setParams(kWheelKp, kWheelKi, kWheelKd);
      pids[i].setLimit(kWheelOutMax, kWheelIntegMax);
    }
  }
  is_inited = true;

  return &pids[idx];
}

hello_world::pid::Pid *GetPidMotorSteer(int idx) {
  /* TODO【新人任务】:
   * 返回第 idx 号舵电机角度环 PID 单例，做法同 GetPidMotorWheel()。
   */
  HW_ASSERT(0 <= idx && idx < 4u, "Error steer pid idx: %d", idx);

  static hello_world::pid::Pid pids[4];
  static bool is_inited = false;

  if(!is_inited){
    for(uint8_t i = 0; i < 4u; i++){
      pids[i].setParams(kSteerKp, kSteerKi, kSteerKd);
      pids[i].setLimit(kSteerOutMax, kSteerIntegMax);
    }
  }
  is_inited = true;

  return &pids[idx];
}

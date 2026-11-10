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
  constexpr float kYawKp = 1.2f;              
  constexpr float kYawKi = 0.05f;             
  constexpr float kYawKd = 0.10f;              
  constexpr float kYawOutMax = 2.5f;      // 输出力矩上限 (N·m)
  constexpr float kYawIntegMax = 0.8f;        

  constexpr float kPitchKp = 2.0f;            
  constexpr float kPitchKi = 0.20f;            
  constexpr float kPitchKd = 0.15f;             
  constexpr float kPitchOutMax = 2.5f;    // 输出力矩上限 (N·m)
  constexpr float kPitchIntegMax = 1.0f;      
}  // namespace

hello_world::pid::Pid *GetPidMotorYaw() {
  /* TODO【新人任务】:
   * 返回对应的 PID 单例。
   *
   * 你要做：
   *   1. 先自己实现一个 PID 类（位置式 / 增量式，含 Kp/Ki/Kd、积分限幅、输出限幅），
   *      建议单独新建文件，类名放到 hello_world::pid 命名空间并命名为 Pid（或改这里
   *      的返回类型以匹配你的类名）。
   *   2. 静态定义各控制回路的 PID 对象并整定参数（考核会检查稳 / 快 / 无超调）。
   */

  static hello_world::pid::Pid pid;
  pid.setParams(kYawKp, kYawKi, kYawKd);
  pid.setLimit(kYawOutMax, kYawIntegMax);

  return &pid;
}

hello_world::pid::Pid *GetPidMotorPitch() {
  /* TODO【新人任务】:
   * 返回 pitch 关节位置环 PID 单例，做法同 GetPidMotorYaw()。
   */
  
  static hello_world::pid::Pid pid;
  pid.setParams(kPitchKp, kPitchKi, kPitchKd);
  pid.setLimit(kPitchOutMax, kPitchIntegMax);

  return &pid;
}

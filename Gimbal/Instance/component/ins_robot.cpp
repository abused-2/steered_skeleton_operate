/**
 *******************************************************************************
 * @file      :ins_robot.cpp
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

#include "ins_robot.hpp"
#include "gimbal.hpp"
#include "robot.hpp"

robot::Robot *GetRobot() {
  /* TODO【新人任务】:
   * 返回本板顶层状态机 robot::Robot 单例。
   *
   * 你要做：
   *   1. 在本文件静态定义一个 robot::Robot 对象（静态存储期，程序启动即构造）。
   *   2. 返回它的地址。
   *
   * 约束 / 提示：
   *   - robot::Robot 类的定义由你在 RobotModules/component/robot.hpp 补全（见该文件 TODO），
   *     本文件只负责实例化并返回地址。
   */
  
  // 静态存储期：默认构造后处于 kDead，MainTaskInit 中再注册各模块
  static robot::Robot robot;
  return &robot; 
}

robot::Gimbal *GetGimbal() {
  /* TODO【新人任务】:
   * 返回 Chassis 模块 robot::Chassis 单例，做法同 GetRobot()。
   * robot::Chassis 类定义由你在 RobotModules/component/chassis.hpp 补全。
   */
  
  static robot::Gimbal gimbal;
  return &gimbal;
}


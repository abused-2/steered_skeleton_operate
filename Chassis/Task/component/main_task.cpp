/*
 * @Author: Qian Yicheng abused-2@users.noreply.github.com
 * @Date: 2026-10-27 21:48:23
 * @LastEditors: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @LastEditTime: 2026-11-08 13:39:01
 * @FilePath: \steered_skeleton_operate\Chassis\Task\component\main_task.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
/**
 *******************************************************************************
 * @file      :main_task.cpp
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

#include "main_task.hpp"
#include "HW_can.hpp"
#include "can.h"
#include "chassis.hpp"
#include "comm_task.hpp"
#include "imu.hpp"
#include "ins_all.hpp"
#include "robot.hpp"
#include "tim.h"

namespace {
// 顶层状态机指针，MainTaskInit 中赋值，中断里只读
robot::Robot *g_robot = nullptr;
}  // namespace

void MainTaskInit(void) {
  /* TODO【新人任务】:
   * 一次性初始化（由 main() 在 while(1) 之前调用一次）：
   *   1. 通过 Instance 层的 GetXxx() 拿到电机 / IMU / 遥控器 / 双板通讯 / Robot 等对象，
   *      按依赖顺序完成初始化（电机使能、IMU 零漂采集、CAN 滤波器等）。
   *   2. 把各模块注册进 Robot（registerXxx）。
   *   3. 调用 CommTaskInit() 完成通讯初始化（UART 接收、IWDG 等）。
   * 注意：不要在这里写任何周期性逻辑。
   */

  // 对象初始化 + 注册
  robot::Chassis *chassis = GetChassis();
  for (int i = 0; i < 4; i++) {
    chassis->registerMotorWheel(i, GetMotorWheel(i));
    chassis->registerMotorSteer(i, GetMotorSteer(i));
    chassis->registerPidWheel(i, GetPidMotorWheel(i));
    chassis->registerPidSteer(i, GetPidMotorSteer(i));
  }

  hello_world::imu::Imu *imu = GetImu();
  if (imu != nullptr) { imu->init(); }

  g_robot = GetRobot();
  if (g_robot != nullptr) {
      g_robot->registerChassis(chassis);
      g_robot->registerImu(imu);
      g_robot->registerGimbalChassisComm(GetGimbalChassisComm());
      for (int i = 0; i < 4; i++) {
          g_robot->registerMotorWheel(i, GetMotorWheel(i));
          g_robot->registerMotorSteer(i, GetMotorSteer(i));
      }   // 电机在Chassis里和Robot里要分别注册一次，为了使StopAll可以直接作用于电机
  }

  // 通讯初始化
  CommTaskInit();

}

void MainTask(void) {
  /* TODO【新人任务】:
   * 1ms 控制周期入口：调用 robot->update()，由状态机完成
   * 「读数据 → 迁移 → 生成指令 → 下发」的整条链路。
   * 保持薄薄一层，不要在这里堆业务逻辑。
   */
  if (g_robot != nullptr) { g_robot->update(); }  
}

/* TODO【新人任务】:
 * 实现 htim6 周期中断回调 HAL_TIM_PeriodElapsedCallback：
 *   - 先判中断来自 htim6（避免把别的定时器中断误当成 1ms tick）；
 *   - 再调用 MainTask() 与 CommTask()。
 * 注意：
 *   - main.c 已 HAL_TIM_Base_Start_IT(&htim6)，这里只需补中断里的分发；
 *   - HAL 回调是 C 链接，需用 extern "C" 包裹，否则中断里找不到符号。
 */
extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
    if (htim->Instance == TIM6) {
        MainTask();
        CommTask();
    }
}

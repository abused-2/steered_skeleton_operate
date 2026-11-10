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
#include "comm_task.hpp"
#include "gimbal.hpp"
#include "imu.hpp"
#include "ins_all.hpp"
#include "motor_base.hpp"
#include "robot.hpp"
#include "tim.h"

void MainTaskInit(void) {
  /* TODO【新人任务】:
   * 一次性初始化（由 main() 在 while(1) 之前调用一次）：
   *   1. 通过 Instance 层的 GetXxx() 拿到电机 / IMU / 遥控器 / 双板通讯 / Robot 等对象，
   *      按依赖顺序完成初始化（电机使能、IMU 零漂采集、CAN 滤波器等）。
   *   2. 把各模块注册进 Robot（registerXxx）。
   *   3. 调用 CommTaskInit() 完成通讯初始化（UART 接收、IWDG 等）。
   * 注意：不要在这里写任何周期性逻辑。
   */

  // 取 Instance 层的单例
  robot::Robot* robot = GetRobot();
  robot::Gimbal* gimbal = GetGimbal();
  hello_world::motor::Motor* motor_yaw = GetMotorYaw();
  hello_world::motor::Motor* motor_pitch = GetMotorPitch();
  hello_world::imu::Imu* imu = GetImu();

  // imu初始化
  if (imu != nullptr) { imu->init(); }
  
  // 注册进顶层状态机
  robot->registerGimbal(gimbal);
  robot->registerImu(imu);
  robot->registerRemoteControl(GetRemoteControl());
  robot->registerGimbalChassisComm(GetGimbalChassisComm());
  robot->registerMotor(0, motor_yaw);
  robot->registerMotor(1, motor_pitch);

  // 云台模块单例获取与注册
  gimbal->registerMotor(motor_yaw, motor_pitch);
  gimbal->registerPid(GetPidMotorYaw(), GetPidMotorPitch());
  gimbal->syncTargetToActual();

  //通讯初始化
  CommTaskInit();

}

void MainTask(void) {
  /* TODO【新人任务】:
   * 1ms 控制周期入口：调用 robot->update()，由状态机完成
   * 「读数据 → 迁移 → 生成指令 → 下发」的整条链路。
   * 保持薄薄一层，不要在这里堆业务逻辑。
   */
  robot::Robot* robot = GetRobot();
  if (robot != nullptr) { robot->update(); }
}

/* TODO【新人任务】:
 * 实现 htim6 周期中断回调 HAL_TIM_PeriodElapsedCallback：
 *   - 先判中断来自 htim6（避免把别的定时器中断误当成 1ms tick）；
 *   - 再调用 MainTask() 与 CommTask()。
 * 注意：
 *   - main.c 已 HAL_TIM_Base_Start_IT(&htim6)，这里只需补中断里的分发；
 *   - HAL 回调是 C 链接，需用 extern "C" 包裹，否则中断里找不到符号。
 */
// 定时器中断回调
extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
    if (htim->Instance == TIM6) {
        MainTask();
        CommTask();
    }
}
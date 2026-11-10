/**
 *******************************************************************************
 * @file      :comm_task.cpp
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

#include "comm_task.hpp"
#include "DT7.hpp"
#include "HW_can.hpp"
#include "can.h"
#include "gimbal_chassis_comm.hpp"
#include "ins_all.hpp"
#include "iwdg.h"
#include "robot.hpp"
#include "usart.h"

namespace {
// 4 台 M3508 驱动轮共用 0x200；4 台 GM6020 舵向共用 0x1FF（电压控制）
constexpr uint32_t kWheelTxId = 0x200;
constexpr uint32_t kSteerTxId = 0x1FE;
}  // namespace

void CommTaskInit(void) {
  /* TODO【新人任务】:
   * 通讯层一次性初始化：
   *   1. 初始化 CAN 滤波器：inc/HW_can.hpp 的 CanFilter_Init(&hcan1) / CanFilter_Init(&hcan2)。
   *   2. 启动遥控器串口空闲 + DMA 接收（18 字节 DBUS，见 inc/DT7.hpp 注释）：
   *      调用 HAL_UARTEx_ReceiveToIdle_DMA 并记录接收缓冲区。
   *   3. 启动 / 刷新独立看门狗 IWDG（inc/iwdg.h）。
   */

  // CAN：滤波器 -> 启动 -> 开接收中断
  CanFilter_Init(&hcan1);
  CanFilter_Init(&hcan2);

  if (HAL_CAN_Start(&hcan1) != HAL_OK) {
      Error_Handler();
  }
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);  // M3508 + 双板帧

  if (HAL_CAN_Start(&hcan2) != HAL_OK) {
      Error_Handler();
  }
  HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING);  // GM6020
  
  // 看门狗启动在main.c
  // 这里只喂一次看门狗，防止初始化空窗内超时
  HAL_IWDG_Refresh(&hiwdg);
}

void CommTask(void) {
  /* TODO【新人任务】:
   * 每个通讯周期（1ms）调用一次，把各模块 encode 出的 8 字节 CAN 帧发出去：
   *   1. 遍历电机：调用 encode(data) 得到 8 字节 + tx_id()，再 CAN_Send_Msg 发出。
   *   2. 双板通讯对象 encode 后同样发送。
   *   3. 刷新 IWDG。
   */

  // 4 台 M3508 编码进同一帧后发一次
  uint8_t wheel_frame[8] = {0};   //驱动轮共用的 8 字节缓冲区
  for (int i = 0; i < 4; i++) {
    hello_world::motor::Motor* motor = GetMotorWheel(i);
    if (motor != nullptr) { motor->encode(wheel_frame); }
  }
  CAN_Send_Msg(&hcan2, wheel_frame, kWheelTxId, 8);

  // 4 台 GM6020 编码进同一帧后发一次
  uint8_t steer_frame[8] = {0};   //转向轮共用的 8 字节缓冲区
  for (int i = 0; i < 4; ++i) {
    hello_world::motor::Motor* motor = GetMotorSteer(i);
    if (motor != nullptr) { motor->encode(steer_frame); }
  }
  CAN_Send_Msg(&hcan1, steer_frame, kSteerTxId, 8);

  // 双板帧：C2G，走 CAN1
  robot::GimbalChassisComm* comm = GetGimbalChassisComm();
  if (comm != nullptr) {
    uint8_t comm_frame[8] = {0};
    comm->encode(comm_frame);
    CAN_Send_Msg(&hcan2, comm_frame, comm->tx_id(), 8);
  }

  // 通讯周期正常结束才喂狗
  HAL_IWDG_Refresh(&hiwdg);
}

/* TODO【新人任务】:
 * 实现 UART 空闲中断回调 HAL_UARTEx_RxEventCallback：
 *   - 收到满 18 字节后调用 DT7.decode() 解包遥控器数据，并重新启动下一轮接收；
 *   - 该回调同样是 C 链接（extern "C"）。
 * 注意：CAN 的接收分发在 src/HW_can.cpp 的两个回调里（电机 decode / 双板协议解码），
 *      不要在这里重复写 CAN 接收逻辑。
 */
// 底盘板不接收遥控器信号！
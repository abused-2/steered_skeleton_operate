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
#include "can.h"
#include "iwdg.h"
#include "usart.h"
#include "DT7.hpp"
#include "HW_can.hpp"
#include "gimbal_chassis_comm.hpp"
#include "ins_all.hpp"
#include "motor_DaMiao.hpp"
#include "robot.hpp"

extern "C" {
extern DMA_HandleTypeDef hdma_usart3_rx;
}

// ---------- 调试镜像（仅供 Ozone 观测，不参与控制逻辑）----------
volatile uint8_t  g_dbg_dm_encode_ok[2] = {0u, 0u};  // encode() 返回值
volatile uint8_t  g_dbg_dm_mode[2]      = {0u, 0u};  // current_ctrl_mode
volatile uint8_t  g_dbg_dm_status[2]    = {0u, 0u};  // status_code
volatile uint32_t g_dbg_can2_tx_cnt     = 0u;        // CAN2 发送次数
volatile uint32_t g_dbg_can1_tx_cnt     = 0u;        // CAN1 发送次数
volatile uint8_t  g_dbg_scan_id         = 2u;        // 当前扫描的电机 ID
// --------------------------------------------------------------

namespace {
// 遥控器接收缓冲：DBUS 固定 18 字节
uint8_t dbus_rx_buf[hello_world::remote_control::kRcRxDataLen] = {0};
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
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);  // 双板帧

  if (HAL_CAN_Start(&hcan2) != HAL_OK) {
      Error_Handler();
  }
  HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING);  // 达妙反馈

  // 启动遥控器接收：空闲中断 + DMA
  HAL_UARTEx_ReceiveToIdle_DMA(&huart3, dbus_rx_buf, sizeof(dbus_rx_buf));
  __HAL_DMA_DISABLE_IT(&hdma_usart3_rx, DMA_IT_HT);  // 关半传输中断

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

  uint8_t tx_data[8] = {0};

  // 1. 电机控制帧
  hello_world::motor::Motor* motors[2] = {GetMotorYaw(), GetMotorPitch()};
  for (uint8_t i = 0; i < 2u; ++i) {
    hello_world::motor::Motor* motor = motors[i];
    if (motor == nullptr) { g_dbg_dm_encode_ok[i] = 0u; continue; }
    hello_world::motor::DaMiao* dm =
    static_cast<hello_world::motor::DaMiao*>(motor);
    g_dbg_dm_mode[i]   = static_cast<uint8_t>(dm->current_ctrl_mode());
    g_dbg_dm_status[i] = static_cast<uint8_t>(dm->status_code());
    const bool enc_ok = motor->encode(tx_data);
    g_dbg_dm_encode_ok[i] = enc_ok ? 1u : 0u;
    if (!enc_ok) { continue; }
    // yaw 在 CAN1（与底盘 CAN2 同一条线），pitch 在 CAN2
    CAN_HandleTypeDef* bus = (i == 0u) ? &hcan1 : &hcan2;
    CAN_Send_Msg(bus, tx_data, dm->tx_id(), 8u);
    if (bus == &hcan2) { ++g_dbg_can2_tx_cnt; } else { ++g_dbg_can1_tx_cnt; }
  }

  // 2. 双板帧 G2C
  robot::GimbalChassisComm* comm = GetGimbalChassisComm();
  if (comm != nullptr) {
    comm->encode(tx_data);
    CAN_Send_Msg(&hcan1, tx_data, comm->tx_id(), 8u);
  }

  // 3. 所有帧都发完再喂狗：任务停摆时看门狗才能正常复位
  HAL_IWDG_Refresh(&hiwdg);

}

/* TODO【新人任务】:
 * 实现 UART 空闲中断回调 HAL_UARTEx_RxEventCallback：
 *   - 收到满 18 字节后调用 DT7.decode() 解包遥控器数据，并重新启动下一轮接收；
 *   - 该回调同样是 C 链接（extern "C"）。
 * 注意：CAN 的接收分发在 src/HW_can.cpp 的两个回调里（电机 decode / 双板协议解码），
 *      不要在这里重复写 CAN 接收逻辑。
 */
extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart, uint16_t Size) {
    if (huart != &huart3) { return; }

    // 收满 18 字节才是一帧完整的 DBUS 数据
    if (Size >= hello_world::remote_control::kRcRxDataLen) {
        hello_world::remote_control::DT7* dt7 = GetRemoteControl();
        if (dt7 != nullptr) { dt7->decode(dbus_rx_buf); }
        // 通知状态机刷新遥控器在线计时
        robot::Robot* robot = GetRobot();
        if (robot != nullptr) {
            robot->notifyRemoteControlRx();
        }
    } else {
        // 半帧说明中途丢了字节，帧边界已经错位：先中止，下面重新挂载接收来对齐
        HAL_UART_AbortReceive(&huart3);
    }

    // DMA 接收需要每帧重新挂载（正常整帧时 DMA 是循环模式，这一步相当于空操作，
    // 半帧中止后则从缓冲区头部重新开始，避免一直错位）
    HAL_UARTEx_ReceiveToIdle_DMA(&huart3, dbus_rx_buf, sizeof(dbus_rx_buf));
    __HAL_DMA_DISABLE_IT(&hdma_usart3_rx, DMA_IT_HT);
}

extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef* huart) {
    // 串口出错（溢出 / 噪声 / DMA 错误）时 HAL 已经停下了接收链路，
    // 这里重新挂载，否则遥控器会永久离线（离线本身是安全的，但机器人动不了）
    if (huart == &huart3) {
        HAL_UARTEx_ReceiveToIdle_DMA(&huart3, dbus_rx_buf, sizeof(dbus_rx_buf));
        __HAL_DMA_DISABLE_IT(&hdma_usart3_rx, DMA_IT_HT);
    }
}


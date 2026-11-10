/**
 *******************************************************************************
 * @file      : HW_can.cpp
 * @brief     : CAN 收发基础封装实现
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-09-18      Hello World     1. 考核骨架：CanFilter_Init + CAN_Send_Msg
 *                                                全量提供，接收分发回调掏空
 *******************************************************************************
 * @attention :
 *  接收中断回调 HAL_CAN_RxFifo0/1MsgPendingCallback 已在此预留，仅完成「取 8 字节 +
 *  再次使能接收中断」，具体「按帧 ID 分发」逻辑留给新人：
 *    - M3508 / DM 电机反馈：调用对应电机的 decode(can?_rx_data)
 *    - 双板通讯帧：调用新人自写的协议解码
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */

#include "HW_can.hpp"
#include "can.h"
#include "gimbal_chassis_comm.hpp"
#include "ins_chassis_gimbal_comm.hpp"
#include "ins_motor.hpp"

static CAN_RxHeaderTypeDef rx_header1;
static CAN_RxHeaderTypeDef rx_header2;
static uint8_t can1_rx_data[8];
static uint8_t can2_rx_data[8];
static uint32_t p_tx_mailbox;

void CanFilter_Init(CAN_HandleTypeDef *hcan)
{
  CAN_FilterTypeDef can_filter;

  /* 全收：掩码全 0，任何 ID 均通过 */
  can_filter.FilterMode = CAN_FILTERMODE_IDMASK;
  can_filter.FilterScale = CAN_FILTERSCALE_32BIT;
  can_filter.FilterIdHigh = 0x0000;
  can_filter.FilterIdLow = 0x0000;
  can_filter.FilterMaskIdHigh = 0x0000;
  can_filter.FilterMaskIdLow = 0x0000;
  can_filter.FilterActivation = ENABLE;

  if (hcan == &hcan1) {
    can_filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    can_filter.FilterBank = 0;
  } else if (hcan == &hcan2) {
    can_filter.FilterFIFOAssignment = CAN_FILTER_FIFO1;
    can_filter.FilterBank = 14;
  } else {
    return;
  }

  if (HAL_CAN_ConfigFilter(hcan, &can_filter) != HAL_OK) {
    Error_Handler();
  }
}

void CAN_Send_Msg(CAN_HandleTypeDef *hcan, uint8_t *msg, uint32_t id,
                  uint8_t len)
{
  CAN_TxHeaderTypeDef tx_header = {0};
  tx_header.StdId = id;
  tx_header.IDE = CAN_ID_STD;
  tx_header.RTR = CAN_RTR_DATA;
  tx_header.DLC = len;
  HAL_CAN_AddTxMessage(hcan, &tx_header, msg, &p_tx_mailbox);
}

// ---------- 临时排查用镜像（验证完删除）----------
volatile uint32_t g_dbg_pitch_fallback_rx = 0u;  // 未识别帧数量
volatile uint32_t g_dbg_pitch_fallback_id = 0u;  // 最近一个未识别帧的 StdId
volatile uint32_t g_dbg_rx_cnt[2] = {0u, 0u};    // 两条总线各自的收帧数 [0]=CAN1 [1]=CAN2
volatile uint32_t g_dbg_unk_cnt[2] = {0u, 0u};   // 各自的未识别帧数
volatile uint32_t g_dbg_unk_id[2] = {0u, 0u};    // 各自最近一个未识别帧的 StdId
// 未知 ID 清单：记录除 DJI 电机帧(0x200~0x204)外的所有不同 ID
volatile uint32_t g_dbg_unk_list[8] = {0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u};
volatile uint32_t g_dbg_unk_list_cnt = 0u;
// ------------------------------------------------

namespace {
// 按 StdId 分发一帧（bus: 0=CAN1, 1=CAN2）
void CanDispatch(uint32_t std_id, const uint8_t* data, uint8_t bus)
{
    ++g_dbg_rx_cnt[bus & 1u];

    // 0->yaw  1->pitch
    for (uint8_t i = 0; i < 2u; i++) {
        hello_world::motor::Motor* motor =
            (i == 0u) ? GetMotorYaw() : GetMotorPitch();
        if (motor != nullptr &&
            hello_world::motor::DaMiao::GetRxId(motor->id()) == std_id) {
            motor->decode(data);
            return;
        }
    }

    robot::GimbalChassisComm* comm = GetGimbalChassisComm();
    if (comm != nullptr && comm->rx_id() == std_id) {
        comm->decode(data);
        return;
    }

    // 兜底统计：只记录，不解码（避免用底盘 0x200 轮控帧污染 pitch 状态）
    ++g_dbg_pitch_fallback_rx;
    ++g_dbg_unk_cnt[bus & 1u];
    // 0x200 是底盘下发的轮控帧，属于已知无关帧，记录时跳过它，
    // 以便把其它未知 ID（极可能是 pitch 的真实反馈 ID）暴露出来
    if (std_id != 0x200u) {
        g_dbg_pitch_fallback_id = std_id;
        g_dbg_unk_id[bus & 1u] = std_id;
    }
    // 收集除 DJI 电机帧外的所有不同 ID
    if (std_id < 0x200u || std_id > 0x204u) {
        bool found = false;
        for (uint8_t k = 0; k < g_dbg_unk_list_cnt && k < 8u; ++k) {
            if (g_dbg_unk_list[k] == std_id) { found = true; break; }
        }
        if (!found && g_dbg_unk_list_cnt < 8u) {
            g_dbg_unk_list[g_dbg_unk_list_cnt] = std_id;
            ++g_dbg_unk_list_cnt;
        }
    }
}
}  // namespace

// 接收中断回调(取帧头/取8字节数据/按帧ID分发/重新使能)
/**
 * @brief   CAN1 FIFO0 接收中断回调（新人填写分发逻辑）
 * @param   hcan: CAN 句柄
 * @retval  None
 * @note    此处只取了 8 字节并重新使能接收中断，具体分发由新人实现
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  if (hcan == &hcan1) {
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header1,
                             can1_rx_data) == HAL_OK) {
      CanDispatch(rx_header1.StdId, can1_rx_data, 0u);                      
    }
  }
  HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
}

/**
 * @brief   CAN2 FIFO1 接收中断回调（新人填写分发逻辑）
 * @param   hcan: CAN 句柄
 * @retval  None
 * @note    同上，具体分发由新人实现
 */
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  if (hcan == &hcan2) {
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &rx_header2,
                             can2_rx_data) == HAL_OK) {
      CanDispatch(rx_header2.StdId, can2_rx_data, 1u);
    }
  }
  HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO1_MSG_PENDING);
}

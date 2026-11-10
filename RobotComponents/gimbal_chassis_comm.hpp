/**
 *******************************************************************************
 * @file      :gimbal_chassis_comm.hpp
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

#ifndef GIMBAL_CHASSIS_COMM_HPP_
#define GIMBAL_CHASSIS_COMM_HPP_

#include <cstdint>
#include "tick.hpp"
#include "module_state.hpp"
#include "DT7.hpp"

namespace robot {

class GimbalChassisComm {
 public:

  //传输方向枚举  0:G2C  1:C2G  (本板是 Chassis 还是 Gimbal)
  enum class CodePart : uint8_t { Chassis = 0, Gimbal = 1 };
  
  //C2G数据
  struct ChassisData {

    bool ready = false;                         // 底盘自检完成
    LifeState life_state = LifeState::kDead;    // 底盘生命状态
    ChassisMode mode = ChassisMode::kSeperate;  // 当前底盘工作模式
    float yaw = 0.0f;                           // 底盘 IMU 偏航角，单位 rad
    float w = 0.0f;                             // 底盘偏航角速度，单位 rad/s
  };
  
  //G2C数据
  struct GimbalData {

    bool ready = false;                         // 云台自检完成
    LifeState life_state = LifeState::kDead;    // 云台生命状态
    float delta_joint = 0.0f;                   // 云台相对底盘偏航角 δ，单位 rad
    float pitch_joint = 0.0f;                   // 云台pitch角，单位 rad
    float rc_lv = 0.0f;                         // 左摇杆竖直
    float rc_lh = 0.0f;                         // 左摇杆水平
    float rc_wheel = 0.0f;                      // 拨轮
    bool rc_valid = false;                      // 遥控数据是否有效
    hello_world::remote_control::SwitchState rc_l_switch =
        hello_world::remote_control::kSwitchStateUp;  // 左拨杆
    hello_world::remote_control::SwitchState rc_r_switch =
        hello_world::remote_control::kSwitchStateUp;  // 右拨杆
  };

  GimbalChassisComm(CodePart code_part, uint32_t tx_id, uint32_t rx_id);

  void encode(uint8_t tx_data[8]);
  void decode(const uint8_t rx_data[8]);

  bool isOffline() const;

  uint32_t tx_id() const { return tx_id_; }
  uint32_t rx_id() const { return rx_id_; }

  //数据访问（供 RobotModules / Task 读写）  双板协议对外的唯一接口
  ChassisData &chassis_data() { return chassis_data_; }
  GimbalData &gimbal_data() { return gimbal_data_; }
  const ChassisData &chassis_data() const { return chassis_data_; }
  const GimbalData &gimbal_data() const { return gimbal_data_; }

 private:
  CodePart code_part_;
  uint32_t tx_id_;
  uint32_t rx_id_;
  uint32_t last_rx_tick_;  // 上次收到对方帧的 GetTickMs() 时间戳
  ChassisData chassis_data_;
  GimbalData gimbal_data_;
};

}  // namespace robot

#endif /* GIMBAL_CHASSIS_COMM_HPP_ */

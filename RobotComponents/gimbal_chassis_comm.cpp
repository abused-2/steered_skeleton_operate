/**
 *******************************************************************************
 * @file      :gimbal_chassis_comm.cpp
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

#include "gimbal_chassis_comm.hpp"

namespace robot {

namespace {

constexpr uint32_t kOfflineThresMs = 100;  // 离线阈值，单位 ms

constexpr float kStickScale = 127.0f;    // 摇杆
constexpr float kPitchScale = 200.0f;    // pitch（覆盖软件限位[-0.35, +0.55]范围）
constexpr float kAngleScale = 1000.0f;   // delta / 底盘 yaw / 底盘 w

int8_t FloatToInt8(float value, float scale) {
  float scaled = value * scale;
  if (scaled > 127.0f) {  scaled = 127.0f;  } 
  else if (scaled < -128.0f) {  scaled = -128.0f; }
  return static_cast<int8_t>(scaled);
}

float Int8ToFloat(int8_t raw, float scale) { 
  return static_cast<float>(raw) / scale; 
}

int16_t FloatToInt16Be(float value, float scale) {
  float scaled = value * scale;
  if (scaled > 32767.0f) {  scaled = 32767.0f; } 
  else if (scaled < -32768.0f) {  scaled = -32768.0f; }
  return static_cast<int16_t>(scaled);
}

// 大端写入与读取
void SetBigEndian16(uint8_t* dst, int16_t value) {
  const uint16_t raw = static_cast<uint16_t>(value);
  dst[0] = static_cast<uint8_t>(raw >> 8);
  dst[1] = static_cast<uint8_t>(raw & 0xFFu);
}
int16_t GetBigEndian16(const uint8_t* src) {
  return static_cast<int16_t>((static_cast<uint16_t>(src[0]) << 8) |
                              static_cast<uint16_t>(src[1]));
}

// 拨杆档位压进 2 bit（1 = Up, 2 = Down, 3 = Mid，0 = 非法）
uint8_t SwitchToBits(hello_world::remote_control::SwitchState sw) {
  const uint8_t raw = static_cast<uint8_t>(sw);
  return (raw >= 1u && raw <= 3u) ? raw : 0u;
}
hello_world::remote_control::SwitchState BitsToSwitch(uint8_t bits) {
  switch (bits) {
    case 1u:
      return hello_world::remote_control::kSwitchStateUp;
    case 2u:
      return hello_world::remote_control::kSwitchStateDown;
    case 3u:
      return hello_world::remote_control::kSwitchStateMid;
    default:   // 非法值保守处理
      return hello_world::remote_control::kSwitchStateDown;
  }
}

}  // namespace


// 构造函数
GimbalChassisComm::GimbalChassisComm(GimbalChassisComm::CodePart code_part,
                                     uint32_t tx_id, uint32_t rx_id) {
  code_part_ = code_part;
  tx_id_ = tx_id;
  rx_id_ = rx_id;
  last_rx_tick_ = hello_world::tick::GetTickMs() - kOfflineThresMs - 1u;
}

void GimbalChassisComm::encode(uint8_t tx_data[8]) {
  // 初始化
  for (uint8_t i = 0; i < 8u; i++) { tx_data[i] = 0u; }

  if (code_part_ == CodePart::Chassis) {          // C2G
    tx_data[0] = chassis_data_.ready ? 0x01u : 0x00u;
    tx_data[1] = static_cast<uint8_t>(chassis_data_.life_state);
    tx_data[2] = static_cast<uint8_t>(chassis_data_.mode);
    tx_data[3] = 0u;  // 保留
    SetBigEndian16(&tx_data[4], FloatToInt16Be(chassis_data_.yaw, kAngleScale));
    SetBigEndian16(&tx_data[6], FloatToInt16Be(chassis_data_.w, kAngleScale));
  } else {                                        // G2C
    const uint8_t flags = static_cast<uint8_t>(
        (gimbal_data_.ready ? 0x01u : 0x00u) |
        static_cast<uint8_t>(SwitchToBits(gimbal_data_.rc_r_switch) << 1) |
        static_cast<uint8_t>(SwitchToBits(gimbal_data_.rc_l_switch) << 3) |
        (gimbal_data_.rc_valid ? 0x20u : 0x00u));
    tx_data[0] = flags;
    tx_data[1] = static_cast<uint8_t>(gimbal_data_.life_state);
    tx_data[2] = static_cast<uint8_t>(FloatToInt8(gimbal_data_.rc_lv, kStickScale));
    tx_data[3] = static_cast<uint8_t>(FloatToInt8(gimbal_data_.rc_lh, kStickScale));
    tx_data[4] = static_cast<uint8_t>(FloatToInt8(gimbal_data_.rc_wheel, kStickScale));
    tx_data[5] = static_cast<uint8_t>(FloatToInt8(gimbal_data_.pitch_joint, kPitchScale));
    SetBigEndian16(&tx_data[6], FloatToInt16Be(gimbal_data_.delta_joint, kAngleScale));
  }
}

void GimbalChassisComm::decode(const uint8_t rx_data[8]) {
  const uint8_t flags = rx_data[0];
  const bool ready = (flags & 0x01u) != 0u;
  const uint8_t state_raw = rx_data[1];

  const LifeState life_state =
      IsValidLifeState(state_raw) ? static_cast<LifeState>(state_raw) : LifeState::kDead;

  if (code_part_ == CodePart::Chassis) {        // G2C
    GimbalData &data = gimbal_data_;
    data.ready = ready;
    data.life_state = life_state;
    data.rc_lv = Int8ToFloat(static_cast<int8_t>(rx_data[2]), kStickScale);
    data.rc_lh = Int8ToFloat(static_cast<int8_t>(rx_data[3]), kStickScale);
    data.rc_wheel = Int8ToFloat(static_cast<int8_t>(rx_data[4]), kStickScale);
    data.pitch_joint = Int8ToFloat(static_cast<int8_t>(rx_data[5]), kPitchScale);
    data.delta_joint = static_cast<float>(GetBigEndian16(&rx_data[6])) / kAngleScale;
    data.rc_r_switch = BitsToSwitch(static_cast<uint8_t>((flags >> 1) & 0x03u));
    data.rc_l_switch = BitsToSwitch(static_cast<uint8_t>((flags >> 3) & 0x03u));
    data.rc_valid = (flags & 0x20u) != 0u;
  } else {                                     // C2G
    ChassisData &data = chassis_data_;
    const uint8_t mode_raw = rx_data[2];
    data.ready = ready;
    data.life_state = life_state;
    data.mode = IsValidChassisMode(mode_raw) ? static_cast<ChassisMode>(mode_raw) : ChassisMode::kSeperate;
    data.yaw = static_cast<float>(GetBigEndian16(&rx_data[4])) / kAngleScale;
    data.w = static_cast<float>(GetBigEndian16(&rx_data[6])) / kAngleScale;
  }

  last_rx_tick_ = hello_world::tick::GetTickMs();
}

bool GimbalChassisComm::isOffline() const {
  return (hello_world::tick::GetTickMs() - last_rx_tick_) > kOfflineThresMs;
}

}  // namespace robot

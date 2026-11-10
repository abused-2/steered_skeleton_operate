/**
 *******************************************************************************
 * @file      :module_state.hpp
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

#ifndef MODULE_STATE_HPP_
#define MODULE_STATE_HPP_

#include<cstdint>

/* ============================================================================
 * 【新人任务】定义底盘板与云台板共享的状态类型与枚举（namespace robot）。
 *
 * 本文件会被底盘板、云台板同时编译，只能放「纯类型 / 枚举 / 内联函数」，
 * 严禁依赖任何具体硬件（电机 / CAN / HAL），否则另一块板编不过。
 *
 * 你要做：
 *   1. 生命状态枚举：至少区分「上电未就绪 / 恢复自检中 / 正常工作中」三种，
 *      供 RobotModules 的状态机在 Dead / Resurrection / Working 三态间迁移使用。
 *   2. 手动控制源枚举：遥控器 / 键鼠（如有），说明当前控制指令来源。
 *   3. 底盘工作模式枚举：分离 / 随动 / 小陀螺；云台工作模式枚举（关节空间控 yaw、pitch）。
 *   4. 底盘运动状态结构（平移速度 vx、vy + 旋转角速度 w，单位自定并全工程一致）与
 *      云台运动状态结构（yaw、pitch），并给出加减、数乘等常用运算。
 *
 * 约束 / 提示：
 *   - 这些类型会被 RobotModules 状态机、RobotComponents 双板通讯帧、Instance 接线
 *     三处共同使用，字段命名 / 枚举值必须三处一致——动手前先画一张字段对照表。
 *   - 只需覆盖本骨架用到的状态（裁判系统 / 摩擦轮 / 超电相关状态本骨架用不到，可省略）
 * ==========================================================================*/

namespace robot{

enum class LifeState : uint8_t {
  kDead = 0,         //上电未就绪
  kResurrection = 1, //恢复自检中
  kWorking = 2       //正常工作中
};

enum class ControlSourse : uint8_t {
  kRemoteControl = 0,
  kKeyboard = 1
};

enum class ChassisMode : uint8_t {
  kSeperate = 0,  //分离
  kFollow = 1,    //跟随
  kSpin = 2       //小陀螺
};

enum class GimbalMode : uint8_t {
  kJointSpace = 0,  //未知
  kUnknown = 1      //未知
};

struct ChassisMotion {

    float vx = 0.0f;
    float vy = 0.0f;
    float w = 0.0f;

    // 构造函数
    ChassisMotion() = default;
    ChassisMotion(float vx_in, float vy_in, float w_in) : vx(vx_in), vy(vy_in), w(w_in) {}
    
    // 运算符重载
    ChassisMotion operator+(const ChassisMotion& rhs) const{
      return ChassisMotion(vx+rhs.vx, vy+rhs.vy, w+rhs.w);
    }

    ChassisMotion operator-(const ChassisMotion& rhs) const{
      return ChassisMotion(vx-rhs.vx, vy-rhs.vy, w-rhs.w);
    }

    ChassisMotion operator-() const{
      return ChassisMotion(-vx, -vy, -w);
    }

    ChassisMotion operator*(float k) const{
      return ChassisMotion(vx*k, vy*k, w*k);
    }

    ChassisMotion operator/(float k) const{
      if( k != 0 )   return ChassisMotion(vx/k, vy/k, w/k);
    }

    ChassisMotion& operator+=(const ChassisMotion& rhs)
    {
        vx += rhs.vx;  vy += rhs.vy;  w += rhs.w;
        return *this;
    }

    ChassisMotion& operator-=(const ChassisMotion& rhs)
    {
        vx -= rhs.vx;  vy -= rhs.vy;  w -= rhs.w;
        return *this;
    }

    ChassisMotion& operator*=(float k)
    {
        vx *= k;  vy *= k;  w *= k;
        return *this;
    }
    
    ChassisMotion& operator/=(float k)
    {
        if( k != 0 ){
        vx /= k;  vy /= k;  w /= k;
        return *this;}
    }

    // 比较
    bool operator==(const ChassisMotion& rhs) const {
      return vx == rhs.vx && vy == rhs.vy && w == rhs.w;
    }

    bool operator!=(const ChassisMotion& rhs) const { 
      return !(*this == rhs); 
    }

    bool isNear(const ChassisMotion& rhs, float eps = 1e-4f) const {
        return (vx - rhs.vx < eps && rhs.vx - vx < eps) &&
               (vy - rhs.vy < eps && rhs.vy - vy < eps) &&
               (w - rhs.w < eps && rhs.w - w < eps);
    }

    bool isZer0(float eps = 1e-4f) const {
      return isNear(ChassisMotion(), eps);
    }

    void reset() { vx = vy = w = 0.0f; }
};

struct GimbalMotion {

    float yaw = 0.0f;
    float pitch = 0.0f;

    // 构造函数
    GimbalMotion() = default;
    GimbalMotion(float yaw_in, float pitch_in) : yaw(yaw_in), pitch(pitch_in) {}

    // 运算符重构
    GimbalMotion operator+(const GimbalMotion& rhs) const {
        return GimbalMotion(yaw + rhs.yaw, pitch + rhs.pitch);
    }

    GimbalMotion operator-(const GimbalMotion& rhs) const {
        return GimbalMotion(yaw - rhs.yaw, pitch - rhs.pitch);
    }

    GimbalMotion operator-() const { 
      return GimbalMotion(-yaw, -pitch); 
    }

    GimbalMotion operator*(float k) const {
        return GimbalMotion(yaw * k, pitch * k);
    }

    GimbalMotion operator/(float k) const
    {
        if(k != 0)
        return GimbalMotion(yaw / k, pitch / k);
    }

    GimbalMotion& operator+=(const GimbalMotion& rhs) {
        yaw += rhs.yaw;
        pitch += rhs.pitch;
        return *this;
    }

    GimbalMotion& operator-=(const GimbalMotion& rhs) {
        yaw -= rhs.yaw;
        pitch -= rhs.pitch;
        return *this;
    }

    GimbalMotion& operator*=(float k) {
        yaw *= k;
        pitch *= k;
        return *this;
    }

    GimbalMotion& operator/=(float k) {
        if( k != 0 ){
        yaw /= k; pitch /= k;
        return *this;}
    }
    
    // 比较
    bool operator==(const GimbalMotion& rhs) const {
        return yaw == rhs.yaw && pitch == rhs.pitch;
    }

    bool operator!=(const GimbalMotion& rhs) const { 
      return !(*this == rhs); 
    }
    
    bool isNear(const GimbalMotion& rhs, float eps = 1e-4f) const {
        return (yaw - rhs.yaw < eps && rhs.yaw - yaw < eps) &&
               (pitch - rhs.pitch < eps && rhs.pitch - pitch < eps);
    }

    bool isZero(float eps = 1e-4f) const { 
      return isNear(GimbalMotion(), eps); 
    }

    void reset() { yaw = pitch = 0.0f; }
};

// 合法性检查
inline bool IsValidLifeState(uint8_t v) { return v<=2u; }
inline bool IsValidControlSource(uint8_t v) { return v <= 2u; }
inline bool IsValidChassisMode(uint8_t v) { return v <= 2u; }
inline bool IsValidGimbalMode(uint8_t v) { return v <= 1u; }

} //namespace
#endif /* MODULE_STATE_HPP_ */

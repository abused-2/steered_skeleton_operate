/*
 * @Author: Qian Yicheng abused-2@users.noreply.github.com
 * @Date: 2026-11-01 01:08:14
 * @LastEditors: Qian Yicheng abused-2@users.noreply.github.com
 * @LastEditTime: 2026-11-01 17:56:34
 * @FilePath: \steered_skeleton_operate\Chassis\src\imu.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "imu.hpp"
#include "iwdg.h"
#include "spi.h"

namespace hello_world {
namespace imu {

// 参数
namespace {

constexpr uint32_t kZeroDriftSamples = 500;  // 零漂采样次数（每 1 ms 一次，共 0.5 s）
constexpr float kSampFreq = 1000.0f;         // 姿态解算采样频率，与调用 update 的频率一致
constexpr float kMahonyKp = 0.5f;
constexpr float kMahonyKi = 0.01f;

// 旋转矩阵（默认同向）
constexpr float kRotMat[9] = {1.0f, 0.0f, 0.0f,
                              0.0f, 1.0f, 0.0f,
                              0.0f, 0.0f, 1.0f};

}  // namespace

bool Imu::init()
{
    // 硬件配置：SPI1/加速度计片选 PA4/陀螺仪片选 PB0（C 型板）
    BMI088HWConfig hw{};
    hw.hspi = &hspi1;
    hw.acc_cs_port = GPIOA;
    hw.acc_cs_pin = GPIO_PIN_4;
    hw.gyro_cs_port = GPIOB;
    hw.gyro_cs_pin = GPIO_PIN_0;

    bmi_.init(hw, kRotMat, BMI088Config{});
    inited_ = true;

    // 芯片自检与配置（内部有 ms 级阻塞，不能放在中断里）
    if (bmi_.imuInit(false) != kBMI088ErrStateNoErr) {
        ready_ = false;
        return false;
    }

    // 零漂采样（边采边喂狗，防止500ms超时）
    float sum[3] = {0.0f, 0.0f, 0.0f};
    float acc_tmp[3] = {0.0f, 0.0f, 0.0f};
    float gyro_tmp[3] = {0.0f, 0.0f, 0.0f};

    for (uint32_t i = 0; i < kZeroDriftSamples; i++) {
        bmi_.getData(acc_tmp, gyro_tmp, nullptr);
        for (uint8_t k = 0; k < 3u; k++) {
            sum[k] += gyro_tmp[k];
        }
        HAL_IWDG_Refresh(&hiwdg);
        HAL_Delay(1);
    }

    for (uint8_t k = 0; k < 3u; k++) {
        gyro_bias_[k] = sum[k] / static_cast<float>(kZeroDriftSamples);
    }

    // 姿态解算初始化，并用重力加速度把 roll/pitch 收敛到正确值
    mahony_.init(kSampFreq, kMahonyKp, kMahonyKi);
    for (uint32_t i = 0; i < kZeroDriftSamples; i++) {
        bmi_.getData(acc_, gyro_, nullptr);
        for (uint8_t k = 0; k < 3u; k++) {
            gyro_[k] -= gyro_bias_[k];
        }
        mahony_.update(acc_, gyro_);
        HAL_IWDG_Refresh(&hiwdg);
        HAL_Delay(1);
    }
    mahony_.getEulerAngle(euler_);

    ready_ = true;
    return true;
}

void Imu::update(){
    if (!inited_) {
        return;
    }

    bmi_.getData(acc_, gyro_, nullptr);
    for (uint8_t k = 0; k < 3u; k++) {
        gyro_[k] -= gyro_bias_[k];
    }

    mahony_.update(acc_, gyro_);
    mahony_.getEulerAngle(euler_);
}

}}  // namespace

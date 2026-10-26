#pragma once

#include "bmi088.hpp"
#include "vqf.hpp"

namespace Imu_N
{
inline constexpr uint32_t sample_freq_hz = 800;
inline constexpr float sample_dt = 0.00125f;
inline constexpr uint32_t drdy_timeout_ms = 5;
inline constexpr uint32_t dt_min_us = 400;
inline constexpr uint32_t dt_max_us = 5000;
inline constexpr float gyro_avg_tau = 5.0f;
inline constexpr float pitch_sign = -1.0f;
}

inline Bmi088* imu_bmi088 = nullptr;
inline Vqf* imu_vqf = nullptr;
inline int32_t imu_init_ans = 0;
inline uint8_t imu_acc_id = 0;
inline uint8_t imu_gyro_id = 0;

extern volatile float imu_euler[3];

inline void Imu_Attach(Bmi088& bmi088_, Vqf& vqf_)
{
	imu_bmi088 = &bmi088_;
	imu_vqf = &vqf_;
}

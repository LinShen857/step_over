#include "threads.hpp"
#include "modules/imu.hpp"
#include "main.h"

volatile float imu_gyro[3] = {0.f, 0.f, 0.f};
volatile float imu_accel[3] = {0.f, 0.f, 0.f};
volatile float imu_quat[4] = {1.f, 0.f, 0.f, 0.f};
volatile float imu_euler[3] = {0.f, 0.f, 0.f};
volatile float imu_bias[3] = {0.f, 0.f, 0.f};
volatile float imu_gyro_avg[3] = {0.f, 0.f, 0.f};
volatile float imu_temp = 0.f;
volatile float imu_rest_dev_gyr = 0.f;
volatile float imu_rest_dev_acc = 0.f;
volatile uint32_t imu_dt_us = 0;
volatile uint32_t imu_rest_count = 0;
volatile uint8_t imu_rest_detected = 0;
volatile uint32_t imu_timeout_count = 0;
volatile uint32_t imu_sample_count = 0;

namespace
{
inline uint32_t Micros()
{
	return HAL_GetTick() * 1000u + TIM6->CNT;
}
}

void TaskImu(void *argument)
{
	(void)argument;

	uint32_t last_us = Micros();

	for (;;)
	{
		Bmi088* bmi088 = imu_bmi088;
		Vqf* vqf = imu_vqf;
		if (bmi088 == nullptr || vqf == nullptr)
		{
			osDelay(1);
			last_us = Micros();
			continue;
		}

		if (bmi088->WaitDataReady(Imu_N::drdy_timeout_ms) != LibXR::ErrorCode::OK)
		{
			imu_timeout_count = imu_timeout_count + 1;
		}

		const uint32_t now_us = Micros();
		uint32_t dt_us = now_us - last_us;
		last_us = now_us;
		if (dt_us < Imu_N::dt_min_us) dt_us = Imu_N::dt_min_us;
		if (dt_us > Imu_N::dt_max_us) dt_us = Imu_N::dt_max_us;
		imu_dt_us = dt_us;

		bmi088->Read();
		vqf->Update(bmi088->data.gyro, bmi088->data.accel, static_cast<float>(dt_us) * 1e-6f);

		float q[4] {};
		float rpy[3] {};
		float bias[3] {};
		vqf->GetQuat(q);
		vqf->GetEuler(rpy);
		vqf->GetBias(bias);

		for (uint8_t i = 0; i < 3; ++i)
		{
			imu_gyro[i] = bmi088->data.gyro[i];
			imu_accel[i] = bmi088->data.accel[i];
			imu_euler[i] = rpy[i];
			imu_bias[i] = bias[i];
			imu_gyro_avg[i] += (bmi088->data.gyro[i] - bias[i] - imu_gyro_avg[i]) *
				(static_cast<float>(dt_us) * 1e-6f / Imu_N::gyro_avg_tau);
		}
		for (uint8_t i = 0; i < 4; ++i)
		{
			imu_quat[i] = q[i];
		}
		const float dev_gyr = vqf->RestDevGyr();
		const float dev_acc = vqf->RestDevAcc();
		imu_rest_dev_gyr = dev_gyr > imu_rest_dev_gyr ? dev_gyr : imu_rest_dev_gyr * 0.999f;
		imu_rest_dev_acc = dev_acc > imu_rest_dev_acc ? dev_acc : imu_rest_dev_acc * 0.999f;
		imu_temp = bmi088->data.temperature;
		if (vqf->RestDetected())
		{
			imu_rest_detected = 1u;
			imu_rest_count = imu_rest_count + 1;
		}
		else
		{
			imu_rest_detected = 0u;
		}
		imu_sample_count = imu_sample_count + 1;
	}
}

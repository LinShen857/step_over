#include "threads.hpp"
#include "dbus_callback.hpp"
#include "math/balance.hpp"
#include "modules/chassis.hpp"
#include "modules/imu.hpp"
#include "motor_gm.hpp"
#include "motor_dm.hpp"


namespace
{

inline constexpr int16_t rc_deadzone = 20;
inline constexpr float rc_range = 660.f;

bool balance_active = false;
float balance_pitch_target = 0.f;

float NormStick(const int16_t value_)
{
	if (value_ > -rc_deadzone && value_ < rc_deadzone) return 0.f;

	const float norm = static_cast<float>(value_) / rc_range;
	return norm > 1.f ? 1.f : (norm < -1.f ? -1.f : norm);
}
}


volatile uint32_t chassis_overrun_count = 0;

volatile float test_speed = 0.5f;

volatile float fb_speed_lf = 0.f;
volatile float fb_speed_rf = 0.f;
volatile float fb_speed_rb = 0.f;
volatile float fb_speed_lb = 0.f;

/* 四个 GM3508 的 0x200 帧按 1 kHz 周期下发。
   每拍：读遥控 -> 写目标速度 -> 刷轮速/正解/逆解(含限幅) -> 速度环 -> 下发一帧 0x200。
   Chassis_Attach() 由 app_main() 调用，而 chassisTask 可能比它先就绪，
   未绑定期间直接跳过本周期。
   绝对时间调度（osDelayUntil）避免相对延时累计漂移。 */
void TaskChassis(void *argument)
{
	(void)argument;

	const uint32_t cycle_ticks = osKernelGetTickFreq() / Chassis_N::ctrl_freq_hz;
	uint32_t cycle_start = osKernelGetTickCount();

	for (;;)
	{
		Chassis* chassis = Chassis_Get();
		if (chassis != nullptr)
		{
			static Balance balance(chassis->strut_lf.x, chassis->strut_lf.z,
				Leg_N::orbit_radius, Leg_N::orbit_center_z, chassis->leg_lb.stretch_angle,
				chassis->leg_lb.shrink_angle);

			const USER_DBUS::rc_ctrl_s rc = USER_DBUS::Snapshot();
			chassis->SetCmdNorm(NormStick(rc.rc.ch3), NormStick(rc.rc.ch2),
				NormStick(rc.rc.ch0), NormStick(rc.rc.ch1));

			const float pitch = Imu_N::pitch_sign * imu_euler[1];

			if (rc.rc.s2 == USER_DBUS::sw_mid)
			{
				if (!balance_active)
				{
					balance_active = true;
					balance_pitch_target = pitch-0.08f;
				}
				chassis->leg_lb.target_angle =
					balance.QBalance(chassis->leg_lb.angle, pitch, balance_pitch_target);
				chassis->leg_rb.target_angle =
					balance.QBalance(chassis->leg_rb.angle, pitch, balance_pitch_target);
			}
			else
			{
				balance_active = false;

				if (rc.rc.s1 == USER_DBUS::sw_mid)
				{
					chassis->leg_lb.target_angle = chassis->leg_lb.shrink_angle;
					chassis->leg_rb.target_angle = chassis->leg_rb.shrink_angle;
				}
				else if (rc.rc.s1 == USER_DBUS::sw_up)
				{
					chassis->leg_lb.target_angle = chassis->leg_lb.stretch_angle;
					chassis->leg_rb.target_angle = chassis->leg_rb.stretch_angle;
				}
			}

			chassis->Update(pitch);
			/* chassis->wheel_lf.UpdateFbSpeed();
			chassis->wheel_rf.UpdateFbSpeed();
			chassis->wheel_rb.UpdateFbSpeed();
			chassis->wheel_lb.UpdateFbSpeed();
			fb_speed_lf = chassis->wheel_lf.speed;
			fb_speed_rf = chassis->wheel_rf.speed;
			fb_speed_rb = chassis->wheel_rb.speed;
			fb_speed_lb = chassis->wheel_lb.speed;
			
			chassis->SolveInverse();

			chassis->wheel_lf.pid.Update(chassis->wheel_lf.target_speed, chassis->wheel_lf.speed);
			chassis->wheel_rf.pid.Update(chassis->wheel_rf.target_speed, chassis->wheel_rf.speed);
			chassis->wheel_rb.pid.Update(chassis->wheel_rb.target_speed, chassis->wheel_rb.speed);
			chassis->wheel_lb.pid.Update(chassis->wheel_lb.target_speed, chassis->wheel_lb.speed);
			//chassis->Transmit();  */

		}
		cycle_start += cycle_ticks;
		const int32_t remaining = static_cast<int32_t>(cycle_start - osKernelGetTickCount());
		if (remaining > 0)
		{
			osDelayUntil(cycle_start);
		}
		else
		{
			chassis_overrun_count = chassis_overrun_count + 1;  // C++20 起 ++ 用于 volatile 已弃用
			cycle_start = osKernelGetTickCount();
			osDelay(1);  // 超期时仍让出 CPU，生命周期任务才能执行挂起
		}
	}
}

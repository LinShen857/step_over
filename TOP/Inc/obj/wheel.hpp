#pragma once

#include "math/filter.hpp"
#include "math/pid.hpp"
#include "motor_gm.hpp"

/* 轮端扭矩由电流换算而来，量纲上是电机输出扭矩，匀速巡航时地面扭矩同样很小，
   因此不能按扭矩绝对值判空转。判据改成「扭矩能否被空转模型解释」：
   tau ~= J_air*alpha + b_air*omega + tau_air*sgn(omega)，用归一化残差
   rho = |tau - tau_hat| / (J_air*|alpha| + |b_air*omega| + tau_air)，
   离地时 rho 很小，带上负载时 rho 显著变大。三个系数逐轮离线标定。 */
namespace Wheel_Slip_N
{
inline constexpr float ctrl_dt = 0.001f;
inline constexpr float rho_low = 0.30f;
inline constexpr float rho_high = 0.70f;
inline constexpr float score_rise = 30.f;
inline constexpr float score_decay = 10.f;
inline constexpr float enter_level = 0.90f;
inline constexpr float exit_level = 0.35f;
inline constexpr float dwell_time = 0.08f;
inline constexpr float speed_min = 1.5f;
inline constexpr uint16_t rx_stale_max = 5;
}

struct slip_cfg_s
{
	const float j_air;
	const float b_air;
	const float tau_air;
};

class Wheel
{
	public:
	explicit Wheel(Motor_GM& motor_, const float x_, const float y_, const float z_,
		const float radius_, const lowpass_cfg_s& speed_filter_cfg_, const pid_vel_cfg_s& pid_cfg_,
		const slip_cfg_s& slip_cfg_)
		: motor(motor_), x(x_), y(y_), z(z_), radius(radius_), roller_sign(0.f),
		motor_sign(0.f), speed(0.f), torque(0.f), target_speed(0.f), cmd_current(0.f),
		speed_filter(speed_filter_cfg_), pid(pid_cfg_), slip_cfg(slip_cfg_), accel(0.f),
		slip_score(0.f), slip_rho(1.f), dwell(0.f), airborne(false), speed_prev(0.f),
		torque_lpf(0.f), torque_filter(speed_filter_cfg_), rx_prev(0), rx_stale(0)
	{
		const float side = y_ > 0.f ? 1.f : -1.f;
		const float front = x_ > 0.f ? 1.f : -1.f;
		roller_sign = front * side;
		motor_sign = -side;
	}

	void UpdateFbdata()
	{

		speed = speed_filter.Update(motor.GetSpeedRadS() * motor_sign);
		torque = motor.GetTorqueNm() * motor_sign;

		const uint16_t rx = motor.GetRxCount();
		rx_stale = rx == rx_prev ? rx_stale + 1U : 0U;
		rx_prev = rx;

		UpdateSlip();
	}

	void UpdateSlip()
	{
		accel = (speed - speed_prev) / Wheel_Slip_N::ctrl_dt;
		speed_prev = speed;
		torque_lpf = torque_filter.Update(torque);

		const float drag = slip_cfg.b_air * speed +
			(speed > 0.f ? slip_cfg.tau_air : -slip_cfg.tau_air);
		const float pred = slip_cfg.j_air * accel + drag;
		const float scale = slip_cfg.j_air * Abs(accel) + Abs(slip_cfg.b_air * speed) +
			slip_cfg.tau_air;

		float target = 0.f;
		if (rx_stale > Wheel_Slip_N::rx_stale_max || Abs(speed) < Wheel_Slip_N::speed_min)
		{
			slip_rho = 1.f;
		}
		else
		{
			slip_rho = Abs(torque_lpf - pred) / scale;
			target = slip_rho < Wheel_Slip_N::rho_low ? 1.f
				: (slip_rho < Wheel_Slip_N::rho_high
					? (Wheel_Slip_N::rho_high - slip_rho) /
						(Wheel_Slip_N::rho_high - Wheel_Slip_N::rho_low) : 0.f);
		}

		StepScore(target);

		dwell = slip_score > Wheel_Slip_N::enter_level
			? (dwell + Wheel_Slip_N::ctrl_dt < Wheel_Slip_N::dwell_time
				? dwell + Wheel_Slip_N::ctrl_dt : Wheel_Slip_N::dwell_time) : 0.f;

		if (!airborne && dwell >= Wheel_Slip_N::dwell_time) airborne = true;
		else if (airborne && slip_score < Wheel_Slip_N::exit_level) airborne = false;
	}

	void Send()
	{
		motor.SetCmdCurrent(cmd_current * motor_sign);
	}

	Motor_GM& motor;
	float x;
	float y;
	float z;
	float radius;
	float roller_sign;
	float motor_sign;
	float speed;
	float torque;
	float target_speed;
	float cmd_current;
	lowpass_s speed_filter;
	pid_vel_s pid;
	const slip_cfg_s slip_cfg;
	float accel;
	float slip_score;
	float slip_rho;
	float dwell;
	bool airborne;
	float speed_prev;
	float torque_lpf;
	lowpass_s torque_filter;
	uint16_t rx_prev;
	uint16_t rx_stale;

	private:
	void StepScore(const float target_)
	{
		const float step = Wheel_Slip_N::ctrl_dt * (target_ > slip_score
			? Wheel_Slip_N::score_rise : Wheel_Slip_N::score_decay);
		slip_score = target_ > slip_score
			? (slip_score + step > target_ ? target_ : slip_score + step)
			: (slip_score - step < target_ ? target_ : slip_score - step);
	}

	static float Abs(const float value_)
	{
		return value_ < 0.f ? -value_ : value_;
	}
};

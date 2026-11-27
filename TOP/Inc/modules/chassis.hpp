#pragma once

#include <cstdint>

#include "leg.hpp"
#include "power_control.hpp"
#include "strut.hpp"
#include "wheel.hpp"

// 底盘全局坐标系：x 前为正，y 左为正，w 逆时针为正
namespace Chassis_N
{
	inline constexpr uint8_t wheel_num = 4;
	inline constexpr uint32_t ctrl_freq_hz = 1000;
	inline constexpr float ctrl_dt = 0.001f;
	inline constexpr uint8_t lift_per_tick = 2;
	inline constexpr float wheel_speed_max = 20.f;
	inline constexpr float strut_rate_max = 4.f;
	inline constexpr float track_err_hi = 1.2f;
	inline constexpr float track_err_lo = 0.4f;
	inline constexpr float track_rise_min = 0.4f;
	inline constexpr float track_peak_lpf = 0.005f;
	inline constexpr float track_power_ratio = 0.98f;
	inline constexpr float track_decay = 0.997f;
	inline constexpr float track_rise = 0.4f;
	inline constexpr float track_cheap_ratio = 0.80f;
	inline constexpr float track_active_ratio = 0.05f;
	inline constexpr float track_cap_ratio = 0.95f;
	inline constexpr float limit_vx_init = 1.f;
	inline constexpr float limit_vy_init = 0.42f;
	inline constexpr float limit_w_init = 1.f;
	inline constexpr float twist_limit_min = 0.15f;
}

class Chassis
{
	public:
	Chassis(Wheel& lf_, Wheel& rf_, Wheel& rb_, Wheel& lb_, Leg& leg_lb_, Leg& leg_rb_,
		Strut& strut_lf_, Strut& strut_rf_, const PowerControl_N::cfg_s& power_cfg_)
		: target_vx(0.f), target_vy(0.f), target_w(0.f), target_strut_rate(0.f), fb_vx(0.f), fb_vy(0.f), fb_w(0.f),
		cmd_vx(0.f), cmd_vy(0.f), cmd_w(0.f),
		limit_vx(Chassis_N::limit_vx_init), limit_vy(Chassis_N::limit_vy_init),
		limit_w(Chassis_N::limit_w_init), speed_peak(0.f),
		slip_cut_enable(false), slip_cut_gain(1.f),
		wheel_lf(lf_), wheel_rf(rf_), wheel_rb(rb_), wheel_lb(lb_),
		leg_lb(leg_lb_), leg_rb(leg_rb_), strut_lf(strut_lf_), strut_rf(strut_rf_),
		power_control(power_cfg_), lift_index(0)
	{}

	void SetSlipCut(const bool enable_)
	{
		slip_cut_enable = enable_;
	}

	void SetSlipCutGain(const float gain_)
	{
		slip_cut_gain = ClampNorm(gain_);
	}

	uint8_t AirborneMask() const
	{
		return static_cast<uint8_t>((wheel_lf.airborne ? 1U : 0U) | (wheel_rf.airborne ? 2U : 0U) |
			(wheel_rb.airborne ? 4U : 0U) | (wheel_lb.airborne ? 8U : 0U));
	}

	void SetCmdNorm(const float nx_, const float ny_, const float nz_, const float ns_)
	{
		cmd_vx = ClampNorm(nx_) * VMax();
		cmd_vy = ClampNorm(ny_) * VMax();
		cmd_w = ClampNorm(nz_) * WMax();
		target_strut_rate = ClampNorm(ns_) * Chassis_N::strut_rate_max;
	}

	void SetPowerLimit(const float limit_)
	{
		power_control.SetLimit(limit_);
	}
	float VMax() const { return Chassis_N::wheel_speed_max * wheel_lf.radius; }

	float WMax() const
	{
		const Wheel* wheel[Chassis_N::wheel_num] = {&wheel_lf, &wheel_rf, &wheel_rb, &wheel_lb};
		float lever_max = 0.f;
		for (uint8_t i = 0; i < Chassis_N::wheel_num; ++i)
		{
			const float lever = wheel[i]->y + wheel[i]->roller_sign * wheel[i]->x;
			const float lever_abs = lever < 0.f ? -lever : lever;
			if (lever_abs > lever_max) lever_max = lever_abs;
		}
		return lever_max > 0.f ? VMax() / lever_max : 0.f;
	}

	void SolveInverse()
	{
		SolveWheel(wheel_lf, target_vx, target_vy, target_w);
		SolveWheel(wheel_rf, target_vx, target_vy, target_w);
		SolveWheel(wheel_rb, target_vx, target_vy, target_w);
		SolveWheel(wheel_lb, target_vx, target_vy, target_w);
	}

	void SolveForward()
	{
		const Wheel* wheel[Chassis_N::wheel_num] = {&wheel_lf, &wheel_rf, &wheel_rb, &wheel_lb};
		float a[Chassis_N::wheel_num][3] {};
		float b[Chassis_N::wheel_num] {};
		float m[3][3] {};
		float c[3] {};

		for (uint8_t i = 0; i < Chassis_N::wheel_num; ++i)
		{
			a[i][0] = 1.f;
			a[i][1] = wheel[i]->roller_sign;
			a[i][2] = wheel[i]->y + wheel[i]->roller_sign * wheel[i]->x;
			b[i] = wheel[i]->speed * wheel[i]->radius;
		}

		for (uint8_t i = 0; i < Chassis_N::wheel_num; ++i)
		{
			for (uint8_t row = 0; row < 3; ++row)
			{
				c[row] += a[i][row] * b[i];
				for (uint8_t col = 0; col < 3; ++col)
				{
					m[row][col] += a[i][row] * a[i][col];
				}
			}
		}

		const float det = Det3(m);
		if (det == 0.f) return;

		const float mx[3][3] = {{c[0], m[0][1], m[0][2]}, {c[1], m[1][1], m[1][2]},
			{c[2], m[2][1], m[2][2]}};
		const float my[3][3] = {{m[0][0], c[0], m[0][2]}, {m[1][0], c[1], m[1][2]},
			{m[2][0], c[2], m[2][2]}};
		const float mw[3][3] = {{m[0][0], m[0][1], c[0]}, {m[1][0], m[1][1], c[1]},
			{m[2][0], m[2][1], c[2]}};

		fb_vx = Det3(mx) / det;
		fb_vy = Det3(my) / det;
		fb_w = Det3(mw) / det;
	}

	void UpdateFeedback()
	{
		wheel_lf.UpdateFbdata();
		wheel_rf.UpdateFbdata();
		wheel_rb.UpdateFbdata();
		wheel_lb.UpdateFbdata();

		strut_lf.UpdateFb();
		strut_rf.UpdateFb();
		leg_lb.UpdateFb();
		leg_rb.UpdateFb();

		leg_lb.CalcWheelCenter(wheel_lb.x, wheel_lb.z);
		leg_rb.CalcWheelCenter(wheel_rb.x, wheel_rb.z);

		SolveForward();
	}

	void Update(const float pitch_)
	{
		UpdateStrutTarget();
		UpdateFeedback();
		SolveTwist(pitch_);

		wheel_lf.pid.Update(wheel_lf.target_speed, wheel_lf.speed);
		wheel_rf.pid.Update(wheel_rf.target_speed, wheel_rf.speed);
		wheel_rb.pid.Update(wheel_rb.target_speed, wheel_rb.speed);
		wheel_lb.pid.Update(wheel_lb.target_speed, wheel_lb.speed);

		LimitPower();

		UpdateLift(lift_index);
		Transmit();
		UpdateLift(lift_index + 1);
		lift_index = lift_index + Chassis_N::lift_per_tick < motor_dm_num ? lift_index + Chassis_N::lift_per_tick : 0;
	}

	void Transmit()
	{
		wheel_lf.Send();
		wheel_rf.Send();
		wheel_rb.Send();
		wheel_lb.Send();
		Motor_GM::Tx(wheel_lf.motor, wheel_rf.motor, wheel_rb.motor, wheel_lb.motor);
	}

	float target_vx;
	float target_vy;
	float target_w;
	float target_strut_rate;
	float fb_vx;
	float fb_vy;
	float fb_w;
	float cmd_vx;
	float cmd_vy;
	float cmd_w;
	float limit_vx;
	float limit_vy;
	float limit_w;
	float speed_peak;
	bool slip_cut_enable;
	float slip_cut_gain;
	Wheel& wheel_lf;
	Wheel& wheel_rf;
	Wheel& wheel_rb;
	Wheel& wheel_lb;
	Leg& leg_lb;
	Leg& leg_rb;
	Strut& strut_lf;
	Strut& strut_rf;
	PowerControl power_control;

	private:
	uint8_t lift_index;

	void LimitPower()
	{
		const float speed[Chassis_N::wheel_num] = {wheel_lf.speed, wheel_rf.speed, wheel_rb.speed,
			wheel_lb.speed};
		float ref[Chassis_N::wheel_num] = {wheel_lf.pid.output, wheel_rf.pid.output,
			wheel_rb.pid.output, wheel_lb.pid.output};

		if (slip_cut_enable)
		{
			Wheel* wheel[Chassis_N::wheel_num] = {&wheel_lf, &wheel_rf, &wheel_rb, &wheel_lb};
			for (uint8_t i = 0; i < Chassis_N::wheel_num; ++i)
			{
				ref[i] *= 1.f - slip_cut_gain * wheel[i]->slip_score;
				if (wheel[i]->airborne) wheel[i]->pid.integral *= 0.9f;
			}
		}

		const float err[Chassis_N::wheel_num] = {wheel_lf.target_speed - wheel_lf.speed,
			wheel_rf.target_speed - wheel_rf.speed, wheel_rb.target_speed - wheel_rb.speed,
			wheel_lb.target_speed - wheel_lb.speed};

		power_control.Update(ref, speed, err);

		wheel_lf.cmd_current = power_control.cmd[0];
		wheel_rf.cmd_current = power_control.cmd[1];
		wheel_rb.cmd_current = power_control.cmd[2];
		wheel_lb.cmd_current = power_control.cmd[3];

		UpdateTrackLimit();
	}

	void UpdateStrutTarget()
	{
		const float delta = target_strut_rate * Chassis_N::ctrl_dt;
		strut_lf.target_pos += delta;
		strut_rf.target_pos += delta;
	}

	void UpdateLift(const uint8_t index_)
	{
		const float dt = Chassis_N::ctrl_dt * Chassis_N::lift_per_tick;
		switch (index_)
		{
			case 0: 
				strut_lf.UpdatePlan(dt);
				strut_lf.pid.Update(strut_lf.plan_pos, strut_lf.angle_cont);
				strut_lf.Send(); 
				break;
			case 1: 
				strut_rf.UpdatePlan(dt);
				strut_rf.pid.Update(strut_rf.plan_pos, strut_rf.angle_cont);
				strut_rf.Send(); 
				break;
			case 2:
				leg_rb.UpdatePlan(dt);
				leg_rb.pid.Update(leg_rb.plan_pos, leg_rb.angle);
				leg_rb.Send();
				break;
			default:
				leg_lb.UpdatePlan(dt);
				leg_lb.pid.Update(leg_lb.plan_pos, leg_lb.angle);
				leg_lb.Send();
				break;
		}
	}

	static void SolveWheel(Wheel& wheel_, const float vx_, const float vy_, const float w_)
	{
		const float v = vx_ + wheel_.roller_sign * vy_ + w_ * (wheel_.y + wheel_.roller_sign * wheel_.x);
		wheel_.target_speed = v / wheel_.radius;
	}
	// 机械上限（含腿补偿）约束，返回统一的 twist 比例
	float TwistScale(const float* comp_) const
	{
		const Wheel* wheel[Chassis_N::wheel_num] = {&wheel_lf, &wheel_rf, &wheel_rb, &wheel_lb};
		float s_mech = 1.f;

		for (uint8_t i = 0; i < Chassis_N::wheel_num; ++i)
		{
			const float w = wheel[i]->target_speed;
			const float c = comp_[i];

			if (w > 0.f)
			{
				const float bound = (Chassis_N::wheel_speed_max - c) / w;
				if (bound < s_mech) s_mech = bound;
			}
			else if (w < 0.f)
			{
				const float bound = (Chassis_N::wheel_speed_max + c) / -w;
				if (bound < s_mech) s_mech = bound;
			}
		}

		if (s_mech > 1.f) s_mech = 1.f;
		return s_mech > 0.f ? s_mech : 0.f;
	}

	static float ClampNorm(const float value_)
	{
		return value_ > 1.f ? 1.f : (value_ < -1.f ? -1.f : value_);
	}

	// 腿补偿 + twist 统一缩放：不可达一律表现为整体变慢，不破坏运动学模式
	void SolveTwist(const float pitch_)
	{
		float comp[Chassis_N::wheel_num] {};
		comp[2] = LegComp(leg_rb, wheel_rb, pitch_);
		comp[3] = LegComp(leg_lb, wheel_lb, pitch_);

		target_vx = cmd_vx * limit_vx;
		target_vy = cmd_vy * limit_vy;
		target_w = cmd_w * limit_w;
		SolveInverse();

		const float scale = TwistScale(comp);
		if (scale < 1.f)
		{
			target_vx *= scale;
			target_vy *= scale;
			target_w *= scale;
			SolveInverse();
		}

		wheel_rb.target_speed += comp[2];
		wheel_lb.target_speed += comp[3];
	}

	static float LegComp(const Leg& leg_, const Wheel& wheel_, const float pitch_)
	{
		const float v = Leg_N::orbit_radius * leg_.vel * cosf(leg_.angle + pitch_);
		return v / wheel_.radius;
	}

	// 每轴独立：追不上且已停涨→只收缩在用的轴；顶到上限、跟得上且功率有富余→缓慢放开
	void UpdateTrackLimit()
	{
		const Wheel* wheel[Chassis_N::wheel_num] = {&wheel_lf, &wheel_rf, &wheel_rb, &wheel_lb};
		float deficit = 0.f;
		float speed_max = 0.f;

		for (uint8_t i = 0; i < Chassis_N::wheel_num; ++i)
		{
			const float t = wheel[i]->target_speed;
			const float s = wheel[i]->speed;
			const float t_abs = t < 0.f ? -t : t;
			const float s_abs = s < 0.f ? -s : s;

			if (t_abs - s_abs > deficit) deficit = t_abs - s_abs;
			if (s_abs > speed_max) speed_max = s_abs;
		}

		const float rise = speed_max - speed_peak;
		speed_peak += (speed_max - speed_peak) * Chassis_N::track_peak_lpf;

		const float budget = power_control.budget;
		const float power = power_control.power_total;
		const float v_active = VMax() * Chassis_N::track_active_ratio;
		const float w_active = WMax() * Chassis_N::track_active_ratio;
		const float step = Chassis_N::track_rise * Chassis_N::ctrl_dt;

		if (power >= budget * Chassis_N::track_power_ratio && deficit > Chassis_N::track_err_hi &&
			rise < Chassis_N::track_rise_min)
		{
			if (Abs(cmd_vx) > v_active) limit_vx *= Chassis_N::track_decay;
			if (Abs(cmd_vy) > v_active) limit_vy *= Chassis_N::track_decay;
			if (Abs(cmd_w) > w_active) limit_w *= Chassis_N::track_decay;
		}
		else if (deficit < Chassis_N::track_err_lo && power < budget * Chassis_N::track_cheap_ratio)
		{
			if (Abs(cmd_vx) > VMax() * Chassis_N::track_cap_ratio) limit_vx += step;
			if (Abs(cmd_vy) > VMax() * Chassis_N::track_cap_ratio) limit_vy += step;
			if (Abs(cmd_w) > WMax() * Chassis_N::track_cap_ratio) limit_w += step;
		}

		limit_vx = ClampLimit(limit_vx);
		limit_vy = ClampLimit(limit_vy);
		limit_w = ClampLimit(limit_w);
	}

	static float ClampLimit(const float value_)
	{
		return value_ > 1.f ? 1.f : (value_ < Chassis_N::twist_limit_min ? Chassis_N::twist_limit_min : value_);
	}

	static float Abs(const float value_)
	{
		return value_ < 0.f ? -value_ : value_;
	}

	static float Det3(const float m_[3][3])
	{
		return m_[0][0] * (m_[1][1] * m_[2][2] - m_[1][2] * m_[2][1]) -
			m_[0][1] * (m_[1][0] * m_[2][2] - m_[1][2] * m_[2][0]) +
			m_[0][2] * (m_[1][0] * m_[2][1] - m_[1][1] * m_[2][0]);
	}
};

inline Chassis* chassis_ptr = nullptr;

inline void Chassis_Attach(Chassis& chassis_)
{
	chassis_ptr = &chassis_;
}

inline Chassis* Chassis_Get()
{
	return chassis_ptr;
}

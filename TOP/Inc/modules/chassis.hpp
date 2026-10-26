#pragma once

#include <cstdint>

#include "leg.hpp"
#include "strut.hpp"
#include "wheel.hpp"

// 底盘全局坐标系：x 前为正，y 左为正，w 逆时针为正
namespace Chassis_N
{
	inline constexpr uint8_t wheel_num = 4;
	inline constexpr uint32_t ctrl_freq_hz = 1000;
	inline constexpr float ctrl_dt = 0.001f;
	inline constexpr uint8_t lift_per_tick = 2;
	inline constexpr float wheel_speed_max = 15.f;
}

class Chassis
{
	public:
	Chassis(Wheel& lf_, Wheel& rf_, Wheel& rb_, Wheel& lb_, Leg& leg_lb_, Leg& leg_rb_,
		Strut& strut_lf_, Strut& strut_rf_)
		: target_vx(0.f), target_vy(0.f), target_w(0.f), target_struts(0.f), fb_vx(0.f), fb_vy(0.f), fb_w(0.f),
		last_scale(1.f), wheel_lf(lf_), wheel_rf(rf_), wheel_rb(rb_), wheel_lb(lb_),
		leg_lb(leg_lb_), leg_rb(leg_rb_), strut_lf(strut_lf_), strut_rf(strut_rf_), lift_index(0)
	{}

	void SetCmdNorm(const float nx_, const float ny_, const float nz_, const float ns_)
	{
		target_vx = ClampNorm(nx_) * VMax();
		target_vy = ClampNorm(ny_) * VMax();
		target_w = ClampNorm(nz_) * WMax();
		target_struts = ClampNorm(ns_) * 5.f;
		strut_lf.target_speed = target_struts;
		strut_rf.target_speed = target_struts;
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
		LimitWheelTarget();
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
		SolveInverse();
	}

	void Update(const float pitch_)
	{
		UpdateFeedback();
		CompensateLegMotion(pitch_);

		wheel_lf.pid.Update(wheel_lf.target_speed, wheel_lf.speed);
		wheel_rf.pid.Update(wheel_rf.target_speed, wheel_rf.speed);
		wheel_rb.pid.Update(wheel_rb.target_speed, wheel_rb.speed);
		wheel_lb.pid.Update(wheel_lb.target_speed, wheel_lb.speed);

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
	float target_struts;
	float fb_vx;
	float fb_vy;
	float fb_w;
	float last_scale;	
	Wheel& wheel_lf;
	Wheel& wheel_rf;
	Wheel& wheel_rb;
	Wheel& wheel_lb;
	Leg& leg_lb;
	Leg& leg_rb;
	Strut& strut_lf;
	Strut& strut_rf;

	private:
	uint8_t lift_index;

	void UpdateLift(const uint8_t index_)
	{
		const float dt = Chassis_N::ctrl_dt * Chassis_N::lift_per_tick;
		switch (index_)
		{
			case 0: 
				strut_lf.pid.Update(strut_lf.target_speed, strut_lf.vel);
				strut_lf.Send(); 
				break;
			case 1: 
				strut_rf.pid.Update(strut_rf.target_speed, strut_rf.vel);
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

	void LimitWheelTarget()
	{
		Wheel* wheel[Chassis_N::wheel_num] = {&wheel_lf, &wheel_rf, &wheel_rb, &wheel_lb};
		float speed_max = 0.f;
		for (uint8_t i = 0; i < Chassis_N::wheel_num; ++i)
		{
			const float speed = wheel[i]->target_speed;
			const float speed_abs = speed < 0.f ? -speed : speed;
			if (speed_abs > speed_max) speed_max = speed_abs;
		}

		last_scale = 1.f;
		if (speed_max > Chassis_N::wheel_speed_max && speed_max > 0.f)
		{
			last_scale = Chassis_N::wheel_speed_max / speed_max;
			for (uint8_t i = 0; i < Chassis_N::wheel_num; ++i)
			{
				wheel[i]->target_speed *= last_scale;
			}
		}
	}

	static float ClampNorm(const float value_)
	{
		return value_ > 1.f ? 1.f : (value_ < -1.f ? -1.f : value_);
	}

	void CompensateLegMotion(const float pitch_)
	{
		CompensateWheel(wheel_rb, leg_rb, pitch_);
		CompensateWheel(wheel_lb, leg_lb, pitch_);
	}

	static void CompensateWheel(Wheel& wheel_, const Leg& leg_, const float pitch_)
	{
		const float v = Leg_N::orbit_radius * leg_.vel * cosf(leg_.angle + pitch_);
		wheel_.target_speed = ClampAbs(wheel_.target_speed + v / wheel_.radius,
			Chassis_N::wheel_speed_max);
	}

	static float ClampAbs(const float value_, const float limit_)
	{
		return value_ > limit_ ? limit_ : (value_ < -limit_ ? -limit_ : value_);
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

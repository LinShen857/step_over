#pragma once

#include <cmath>

#include "math/pid.hpp"
#include "math/planning.hpp"
#include "motor_dm.hpp"

namespace Leg_N
{
inline constexpr float deg_to_rad = 0.017453292519943295f;
// 轮心在 xz 平面内绕 (0, 0, orbit_center_z)、半径 orbit_radius 的圆运动；电机编码经零点与镜像换算后即为圆上的几何角
inline constexpr float orbit_center_z = -0.235f;
inline constexpr float orbit_radius = 0.235f;
inline constexpr float shrink_angle = -34.f * deg_to_rad;
inline constexpr float stretch_angle = -90.f * deg_to_rad;
}

class Leg
{
	public:
		explicit Leg(Motor_DM& motor_, const float dir_, const mit_cfg_s& mit_cfg_,
		const pid_pos_cfg_s& pid_cfg_, const Planning_N::planning_cfg_s& plan_cfg_,
		const float shrink_angle_, const float stretch_angle_)
			: motor(motor_), dir(dir_), L(0.f), angle(0.f), vel(0.f), torque(0.f),
			target_angle(0.f), plan_pos(0.f), plan_vel(0.f), plan_cfg(plan_cfg_), plan{},
			resync(false), shrink_angle(shrink_angle_), stretch_angle(stretch_angle_),
			mit(mit_cfg_), pid(pid_cfg_)
		{}

	void UpdateFb()
	{
		angle = motor.GetPosRadZeroed() * dir;
		vel = motor.GetVelRadS() * dir;
		torque = motor.GetTorqueNm() * dir;
	}

	void UpdatePlan(const float dt_)
	{
		if (resync || target_angle != plan.target)
		{
			const Planning_N::cubic_ref_s now = resync
				? Planning_N::cubic_ref_s{angle, vel, 0.f} : plan.Ref();
			resync = false;
			const float time_total = Planning_N::CubicTimeByLimits(target_angle - now.pos,
				now.vel, 0.f, plan_cfg.vel_max, plan_cfg.acc_max, plan_cfg.time_base);
			plan.Plan(target_angle, time_total, 0.f, now.pos, now.vel);
		}

		const Planning_N::cubic_ref_s ref = plan.Update(dt_);
		plan_pos = ref.pos;
		plan_vel = ref.vel;
	}

	void Resync()
	{
		resync = true;
		pid.Clear();
	}

	void CalcWheelCenter(float& x_, float& z_) const
	{
		x_ = Leg_N::orbit_radius * sinf(angle);
		z_ = Leg_N::orbit_center_z + Leg_N::orbit_radius * cosf(angle);
	}

	void Send()
	{
		motor.Ctrl_MIT(0.f, (plan_vel + pid.output) * dir, mit._kp, mit._kd, 0.f);
	}

	Motor_DM& motor;
	float dir;
	float L;
	float angle;
	float vel;
	float torque;
	float target_angle;
	float plan_pos;
	float plan_vel;
	Planning_N::planning_cfg_s plan_cfg;
	Planning_N::cubic_plan_s plan;
	bool resync;
	float shrink_angle;
	float stretch_angle;
	mit_cfg_s mit;
	pid_pos_s pid;
};

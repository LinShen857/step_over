#pragma once

#include <cmath>
#include <cstdint>

#include "math/pid.hpp"
#include "math/planning.hpp"
#include "motor_dm.hpp"

namespace  Joint_N{
	struct MDH_cfg_s
	{
		float alpha;
		float a;
		float theta_offset;
		float d;
	}
	struct vectors_s{
		float x;
		float y;
		float z;
	}
}

class Joint
{
    public:
    explicit Leg(Motor_DM& motor_, const float dir_, const mit_cfg_s& mit_cfg_,
		const pid_pos_cfg_s& pid_cfg_, const Planning_N::planning_cfg_s& plan_cfg_,const Joint_N::MDH_cfg_s& mdh_cfg_)
			: motor(motor_), dir(dir_), angle(0.f), theta(0.f),vel(0.f), torque(0.f),
			target_angle(0.f), plan_pos(0.f), plan_vel(0.f), plan_cfg(plan_cfg_), plan{},
			resync(false), mit(mit_cfg_), pid(pid_cfg_),mdh_cfg(mdh_cfg_),vector{}
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
    void Send()
	{
		motor.Ctrl_MIT(0.f, (plan_vel + pid.output) * dir, mit._kp, mit._kd, 0.f);
	}

	Motor_DM& motor;
	float dir;
	float angle;
	float theta;
	float vel;
	float torque;
	float target_angle;
	float plan_pos;
	float plan_vel;
	Planning_N::planning_cfg_s plan_cfg;
	Planning_N::cubic_plan_s plan;
	bool resync;
	mit_cfg_s mit;
	pid_pos_s pid;
	Joint_N::MDH_cfg_s;
	Joint_N::vectors_s;
};
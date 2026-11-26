#pragma once

#include "math/angle.hpp"
#include "math/pid.hpp"
#include "math/planning.hpp"
#include "motor_dm.hpp"

class Strut
{
	public:
	explicit Strut(Motor_DM& motor_, const float dir_, const float x_, const float y_, const float z_,
		const mit_cfg_s& mit_cfg_, const pid_pos_cfg_s& pid_cfg_,
		const Planning_N::planning_cfg_s& plan_cfg_)
		: motor(motor_), dir(dir_), x(x_), y(y_), z(z_),
		angle(0.f), angle_cont(0.f), vel(0.f), torque(0.f),
		target_pos(0.f), last_target_pos(0.f), plan_pos(0.f), plan_vel(0.f),
		plan_cfg(plan_cfg_), plan{}, homing(false), mit(mit_cfg_), pid(pid_cfg_)
	{}

	void UpdateFb()
	{
		const float pos_wrapped = motor.GetPosRadZeroed() * dir;
		angle = pos_wrapped;
		angle_cont = Angle_N::UnwrapNear(pos_wrapped, angle_cont);
		vel = motor.GetVelRadS() * dir;
		torque = motor.GetTorqueNm() * dir;
	}

	void ResetPos()
	{
		target_pos = Angle_N::UnwrapNear(0.f, angle_cont);
		plan.Plan(target_pos,
			Planning_N::CubicTimeByLimits(target_pos - angle_cont, vel, 0.f,
				plan_cfg.vel_max, plan_cfg.acc_max, plan_cfg.time_base),
			0.f, angle_cont, vel);
		homing = true;
		pid.Clear();
	}

	void UpdatePlan(const float dt_)
	{
		if (homing)
		{
			const Planning_N::cubic_ref_s ref = plan.Update(dt_);
			plan_pos = ref.pos;
			plan_vel = ref.vel;
			if (plan.IsArrived())
			{
				homing = false;
				target_pos = plan_pos;
				last_target_pos = target_pos;
			}
			return;
		}

		plan_vel = (target_pos - last_target_pos) / dt_;
		plan_pos = target_pos;
		last_target_pos = target_pos;
	}

	void Send()
	{
		motor.Ctrl_MIT(0.f, (plan_vel + pid.output) * dir, mit._kp, mit._kd, 0.f);
	}

	Motor_DM& motor;
	float dir;
	float x;
	float y;
	float z;
	float angle;
	float angle_cont;
	float vel;
	float torque;
	float target_pos;
	float last_target_pos;
	float plan_pos;
	float plan_vel;
	Planning_N::planning_cfg_s plan_cfg;
	Planning_N::cubic_plan_s plan;
	bool homing;
	mit_cfg_s mit;
	pid_pos_s pid;
};

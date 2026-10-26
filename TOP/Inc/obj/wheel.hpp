#pragma once

#include "math/filter.hpp"
#include "math/pid.hpp"
#include "motor_gm.hpp"

class Wheel
{
	public:
	explicit Wheel(Motor_GM& motor_, const float x_, const float y_, const float z_,
		const float radius_, const lowpass_cfg_s& speed_filter_cfg_, const pid_vel_cfg_s& pid_cfg_)
		: motor(motor_), x(x_), y(y_), z(z_), radius(radius_), roller_sign(0.f),
		motor_sign(0.f), speed(0.f), torque(0.f), target_speed(0.f), speed_filter(speed_filter_cfg_),
		pid(pid_cfg_)
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
	}

	void Send()
	{
		motor.SetCmdCurrent(pid.output * motor_sign);
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
	lowpass_s speed_filter;
	pid_vel_s pid;
};

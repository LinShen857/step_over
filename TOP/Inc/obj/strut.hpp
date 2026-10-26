#pragma once

#include "math/pid.hpp"
#include "motor_dm.hpp"

class Strut
{
	public:
	explicit Strut(Motor_DM& motor_, const float dir_, const float x_, const float y_, const float z_,
		const mit_cfg_s& mit_cfg_,const pid_vel_cfg_s& pid_cfg_)
		: motor(motor_), dir(dir_), x(x_), y(y_), z(z_),
		angle(0.f), vel(0.f), torque(0.f), target_speed(0.f), mit(mit_cfg_), pid(pid_cfg_)	
	{}

	void UpdateFb()
	{
		angle = motor.GetPosRad() * dir;
		vel = motor.GetVelRadS() * dir;
		torque = motor.GetTorqueNm() * dir;
	}

	void Send()
	{
		motor.Ctrl_MIT(0.f, 0.f, mit._kp, mit._kd, pid.output * dir);
	}

	Motor_DM& motor;
	float dir;
	float x;
	float y;
	float z;
	float angle;
	float vel;
	float torque;
	float target_speed;
	mit_cfg_s mit;
	pid_vel_s pid;
};

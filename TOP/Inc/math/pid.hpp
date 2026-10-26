#pragma once

#include <cmath>

[[gnu::always_inline]] inline float Limit(const float value, const float max, const float min)
{
	return fmaxf(fminf(value, max), min);
}

inline constexpr float pid_nominal_ts = 0.001f;

struct pid_pos_cfg_s
{
	const float _kp;
	const float _ki;
	const float _kd;
	const float _deadzone;
	const float _integral_limit;
	const float _output_limit;
	const float _ts = pid_nominal_ts;
};

struct pid_pos_s
{
	float kp;
	float ki;
	float kd;
	float deadzone;
	float error;
	float error_prev;
	float integral_limit;
	float integral;
	float output_limit;
	float output;
	float ts;
	explicit pid_pos_s(const pid_pos_cfg_s& _pid_cfg)
		: kp(_pid_cfg._kp), ki(_pid_cfg._ki), kd(_pid_cfg._kd), deadzone(_pid_cfg._deadzone), error(0), error_prev(0),
		integral_limit(_pid_cfg._integral_limit), integral(0), output_limit(_pid_cfg._output_limit), output(0),
		ts(_pid_cfg._ts)
	{}
	void Update(const float _target, const float _feedback)
	{
		error_prev = error;
		error = _target - _feedback;
		if (error < deadzone && error > -deadzone) return;
		integral = Limit(integral + error, integral_limit, -integral_limit);
		output = (error - error_prev) * kd * (pid_nominal_ts / ts) + integral * ki * (ts / pid_nominal_ts) + error * kp;
		output = Limit(output, output_limit, -output_limit);
	}
	void Clear()
	{
		error = 0;
		error_prev = 0;
		integral = 0;
		output = 0;
	}
};

struct pid_vel_cfg_s
{
	const float _kp;
	const float _ki;
	const float _kd;
	const float _deadzone;
	const float _integral_limit;
	const float _output_limit;
	const float _ts = pid_nominal_ts;
};

struct pid_vel_s
{
	float kp;
	float ki;
	float kd;
	float deadzone;
	float error;
	float error_prev;
	float integral_limit;
	float integral;
	float output_limit;
	float output;
	float ts;
	explicit pid_vel_s(const pid_vel_cfg_s& _pid_cfg)
		: kp(_pid_cfg._kp), ki(_pid_cfg._ki), kd(_pid_cfg._kd), deadzone(_pid_cfg._deadzone), error(0), error_prev(0),
		integral_limit(_pid_cfg._integral_limit), integral(0), output_limit(_pid_cfg._output_limit), output(0),
		ts(_pid_cfg._ts)
	{}
	void Update(const float _target, const float _feedback)
	{
		error_prev = error;
		error = _target - _feedback;
		if (error < deadzone && error > -deadzone) return;
		integral = Limit(integral + error, integral_limit, -integral_limit);
		output = (error - error_prev) * kd * (pid_nominal_ts / ts) + integral * ki * (ts / pid_nominal_ts) + error * kp;
		output = Limit(output, output_limit, -output_limit);
	}
	void Clear()
	{
		error = 0;
		error_prev = 0;
		integral = 0;
		output = 0;
	}
};

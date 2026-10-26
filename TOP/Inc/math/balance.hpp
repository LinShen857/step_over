#pragma once

#include <cmath>

namespace Balance_N
{
inline constexpr float kEps = 1e-6f;
}

class Balance
{
	public:
	explicit Balance(const float front_x_, const float front_z_, const float orbit_radius_,
		const float orbit_center_z_, const float q_min_, const float q_max_)
		: front_x(front_x_), front_z(front_z_), orbit_radius(orbit_radius_),
		orbit_center_z(orbit_center_z_), q_min(q_min_), q_max(q_max_)
	{}

	float Theta(const float q_) const
	{
		const float dx = front_x - orbit_radius * sinf(q_);
		const float dz = front_z - orbit_center_z - orbit_radius * cosf(q_);
		return dx > Balance_N::kEps ? atanf(dz / dx) : 0.f;
	}

	float QFromTheta(const float theta_) const
	{
		const float c = cosf(theta_) * (front_z - orbit_center_z -
			front_x * tanf(theta_)) / orbit_radius;
		const float cc = c > 1.f ? 1.f : (c < -1.f ? -1.f : c);
		return Clamp(-acosf(cc) - theta_);
	}

	float QBalance(const float q_now_, const float pitch_now_, const float pitch_target_) const
	{
		return QFromTheta(pitch_target_ - pitch_now_ + Theta(q_now_));
	}

	float Clamp(const float q_) const
	{
		return q_ > q_max ? q_max : (q_ < q_min ? q_min : q_);
	}

	float front_x;
	float front_z;
	float orbit_radius;
	float orbit_center_z;
	float q_min;
	float q_max;
};

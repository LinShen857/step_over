#pragma once

#include <cmath>

namespace Angle_N
{
inline constexpr float pi = 3.14159265358979323846f;
inline constexpr float two_pi = 6.28318530717958647692f;

[[gnu::always_inline]] inline float WrapRad(const float angle_)
{
	return angle_ - two_pi * floorf((angle_ + pi) / two_pi);
}

[[gnu::always_inline]] inline float UnwrapNear(const float angle_, const float ref_)
{
	return ref_ + WrapRad(angle_ - ref_);
}
}

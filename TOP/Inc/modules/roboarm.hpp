#pragma once

#include <array>
#include <cmath>
#include <cstdint>

#include "joint.hpp"

namespace Roboarm_N{
    inline constexpr uint8_t J_Count=7;
	inline constexpr std::array<float, Roboarm_N::J_Count>KMass=
    {0.f,0.f,0.f,0.f,0.f,0.f,0.f};
    inline constexpr std::array<float, Roboarm_N::J_Count>KLen=
    {0.f,0.f,0.f,0.f,0.f,0.f,0.f};
};
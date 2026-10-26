#pragma once

#include <cmath>

// 三次式路径规划
namespace Planning_N
{
	inline constexpr float time_min = 0.002f;
	inline constexpr float time_max = 60.f;
	inline constexpr float time_base = 0.02f;

	[[gnu::always_inline]] inline float Abs(const float value_)
	{
		return fabsf(value_);
	}

	struct planning_cfg_s
	{
		float vel_max;
		float acc_max;
		float time_base;
	};

	struct cubic_cfg_s
	{
		float pos_start;
		float vel_start;
		float pos_target;
		float vel_target;
		float time_total;
	};

	struct cubic_ref_s
	{
		float pos;
		float vel;
		float acc;
	};

	struct cubic_coeff_s
	{
		float b0;
		float b1;
		float b2;
		float b3;
		float t;
		float inv_t;
		float inv_t2;
	};

	struct cubic_peak_s
	{
		float vel;
		float acc;
	};

	[[nodiscard]] inline cubic_coeff_s CubicCoeff(const cubic_cfg_s& cfg_)
	{
		const float t_total = (cfg_.time_total > time_min) ? cfg_.time_total : time_min;
		const float dq = cfg_.pos_target - cfg_.pos_start;
		cubic_coeff_s c;
		c.b0 = cfg_.pos_start;
		c.b1 = t_total * cfg_.vel_start;
		c.b2 = 3.f * dq - t_total * (2.f * cfg_.vel_start + cfg_.vel_target);
		c.b3 = -2.f * dq + t_total * (cfg_.vel_start + cfg_.vel_target);
		c.t = t_total;
		c.inv_t = 1.f / t_total;
		c.inv_t2 = c.inv_t * c.inv_t;
		return c;
	}

	[[nodiscard]] inline cubic_ref_s CubicSample(const cubic_coeff_s& c_, const float time_)
	{
		const float tc = (time_ > 0.f) ? ((time_ < c_.t) ? time_ : c_.t) : 0.f;
		const float u = tc * c_.inv_t;
		cubic_ref_s out;
		out.pos = c_.b0 + u * (c_.b1 + u * (c_.b2 + u * c_.b3));
		out.vel = (c_.b1 + u * (2.f * c_.b2 + 3.f * c_.b3 * u)) * c_.inv_t;
		out.acc = (2.f * c_.b2 + 6.f * c_.b3 * u) * c_.inv_t2;
		return out;
	}

	[[nodiscard]] inline cubic_peak_s CubicPeaks(const cubic_coeff_s& c_)
	{
		cubic_peak_s p;
		p.acc = fmaxf(Abs(2.f * c_.b2), Abs(2.f * c_.b2 + 6.f * c_.b3)) * c_.inv_t2;
		p.vel = fmaxf(Abs(c_.b1), Abs(c_.b1 + 2.f * c_.b2 + 3.f * c_.b3)) * c_.inv_t;
		if (c_.b3 != 0.f)
		{
			const float u = -c_.b2 / (3.f * c_.b3);
			if ((u > 0.f) && (u < 1.f))
			{
				const float v = Abs((c_.b1 + u * (2.f * c_.b2 + 3.f * c_.b3 * u)) * c_.inv_t);
				if (v > p.vel) p.vel = v;
			}
		}
		return p;
	}

	[[nodiscard]] inline bool CubicWithinLimits(const float dq_, const float vel_start_, const float vel_target_,
		const float time_, const float vel_max_, const float acc_max_)
	{
		const cubic_cfg_s cfg{0.f, vel_start_, dq_, vel_target_, time_};
		const cubic_peak_s p = CubicPeaks(CubicCoeff(cfg));
		if ((vel_max_ > 0.f) && (p.vel > vel_max_)) return false;
		if ((acc_max_ > 0.f) && (p.acc > acc_max_)) return false;
		return true;
	}

	[[nodiscard]] inline float CubicTimeByLimits(const float dq_, const float vel_start_, const float vel_target_,
		const float vel_max_, const float acc_max_, const float base_time_ = time_base)
	{
		float t_lo = (base_time_ > time_min) ? base_time_ : time_min;
		if (t_lo > time_max) return time_max;
		if ((vel_max_ <= 0.f) && (acc_max_ <= 0.f)) return t_lo;
		if (CubicWithinLimits(dq_, vel_start_, vel_target_, t_lo, vel_max_, acc_max_)) return t_lo;

		float t_hi = t_lo;
		for (int i = 0; (i < 24) && (t_hi < time_max); ++i)
		{
			t_hi = fminf(t_hi * 2.f, time_max);
			if (CubicWithinLimits(dq_, vel_start_, vel_target_, t_hi, vel_max_, acc_max_)) break;
		}
		if (!CubicWithinLimits(dq_, vel_start_, vel_target_, t_hi, vel_max_, acc_max_)) return t_hi;
		const float t_ok = t_hi;

		for (int i = 0; i < 24; ++i)
		{
			const float t_m = 0.5f * (t_lo + t_hi);
			if (CubicWithinLimits(dq_, vel_start_, vel_target_, t_m, vel_max_, acc_max_)) t_hi = t_m;
			else t_lo = t_m;
		}
		return CubicWithinLimits(dq_, vel_start_, vel_target_, t_hi, vel_max_, acc_max_) ? t_hi : t_ok;
	}

	struct cubic_plan_s
	{
		void Plan(const float pos_target_, const float time_total_, const float vel_target_,
			const float pos_start_, const float vel_start_)
		{
			cfg.pos_start = pos_start_;
			cfg.vel_start = vel_start_;
			cfg.pos_target = pos_target_;
			cfg.vel_target = vel_target_;
			cfg.time_total = (time_total_ > time_min) ? time_total_ : time_min;
			coeff = CubicCoeff(cfg);
			t = 0.f;
			running = true;
			target = pos_target_;
		}

		void Replan(const float pos_target_, const float time_total_, const float vel_target_)
		{
			const cubic_ref_s now = Ref();
			Plan(pos_target_, time_total_, vel_target_, now.pos, now.vel);
		}

		cubic_ref_s Update(const float dt_)
		{
			if (!running) return End();
			t += (dt_ > 0.f) ? dt_ : 0.f;
			if (t >= cfg.time_total)
			{
				running = false;
				return End();
			}
			return CubicSample(coeff, t);
		}

		[[nodiscard]] cubic_ref_s Ref() const
		{
			return running ? CubicSample(coeff, t) : End();
		}

		[[nodiscard]] bool IsArrived() const { return !running; }

		[[nodiscard]] cubic_ref_s End() const
		{
			cubic_ref_s out;
			out.pos = cfg.pos_target;
			out.vel = cfg.vel_target;
			out.acc = 0.f;
			return out;
		}

		cubic_cfg_s cfg{};
		cubic_coeff_s coeff{};
		float t{0.f};
		float target{0.f};
		bool running{false};
	};
}

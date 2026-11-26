#pragma once

#include <cmath>
#include <cstdint>

namespace PowerControl_N
{
	inline constexpr uint8_t wheel_num = 4;
	inline constexpr uint8_t bisect_iter = 20;
	inline constexpr uint8_t expand_iter = 24;
	inline constexpr float lambda_start = 1.f;
	inline constexpr float lambda_expand = 4.f;
	inline constexpr float weight_min = 1e-3f;
	inline constexpr float power_limit = 50.f;

	// p(i, w) = copper * i^2 + torque * i * w + friction * w^2 + standby
	struct model_cfg_s
	{
		const float copper;	  // W/A^2
		const float torque;	  // N*m/A
		const float friction; // W/(rad/s)^2
		const float standby;  // W
	};

	inline constexpr model_cfg_s gm3508_model{1.4138e-1f, 2.8265e-1f, 7.880e-3f, 1.8848f};

	struct cfg_s
	{
		const model_cfg_s model;
		const float current_max;
		const float weight[wheel_num];
	};

	inline constexpr cfg_s gm3508_cfg{gm3508_model, 20.f, {1.f, 1.f, 1.f, 1.f}};
}

/* 先按 limit/ref_total 缩放各轮功率，再用求根公式反解电流（参考工程的做法）；
   判别式无解或解被限幅后超预算时，才改解二次凸优化：
   min sum_k w_k (i_k - ref_k)^2  s.t.  sum_k p_k(i_k) <= limit,  |i_k| <= current_max，
   对偶变量 lambda 二分至约束紧贴；连零电流都超预算时直接零电流。
   功率上限统一由 PowerControl_N::power_limit 给出，运行时可用 SetLimit() 修改；
   功率模型只用电机电流与转速，不掺入功率计反馈。 */
class PowerControl
{
	public:
	explicit PowerControl(const PowerControl_N::cfg_s& cfg_)
		: cfg(cfg_), limit(PowerControl_N::power_limit), lambda(0.f), ref_total(0.f),
		power_total(4.f * cfg_.model.standby), fallback(false), cmd{}, power{}
	{}

	void Update(const float* ref_, const float* speed_)
	{
		Solve(ref_, speed_);
	}

	void SetLimit(const float limit_)
	{
		limit = limit_;
	}

	const PowerControl_N::cfg_s& cfg;
	float limit;
	float lambda;
	float ref_total;
	float power_total;
	bool fallback;
	float cmd[PowerControl_N::wheel_num];
	float power[PowerControl_N::wheel_num];

	private:
	void Solve(const float* ref_, const float* speed_)
	{
		ref_total = TotalPower(ref_, speed_, 0.f);
		if (ref_total <= limit)
		{
			fallback = false;
			lambda = 0.f;
			Commit(ref_, speed_, 0.f);
			return;
		}

		const float zero_ref[PowerControl_N::wheel_num] = {};
		if (limit <= TotalPower(zero_ref, speed_, 0.f))
		{
			fallback = true;
			lambda = 0.f;
			Commit(zero_ref, speed_, 0.f);
			return;
		}

		fallback = !RootSolve(ref_, speed_);
		if (fallback)
		{
			Bisect(ref_, speed_);
		}
	}

	bool RootSolve(const float* ref_, const float* speed_)
	{
		const float scale = limit / ref_total;
		float cand[PowerControl_N::wheel_num];
		float total = 0.f;

		for (uint8_t k = 0; k < PowerControl_N::wheel_num; ++k)
		{
			const float b = cfg.model.torque * speed_[k];
			const float c = cfg.model.friction * speed_[k] * speed_[k] + cfg.model.standby;
			const float p_target = PowerAt(ref_[k], b, speed_[k]) * scale;
			const float disc = b * b - 4.f * cfg.model.copper * (c - p_target);
			if (disc < 0.f) return false;

			const float sq = sqrtf(disc);
			const float root = ref_[k] >= 0.f ? -b + sq : -b - sq;
			cand[k] = Clamp(root / (2.f * cfg.model.copper), cfg.current_max, -cfg.current_max);
			total += PowerAt(cand[k], b, speed_[k]);
		}

		if (total > limit * 1.001f + 1e-3f) return false;

		lambda = 0.f;
		power_total = total;
		for (uint8_t k = 0; k < PowerControl_N::wheel_num; ++k)
		{
			cmd[k] = cand[k];
			power[k] = PowerAt(cand[k], cfg.model.torque * speed_[k], speed_[k]);
		}
		return true;
	}

	void Bisect(const float* ref_, const float* speed_)
	{
		float lo = 0.f;
		float hi = PowerControl_N::lambda_start;
		for (uint8_t k = 0; k < PowerControl_N::expand_iter && TotalPower(ref_, speed_, hi) > limit; ++k)
		{
			lo = hi;
			hi *= PowerControl_N::lambda_expand;
		}
		for (uint8_t k = 0; k < PowerControl_N::bisect_iter; ++k)
		{
			const float mid = 0.5f * (lo + hi);
			if (TotalPower(ref_, speed_, mid) > limit)
			{
				lo = mid;
			}
			else
			{
				hi = mid;
			}
		}

		lambda = hi;
		Commit(ref_, speed_, hi);
	}

	float CurrentAt(const float ref_, const float b_, const float weight_, const float lambda_) const
	{
		const float w = weight_ > PowerControl_N::weight_min ? weight_ : PowerControl_N::weight_min;
		const float i = (2.f * w * ref_ - lambda_ * b_) / (2.f * (w + lambda_ * cfg.model.copper));
		return Clamp(i, cfg.current_max, -cfg.current_max);
	}

	float PowerAt(const float i_, const float b_, const float speed_) const
	{
		return cfg.model.copper * i_ * i_ + b_ * i_ + cfg.model.friction * speed_ * speed_ +
			cfg.model.standby;
	}

	float TotalPower(const float* ref_, const float* speed_, const float lambda_) const
	{
		float total = 0.f;
		for (uint8_t k = 0; k < PowerControl_N::wheel_num; ++k)
		{
			const float b = cfg.model.torque * speed_[k];
			total += PowerAt(CurrentAt(ref_[k], b, cfg.weight[k], lambda_), b, speed_[k]);
		}
		return total;
	}

	void Commit(const float* ref_, const float* speed_, const float lambda_)
	{
		power_total = 0.f;
		for (uint8_t k = 0; k < PowerControl_N::wheel_num; ++k)
		{
			const float b = cfg.model.torque * speed_[k];
			cmd[k] = CurrentAt(ref_[k], b, cfg.weight[k], lambda_);
			power[k] = PowerAt(cmd[k], b, speed_[k]);
			power_total += power[k];
		}
	}

	static float Clamp(const float value_, const float max_, const float min_)
	{
		return value_ > max_ ? max_ : (value_ < min_ ? min_ : value_);
	}
};

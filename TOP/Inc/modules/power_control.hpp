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
	inline constexpr float power_margin = 4.f;
	inline constexpr float blend_power_low = 1.05f;
	inline constexpr float blend_power_high = 1.60f;
	inline constexpr float error_weight_max = 0.85f;

	enum class alloc_e : uint8_t
	{
		scaled,
		big_p
	};

	inline constexpr alloc_e alloc_mode = alloc_e::big_p;

	// p(i, w) = copper * i^2 + torque * i * w（制动侧乘 brake_ratio）+ friction * w^2 + standby
	struct model_cfg_s
	{
		const float copper;		  // W/A^2
		const float torque;		  // N*m/A
		const float friction;	  // W/(rad/s)^2
		const float standby;	  // W
		const float brake_ratio; // 制动回馈系数（<1 表示回馈不足全额）
	};

	inline constexpr model_cfg_s gm3508_model{1.3597e-1f, 3.0842e-1f, 3.098e-3f, 1.8197f, 7.579e-1f};

	struct cfg_s
	{
		const model_cfg_s model;
		const float current_max;
		const float weight[wheel_num];
	};

	inline constexpr cfg_s gm3508_cfg{gm3508_model, 20.f, {1.f, 1.f, 1.f, 1.f}};
}

/* big_p  —— 低功率消耗时按需求逐轮等比，高功率消耗时向大 P 过渡：每轮先算最小功率 base
             与可分配增量 demand，权重 = (1-blend)*demand 占比 + blend*误差占比，
             blend 由 ref_total/limit 在 blend_power_low..blend_power_high 间映射（上限 error_weight_max），
             再把 limit-Σbase 按权重注水分配，逐轮用单调分支求根反解电流；
   scaled —— 按 limit/ref_total 等比缩放功率后逐轮求根，判别式无解时改解二次凸优化。
   两者都保证总功率贴住 budget = limit - power_margin（余量吸收模型误差）；连零电流都超限时直接零电流。
   功率上限统一由 PowerControl_N::power_limit 给出，运行时可用 SetLimit() 修改；
   功率模型只用电机电流与转速，不掺入功率计反馈。 */
class PowerControl
{
	public:
	explicit PowerControl(const PowerControl_N::cfg_s& cfg_)
		: cfg(cfg_), limit(PowerControl_N::power_limit),
		budget(PowerControl_N::power_limit - PowerControl_N::power_margin), lambda(0.f),
		ref_total(0.f), power_total(4.f * cfg_.model.standby), confidence(0.f), fallback(false),
		cmd{}, power{}, err{}
	{}

	void Update(const float* ref_, const float* speed_, const float* err_)
	{
		Solve(ref_, speed_, err_);
	}

	void SetLimit(const float limit_)
	{
		limit = limit_;
	}

	const PowerControl_N::cfg_s& cfg;
	float limit;
	float budget;
	float lambda;
	float ref_total;
	float power_total;
	float confidence;
	bool fallback;
	float cmd[PowerControl_N::wheel_num];
	float power[PowerControl_N::wheel_num];
	float err[PowerControl_N::wheel_num];

	private:
	void Solve(const float* ref_, const float* speed_, const float* err_)
	{
		budget = limit - PowerControl_N::power_margin;
		if (budget < 0.f) budget = 0.f;

		ref_total = TotalPower(ref_, speed_, 0.f);
		if (ref_total <= budget)
		{
			fallback = false;
			lambda = 0.f;
			Commit(ref_, speed_, 0.f);
			return;
		}

		const float zero_ref[PowerControl_N::wheel_num] = {};
		if (budget <= TotalPower(zero_ref, speed_, 0.f))
		{
			fallback = true;
			lambda = 0.f;
			Commit(zero_ref, speed_, 0.f);
			return;
		}

		fallback = false;
		if (PowerControl_N::alloc_mode == PowerControl_N::alloc_e::big_p && BigP(ref_, speed_, err_))
		{
			return;
		}

		fallback = !RootSolve(ref_, speed_);
		if (fallback)
		{
			Bisect(ref_, speed_);
		}
	}

	bool BigP(const float* ref_, const float* speed_, const float* err_)
	{
		const float quad = cfg.model.copper;
		float linear[PowerControl_N::wheel_num];
		float loss[PowerControl_N::wheel_num];
		float dir[PowerControl_N::wheel_num];
		float request[PowerControl_N::wheel_num];
		float x_min[PowerControl_N::wheel_num];
		float base[PowerControl_N::wheel_num];
		float demand[PowerControl_N::wheel_num];
		float weight[PowerControl_N::wheel_num];
		float allocated[PowerControl_N::wheel_num];
		float minimum = 0.f;
		float demand_sum = 0.f;
		float error_sum = 0.f;

		for (uint8_t k = 0; k < PowerControl_N::wheel_num; ++k)
		{
			const float speed = speed_[k];

			dir[k] = ref_[k] < 0.f ? -1.f : 1.f;
			request[k] = fminf(fabsf(ref_[k]), cfg.current_max);
			linear[k] = dir[k] * MechB(ref_[k], speed);
			loss[k] = cfg.model.friction * speed * speed + cfg.model.standby;
			err[k] = err_ != nullptr ? fabsf(err_[k]) : 0.f;

			x_min[k] = 0.f;
			if (linear[k] < 0.f)
			{
				x_min[k] = quad > 0.f ? fminf(request[k], -0.5f * linear[k] / quad) : request[k];
			}

			base[k] = fmaxf(0.f, (quad * x_min[k] + linear[k]) * x_min[k] + loss[k]);
			demand[k] = (quad * request[k] + linear[k]) * request[k] + loss[k] - base[k];
			demand[k] = demand[k] > 0.f ? demand[k] : 0.f;

			minimum += base[k];
			demand_sum += demand[k];
			if (demand[k] > 0.f)
			{
				error_sum += err[k];
			}
		}

		if (demand_sum <= 0.f)
		{
			return false;
		}

		const float demand_ratio = budget > 1.f ? ref_total / budget : PowerControl_N::blend_power_high;
		const float blend = PowerControl_N::error_weight_max *
			Clamp((demand_ratio - PowerControl_N::blend_power_low) /
				(PowerControl_N::blend_power_high - PowerControl_N::blend_power_low), 1.f, 0.f);
		confidence = blend;

		for (uint8_t k = 0; k < PowerControl_N::wheel_num; ++k)
		{
			const float proportional = demand[k] / demand_sum;
			const float big_p = error_sum > 0.f ? err[k] / error_sum : proportional;
			weight[k] = cfg.weight[k] * ((1.f - blend) * proportional + blend * big_p);
		}

		const float alloc_budget = budget - minimum - 1e-3f;
		Allocate(demand, weight, alloc_budget > 0.f ? alloc_budget : 0.f, allocated);

		power_total = 0.f;
		for (uint8_t k = 0; k < PowerControl_N::wheel_num; ++k)
		{
			float x = request[k];
			if (allocated[k] < demand[k])
			{
				x = SolveCurrent(quad, linear[k], loss[k], x_min[k], request[k],
					base[k] + allocated[k]);
			}
			cmd[k] = dir[k] * x;
			power[k] = PowerAt(cmd[k], speed_[k]);
			power_total += power[k];
		}
		lambda = 0.f;
		return true;
	}

	static void Allocate(const float* demand_, const float* weight_, const float budget_, float* out_)
	{
		float leftover = budget_;

		for (uint8_t k = 0; k < PowerControl_N::wheel_num; ++k)
		{
			out_[k] = 0.f;
		}

		for (uint8_t pass = 0; pass < PowerControl_N::wheel_num && leftover > 0.f; ++pass)
		{
			float free_weight = 0.f;
			for (uint8_t k = 0; k < PowerControl_N::wheel_num; ++k)
			{
				if (demand_[k] > out_[k]) free_weight += weight_[k];
			}
			if (free_weight <= 0.f) return;

			const float share = leftover / free_weight;
			leftover = 0.f;
			for (uint8_t k = 0; k < PowerControl_N::wheel_num; ++k)
			{
				const float gap = demand_[k] - out_[k];
				if (gap <= 0.f) continue;

				const float want = weight_[k] * share;
				if (want >= gap)
				{
					out_[k] = demand_[k];
					leftover += want - gap;
				}
				else
				{
					out_[k] += want;
				}
			}
		}
	}

	static float SolveCurrent(const float quad_, const float linear_, const float loss_,
		const float x_min_, const float x_max_, const float target_)
	{
		const float gap = target_ - loss_;
		float x;

		if (quad_ <= 0.f)
		{
			x = linear_ > 0.f ? gap / linear_ : x_max_;
		}
		else if (linear_ >= 0.f)
		{
			x = gap > 0.f ? 2.f * gap / (linear_ + sqrtf(linear_ * linear_ + 4.f * quad_ * gap)) : x_min_;
		}
		else
		{
			const float disc = linear_ * linear_ - 4.f * quad_ * (loss_ - target_);
			x = (-linear_ + sqrtf(disc > 0.f ? disc : 0.f)) / (2.f * quad_);
		}
		return Clamp(x, x_max_, x_min_);
	}

	bool RootSolve(const float* ref_, const float* speed_)
	{
		const float scale = budget / ref_total;
		float cand[PowerControl_N::wheel_num];
		float total = 0.f;

		for (uint8_t k = 0; k < PowerControl_N::wheel_num; ++k)
		{
			const float b = MechB(ref_[k], speed_[k]);
			const float c = cfg.model.friction * speed_[k] * speed_[k] + cfg.model.standby;
			const float p_target = PowerAt(ref_[k], speed_[k]) * scale;
			const float disc = b * b - 4.f * cfg.model.copper * (c - p_target);
			if (disc < 0.f) return false;

			const float sq = sqrtf(disc);
			const float root = ref_[k] >= 0.f ? -b + sq : -b - sq;
			cand[k] = Clamp(root / (2.f * cfg.model.copper), cfg.current_max, -cfg.current_max);
			total += PowerAt(cand[k], speed_[k]);
		}

		if (total > budget * 1.001f + 1e-3f) return false;

		lambda = 0.f;
		power_total = total;
		for (uint8_t k = 0; k < PowerControl_N::wheel_num; ++k)
		{
			cmd[k] = cand[k];
			power[k] = PowerAt(cand[k], speed_[k]);
		}
		return true;
	}

	void Bisect(const float* ref_, const float* speed_)
	{
		float lo = 0.f;
		float hi = PowerControl_N::lambda_start;
		for (uint8_t k = 0; k < PowerControl_N::expand_iter && TotalPower(ref_, speed_, hi) > budget; ++k)
		{
			lo = hi;
			hi *= PowerControl_N::lambda_expand;
		}
		for (uint8_t k = 0; k < PowerControl_N::bisect_iter; ++k)
		{
			const float mid = 0.5f * (lo + hi);
			if (TotalPower(ref_, speed_, mid) > budget)
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

	float MechB(const float ref_, const float speed_) const
	{
		const float b = cfg.model.torque * speed_;
		return ref_ * speed_ > 0.f ? b : b * cfg.model.brake_ratio;
	}

	float CurrentAt(const float ref_, const float speed_, const float weight_, const float lambda_) const
	{
		const float w = weight_ > PowerControl_N::weight_min ? weight_ : PowerControl_N::weight_min;
		const float b = MechB(ref_, speed_);
		const float i = (2.f * w * ref_ - lambda_ * b) / (2.f * (w + lambda_ * cfg.model.copper));
		return Clamp(i, cfg.current_max, -cfg.current_max);
	}

	float PowerAt(const float i_, const float speed_) const
	{
		const float mech = cfg.model.torque * i_ * speed_;
		return cfg.model.copper * i_ * i_ + (mech > 0.f ? mech : mech * cfg.model.brake_ratio) +
			cfg.model.friction * speed_ * speed_ + cfg.model.standby;
	}

	float TotalPower(const float* ref_, const float* speed_, const float lambda_) const
	{
		float total = 0.f;
		for (uint8_t k = 0; k < PowerControl_N::wheel_num; ++k)
		{
			total += PowerAt(CurrentAt(ref_[k], speed_[k], cfg.weight[k], lambda_), speed_[k]);
		}
		return total;
	}

	void Commit(const float* ref_, const float* speed_, const float lambda_)
	{
		power_total = 0.f;
		for (uint8_t k = 0; k < PowerControl_N::wheel_num; ++k)
		{
			cmd[k] = CurrentAt(ref_[k], speed_[k], cfg.weight[k], lambda_);
			power[k] = PowerAt(cmd[k], speed_[k]);
			power_total += power[k];
		}
	}

	static float Clamp(const float value_, const float max_, const float min_)
	{
		return value_ > max_ ? max_ : (value_ < min_ ? min_ : value_);
	}
};

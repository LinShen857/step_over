#pragma once

struct lowpass_cfg_s
{
	const float alpha;
};

struct lowpass_s
{
	float alpha;
	float output;

	explicit lowpass_s(const lowpass_cfg_s& cfg_) : alpha(cfg_.alpha), output(0.f) {}

	float Update(const float input_)
	{
		output += alpha * (input_ - output);
		return output;
	}

	void Clear()
	{
		output = 0.f;
	}
};

#pragma once

#include <cstdint>

#include "can.hpp"
#include "can_device.hpp"

namespace Motor_GM_N
{
	enum class model_e : uint8_t
	{
		gm2006,
		gm3508,
		gm6020
	};

	struct param_factor_s
	{
		const int16_t cmd_max;
		const float bit_to_angle;
		const float rpm_to_rad_s;
		const float bit_to_current;
		const float current_to_torque;
	};

	constexpr float two_pi = 6.28318530718f;
	constexpr float two_pi_over_60 = two_pi / 60.f;

	constexpr param_factor_s MakeFactors(const model_e model)
	{
		switch (model)
		{
			case model_e::gm2006:
				return {10000, two_pi / 8192.f, two_pi_over_60 / 36.f, 10.f / 10000.f, 0.18f};
			case model_e::gm3508:
				return {16384, two_pi / 8192.f, two_pi_over_60 * 17.f / 268.f, 20.f / 16384.f, 0.2463f};
				//xroll减速箱 减速比为 17:268，转矩常数变为 0.3*268/17/(3591/187) = 0.2463
			default:
				return {30000, two_pi / 8192.f, two_pi_over_60, 3.f / 30000.f, 0.741f};
		}
	}

	constexpr uint32_t MakeTxId(const model_e model, const uint32_t id)
	{
		if (model == model_e::gm6020)
		{
			return id <= 0x208 ? 0x1FF : 0x2FF;
		}
		return id <= 0x204 ? 0x200 : 0x1FF;
	}
}

class Motor_GM : public CanDevice
{
	public:
	struct raw_s
	{
		uint16_t angle;
		int16_t speed;
		int16_t current;
	};

	explicit Motor_GM(LibXR::CAN& can_, const Motor_GM_N::model_e model_, const uint32_t id_)
		: CanDevice(Motor_GM_N::MakeTxId(model_, id_), id_),
		can(can_), model(model_), id(id_), facs(Motor_GM_N::MakeFactors(model_)), raw{},
		angle_rad(0.f), speed_rad_s(0.f), torque(0.f), current(0.f), temperature(0),
		cmd(0), rx_cnt(0)
	{}

	void UpdateFeedback(const uint8_t* data_)
	{
		if (__builtin_expect(data_ == nullptr, 0)) return;
		raw.angle = static_cast<uint16_t>((data_[0] << 8 | data_[1]) & 0x1FFF);
		raw.speed = BitToSigned(data_[2], data_[3]);
		raw.current = BitToSigned(data_[4], data_[5]);

		angle_rad = static_cast<float>(raw.angle) * facs.bit_to_angle;
		speed_rad_s = static_cast<float>(raw.speed) * facs.rpm_to_rad_s;
		current = static_cast<float>(raw.current) * facs.bit_to_current;
		torque = current * facs.current_to_torque;
		temperature = static_cast<int8_t>(data_[6]);
		++rx_cnt;
	}

	void OnClassicFrame(const LibXR::CAN::ClassicPack& frame) override
	{
		UpdateFeedback(frame.data);
	}

	void SetCmd(const int16_t cmd_)
	{
		cmd = Clamp(cmd_);
	}

	void SetCmdNorm(const float cmd_)
	{
		const float norm = cmd_ > 1.f ? 1.f : (cmd_ < -1.f ? -1.f : cmd_);
		cmd = Clamp(static_cast<int16_t>(norm * static_cast<float>(facs.cmd_max)));
	}

	float GetSpeedRadS() const { return speed_rad_s; }
	float GetAngleRad() const { return angle_rad; }
	float GetTorqueNm() const { return torque; }
	uint16_t GetRxCount() const { return rx_cnt; }

	void SetCmdCurrent(const float current_)
	{
		cmd = Clamp(static_cast<int16_t>(current_ / facs.bit_to_current));
	}

	template <typename... Motor>
	static void Tx(Motor_GM& first, Motor&... rest)
	{
		LibXR::CAN::ClassicPack pack{};
		pack.id = first.host_id;
		pack.type = LibXR::CAN::Type::STANDARD;
		pack.dlc = 8;

		Write(pack, first);
		(Write(pack, rest), ...);

		first.can.AddMessage(pack);
	}

	protected:
	static int16_t BitToSigned(const uint8_t high_, const uint8_t low_)
	{
		const uint16_t raw = static_cast<uint16_t>((static_cast<uint16_t>(high_) << 8) |
			static_cast<uint16_t>(low_));
		return (raw & 0x8000U) != 0U
			? static_cast<int16_t>(static_cast<int32_t>(raw) - 0x10000)
			: static_cast<int16_t>(raw);
	}

	int16_t Clamp(const int16_t cmd_) const
	{
		return cmd_ > facs.cmd_max ? facs.cmd_max : (cmd_ < -facs.cmd_max ? -facs.cmd_max : cmd_);
	}

	static void Write(LibXR::CAN::ClassicPack& pack_, const Motor_GM& motor_)
	{
		const auto slot = static_cast<uint8_t>((motor_.id - 1) & 0x03);
		const auto value = static_cast<uint16_t>(motor_.cmd);
		pack_.data[slot * 2] = static_cast<uint8_t>(value >> 8);
		pack_.data[slot * 2 + 1] = static_cast<uint8_t>(value);
	}

	LibXR::CAN& can;
	const Motor_GM_N::model_e model;
	const uint32_t id;
	const Motor_GM_N::param_factor_s facs;
	raw_s raw;
	float angle_rad;
	float speed_rad_s;
	float torque;
	float current;
	int8_t temperature;
	int16_t cmd;
	uint16_t rx_cnt;
};

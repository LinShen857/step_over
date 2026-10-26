#pragma once

#include <cstdint>

#include "can.hpp"
#include "can_device.hpp"

namespace Motor_DM_N
{
	enum class error_state_e : uint8_t
	{
		disabled = 0x00,
		enabled = 0x01,
		over_voltage = 0x08,
		under_voltage = 0x09,
		over_current = 0x0A,
		over_temperature_mos = 0x0B,
		over_temperature_rotor = 0x0C,
		comms_lost = 0x0D,
		overload = 0x0E
	};

	enum class order_byte_e : uint8_t
	{
		clear_error = 0xFB,
		enable = 0xFC,
		disable = 0xFD,
		set_zero = 0xFE
	};

	struct param_factor_s
	{
		const float bit_to_pos;
		const float bit_to_vel;
		const float bit_to_torque;
		const float pos_to_bit;
		const float vel_to_bit;
		const float torque_to_bit;
	};

	constexpr param_factor_s MakeFactors(const float pos_max, const float vel_max, const float torque_max)
	{
		return {pos_max / 32767.5f, vel_max / 2047.5f,
			torque_max / 2047.5f, 32767.5f / pos_max,
			2047.5f / vel_max, 2047.5f / torque_max};
	}
}

struct mit_cfg_s
{
	const uint16_t _kp;
	const uint16_t _kd;
};

class Motor_DM : public CanDevice
{
	public:
	struct raw_s
	{
		uint16_t pos;
		uint16_t vel;
		uint16_t torque;
	};

	explicit Motor_DM(uint32_t host_id_, uint32_t motor_id_, const Motor_DM_N::param_factor_s& facs_, LibXR::FDCAN& can_)
		: CanDevice(host_id_, motor_id_),
		error(Motor_DM_N::error_state_e::disabled), raw{},
		pos(0.f), vel(0.f), torque(0.f), zero_offset(0.f),
		temperature_mos(0), temperature_rotor(0),
		facs(facs_), can(can_), tx_cnt(0), rx_cnt(0)
	{}

	void UpdateFeedback(const uint8_t* data_)
    {
        if (__builtin_expect(data_ == nullptr, 0)) return;
        error = static_cast<Motor_DM_N::error_state_e>(data_[0] >> 4);
        raw.pos = static_cast<uint16_t>(data_[1] << 8 | data_[2]);
        raw.vel = static_cast<uint16_t>(data_[3] << 4 | data_[4] >> 4);
        raw.torque = static_cast<uint16_t>((data_[4] & 0xF) << 8 | data_[5]);

        pos = (static_cast<float>(raw.pos) - 32767.f) * facs.bit_to_pos;
        vel = (static_cast<float>(raw.vel) - 2047.f) * facs.bit_to_vel;
        torque = (static_cast<float>(raw.torque) - 2047.f) * facs.bit_to_torque;
        temperature_mos = static_cast<int8_t>(data_[6]);
        temperature_rotor = static_cast<int8_t>(data_[7]);
        ++rx_cnt;
    };

	void OnClassicFrame(const LibXR::CAN::ClassicPack& frame) override
	{
		UpdateFeedback(frame.data);
	}

	void OnFdFrame(const LibXR::FDCAN::FDPack& frame) override
	{
		UpdateFeedback(frame.data);
	}

	void Ctrl_MIT(float pos_, float vel_, uint16_t kp_, uint16_t kd_, float torque_ff_) const
    {
        const auto angle = static_cast<uint16_t>(pos_ * facs.pos_to_bit + 32767.5f);
        const auto speed = static_cast<uint16_t>(vel_ * facs.vel_to_bit + 2047.5f);
        const auto torque_ff = static_cast<uint16_t>(torque_ff_ * facs.torque_to_bit + 2047.5f);
        const uint8_t tx_data[8] {static_cast<uint8_t>(angle >> 8), static_cast<uint8_t>(angle),
            static_cast<uint8_t>(speed >> 4), static_cast<uint8_t>((speed & 0xF) << 4 | (kp_ >> 8 & 0xF)),
            static_cast<uint8_t>(kp_), static_cast<uint8_t>(kd_ >> 4),
            static_cast<uint8_t>((kd_ & 0xF) << 4 | (torque_ff >> 8 & 0xF)), static_cast<uint8_t>(torque_ff)};
        Transmit(tx_data);
        ++tx_cnt;
    };

	void Ctrl_CMD(Motor_DM_N::order_byte_e order_) const
    {
        const uint8_t tx_data[8] {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, static_cast<uint8_t>(order_)};
        Transmit(tx_data);
        ++tx_cnt;
    };

	float GetPosRad() const { return pos; }
	float GetPosRadZeroed() const { return pos - zero_offset; }
	float GetZeroOffset() const { return zero_offset; }
	void SetZeroOffset(const float offset_) { zero_offset = offset_; }
	float GetVelRadS() const { return vel; }
	float GetTorqueNm() const { return torque; }

	protected:
	void Transmit(const uint8_t* tx_data_) const
	{
		LibXR::FDCAN::FDPack fd{};
		fd.id = host_id;
		fd.type = LibXR::FDCAN::Type::STANDARD;
		fd.len = 8;
		for (uint8_t i = 0; i < 8; ++i)
		{
			fd.data[i] = tx_data_[i];
		}
		can.AddMessage(fd);
	}

	Motor_DM_N::error_state_e error;  
	raw_s raw;
	float pos;                        // rad
	float vel;                        // rad/s
	float torque;                     // Nm
	float zero_offset;                // 绝对编码零点（几何零位对应的编码角，rad）
	int8_t temperature_mos;
	int8_t temperature_rotor;
	const Motor_DM_N::param_factor_s& facs;
	LibXR::FDCAN& can;
	mutable uint16_t tx_cnt;  
	uint16_t rx_cnt;
};

inline constexpr uint8_t motor_dm_num = 4;

inline Motor_DM* motor_dm_ptr[motor_dm_num] = {nullptr, nullptr, nullptr, nullptr};

inline void Motor_DM_Attach(const uint8_t index_, Motor_DM& motor_)
{
	if (index_ < motor_dm_num)
	{
		motor_dm_ptr[index_] = &motor_;
	}
}

inline Motor_DM* Motor_DM_Get(const uint8_t index_)
{
	return index_ < motor_dm_num ? motor_dm_ptr[index_] : nullptr;
}
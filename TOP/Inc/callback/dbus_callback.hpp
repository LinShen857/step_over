#pragma once

#include <cstdint>

#include "uart.hpp"

namespace USER_DBUS
{
inline constexpr uint16_t frame_length = 18;
inline constexpr uint16_t rx_buf_length = 36;
inline constexpr uint8_t sw_up = 1;
inline constexpr uint8_t sw_mid = 3;
inline constexpr uint8_t sw_down = 2;
inline constexpr int16_t ch_value_mid = 1024;
inline constexpr uint32_t rx_flag = 1U;

struct rc_ctrl_s
{
	struct
	{
		int16_t ch0;
		int16_t ch1;
		int16_t ch2;
		int16_t ch3;
		int16_t ch4;
		uint8_t s1;
		uint8_t s2;
	} rc;

	struct
	{
		int16_t x;
		int16_t y;
		int16_t z;
		uint8_t press_l;
		uint8_t press_r;
	} mouse;

	struct
	{
		uint16_t v;
	} key;

	uint32_t update_tick;
};

struct rc_status_s
{
	uint32_t rx_event_count;
	uint32_t valid_frame_count;
	uint32_t invalid_frame_count;
	uint32_t error_count;
	uint16_t last_rx_size;
};

void Register(LibXR::UART& uart);
void Process();
rc_ctrl_s Snapshot();
rc_status_s Status();
void SetLinkTimeout(bool timeout);
bool IsLinkTimeout();
bool IsHoldRequested();
bool IsEStopRequested();
}

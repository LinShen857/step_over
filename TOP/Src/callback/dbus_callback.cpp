#include "dbus_callback.hpp"

#include "cmsis_os2.h"
#include "main.h"

extern osThreadId_t remoteCtrlTaskHandle;

namespace USER_DBUS
{
namespace
{
LibXR::UART* uart_ptr = nullptr;
LibXR::Callback<LibXR::ErrorCode> cb_wait;
LibXR::Callback<LibXR::ErrorCode> cb_read;
LibXR::ReadOperation op_wait;
LibXR::ReadOperation op_read;

alignas(32) uint8_t rx_buf[rx_buf_length] {};
volatile size_t rx_len = 0;
volatile bool frame_pending = false;
rc_ctrl_s rc_ctrl {};
rc_status_s rc_status {};
volatile bool link_timeout = false;

uint8_t DecodeFrame(const uint8_t* f)
{
	const uint8_t s1 = static_cast<uint8_t>((f[5] >> 4) & 0x03U);
	const uint8_t s2 = static_cast<uint8_t>((f[5] >> 6) & 0x03U);

	if (s1 == 0U || s2 == 0U)
	{
		++rc_status.invalid_frame_count;
		return 0U;
	}

	rc_ctrl.rc.ch0 = static_cast<int16_t>((f[0] | (f[1] << 8)) & 0x07FF) - ch_value_mid;
	rc_ctrl.rc.ch1 = static_cast<int16_t>(((f[1] >> 3) | (f[2] << 5)) & 0x07FF) - ch_value_mid;
	rc_ctrl.rc.ch2 =
		static_cast<int16_t>(((f[2] >> 6) | (f[3] << 2) | (f[4] << 10)) & 0x07FF) - ch_value_mid;
	rc_ctrl.rc.ch3 = static_cast<int16_t>(((f[4] >> 1) | (f[5] << 7)) & 0x07FF) - ch_value_mid;
	rc_ctrl.rc.ch4 = static_cast<int16_t>((f[16] | (f[17] << 8)) & 0x07FF) - ch_value_mid;

	rc_ctrl.rc.s1 = s1;
	rc_ctrl.rc.s2 = s2;

	rc_ctrl.mouse.x = static_cast<int16_t>(f[6] | (f[7] << 8));
	rc_ctrl.mouse.y = static_cast<int16_t>(f[8] | (f[9] << 8));
	rc_ctrl.mouse.z = static_cast<int16_t>(f[10] | (f[11] << 8));
	rc_ctrl.mouse.press_l = f[12];
	rc_ctrl.mouse.press_r = f[13];

	rc_ctrl.key.v = static_cast<uint16_t>(f[14] | (f[15] << 8));

	rc_ctrl.update_tick = HAL_GetTick();
	++rc_status.valid_frame_count;
	return 1U;
}

void WaitCallback(bool in_isr, LibXR::UART* uart, LibXR::ErrorCode ec)
{
	if (ec != LibXR::ErrorCode::OK)
	{
		++rc_status.error_count;
	}

	const size_t avail = uart->read_port_->Size();
	rx_len = avail > rx_buf_length ? rx_buf_length : avail;
	(void)uart->Read({rx_buf, rx_len}, op_read, in_isr);
}

void ReadCallback(bool in_isr, LibXR::UART*, LibXR::ErrorCode ec)
{
	(void)in_isr;

	if (ec != LibXR::ErrorCode::OK)
	{
		++rc_status.error_count;
	}

	frame_pending = true;
	if (remoteCtrlTaskHandle != nullptr)
	{
		osThreadFlagsSet(remoteCtrlTaskHandle, rx_flag);
	}
}
}

void Register(LibXR::UART& uart)
{
	uart_ptr = &uart;
	cb_wait = LibXR::Callback<LibXR::ErrorCode>::CreateGuarded(WaitCallback, uart_ptr);
	cb_read = LibXR::Callback<LibXR::ErrorCode>::CreateGuarded(ReadCallback, uart_ptr);
	op_wait = LibXR::ReadOperation(cb_wait);
	op_read = LibXR::ReadOperation(cb_read);
	(void)uart.Read({nullptr, 0}, op_wait, false);
}

void Process()
{
	if (!frame_pending) return;
	frame_pending = false;

	const size_t size = rx_len;

	++rc_status.rx_event_count;
	rc_status.last_rx_size = static_cast<uint16_t>(size);

	if (size >= frame_length)
	{
		const size_t offset = size - (size % frame_length) - frame_length;
		(void)DecodeFrame(&rx_buf[offset]);
	}

	if (uart_ptr != nullptr)
	{
		(void)uart_ptr->Read({nullptr, 0}, op_wait, false);
	}
}

rc_ctrl_s Snapshot()
{
	rc_ctrl_s snapshot;
	const uint32_t primask = __get_PRIMASK();
	__disable_irq();
	snapshot = rc_ctrl;
	__set_PRIMASK(primask);
	return snapshot;
}

rc_status_s Status()
{
	rc_status_s status;
	const uint32_t primask = __get_PRIMASK();
	__disable_irq();
	status = rc_status;
	__set_PRIMASK(primask);
	return status;
}

bool IsHoldRequested()
{
	const rc_ctrl_s snapshot = Snapshot();
	return (snapshot.rc.s1 == sw_down) && (snapshot.rc.s2 == sw_down);
}

void SetLinkTimeout(const bool timeout_)
{
	link_timeout = timeout_;
}

bool IsLinkTimeout()
{
	return link_timeout;
}

bool IsEStopRequested()
{
	if (Status().valid_frame_count == 0U) return true;
	return IsHoldRequested() || link_timeout;
}
}

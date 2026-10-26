#include "threads.hpp"

#include "dbus_callback.hpp"

namespace
{
inline constexpr uint32_t rc_timeout_ms = 3000;
inline constexpr uint32_t daemon_period_ms = 10;
}

/* 周期性统计“遥控器是否超过 3 s 没有新帧”，超时写入 USER_DBUS 并唤醒 TaskError；
   连续无新帧按周期累加计数，新帧到达即清零。 */
void TaskDaemon(void *argument)
{
	(void)argument;

	uint32_t last_frame_tick = 0;
	uint32_t stale_ms = 0;
	bool last_timeout = false;

	for (;;)
	{
		const USER_DBUS::rc_ctrl_s rc = USER_DBUS::Snapshot();
		const USER_DBUS::rc_status_s status = USER_DBUS::Status();

		if (rc.update_tick != last_frame_tick)
		{
			last_frame_tick = rc.update_tick;
			stale_ms = 0;
		}
		else if (stale_ms < rc_timeout_ms)
		{
			stale_ms += daemon_period_ms;
		}

		const bool timeout = status.valid_frame_count > 0U && stale_ms >= rc_timeout_ms;
		USER_DBUS::SetLinkTimeout(timeout);

		if (timeout && !last_timeout)
		{
			TaskError_Notify();
		}
		last_timeout = timeout;

		osDelay(daemon_period_ms);
	}
}

#include "threads.hpp"
#include "dbus_callback.hpp"

void TaskRemoteCtrl(void *argument)
{
	(void)argument;

	for (;;)
	{
		osThreadFlagsWait(USER_DBUS::rx_flag, osFlagsWaitAny, osWaitForever);
		USER_DBUS::Process();

		if (USER_DBUS::IsEStopRequested())
		{
			TaskError_Notify();
		}
	}
}

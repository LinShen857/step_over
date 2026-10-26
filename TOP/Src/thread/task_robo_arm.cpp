#include "threads.hpp"

void TaskRoboarm(void *argument)
{
	(void)argument;

	for (;;)
	{
		osDelay(1);
	}
}

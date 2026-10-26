#include "threads.hpp"

void TaskHost(void *argument)
{
	(void)argument;

	for (;;)
	{
		osDelay(1);
	}
}

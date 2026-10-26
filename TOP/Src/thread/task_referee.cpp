#include "threads.hpp"

void TaskReferee(void *argument)
{
	(void)argument;

	for (;;)
	{
		osDelay(1);
	}
}

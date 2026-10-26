#include "threads.hpp"

void Taskwdg(void *argument)
{
	(void)argument;

	for (;;)
	{
		osDelay(1);
	}
}

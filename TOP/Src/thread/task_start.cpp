#include "threads.hpp"
#include "usb_device.h"

#ifdef __cplusplus
extern "C" {
#endif

extern osThreadId_t defaultTaskHandle;

#ifdef __cplusplus
}
#endif

void TaskStart(void *argument)
{
	(void)argument;

	MX_USB_DEVICE_Init();
    app_main();
	osThreadTerminate(defaultTaskHandle);
}

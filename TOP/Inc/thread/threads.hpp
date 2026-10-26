#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "app_main.h"
#include "cmsis_os2.h"

void TaskStart(void *argument);
void TaskError(void *argument);
void TaskError_Notify(void);
void TaskRemoteCtrl(void *argument);
void TaskDaemon(void *argument);
void TaskHost(void *argument);
void Taskwdg(void *argument);
void TaskChassis(void *argument);
void TaskRoboarm(void *argument);
void TaskImu(void *argument);
void TaskReferee(void *argument);

#ifdef __cplusplus
}
#endif

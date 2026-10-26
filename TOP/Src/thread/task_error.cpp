#include "threads.hpp"

#include "dbus_callback.hpp"
#include "modules/chassis.hpp"
#include "motor_dm.hpp"

extern osThreadId_t chassisTaskHandle;
extern osThreadId_t roboarmTaskHandle;
extern osThreadId_t errorTaskHandle;

namespace
{
inline constexpr uint32_t wake_flag = 1U;

void GmZero()
{
	Chassis* chassis = Chassis_Get();
	if (chassis == nullptr) return;

	chassis->wheel_lf.motor.SetCmd(0);
	chassis->wheel_rf.motor.SetCmd(0);
	chassis->wheel_rb.motor.SetCmd(0);
	chassis->wheel_lb.motor.SetCmd(0);
	Motor_GM::Tx(chassis->wheel_lf.motor, chassis->wheel_rf.motor, chassis->wheel_rb.motor,
		chassis->wheel_lb.motor);
}

void DmOrder(const Motor_DM_N::order_byte_e order_)
{
	for (uint8_t i = 0; i < 2; ++i)
	{
		if (Motor_DM* motor = Motor_DM_Get(i); motor != nullptr)
		{
			motor->Ctrl_CMD(order_);
		}
	}
	osDelay(1);

	for (uint8_t i = 2; i < motor_dm_num; ++i)
	{
		if (Motor_DM* motor = Motor_DM_Get(i); motor != nullptr)
		{
			motor->Ctrl_CMD(order_);
		}
	}
	osDelay(1);
}

void Stop()
{
	osThreadSuspend(chassisTaskHandle);
	osThreadSuspend(roboarmTaskHandle);
}

void Release()
{
	if (Chassis* chassis = Chassis_Get(); chassis != nullptr)
	{
		chassis->leg_lb.Resync();
		chassis->leg_rb.Resync();
	}

	DmOrder(Motor_DM_N::order_byte_e::clear_error);
	DmOrder(Motor_DM_N::order_byte_e::enable);
	osThreadResume(chassisTaskHandle);
	osThreadResume(roboarmTaskHandle);
}
}

void TaskError_Notify(void)
{
	if (errorTaskHandle != nullptr)
	{
		osThreadFlagsSet(errorTaskHandle, wake_flag);
	}
}

/* 常规挂起；被唤醒或上电默认进入急停：挂起底盘/机械臂任务并失能电机，
   急停期间持续下发达妙失能指令、并刷新反馈与运动学解算（不发控制帧），条件解除后恢复任务。 */
void TaskError(void *argument)
{
	(void)argument;

	bool holding = false;

	for (;;)
	{
		if (USER_DBUS::IsEStopRequested())
		{
			if (!holding)
			{
				holding = true;
				Stop();
				GmZero();
			}

			DmOrder(Motor_DM_N::order_byte_e::disable);

			if (Chassis* chassis = Chassis_Get(); chassis != nullptr)
			{
				chassis->UpdateFeedback();
			}

			osDelay(1);
		}
		else if (holding)
		{
			holding = false;
			Release();
		}
		else
		{
			osThreadFlagsWait(wake_flag, osFlagsWaitAny, osWaitForever);
		}
	}
}

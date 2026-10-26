#pragma once

#include <cstdint>

#include "can.hpp"
#include "can_device.hpp"

namespace USER_CAN
{

enum class Bus : uint8_t
{
  CAN1,
  CAN2,
  CAN3
};

/// 单条总线的接收订阅开关；默认全订阅。
struct BusConfig
{
  bool extended_frames = true;  ///< 订阅扩展帧（29 位 ID）
  bool fd_frames = true;        ///< 订阅 CAN FD 帧
};

using ErrorInfo = CanErrorInfo;

// Call once per bus after constructing its driver.
void RegisterBus(LibXR::FDCAN& can, Bus bus, const BusConfig& cfg = {});

void Attach(Bus bus, CanDevice& device);
ErrorInfo GetErrorInfo(Bus bus);

// These handlers run in the driver's context, normally an ISR. Do not block or
// retain the frame reference; copy it if a task will process it later.
void OnClassicFrame(Bus bus, bool in_isr, const LibXR::CAN::ClassicPack& frame);
void OnFdFrame(Bus bus, bool in_isr, const LibXR::FDCAN::FDPack& frame);

}  // namespace USR_CAN

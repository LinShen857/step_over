#pragma once

#include <cstdint>

#include "can.hpp"

struct CanErrorInfo
{
  uint32_t count = 0;  ///< 错误状态事件累计（TEC/REC 越过 96/128 或 BusOff）
  LibXR::CAN::ErrorID last =
      LibXR::CAN::ErrorID::CAN_ERROR_ID_GENERIC;  ///< 最近一次错误事件
  LibXR::CAN::ErrorState state {};                ///< TEC/REC/BO/EP/EW 快照
};

class CanDevice
{
 public:
  CanDevice(uint32_t host_id_, uint32_t motor_id_)
      : host_id(host_id_), motor_id(motor_id_)
  {
  }

  virtual ~CanDevice() = default;

  virtual void OnClassicFrame(const LibXR::CAN::ClassicPack& frame) = 0;

  virtual void OnFdFrame(const LibXR::FDCAN::FDPack& frame)
  {
    (void)frame;
  }

  
  void SetBusError(const CanErrorInfo& info) { can_error = info; }

  const uint32_t host_id;
  const uint32_t motor_id;
  CanErrorInfo can_error;  
};

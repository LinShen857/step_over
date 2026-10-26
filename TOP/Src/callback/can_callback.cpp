#include "can_callback.hpp"

namespace USER_CAN
{
namespace
{
constexpr uint8_t BUS_NUM = 3;
constexpr uint8_t MAX_DEVICES_PER_BUS = 16;

struct DeviceEntry
{
  CanDevice* device;
  uint32_t id;
};

DeviceEntry device_table[BUS_NUM][MAX_DEVICES_PER_BUS] {};
LibXR::FDCAN* bus_driver[BUS_NUM] = {nullptr, nullptr, nullptr};
ErrorInfo bus_error[BUS_NUM] {};

void HandleError(Bus bus, const LibXR::CAN::ClassicPack& frame)
{
  const auto index = static_cast<uint8_t>(bus);
  ++bus_error[index].count;
  bus_error[index].last = LibXR::CAN::ToErrorID(frame.id);
  if (bus_driver[index] != nullptr)
  {
    bus_driver[index]->GetErrorState(bus_error[index].state);
  }

  for (auto& entry : device_table[index])
  {
    if (entry.device != nullptr)
    {
      entry.device->SetBusError(bus_error[index]);
    }
  }
}

CanDevice* FindDevice(Bus bus, uint32_t id)
{
  for (auto& entry : device_table[static_cast<uint8_t>(bus)])
  {
    if (entry.device != nullptr && entry.id == id)
    {
      return entry.device;
    }
  }
  return nullptr;
}

void ClassicCallback(bool in_isr, Bus bus, const LibXR::CAN::ClassicPack& frame)
{
  OnClassicFrame(bus, in_isr, frame);
}

void FdCallback(bool in_isr, Bus bus, const LibXR::FDCAN::FDPack& frame)
{
  OnFdFrame(bus, in_isr, frame);
}

// 经典帧与 FD 帧的回调句柄形状相同，交给 can.Register 自己选重载。
template <typename CallbackT>
void RegisterFrames(LibXR::FDCAN& can, const CallbackT& cb, bool extended_frames)
{
  can.Register(cb, LibXR::CAN::Type::STANDARD,
               LibXR::CAN::FilterMode::ID_RANGE, 0, 0x7FF);
  if (extended_frames)
  {
    can.Register(cb, LibXR::CAN::Type::EXTENDED,
                 LibXR::CAN::FilterMode::ID_RANGE, 0, 0x1FFFFFFF);
  }
}
} 

void RegisterBus(LibXR::FDCAN& can, Bus bus, const BusConfig& cfg)
{
  bus_driver[static_cast<uint8_t>(bus)] = &can;

  const auto classic = LibXR::CAN::Callback::Create(ClassicCallback, bus);
  RegisterFrames(can, classic, cfg.extended_frames);
  can.Register(classic, LibXR::CAN::Type::ERROR,
               LibXR::CAN::FilterMode::ID_RANGE, 0, UINT32_MAX);

  if (cfg.fd_frames)
  {
    const auto fd = LibXR::FDCAN::CallbackFD::Create(FdCallback, bus);
    RegisterFrames(can, fd, cfg.extended_frames);
  }
}

void Attach(Bus bus, CanDevice& device)
{
  for (auto& entry : device_table[static_cast<uint8_t>(bus)])
  {
    if (entry.device == nullptr)
    {
      entry.device = &device;
      entry.id = device.motor_id;
      return;
    }
  }
}

ErrorInfo GetErrorInfo(Bus bus)
{
  return bus_error[static_cast<uint8_t>(bus)];
}

void OnClassicFrame(Bus bus, bool in_isr, const LibXR::CAN::ClassicPack& frame)
{
  (void)in_isr;
  if (frame.type == LibXR::CAN::Type::ERROR)
  {
    HandleError(bus, frame);
    return;
  }
  if (CanDevice* device = FindDevice(bus, frame.id); device != nullptr)
  {
    device->OnClassicFrame(frame);
  }
}

void OnFdFrame(Bus bus, bool in_isr, const LibXR::FDCAN::FDPack& frame)
{
  (void)in_isr;
  if (CanDevice* device = FindDevice(bus, frame.id); device != nullptr)
  {
    device->OnFdFrame(frame);
  }
}

}  // namespace USR_CAN

#pragma once

#include "can.hpp"
#include "can_device.hpp"

class PowerMeter : public CanDevice
{
    public:
            explicit PowerMeter(uint32_t host_id_, uint32_t motor_id_)
                : CanDevice(host_id_, motor_id_) {}

            float voltage;
            float current;
            float power;
            uint16_t rx_cnt = 0;

        void OnClassicFrame(const LibXR::CAN::ClassicPack& frame) override
        {
            UpdateFeedback(frame.data);
        }

        void UpdateFeedback(const uint8_t* rxdata)
        {
            voltage = (float)((int16_t)(((uint16_t)rxdata[1] << 8) | (uint16_t)rxdata[0])) / 100.0f;
            current = (float)((int16_t)(((uint16_t)rxdata[3] << 8) | (uint16_t)rxdata[2])) / 100.0f;
            power = voltage * current;
            ++rx_cnt;
        }
};

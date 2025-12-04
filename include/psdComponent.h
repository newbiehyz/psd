// 自动生成的配置代码，请勿手动修改
// 生成时间: 2025-10-28 11:05:25
// PACOS配置版本: v0.2-20250916
// 接口列表版本: V1.2_20250919
// 生成工具版本: v0.2.1 (2025-10-16)

#pragma once

#include <iostream>
#include "DataType.h"
#include "core/ComponentBase.h"

namespace apa {

class psdComponent : public pacos::core::ComponentBase {

public:
    psdComponent(std::string name);
    ~psdComponent() = default;

    void initialize() override;

private:
    void Timer50_1TimerTask();
    void UicSelectSlotTriggerCallback(const uint8_t* data, uint32_t length);
    void UicSelectCustomSlotTriggerCallback(const uint8_t* data, uint32_t length);
    void publishDataToapapsdparking_slot_stopper(apa::psd::ParkingSlotStopper* sendData);
    void publishDataToapapsdparking_slot_list(apa::psd::ParkingSlotList* sendData);
    void publishDataToapapsdtarget_slot_label(apa::psd::TargetSlotLabel* sendData);
    void publishDataToapapsduss_slot_label(apa::psd::UssSlotLabel* sendData);
};

} // namespace apa

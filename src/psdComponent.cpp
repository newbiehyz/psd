// 自动生成的配置代码，请勿手动修改
// 生成时间: 2025-10-28 11:05:25
// PACOS配置版本: v0.2-20250916
// 接口列表版本: V1.2_20250919
// 生成工具版本: v0.2.1 (2025-10-16)

#include <iostream>
#include "logger.h"
#include "psdComponent.h"

#include "psd/swc_process/psd_fusion_process.h"

static patac::psd::PsdFusionProcess s_psd_fusion_processor;

namespace apa {

psdComponent::psdComponent(std::string name)
    : pacos::core::ComponentBase(name) {
  
  s_psd_fusion_processor.Initialize("");
}

void psdComponent::initialize() {
    registerTimerTaskCallback("Timer50_1", std::bind(&psdComponent::Timer50_1TimerTask, this));
    registerTriggerTaskCallback("UicSelectSlot", std::bind(&psdComponent::UicSelectSlotTriggerCallback, this, std::placeholders::_1, std::placeholders::_2));
    registerTriggerTaskCallback("UicSelectCustomSlot", std::bind(&psdComponent::UicSelectCustomSlotTriggerCallback, this, std::placeholders::_1, std::placeholders::_2));
}

void psdComponent::Timer50_1TimerTask() {
    LOG_DEBUG() << "psdComponent: Executing timer task (50ms)";

    // input params
    apa::status::Output2ALL status;
    apa::loc::Vehicle_Pose vehicle_pose;
    apa::loc::MapInfo map_info;

    uss::UssSlotList uss_slot_list;
    apa::fus::EmapGridmap gridmap;
    apa::fus::PkEmapObs emap_obs;

    apa::planning::DecPlan2Psd parkable_slot;
    apa::planning::Plan2Status pnc_status;

    // output params
    apa::psd::ParkingSlotList slot_list;
    apa::psd::ParkingSlotStopper slot_stopper;
    apa::psd::TargetSlotLabel target_slot_label;

    {
        const std::vector<uint8_t>* cached_data = nullptr;
        getLatestCacheData("/apa/status/output_ALL", cached_data);
        if (cached_data != nullptr && cached_data->size() > 0) {
        status.deserialize_from_memory(cached_data->data(), cached_data->data() + cached_data->size());
        LOG_DEBUG() << "[/apa/status/output_ALL: (pub_timestamp, sequence, apa_status)] = (" 
                    << status.header.pub_timestamp_us << ", " 
                    << status.header.sequence << ", "
                    << static_cast<int>(status.apaStatusReq) << ", "
                    << static_cast<int>(status.DrivingOrParking1) << ", "
                    << static_cast<int>(status.apaStartParkingReq)
                    << ")";
        }
    }

    {
        const std::vector<uint8_t>* cached_data = nullptr;
        getLatestCacheData("/apa/loc/vehicle_pose", cached_data);
        if (cached_data != nullptr && cached_data->size() > 0) {
        vehicle_pose.deserialize_from_memory(cached_data->data(), cached_data->data() + cached_data->size());
        LOG_DEBUG() << "[/apa/loc/vehicle_pose: (pub_timestamp, sequence, x, y, yaw, timestamp_us)] = (" 
                    << vehicle_pose.header.pub_timestamp_us << ", " 
                    << vehicle_pose.header.sequence << ", "
                    << vehicle_pose.pos_x << ", "
                    << vehicle_pose.pos_y << ", "
                    << vehicle_pose.yaw << ", "
                    << vehicle_pose.timeStamp_us
                    << ")";
        }
    }

    {
        const std::vector<uint8_t>* cached_data = nullptr;
        getLatestCacheData("/apa/loc/map_info", cached_data);
        if (cached_data != nullptr && cached_data->size() > 0) {
        map_info.deserialize_from_memory(cached_data->data(), cached_data->data() + cached_data->size());
        LOG_DEBUG() << "[/apa/loc/map_info: (pub_timestamp, sequence, mapId, ParkingSlot_num)] = (" 
                    << map_info.header.pub_timestamp_us << ", " 
                    << map_info.header.sequence << ", "
                    << map_info.mapId << ", "
                    << map_info.ParkingSlot.size() << ", "
                    << ")";
        }
    }

    {
        const std::vector<uint8_t>* cached_data = nullptr;
        getLatestCacheData("/apa/uss/uss_parking_slot_list", cached_data);
        if (cached_data != nullptr && cached_data->size() > 0) {
        uss_slot_list.deserialize_from_memory(cached_data->data(), cached_data->data() + cached_data->size());
        LOG_DEBUG() << "[/apa/uss/uss_parking_slot_list: received, slot0_timestamp = " 
                    << uss_slot_list.uss_slot[0].timestamp << "]";
        }
    }

    {
        const std::vector<uint8_t>* cached_data = nullptr;
        getLatestCacheData("/apa/fus/gridmap", cached_data);
        if (cached_data != nullptr && cached_data->size() > 0) {
        gridmap.deserialize_from_memory(cached_data->data(), cached_data->data() + cached_data->size());
        LOG_DEBUG() << "[/apa/fus/gridmap: (pub_timestamp, sequence, EmapGrid_TimeStamp, eMapWorkMode)] = (" 
                    << gridmap.header.pub_timestamp_us << ", " 
                    << gridmap.header.sequence << ", "
                    << gridmap.EmapGrid_TimeStamp << ", "
                    << static_cast<int>(gridmap.eMapWorkMode)
                    << ")";
        }
    }

    {
        const std::vector<uint8_t>* cached_data = nullptr;
        getLatestCacheData("/apa/fus/obs", cached_data);
        if (cached_data != nullptr && !cached_data->empty()) {
        emap_obs.deserialize_from_memory(cached_data->data(), cached_data->data() + cached_data->size());
        LOG_DEBUG() << "[/apa/fus/obs: (pub_timestamp, sequence, first_obs_ID, first_obs_Type)] = (" 
                    << emap_obs.header.pub_timestamp_us << ", " 
                    << emap_obs.header.sequence << ", "
                    << emap_obs.pkEmapObs[0].obsID << ", "
                    << static_cast<int>(emap_obs.pkEmapObs[0].obsTyp)
                    << ")";
        }
    }

    {
        const std::vector<uint8_t>* cached_data = nullptr;
        getLatestCacheData("/apa/planning/parkable_slot", cached_data);
        if (cached_data != nullptr && !cached_data->empty()) {
        parkable_slot.deserialize_from_memory(cached_data->data(), cached_data->data() + cached_data->size());
        LOG_DEBUG() << "[/apa/planning/parkable_slot: (pub_timestamp, sequence, frameID, validSlotNum, SlotID, slotParkable)] = (" 
                    << parkable_slot.header.pub_timestamp_us << ", " 
                    << parkable_slot.header.sequence << ", "
                    << parkable_slot.frameID << ", "
                    << static_cast<int>(parkable_slot.validSlotNum) << ", "
                    << parkable_slot.emapSrchSlotsRes.SlotID << ", "
                    << static_cast<int>(parkable_slot.emapSrchSlotsRes.slotParkable)
                    << ")";
        }
    }

    {
        const std::vector<uint8_t>* cached_data = nullptr;
        getLatestCacheData("/apa/planning/pnc_status", cached_data);
        if (cached_data != nullptr && !cached_data->empty()) {
            pnc_status.deserialize_from_memory(cached_data->data(), cached_data->data() + cached_data->size());
            LOG_DEBUG() << "[/apa/planning/pnc_status: (pub_timestamp, sequence, soltWidthStatus, PncStatus)] = (" 
                        << pnc_status.header.pub_timestamp_us << ", " 
                        << pnc_status.header.sequence << ", "
                        << static_cast<int>(pnc_status.soltWidthStatus) << ", "
                        << static_cast<int>(pnc_status.PncStatus)
                        << ")";
        }
    }


    s_psd_fusion_processor.RunTask(status,
                                   vehicle_pose, map_info,
                                   uss_slot_list,
                                   gridmap, emap_obs,
                                   parkable_slot, pnc_status,
                                   slot_list, slot_stopper, target_slot_label
                                );
}

void psdComponent::UicSelectSlotTriggerCallback(const uint8_t* data, uint32_t length){
    if(data == nullptr || length == 0)
    {
        return;
    }

    apa::uic::SelectParkingSlot receivedData;
    receivedData.deserialize_from_memory(data, data + length);

    // Add processing code here
    s_psd_fusion_processor.UpdateSelectSlot(receivedData);
}

void psdComponent::UicSelectCustomSlotTriggerCallback(const uint8_t* data, uint32_t length){
    if(data == nullptr || length == 0)
    {
        return;
    }

    apa::uic::CustomizeParkingSlot receivedData;
    receivedData.deserialize_from_memory(data, data + length);

    // Add processing code here
    s_psd_fusion_processor.UpdateCustomSlot(receivedData);
}

void psdComponent::publishDataToapapsdparking_slot_stopper(apa::psd::ParkingSlotStopper* sendData){
    publishDataToCom("/apa/psd/parking_slot_stopper", sendData);
}

void psdComponent::publishDataToapapsdparking_slot_list(apa::psd::ParkingSlotList* sendData){
    publishDataToCom("/apa/psd/parking_slot_list", sendData);
}

void psdComponent::publishDataToapapsdtarget_slot_label(apa::psd::TargetSlotLabel* sendData){
    publishDataToCom("/apa/psd/target_slot_label", sendData);
}

void psdComponent::publishDataToapapsduss_slot_label(apa::psd::UssSlotLabel* sendData){
    publishDataToCom("/apa/psd/uss_slot_label", sendData);
}


} // namespace apa

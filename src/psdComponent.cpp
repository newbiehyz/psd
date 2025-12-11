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
            LOG_DEBUG() << "[RAW INPUT - /apa/status/output_ALL]"
                        << " header.flag=" << status.header.flag
                        << ", header.version=" << status.header.version
                        << ", header.pub_timestamp_us=" << status.header.pub_timestamp_us
                        << ", header.sequence=" << status.header.sequence
                        << ", apaStatusReq=" << static_cast<int>(status.apaStatusReq)
                        << ", hppStatusReq=" << static_cast<int>(status.hppStatusReq)
                        << ", DrivingOrParking1=" << static_cast<int>(status.DrivingOrParking1)
                        << ", DrivingOrParking2=" << static_cast<int>(status.DrivingOrParking2)
                        << ", apaStartParkingReq=" << static_cast<int>(status.apaStartParkingReq)
                        << ", apaSrchInterupt=" << static_cast<int>(status.apaSrchInterupt)
                        << ", Reserved1=" << static_cast<int>(status.Reserved1)
                        << ", Reserved2=" << static_cast<int>(status.Reserved2);
        }
    }

    {
        const std::vector<uint8_t>* cached_data = nullptr;
        getLatestCacheData("/apa/loc/vehicle_pose", cached_data);
        if (cached_data != nullptr && cached_data->size() > 0) {
            vehicle_pose.deserialize_from_memory(cached_data->data(), cached_data->data() + cached_data->size());
            LOG_DEBUG() << "[RAW INPUT - /apa/loc/vehicle_pose]"
                        << " header.flag=" << vehicle_pose.header.flag
                        << ", header.version=" << vehicle_pose.header.version
                        << ", header.pub_timestamp_us=" << vehicle_pose.header.pub_timestamp_us
                        << ", header.sequence=" << vehicle_pose.header.sequence
                        << ", loc_status=" << static_cast<int>(vehicle_pose.loc_status)
                        << ", pos_x=" << vehicle_pose.pos_x
                        << ", pos_y=" << vehicle_pose.pos_y
                        << ", pos_z=" << vehicle_pose.pos_z
                        << ", quaternion_qx=" << vehicle_pose.quaternion_qx
                        << ", quaternion_qy=" << vehicle_pose.quaternion_qy
                        << ", quaternion_qz=" << vehicle_pose.quaternion_qz
                        << ", quaternion_qw=" << vehicle_pose.quaternion_qw
                        << ", roll=" << vehicle_pose.roll
                        << ", pitch=" << vehicle_pose.pitch
                        << ", yaw=" << vehicle_pose.yaw
                        << ", timeStamp_us=" << vehicle_pose.timeStamp_us;
        }
    }

    {
        const std::vector<uint8_t>* cached_data = nullptr;
        getLatestCacheData("/apa/loc/map_info", cached_data);
        if (cached_data != nullptr && cached_data->size() > 0) {
            map_info.deserialize_from_memory(cached_data->data(), cached_data->data() + cached_data->size());
            LOG_DEBUG() << "[RAW INPUT - /apa/loc/map_info]"
                        << " header.flag=" << map_info.header.flag
                        << ", header.version=" << map_info.header.version
                        << ", header.pub_timestamp_us=" << map_info.header.pub_timestamp_us
                        << ", header.sequence=" << map_info.header.sequence
                        << ", timeStamp=" << map_info.timeStamp
                        << ", mapId=" << map_info.mapId
                        << ", ParkingSlot.size=" << map_info.ParkingSlot.size();
            
            for (size_t i = 0; i < map_info.ParkingSlot.size(); ++i) {
                const auto& slot = map_info.ParkingSlot[i];
                LOG_DEBUG() << "[RAW INPUT - /apa/loc/map_info] ParkingSlot[" << i << "]:"
                            << " id=" << slot.id
                            << ", psType=" << slot.psType
                            << ", width=" << slot.width
                            << ", length=" << slot.length
                            << ", center=(" << slot.center.x << "," << slot.center.y << "," << slot.center.z << ")"
                            << ", wideDirection=(" << slot.wideDirection.x << "," << slot.wideDirection.y << "," << slot.wideDirection.z << ")"
                            << ", longDirection=(" << slot.longDirection.x << "," << slot.longDirection.y << "," << slot.longDirection.z << ")"
                            << ", isOccupancy=" << slot.isOccupancy;
            }
        }
    }

    {
        const std::vector<uint8_t>* cached_data = nullptr;
        getLatestCacheData("/apa/uss/uss_parking_slot_list", cached_data);
        if (cached_data != nullptr && cached_data->size() > 0) {
            uss_slot_list.deserialize_from_memory(cached_data->data(), cached_data->data() + cached_data->size());

            for (size_t i = 0; i < 4; ++i) {
                const auto& slot = uss_slot_list.uss_slot[i];
                LOG_DEBUG() << "[RAW INPUT - /apa/uss/uss_parking_slot_list] uss_slot[" << i << "]:"
                            << " point_ABCD[0]=(" << slot.point_ABCD[0].x << "," << slot.point_ABCD[0].y << ")"
                            << ", point_ABCD[1]=(" << slot.point_ABCD[1].x << "," << slot.point_ABCD[1].y << ")"
                            << ", point_ABCD[2]=(" << slot.point_ABCD[2].x << "," << slot.point_ABCD[2].y << ")"
                            << ", point_ABCD[3]=(" << slot.point_ABCD[3].x << "," << slot.point_ABCD[3].y << ")"
                            << ", bottom_type=" << static_cast<int>(slot.bottom_type)
                            << ", obs_status=" << static_cast<int>(slot.obs_status)
                            << ", leftside_obstacle_position=" << slot.leftside_obstacle_position
                            << ", rightside_obstacle_position=" << slot.rightside_obstacle_position
                            << ", timestamp=" << slot.timestamp;
            }
        }
    }

    {
        const std::vector<uint8_t>* cached_data = nullptr;
        getLatestCacheData("/apa/fus/gridmap", cached_data);
        if (cached_data != nullptr && cached_data->size() > 0) {
            gridmap.deserialize_from_memory(cached_data->data(), cached_data->data() + cached_data->size());
            LOG_DEBUG() << "[RAW INPUT - /apa/fus/gridmap]"
                        << " header.flag=" << gridmap.header.flag
                        << ", header.version=" << gridmap.header.version
                        << ", header.pub_timestamp_us=" << gridmap.header.pub_timestamp_us
                        << ", header.sequence=" << gridmap.header.sequence
                        << ", EmapGrid_TimeStamp=" << gridmap.EmapGrid_TimeStamp
                        << ", eMapWorkMode=" << static_cast<int>(gridmap.eMapWorkMode);
        }
    }

    {
        const std::vector<uint8_t>* cached_data = nullptr;
        getLatestCacheData("/apa/fus/obs", cached_data);
        if (cached_data != nullptr && !cached_data->empty()) {
            emap_obs.deserialize_from_memory(cached_data->data(), cached_data->data() + cached_data->size());
            LOG_DEBUG() << "[RAW INPUT - /apa/fus/obs]"
                        << " header.flag=" << emap_obs.header.flag
                        << ", header.version=" << emap_obs.header.version
                        << ", header.pub_timestamp_us=" << emap_obs.header.pub_timestamp_us
                        << ", header.sequence=" << emap_obs.header.sequence;
            
            for (size_t i = 0; i < 50; ++i) {
                const auto& obs = emap_obs.pkEmapObs[i];
                if (obs.obsTyp != apa::fus::EmapObsTyp::OBS_NULL) {
                    LOG_DEBUG() << "[RAW INPUT - /apa/fus/obs] pkEmapObs[" << i << "]:"
                                << " Frame_Index=" << obs.Frame_Index
                                << ", Fail_Safe=" << static_cast<int>(obs.Fail_Safe)
                                << ", obsID=" << obs.obsID
                                << ", obsTyp=" << static_cast<int>(obs.obsTyp)
                                << ", obsCenter=(" << obs.obsCenter.x << "," << obs.obsCenter.y << "," << obs.obsCenter.z << ")"
                                << ", age=" << obs.age
                                << ", obsDirection=(" << obs.obsDirection.x << "," << obs.obsDirection.y << "," << obs.obsDirection.z << ")"
                                << ", obsBdTyp=" << static_cast<int>(obs.obsBdTyp)
                                << ", obsConf=" << static_cast<int>(obs.obsConf)
                                << ", obsMotionInfo.isValid=" << obs.obsMotionInfo.isValid
                                << ", obsTrajectory.motionStatus=" << static_cast<int>(obs.obsTrajectory.motionStatus)
                                << ", obsTrajectory.confidence=" << obs.obsTrajectory.confidence;
                }
            }
        }
    }

    {
        const std::vector<uint8_t>* cached_data = nullptr;
        getLatestCacheData("/apa/planning/parkable_slot", cached_data);
        if (cached_data != nullptr && !cached_data->empty()) {
            parkable_slot.deserialize_from_memory(cached_data->data(), cached_data->data() + cached_data->size());
            LOG_DEBUG() << "[RAW INPUT - /apa/planning/parkable_slot]"
                        << " header.flag=" << parkable_slot.header.flag
                        << ", header.version=" << parkable_slot.header.version
                        << ", header.pub_timestamp_us=" << parkable_slot.header.pub_timestamp_us
                        << ", header.sequence=" << parkable_slot.header.sequence
                        << ", frameID=" << parkable_slot.frameID
                        << ", validSlotNum=" << static_cast<int>(parkable_slot.validSlotNum)
                        << ", emapSrchSlotsRes.SlotID=" << parkable_slot.emapSrchSlotsRes.SlotID
                        << ", emapSrchSlotsRes.slotParkable=" << static_cast<int>(parkable_slot.emapSrchSlotsRes.slotParkable)
                        << ", emapSrchSlotsRes.slotScore=" << static_cast<int>(parkable_slot.emapSrchSlotsRes.slotScore);
        }
    }

    {
        const std::vector<uint8_t>* cached_data = nullptr;
        getLatestCacheData("/apa/planning/pnc_status", cached_data);
        if (cached_data != nullptr && !cached_data->empty()) {
            pnc_status.deserialize_from_memory(cached_data->data(), cached_data->data() + cached_data->size());
            LOG_DEBUG() << "[RAW INPUT - /apa/planning/pnc_status]"
                        << " header.flag=" << pnc_status.header.flag
                        << ", header.version=" << pnc_status.header.version
                        << ", header.pub_timestamp_us=" << pnc_status.header.pub_timestamp_us
                        << ", header.sequence=" << pnc_status.header.sequence
                        << ", soltWidthStatus=" << static_cast<int>(pnc_status.soltWidthStatus)
                        << ", PncStatus=" << static_cast<int>(pnc_status.PncStatus);
        }
    }


    s_psd_fusion_processor.RunTask(status,
                                   vehicle_pose, map_info,
                                   uss_slot_list,
                                   gridmap, emap_obs,
                                   parkable_slot, pnc_status,
                                   slot_list, slot_stopper, target_slot_label
                                );
    
    publishDataToapapsdparking_slot_list(&slot_list);
    publishDataToapapsdparking_slot_stopper(&slot_stopper);
    publishDataToapapsdtarget_slot_label(&target_slot_label);
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

    LOG_INFO() << "Publish /apa/psd/parking_slot_list(sequence, slot_num): "
               << sendData->header.sequence << ", "
               << sendData->slot_num;
    for (auto item : sendData->slots) {
        if (item.slot_id == 1006) {
            LOG_INFO() << "A(" << item.slot_corners[0].x << ", " << item.slot_corners[0].y << "), "
                       << "B(" << item.slot_corners[1].x << ", " << item.slot_corners[1].y << "), "
                       << "C(" << item.slot_corners[2].x << ", " << item.slot_corners[2].y << "), "
                       << "D(" << item.slot_corners[3].x << ", " << item.slot_corners[3].y << "). ";
        }
    }
}

void psdComponent::publishDataToapapsdtarget_slot_label(apa::psd::TargetSlotLabel* sendData){
    publishDataToCom("/apa/psd/target_slot_label", sendData);
}

void psdComponent::publishDataToapapsduss_slot_label(apa::psd::UssSlotLabel* sendData){
    publishDataToCom("/apa/psd/uss_slot_label", sendData);
}


} // namespace apa

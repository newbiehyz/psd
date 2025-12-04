#include "psd_fusion_process.h"

#include "log.h"

namespace patac {
namespace psd {

bool PsdFusionProcess::Initialize(const std::string& cfg_path) {
  LOG_DEBUG() << "Call PsdFusionProcess::Initilaize" << std::endl;

  return false;
}

bool PsdFusionProcess::UpdateSelectSlot(const apa::uic::SelectParkingSlot& select_slot) {
  LOG_DEBUG() << "Call PsdFusionProcess::UpdateSelectSlot" << std::endl;
  return false;
};

bool PsdFusionProcess::UpdateCustomSlot(const apa::uic::CustomizeParkingSlot& custom_slot) {
  LOG_DEBUG() << "Call PsdFusionProcess::UpdateSelectSlot" << std::endl;
  return false;
};

bool PsdFusionProcess::RunTask(const apa::status::Output2ALL& state_machine,
                               const apa::loc::Vehicle_Pose& vehicle_pose,
                               const apa::loc::MapInfo& map_info,
                               const uss::UssSlotList& uss_slot_list,
                               const apa::fus::EmapGridmap& gridmap,
                               const apa::fus::PkEmapObs& emap_obs,
                               const apa::planning::DecPlan2Psd& parkable_slot,
                               const apa::planning::Plan2Status& pnc_status,
                               apa::psd::ParkingSlotList& output_slot_list,
                               apa::psd::ParkingSlotStopper& output_slot_stopper,
                               apa::psd::TargetSlotLabel& output_target_label
) {
  LOG_DEBUG() << "Call PsdFusionProcess::RunTask" << std::endl;

    // Timing measurement
    auto start_time = std::chrono::steady_clock::now();

    // Get current timestamp
    auto now = std::chrono::system_clock::now();
    auto now_epoch = now.time_since_epoch();
    uint64_t current_timestamp_us = static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::microseconds>(now_epoch).count());


    //=========================================================================
    // Step 1: Convert all inputs to internal types
    //=========================================================================
    LOG_DEBUG() << "PsdFusionProcess::RunTask - Converting inputs";

    PsdInputConverter::ConvertAllInputs(
        state_machine,
        vehicle_pose,
        map_info,
        uss_slot_list,
        gridmap,
        emap_obs,
        parkable_slot,
        pnc_status,
        current_timestamp_us,
        cached_input_);

    // Log input summary
    LOG_DEBUG() << "PsdFusionProcess::RunTask - Input Summary:"
                << " apaStatus=" << static_cast<int>(cached_input_.stateMachine.apaStatusReq)
                << ", vehPose=(" << cached_input_.vehiclePose.posX 
                << "," << cached_input_.vehiclePose.posY
                << "," << cached_input_.vehiclePose.yaw << ")"
                << ", mapSlots=" << cached_input_.mapInfo.parkingSlotCount;













    // Timing measurement
    auto end_time = std::chrono::steady_clock::now();
    auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        end_time - start_time).count();

    LOG_DEBUG() << "PsdFusionProcess::RunTask - Completed in " << elapsed_ms << "ms";


  return false;
}

bool PsdFusionProcess::Reset() {
  return false;
}

}
}

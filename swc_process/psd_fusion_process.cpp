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
  LOG_DEBUG() << "Call PsdFusionProcess::RunTask\n";

  auto start_time = std::chrono::steady_clock::now();
  auto now = std::chrono::system_clock::now();
  auto now_epoch = now.time_since_epoch();
  uint64_t current_timestamp_us = static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::microseconds>(now_epoch).count());

  //=========================================================================
  // Step 1: Convert all inputs to internal types
  //=========================================================================
  LOG_DEBUG() << "PsdFusionProcess::RunTask - Converting inputs\n";

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

  LOG_DEBUG() << "PsdFusionProcess::RunTask - Input Summary:"
              << " apaStatus=" << static_cast<int>(cached_input_.stateMachine.apaStatusReq)
              << ", vehPose=(" << cached_input_.vehiclePose.posX 
              << "," << cached_input_.vehiclePose.posY
              << "," << cached_input_.vehiclePose.yaw << ")"
              << ", mapSlots=" << cached_input_.mapInfo.parkingSlotCount
              << "\n";

  //=========================================================================
  // Step 2: Execute internal processing
  //=========================================================================
  LOG_DEBUG() << "PsdFusionProcess::RunTask - Processing\n";

  cached_output_.clear();
  slot_fusion_.Process(cached_input_, cached_output_);


  LOG_DEBUG() << "PsdFusionProcess::RunTask - Output Summary:"
              << " slotNum=" << cached_output_.slotList.slotNum
              << ", targetSlotId=" << cached_output_.targetLabel.targetSlotId
              <<"\n";


  //=========================================================================
  // Step 3: Convert outputs back to external types
  //=========================================================================
  LOG_DEBUG() << "PsdFusionProcess::RunTask - Converting outputs\n";

  PsdOutputConverter::ConvertAllOutputs(
      cached_output_,
      output_slot_list,
      output_slot_stopper,
      output_target_label);

  //=========================================================================
  // Step 4: Print final outputs after ConvertAllOutputs
  //=========================================================================
  LOG_DEBUG() << "[FINAL OUTPUT - ParkingSlotList]"
              << " header.flag=" << output_slot_list.header.flag
              << ", header.version=" << output_slot_list.header.version
              << ", header.pub_timestamp_us=" << output_slot_list.header.pub_timestamp_us
              << ", header.sequence=" << output_slot_list.header.sequence
              << ", slot_num=" << output_slot_list.slot_num << "\n";

  for (size_t i = 0; i < output_slot_list.slot_num && i < 50; ++i) {
      const auto& slot = output_slot_list.slots[i];
      LOG_DEBUG() << "[FINAL OUTPUT - ParkingSlotList] slots[" << i << "]:"
                  << " slot_id=" << slot.slot_id
                  << ", slot_display_id=" << slot.slot_display_id
                  << ", slot_display_status=" << static_cast<int>(slot.slot_display_status)
                  << ", slot_corners[0]=(" << slot.slot_corners[0].x << "," << slot.slot_corners[0].y << "," << slot.slot_corners[0].z << ")"
                  << ", slot_corners[1]=(" << slot.slot_corners[1].x << "," << slot.slot_corners[1].y << "," << slot.slot_corners[1].z << ")"
                  << ", slot_corners[2]=(" << slot.slot_corners[2].x << "," << slot.slot_corners[2].y << "," << slot.slot_corners[2].z << ")"
                  << ", slot_corners[3]=(" << slot.slot_corners[3].x << "," << slot.slot_corners[3].y << "," << slot.slot_corners[3].z << ")"
                  << ", slot_entry_length=" << slot.slot_entry_length
                  << ", slot_nonentry_length=" << slot.slot_nonentry_length
                  << ", slot_type=" << static_cast<int>(slot.slot_type)
                  << ", slot_surface_type=" << static_cast<int>(slot.slot_surface_type)
                  << ", slot_bottom_type=" << static_cast<int>(slot.slot_bottom_type)
                  << ", slot_obstacle_type=" << static_cast<int>(slot.slot_obstacle_type)
                  << ", slot_obstacle_location=" << static_cast<int>(slot.slot_obstacle_location)
                  << ", slot_stopper_distance=" << slot.slot_stopper_distance
                  << ", slot_confidence=" << slot.slot_confidence
                  << ", slot_source=" << static_cast<int>(slot.slot_source)
                  << ", slot_occupied=" << slot.slot_occupied
                  << ", slot_selected=" << slot.slot_selected
                  << ", park_in_type=" << static_cast<int>(slot.park_in_type)
                  << ", ussslot_oppositespace=" << slot.ussslot_oppositespace
                  << ", ussslot_transversespace=" << slot.ussslot_transversespace << "\n";
  }

  LOG_DEBUG() << "[FINAL OUTPUT - ParkingSlotStopper]"
              << " header.flag=" << output_slot_stopper.header.flag
              << ", header.version=" << output_slot_stopper.header.version
              << ", header.pub_timestamp_us=" << output_slot_stopper.header.pub_timestamp_us
              << ", header.sequence=" << output_slot_stopper.header.sequence
              << ", apc_LimitBarX[0]=" << output_slot_stopper.apc_LimitBarX[0]
              << ", apc_LimitBarX[1]=" << output_slot_stopper.apc_LimitBarX[1]
              << ", apc_LimitBarY[0]=" << output_slot_stopper.apc_LimitBarY[0]
              << ", apc_LimitBarY[1]=" << output_slot_stopper.apc_LimitBarY[1]
              << ", apc_LimitBarYaw[0]=" << output_slot_stopper.apc_LimitBarYaw[0]
              << ", apc_LimitBarYaw[1]=" << output_slot_stopper.apc_LimitBarYaw[1]
              << ", apc_LimitBarID[0]=" << output_slot_stopper.apc_LimitBarID[0]
              << ", apc_LimitBarID[1]=" << output_slot_stopper.apc_LimitBarID[1] << "\n";

  LOG_DEBUG() << "[FINAL OUTPUT - TargetSlotLabel]"
              << " header.flag=" << output_target_label.header.flag
              << ", header.version=" << output_target_label.header.version
              << ", header.pub_timestamp_us=" << output_target_label.header.pub_timestamp_us
              << ", header.sequence=" << output_target_label.header.sequence
              << ", targetSlotID=" << output_target_label.targetSlotID << "\n";


  auto end_time = std::chrono::steady_clock::now();
  auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
      end_time - start_time).count();

  LOG_DEBUG() << "PsdFusionProcess::RunTask - Completed in " << elapsed_ms << "ms\n";


  return true;
}

bool PsdFusionProcess::Reset() {
  return false;
}

}
}

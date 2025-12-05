#pragma once

#include <string>
#include <chrono>

#include "DataType.h"
#include "include/psd_input_converter.h"
#include "include/psd_slot_fusion.h"
#include "include/psd_output_converter.h"

namespace patac {
namespace psd {

class PsdFusionProcess {
 public:
  PsdFusionProcess() = default;
  
  /*
  * @brief:  called by PsdFusionComponent initialize
  * @param:  cfg_path: configuration path for swc pipeline
  * @return: true: function run success; false otherwise
  */
  bool Initialize(const std::string& cfg_path);
  
  /*
  * @brief:  called by PsdFusionComponent Event_SelectSlot
  * @param:  select_slot: UI选择目标车位
  * @return: true: function run success; false otherwise
  */
  bool UpdateSelectSlot(const apa::uic::SelectParkingSlot& select_slot);
  
  /*
  * @brief:  called by PsdFusionComponent Event_CustomSlot  
  * @param:  custom_slot: UI自选车位
  * @return: true: function run success; false otherwise
  */
  bool UpdateCustomSlot(const apa::uic::CustomizeParkingSlot& custom_slot);
  
  /*
  * @brief:  called by PsdFusionComponent Timer50_1
  *          处理所有周期订阅输入并生成输出
  * @param:  state_machine: 状态机输出
  * @param:  vehicle_pose: 车辆位姿信息
  * @param:  map_info: 地图信息
  * @param:  uss_slot_list: 超声波检测的车位列表
  * @param:  gridmap: 栅格地图
  * @param:  emap_obs: 障碍物信息
  * @param:  parkable_slot: 可泊车位信息
  * @param:  pnc_status: 规划控制状态
  * @param:  output_slot_list: [输出] 融合后的车位列表
  * @param:  output_slot_stopper: [输出] 车位内的限位块坐标
  * @param:  output_target_label: [输出] 目标车位ID
  * @return: true: function run success; false otherwise
  */
  bool RunTask(const apa::status::Output2ALL& state_machine,
               const apa::loc::Vehicle_Pose& vehicle_pose,
               const apa::loc::MapInfo& map_info,
               const uss::UssSlotList& uss_slot_list,
               const apa::fus::EmapGridmap& gridmap,
               const apa::fus::PkEmapObs& emap_obs,
               const apa::planning::DecPlan2Psd& parkable_slot,
               const apa::planning::Plan2Status& pnc_status,
               apa::psd::ParkingSlotList& output_slot_list,
               apa::psd::ParkingSlotStopper& output_slot_stopper,
               apa::psd::TargetSlotLabel& output_target_label);

   /*
  * @brief:  called on mode change
  */ 
  bool Reset();

private:
  bool is_initialized_ = false;

  PsdFusionInput cached_input_;
  PsdFusionOutput cached_output_;

  PsdSlotFusion slot_fusion_;
};

}  // namespace psd
}  // namespace patac

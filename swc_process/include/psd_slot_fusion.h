#pragma once

#include <cmath>
#include <string>

#include "psd_internal_type.h"

class PsdSlotFusion {
 public:
  PsdSlotFusion();
  ~PsdSlotFusion() = default;

  void Reset();

  void Process(const PsdFusionInput& input, PsdFusionOutput& output);

  void UpdateSelectSlot(const SelectSlotInput& select_slot);

  void UpdateCustomSlot(const CustomSlotInput& custom_slot);

 private:
  //=========================================================================
  // Processing Steps
  //=========================================================================

  /// Step 1: Clear data based on APA status
  void ResetByStatus(const PsdFusionInput& input, InternalSlotList& slot_list);

  /// Step 2: Convert map parking slots to internal slots
  void ConvertMapInfo(const PsdFusionInput& input, InternalSlotList& slot_list);

  /// Step 3: Convert USS slots to internal slots and merge
  void ConvertUssSlots(const PsdFusionInput& input,
                       InternalSlotList& slot_list);

  /// Step 4: Process obstacles (stopper, lock, etc.)
  void ProcObstacles(const PsdFusionInput& input, InternalSlotList& slot_list);

  /// Step 5: Select target slot (from user selection or recommendation)
  void ProcSelectTargetSlot(const PsdFusionInput& input,
                            InternalSlotList& slot_list);

  /// Step 6: Update display status for all slots
  void ProcUpdateDisplayStatus(const PsdFusionInput& input,
                               InternalSlotList& slot_list);

  /// Step 7: Convert internal slots to output format
  void ConvertToOutput(const PsdFusionInput& input,
                       const InternalSlotList& slot_list,
                       PsdFusionOutput& output);

  //=========================================================================
  // Helper Functions
  //=========================================================================

  /// Convert slot type from map format to internal format
  static SlotType ConvertMapSlotType(uint32_t map_type);

  /// Calculate slot entry length (AB edge)
  static float CalculateEntryLength(const SlotCorners& corners);

  /// Calculate slot non-entry length (AD edge)
  static float CalculateNonEntryLength(const SlotCorners& corners);

  /// Transform point from body to world coordinate
  static Point2f TransformBodyToWorld(const Point2f& pt_body,
                                      const VehiclePoseInput& pose);

  /// Transform point from world to body coordinate
  static Point2f TransformWorldToBody(const Point2f& pt_world,
                                      const VehiclePoseInput& pose);

  /// Adjust corner order (A closest to vehicle, B next on entry side)
  static void AdjustCornerOrder(SlotCorners& corners);

  /// Check if a point is inside a quadrilateral
  static bool PointInQuad(const Point2f& point, const SlotCorners& corners);

  /// Calculate perpendicular distance from point to line segment
  static float CalcPointToLineDistance(const Point2f& point,
                                       const Point2f& lineA,
                                       const Point2f& lineB);

  /// Check if slot is in parking out mode
  bool IsParkOutMode() const { return is_park_out_mode_; }

  /// Check if vehicle is still (not moving)
  bool IsVehicleStill() const { return is_vehicle_still_; }

  /// Get final selected slot ID (combining HMI, VCU, and recommendation)
  uint32_t GetFinalSelectedSlotId() const;

 private:
  //=========================================================================
  // Internal State
  //=========================================================================

  /// Current fused slot list
  InternalSlotList current_slots_;

  /// Saved target slot for guidance phase
  InternalSlot saved_target_slot_;
  bool has_saved_target_slot_ = false;

  /// User selection state
  uint32_t hmi_selected_id_ = 0;
  uint32_t hmi_temp_id_ = 0;
  uint32_t vcu_selected_id_ = 0;
  uint32_t recommended_id_ = 0;
  uint32_t final_selected_id_ = 0;

  /// Vehicle state
  bool is_park_out_mode_ = false;
  bool is_vehicle_still_ = true;
  VehiclePoseInput previous_pose_;

  /// Status tracking
  uint8_t last_apa_status_ = 0;
  bool has_cleared_for_search_ = false;

  // Filter settings (from config)
  bool faraway_filter_enabled_ = false;
  float faraway_left_min_ = -99999.0f;
  float faraway_left_max_ = 0.0f;
  float faraway_right_min_ = 0.0f;
  float faraway_right_max_ = 99999.0f;
  float faraway_rear_ = -99999.0f;
  float faraway_front_ = 99999.0f;

  bool angle_filter_enabled_ = false;
  float angle_filter_limit_ = 45.0f;

  bool narrow_slot_filter_enabled_ = false;
  float narrow_slot_threshold_ = 2.0f;

  // Vehicle parameters
  static constexpr float kVehicleLength = 5259.9f;         // mm
  static constexpr float kRearAxleToCenterRear = 1136.7f;  // mm
  static constexpr float kMmToMeter = 1000.0f;
  static constexpr float kStillThreshold = 0.03f;  // mm for position change
  static constexpr int kStillFrameCount = 5;       // frames for still detection
  int still_frame_counter_ = 0;
};

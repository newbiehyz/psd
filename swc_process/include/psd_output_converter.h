#pragma once

#include "apa/common.h"
#include "apa/fus.h"
#include "apa/gridmap.h"
#include "apa/loc.h"
#include "apa/planning.h"
#include "apa/state_machine.h"
#include "apa/uic.h"
#include "apa/uss.h"
#include "psd_internal_type.h"

struct ParkingSlotList;
struct ParkingSlotStopper;
struct TargetSlotLabel;

class PsdOutputConverter {
 public:
  /// Convert parking slot list output
  static void ConvertSlotList(const ParkingSlotListOutput& input,
                              apa::psd::ParkingSlotList& output);

  /// Convert parking slot stopper output
  static void ConvertSlotStopper(const ParkingSlotStopperOutput& input,
                                 apa::psd::ParkingSlotStopper& output);

  /// Convert target slot label output
  static void ConvertTargetLabel(const TargetSlotLabelOutput& input,
                                 apa::psd::TargetSlotLabel& output);

  /// Convert all outputs from internal to external format
  static void ConvertAllOutputs(const PsdFusionOutput& input,
                                apa::psd::ParkingSlotList& slot_list,
                                apa::psd::ParkingSlotStopper& slot_stopper,
                                apa::psd::TargetSlotLabel& target_label);
};

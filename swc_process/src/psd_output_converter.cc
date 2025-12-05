#include "psd_output_converter.h"

#include "apa/psd.h"

void PsdOutputConverter::ConvertSlotList(
    const ParkingSlotListOutput& input,
    apa::psd::ParkingSlotList& output)
{
    output.header.flag = input.header.flag;
    output.header.version = input.header.version;
    output.header.sequence = input.header.sequence;
    output.header.pub_timestamp_us = input.header.pubTimestampUs;
    
    output.slot_num = input.slotNum;

    for (size_t i = 0; i < input.slotNum && i < Limits::kMaxSlotCount; ++i) {
        const auto& src = input.slots[i];
        auto& dst = output.slots[i];

        dst.slot_id = src.slotId;
        dst.slot_display_id = src.slotDisplayId;
        dst.slot_display_status = src.slotDisplayStatus;

        for (int j = 0; j < 4; ++j) {
            dst.slot_corners[j].x = src.slotCorners[j].x;
            dst.slot_corners[j].y = src.slotCorners[j].y;
            dst.slot_corners[j].z = src.slotCorners[j].z;
        }

        dst.slot_entry_length = src.slotEntryLength;
        dst.slot_nonentry_length = src.slotNonEntryLength;
        dst.slot_type = src.slotType;
        dst.slot_surface_type = src.slotSurfaceType;
        dst.slot_bottom_type = src.slotBottomType;
        dst.slot_obstacle_type = src.slotObstacleType;
        dst.slot_obstacle_location = src.slotObstacleLocation;
        dst.slot_stopper_distance = src.slotStopperDistance;
        dst.slot_confidence = src.slotConfidence;
        dst.slot_source = src.slotSource;
        dst.slot_occupied = src.slotOccupied;
        dst.slot_selected = src.slotSelected;
        dst.park_in_type = static_cast<apa::psd::ParkingDirection>(src.parkInType);
        dst.ussslot_oppositespace = src.ussSlotOppositeSpace;
        dst.ussslot_transversespace = src.ussSlotTransverseSpace;
    }
}

void PsdOutputConverter::ConvertSlotStopper(
    const ParkingSlotStopperOutput& input,
    apa::psd::ParkingSlotStopper& output)
{
    output.header.flag = input.header.flag;
    output.header.version = input.header.version;
    output.header.sequence = input.header.sequence;
    output.header.pub_timestamp_us = input.header.pubTimestampUs;

    for (size_t i = 0; i < Limits::kMaxStopperCount; ++i) {
        output.apc_LimitBarX[i] = input.limitBarX[i];
        output.apc_LimitBarY[i] = input.limitBarY[i];
        output.apc_LimitBarYaw[i] = input.limitBarYaw[i];
        output.apc_LimitBarID[i] = input.limitBarId[i];
    }
}

void PsdOutputConverter::ConvertTargetLabel(
    const TargetSlotLabelOutput& input,
    apa::psd::TargetSlotLabel& output)
{
    output.header.flag = input.header.flag;
    output.header.version = input.header.version;
    output.header.sequence = input.header.sequence;
    output.header.pub_timestamp_us = input.header.pubTimestampUs;

    output.targetSlotID = input.targetSlotId;
}

void PsdOutputConverter::ConvertAllOutputs(
    const PsdFusionOutput& input,
    apa::psd::ParkingSlotList& slot_list,
    apa::psd::ParkingSlotStopper& slot_stopper,
    apa::psd::TargetSlotLabel& target_label)
{
    ConvertSlotList(input.slotList, slot_list);
    ConvertSlotStopper(input.slotStopper, slot_stopper);
    ConvertTargetLabel(input.targetLabel, target_label);
}
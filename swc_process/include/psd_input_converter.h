#pragma once

#include "psd_internal_type.h"

#include "apa/common.h"
#include "apa/state_machine.h"
#include "apa/loc.h"
#include "apa/gridmap.h"
#include "apa/fus.h"
#include "apa/planning.h"
#include "apa/uss.h"
#include "apa/uic.h"

class PsdInputConverter {
public:
    static void ConvertStateMachine(
        const apa::status::Output2ALL& input,
        StateMachineInput& output);

    static void ConvertVehiclePose(
        const apa::loc::Vehicle_Pose& input,
        VehiclePoseInput& output);

    static void ConvertMapInfo(
        const apa::loc::MapInfo& input,
        MapInfoInput& output);

    static void ConvertUssSlots(
        const uss::UssSlotList& input,
        UssSlotListInput& output);

    static void ConvertGridMap(
        const apa::fus::EmapGridmap& input,
        GridMapInput& output);

    static void ConvertObstacles(
        const apa::fus::PkEmapObs& input,
        ObstacleListInput& output);

    static void ConvertPlanningDecision(
        const apa::planning::DecPlan2Psd& input,
        PlanningDecisionInput& output);

    static void ConvertPlanningStatus(
        const apa::planning::Plan2Status& input,
        PlanningStatusInput& output);

    static void ConvertSelectSlot(
        const apa::uic::SelectParkingSlot& input,
        SelectSlotInput& output);

    static void ConvertCustomSlot(
        const apa::uic::CustomizeParkingSlot& input,
        CustomSlotInput& output);

    static void ConvertAllInputs(
        const apa::status::Output2ALL& state_machine,
        const apa::loc::Vehicle_Pose& vehicle_pose,
        const apa::loc::MapInfo& map_info,
        const uss::UssSlotList& uss_slots,
        const apa::fus::EmapGridmap& gridmap,
        const apa::fus::PkEmapObs& emap_obs,
        const apa::planning::DecPlan2Psd& parkable_slot,
        const apa::planning::Plan2Status& pnc_status,
        uint64_t current_timestamp_us,
        PsdFusionInput& output);

private:
    static void ConvertHeader(
        const apa::common::Header& input,
        Header& output);

    static Point3f ConvertPoint3f(const apa::Point3f& input);

    static Point2f ConvertPoint2f(const apa::Point2f& input);
};


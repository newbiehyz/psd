#include "psd_input_converter.h"

void PsdInputConverter::ConvertHeader(
    const apa::common::Header& input,
    Header& output) {
    output.flag = input.flag;
    output.version = input.version;
    output.pubTimestampUs = input.pub_timestamp_us;
    output.sequence = input.sequence;
}

Point3f PsdInputConverter::ConvertPoint3f(const apa::Point3f& input) {
    Point3f output;
    output.x = input.x;
    output.y = input.y;
    output.z = input.z;
    return output;
}

Point2f PsdInputConverter::ConvertPoint2f(const apa::Point2f& input) {
    Point2f output;
    output.x = input.x;
    output.y = input.y;
    return output;
}

//-----------------------------------------------------------------------------
// State Machine Conversion
//-----------------------------------------------------------------------------

void PsdInputConverter::ConvertStateMachine(
    const apa::status::Output2ALL& input,
    StateMachineInput& output)
{
    ConvertHeader(input.header, output.header);
    output.apaStatusReq = input.apaStatusReq;
    output.hppStatusReq = input.hppStatusReq;
    output.drivingOrParking1 = input.DrivingOrParking1;
    output.drivingOrParking2 = input.DrivingOrParking2;
    output.apaStartParkingReq = input.apaStartParkingReq;
    output.apaSrchInterrupt = input.apaSrchInterupt;
}

//-----------------------------------------------------------------------------
// Vehicle Pose Conversion
//-----------------------------------------------------------------------------

void PsdInputConverter::ConvertVehiclePose(
    const apa::loc::Vehicle_Pose& input,
    VehiclePoseInput& output)
{
    ConvertHeader(input.header, output.header);
    output.locStatus = static_cast<LocStatus>(input.loc_status);
    output.posX = input.pos_x;
    output.posY = input.pos_y;
    output.posZ = input.pos_z;
    output.quaternionQx = input.quaternion_qx;
    output.quaternionQy = input.quaternion_qy;
    output.quaternionQz = input.quaternion_qz;
    output.quaternionQw = input.quaternion_qw;
    output.roll = input.roll;
    output.pitch = input.pitch;
    output.yaw = input.yaw;
    output.timestampUs = input.timeStamp_us;
}

//-----------------------------------------------------------------------------
// Map Info Conversion
//-----------------------------------------------------------------------------

void PsdInputConverter::ConvertMapInfo(
    const apa::loc::MapInfo& input,
    MapInfoInput& output)
{
    ConvertHeader(input.header, output.header);
    output.timestamp = input.timeStamp;
    output.mapId = input.mapId;

    // Convert parking slots
    output.parkingSlotCount = 0;
    for (size_t i = 0; i < input.ParkingSlot.size() && i < Limits::kMaxSlotCount; ++i) {
        const auto& src = input.ParkingSlot[i];
        auto& dst = output.parkingSlots[output.parkingSlotCount];
        
        dst.id = src.id;
        dst.psType = src.psType;
        dst.width = src.width;
        dst.length = src.length;
        dst.center = ConvertPoint3f(src.center);
        dst.wideDirection = ConvertPoint3f(src.wideDirection);
        dst.longDirection = ConvertPoint3f(src.longDirection);
        dst.isOccupied = src.isOccupancy;
        
        ++output.parkingSlotCount;
    }

    // Convert lane lines
    output.laneLineCount = 0;
    for (size_t i = 0; i < input.LaneLine.size() && i < Limits::kMaxLaneLines; ++i) {
        const auto& src = input.LaneLine[i];
        auto& dst = output.laneLines[output.laneLineCount];
        
        dst.id = src.id;
        dst.lineType = src.lineType;
        dst.pointCount = 0;
        for (size_t j = 0; j < src.points.size() && j < 100; ++j) {
            dst.points[dst.pointCount++] = ConvertPoint3f(src.points[j]);
        }
        
        ++output.laneLineCount;
    }

    // Convert stop lines
    output.stopLineCount = 0;
    for (size_t i = 0; i < input.StopLine.size() && i < Limits::kMaxStopLines; ++i) {
        const auto& src = input.StopLine[i];
        auto& dst = output.stopLines[output.stopLineCount];
        
        dst.id = src.id;
        dst.startPt = ConvertPoint3f(src.startPt);
        dst.endPt = ConvertPoint3f(src.endPt);
        
        ++output.stopLineCount;
    }

    // Convert bumps
    output.bumpCount = 0;
    for (size_t i = 0; i < input.Bump.size() && i < Limits::kMaxBumps; ++i) {
        const auto& src = input.Bump[i];
        auto& dst = output.bumps[output.bumpCount];
        
        dst.id = src.id;
        dst.width = src.width;
        dst.length = src.length;
        dst.endPt = ConvertPoint3f(src.endPt);
        dst.wideDirection = ConvertPoint3f(src.wideDirection);
        dst.longDirection = ConvertPoint3f(src.longDirection);
        
        ++output.bumpCount;
    }

    // Convert cross walks
    output.crossWalkCount = 0;
    for (size_t i = 0; i < input.CrossWalk.size() && i < Limits::kMaxCrossWalks; ++i) {
        const auto& src = input.CrossWalk[i];
        auto& dst = output.crossWalks[output.crossWalkCount];
        
        dst.id = src.id;
        dst.width = src.width;
        dst.length = src.length;
        dst.endPt = ConvertPoint3f(src.endPt);
        dst.wideDirection = ConvertPoint3f(src.wideDirection);
        dst.longDirection = ConvertPoint3f(src.longDirection);
        
        ++output.crossWalkCount;
    }

    // Convert arrows
    output.arrowCount = 0;
    for (size_t i = 0; i < input.Arrow.size() && i < Limits::kMaxArrows; ++i) {
        const auto& src = input.Arrow[i];
        auto& dst = output.arrows[output.arrowCount];
        
        dst.id = src.id;
        dst.arrowType = src.arrowType;
        dst.pointCount = 0;
        for (size_t j = 0; j < src.points.size() && j < 20; ++j) {
            dst.points[dst.pointCount++] = ConvertPoint3f(src.points[j]);
        }
        
        ++output.arrowCount;
    }
}

//-----------------------------------------------------------------------------
// USS Slots Conversion
//-----------------------------------------------------------------------------

void PsdInputConverter::ConvertUssSlots(
    const uss::UssSlotList& input,
    UssSlotListInput& output)
{
    for (size_t i = 0; i < Limits::kMaxUssSlots; ++i) {
        const auto& src = input.uss_slot[i];
        auto& dst = output.slots[i];

        for (size_t j = 0; j < Limits::kMaxCornerPoints; ++j) {
            dst.pointABCD[j].x = src.point_ABCD[j].x;
            dst.pointABCD[j].y = src.point_ABCD[j].y;
        }
        dst.bottomType = static_cast<SlotBottomType>(src.bottom_type);
        dst.obsStatus = static_cast<InSlotObstacleLocation>(src.obs_status);
        dst.leftSideObstaclePosition = src.leftside_obstacle_position;
        dst.rightSideObstaclePosition = src.rightside_obstacle_position;
        dst.timestamp = src.timestamp;
    }
}

//-----------------------------------------------------------------------------
// Grid Map Conversion
//-----------------------------------------------------------------------------

void PsdInputConverter::ConvertGridMap(
    const apa::fus::EmapGridmap& input,
    GridMapInput& output)
{
    ConvertHeader(input.header, output.header);
    output.timestamp = input.EmapGrid_TimeStamp;
    output.workMode = static_cast<GridMapWorkMode>(input.eMapWorkMode);
    
    // Vague center point
    output.vagueCenterPoint.x = input.VagueEmapGrid.vaguecenterPoint.x;
    output.vagueCenterPoint.y = input.VagueEmapGrid.vaguecenterPoint.y;
    output.vagueCenterPoint.z = input.VagueEmapGrid.vaguecenterPoint.z;
    output.vagueCenterPoint.yaw = input.VagueEmapGrid.vaguecenterPoint.yaw;
    
    // Precise center point
    output.preciseCenterPoint.x = input.PreciseEmapGrid.PrecisecenterPoint.x;
    output.preciseCenterPoint.y = input.PreciseEmapGrid.PrecisecenterPoint.y;
    output.preciseCenterPoint.z = input.PreciseEmapGrid.PrecisecenterPoint.z;
    output.preciseCenterPoint.yaw = input.PreciseEmapGrid.PrecisecenterPoint.yaw;
}

//-----------------------------------------------------------------------------
// Obstacle List Conversion
//-----------------------------------------------------------------------------

void PsdInputConverter::ConvertObstacles(
    const apa::fus::PkEmapObs& input,
    ObstacleListInput& output)
{
    ConvertHeader(input.header, output.header);

    for (size_t i = 0; i < Limits::kMaxObstacleCount; ++i) {
        const auto& src = input.pkEmapObs[i];
        auto& dst = output.obstacles[i];

        dst.frameIndex = src.Frame_Index;
        dst.failSafe = src.Fail_Safe;
        dst.obsId = src.obsID;
        dst.obsType = static_cast<ObstacleType>(src.obsTyp);
        dst.obsCenter = ConvertPoint3f(src.obsCenter);
        dst.age = src.age;
        dst.obsDirection = ConvertPoint3f(src.obsDirection);
        
        // Motion info
        dst.motionInfo.isValid = src.obsMotionInfo.isValid;
        dst.motionInfo.center = ConvertPoint3f(src.obsMotionInfo.center);
        dst.motionInfo.centerUncertainty = ConvertPoint3f(src.obsMotionInfo.centerUncertainty);
        dst.motionInfo.velocity = ConvertPoint3f(src.obsMotionInfo.velocity);
        dst.motionInfo.velocityUncertainty = ConvertPoint3f(src.obsMotionInfo.velocityUncertainty);
        dst.motionInfo.acceleration = ConvertPoint3f(src.obsMotionInfo.acceleration);
        dst.motionInfo.accelerationUncertainty = ConvertPoint3f(src.obsMotionInfo.accelerationUncertainty);
        dst.motionInfo.velocityHeading = src.obsMotionInfo.velocityHeading;
        dst.motionInfo.velocityHeadingUncertainty = src.obsMotionInfo.velocityHeadingUncertainty;
        dst.motionInfo.velocityHeadingRate = src.obsMotionInfo.velocityHeadingRate;
        dst.motionInfo.velocityHeadingRateUncertainty = src.obsMotionInfo.velocityHeadingRateUncertainty;
        
        // Trajectory
        dst.trajectory.motionStatus = static_cast<MotionStatus>(src.obsTrajectory.motionStatus);
        dst.trajectory.confidence = src.obsTrajectory.confidence;
        dst.trajectory.pointCount = 0;
        for (size_t j = 0; j < src.obsTrajectory.points.size() && j < Limits::kMaxTrajectoryPoints; ++j) {
            const auto& pt = src.obsTrajectory.points[j];
            dst.trajectory.points[dst.trajectory.pointCount].deltaTNs = pt.deltaTNs;
            dst.trajectory.points[dst.trajectory.pointCount].center = ConvertPoint3f(pt.center);
            dst.trajectory.points[dst.trajectory.pointCount].direction = ConvertPoint3f(pt.direction);
            ++dst.trajectory.pointCount;
        }
        
        // Boundary
        dst.boundaryType = static_cast<ObstacleBoundaryType>(src.obsBdTyp);
        dst.boundingBoxCount = 0;
        for (size_t j = 0; j < src.obsBdBox.size() && j < Limits::kMaxBoundingBoxPoints; ++j) {
            dst.boundingBox[dst.boundingBoxCount++] = ConvertPoint3f(src.obsBdBox[j]);
        }
        dst.confidence = src.obsConf;
    }
}

//-----------------------------------------------------------------------------
// Planning Decision Conversion
//-----------------------------------------------------------------------------

void PsdInputConverter::ConvertPlanningDecision(
    const apa::planning::DecPlan2Psd& input,
    PlanningDecisionInput& output)
{
    ConvertHeader(input.header, output.header);
    output.frameId = input.frameID;
    output.validSlotNum = input.validSlotNum;
    
    // Slot result
    output.slotResult.slotId = input.emapSrchSlotsRes.SlotID;
    output.slotResult.slotParkable = static_cast<SlotParkable>(input.emapSrchSlotsRes.slotParkable);
    output.slotResult.slotScore = input.emapSrchSlotsRes.slotScore;
    
    // Parking space
    ConvertHeader(input.parkingSpace.header, output.parkingSpace.header);
    output.parkingSpace.p0 = ConvertPoint2f(input.parkingSpace.p0);
    output.parkingSpace.p1 = ConvertPoint2f(input.parkingSpace.p1);
    output.parkingSpace.p2 = ConvertPoint2f(input.parkingSpace.p2);
    output.parkingSpace.p3 = ConvertPoint2f(input.parkingSpace.p3);
    output.parkingSpace.p4 = ConvertPoint2f(input.parkingSpace.p4);
    output.parkingSpace.p5 = ConvertPoint2f(input.parkingSpace.p5);
    output.parkingSpace.p6 = ConvertPoint2f(input.parkingSpace.p6);
    output.parkingSpace.p7 = ConvertPoint2f(input.parkingSpace.p7);
    output.parkingSpace.chamferP0.aisleSidePoint = ConvertPoint2f(input.parkingSpace.chamferP0.aisleSidePoint);
    output.parkingSpace.chamferP0.slotSidePoint = ConvertPoint2f(input.parkingSpace.chamferP0.slotSidePoint);
    output.parkingSpace.chamferP5.aisleSidePoint = ConvertPoint2f(input.parkingSpace.chamferP5.aisleSidePoint);
    output.parkingSpace.chamferP5.slotSidePoint = ConvertPoint2f(input.parkingSpace.chamferP5.slotSidePoint);
    
    // Target position
    output.targetPos.finalTargetPos.x = input.targetPos.finalTargetPos.x;
    output.targetPos.finalTargetPos.y = input.targetPos.finalTargetPos.y;
    output.targetPos.finalTargetPos.yaw = input.targetPos.finalTargetPos.yaw;
    output.targetPos.finalTargetPos.curv = input.targetPos.finalTargetPos.curv;
    output.targetPos.stageTargetPos.x = input.targetPos.stageTargetPos.x;
    output.targetPos.stageTargetPos.y = input.targetPos.stageTargetPos.y;
    output.targetPos.stageTargetPos.yaw = input.targetPos.stageTargetPos.yaw;
    output.targetPos.stageTargetPos.curv = input.targetPos.stageTargetPos.curv;
}

//-----------------------------------------------------------------------------
// Planning Status Conversion
//-----------------------------------------------------------------------------

void PsdInputConverter::ConvertPlanningStatus(
    const apa::planning::Plan2Status& input,
    PlanningStatusInput& output)
{
    ConvertHeader(input.header, output.header);
    output.slotWidthStatus = input.soltWidthStatus;
    output.pncStatus = input.PncStatus;
}

//-----------------------------------------------------------------------------
// Select Slot Conversion
//-----------------------------------------------------------------------------

void PsdInputConverter::ConvertSelectSlot(
    const apa::uic::SelectParkingSlot& input,
    SelectSlotInput& output)
{
    output.slotId = input.slot_id;
    output.parkingDirection = static_cast<ParkingDirection>(input.parking_direction);
    output.isValid = (input.slot_id != 0);
}

//-----------------------------------------------------------------------------
// Custom Slot Conversion
//-----------------------------------------------------------------------------

void PsdInputConverter::ConvertCustomSlot(
    const apa::uic::CustomizeParkingSlot& input,
    CustomSlotInput& output)
{
    output.pt = input.pt;
    output.parkingSlotType = static_cast<uint8_t>(input.parking_slot_type);
    output.slotSurfaceType = static_cast<uint8_t>(input.slot_surface_type);
    output.targetSlotId = input.target_slot_id;
    output.isValid = (input.target_slot_id != 0);
}

//-----------------------------------------------------------------------------
// Convert All Inputs
//-----------------------------------------------------------------------------

void PsdInputConverter::ConvertAllInputs(
    const apa::status::Output2ALL& state_machine,
    const apa::loc::Vehicle_Pose& vehicle_pose,
    const apa::loc::MapInfo& map_info,
    const uss::UssSlotList& uss_slots,
    const apa::fus::EmapGridmap& gridmap,
    const apa::fus::PkEmapObs& emap_obs,
    const apa::planning::DecPlan2Psd& parkable_slot,
    const apa::planning::Plan2Status& pnc_status,
    uint64_t current_timestamp_us,
    PsdFusionInput& output)
{
    output.currentTimestamp = current_timestamp_us;
    
    ConvertStateMachine(state_machine, output.stateMachine);
    ConvertVehiclePose(vehicle_pose, output.vehiclePose);
    ConvertMapInfo(map_info, output.mapInfo);
    ConvertUssSlots(uss_slots, output.ussSlots);
    ConvertGridMap(gridmap, output.gridMap);
    ConvertObstacles(emap_obs, output.obstacles);
    ConvertPlanningDecision(parkable_slot, output.planningDecision);
    ConvertPlanningStatus(pnc_status, output.planningStatus);
}


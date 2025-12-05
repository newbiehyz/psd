#include "psd_input_converter.h"

void PsdInputConverter::ConvertHeader(
    const apa::common::Header& input,
    Header& internal_input) {
    internal_input.flag = input.flag;
    internal_input.version = input.version;
    internal_input.pubTimestampUs = input.pub_timestamp_us;
    internal_input.sequence = input.sequence;
}

Point3f PsdInputConverter::ConvertPoint3f(const apa::Point3f& input) {
    Point3f internal_input;
    internal_input.x = input.x;
    internal_input.y = input.y;
    internal_input.z = input.z;
    return internal_input;
}

Point2f PsdInputConverter::ConvertPoint2f(const apa::Point2f& input) {
    Point2f internal_input;
    internal_input.x = input.x;
    internal_input.y = input.y;
    return internal_input;
}

//-----------------------------------------------------------------------------
// State Machine Conversion
//-----------------------------------------------------------------------------

void PsdInputConverter::ConvertStateMachine(
    const apa::status::Output2ALL& input,
    StateMachineInput& internal_input)
{
    ConvertHeader(input.header, internal_input.header);
    internal_input.apaStatusReq = input.apaStatusReq;
    internal_input.hppStatusReq = input.hppStatusReq;
    internal_input.drivingOrParking1 = input.DrivingOrParking1;
    internal_input.drivingOrParking2 = input.DrivingOrParking2;
    internal_input.apaStartParkingReq = input.apaStartParkingReq;
    internal_input.apaSrchInterrupt = input.apaSrchInterupt;
}

//-----------------------------------------------------------------------------
// Vehicle Pose Conversion
//-----------------------------------------------------------------------------

void PsdInputConverter::ConvertVehiclePose(
    const apa::loc::Vehicle_Pose& input,
    VehiclePoseInput& internal_input)
{
    ConvertHeader(input.header, internal_input.header);
    internal_input.locStatus = static_cast<LocStatus>(input.loc_status);
    internal_input.posX = input.pos_x;
    internal_input.posY = input.pos_y;
    internal_input.posZ = input.pos_z;
    internal_input.quaternionQx = input.quaternion_qx;
    internal_input.quaternionQy = input.quaternion_qy;
    internal_input.quaternionQz = input.quaternion_qz;
    internal_input.quaternionQw = input.quaternion_qw;
    internal_input.roll = input.roll;
    internal_input.pitch = input.pitch;
    internal_input.yaw = input.yaw;
    internal_input.timestampUs = input.timeStamp_us;
}

//-----------------------------------------------------------------------------
// Map Info Conversion
//-----------------------------------------------------------------------------

void PsdInputConverter::ConvertMapInfo(
    const apa::loc::MapInfo& input,
    MapInfoInput& internal_input)
{
    ConvertHeader(input.header, internal_input.header);
    internal_input.timestamp = input.timeStamp;
    internal_input.mapId = input.mapId;

    // Convert parking slots
    internal_input.parkingSlotCount = 0;
    for (size_t i = 0; i < input.ParkingSlot.size() && i < Limits::kMaxSlotCount; ++i) {
        const auto& src = input.ParkingSlot[i];
        auto& dst = internal_input.parkingSlots[internal_input.parkingSlotCount];
        
        dst.id = src.id;
        dst.psType = src.psType;
        dst.width = src.width;
        dst.length = src.length;
        dst.center = ConvertPoint3f(src.center);
        dst.wideDirection = ConvertPoint3f(src.wideDirection);
        dst.longDirection = ConvertPoint3f(src.longDirection);
        dst.isOccupied = src.isOccupancy;
        
        ++internal_input.parkingSlotCount;
    }

    // Convert lane lines
    internal_input.laneLineCount = 0;
    for (size_t i = 0; i < input.LaneLine.size() && i < Limits::kMaxLaneLines; ++i) {
        const auto& src = input.LaneLine[i];
        auto& dst = internal_input.laneLines[internal_input.laneLineCount];
        
        dst.id = src.id;
        dst.lineType = src.lineType;
        dst.pointCount = 0;
        for (size_t j = 0; j < src.points.size() && j < 100; ++j) {
            dst.points[dst.pointCount++] = ConvertPoint3f(src.points[j]);
        }
        
        ++internal_input.laneLineCount;
    }

    // Convert stop lines
    internal_input.stopLineCount = 0;
    for (size_t i = 0; i < input.StopLine.size() && i < Limits::kMaxStopLines; ++i) {
        const auto& src = input.StopLine[i];
        auto& dst = internal_input.stopLines[internal_input.stopLineCount];
        
        dst.id = src.id;
        dst.startPt = ConvertPoint3f(src.startPt);
        dst.endPt = ConvertPoint3f(src.endPt);
        
        ++internal_input.stopLineCount;
    }

    // Convert bumps
    internal_input.bumpCount = 0;
    for (size_t i = 0; i < input.Bump.size() && i < Limits::kMaxBumps; ++i) {
        const auto& src = input.Bump[i];
        auto& dst = internal_input.bumps[internal_input.bumpCount];
        
        dst.id = src.id;
        dst.width = src.width;
        dst.length = src.length;
        dst.endPt = ConvertPoint3f(src.endPt);
        dst.wideDirection = ConvertPoint3f(src.wideDirection);
        dst.longDirection = ConvertPoint3f(src.longDirection);
        
        ++internal_input.bumpCount;
    }

    // Convert cross walks
    internal_input.crossWalkCount = 0;
    for (size_t i = 0; i < input.CrossWalk.size() && i < Limits::kMaxCrossWalks; ++i) {
        const auto& src = input.CrossWalk[i];
        auto& dst = internal_input.crossWalks[internal_input.crossWalkCount];
        
        dst.id = src.id;
        dst.width = src.width;
        dst.length = src.length;
        dst.endPt = ConvertPoint3f(src.endPt);
        dst.wideDirection = ConvertPoint3f(src.wideDirection);
        dst.longDirection = ConvertPoint3f(src.longDirection);
        
        ++internal_input.crossWalkCount;
    }

    // Convert arrows
    internal_input.arrowCount = 0;
    for (size_t i = 0; i < input.Arrow.size() && i < Limits::kMaxArrows; ++i) {
        const auto& src = input.Arrow[i];
        auto& dst = internal_input.arrows[internal_input.arrowCount];
        
        dst.id = src.id;
        dst.arrowType = src.arrowType;
        dst.pointCount = 0;
        for (size_t j = 0; j < src.points.size() && j < 20; ++j) {
            dst.points[dst.pointCount++] = ConvertPoint3f(src.points[j]);
        }
        
        ++internal_input.arrowCount;
    }
}

//-----------------------------------------------------------------------------
// USS Slots Conversion
//-----------------------------------------------------------------------------

void PsdInputConverter::ConvertUssSlots(
    const uss::UssSlotList& input,
    UssSlotListInput& internal_input)
{
    for (size_t i = 0; i < Limits::kMaxUssSlots; ++i) {
        const auto& src = input.uss_slot[i];
        auto& dst = internal_input.slots[i];

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
    GridMapInput& internal_input)
{
    ConvertHeader(input.header, internal_input.header);
    internal_input.timestamp = input.EmapGrid_TimeStamp;
    internal_input.workMode = static_cast<GridMapWorkMode>(input.eMapWorkMode);
    
    // Vague center point
    internal_input.vagueCenterPoint.x = input.VagueEmapGrid.vaguecenterPoint.x;
    internal_input.vagueCenterPoint.y = input.VagueEmapGrid.vaguecenterPoint.y;
    internal_input.vagueCenterPoint.z = input.VagueEmapGrid.vaguecenterPoint.z;
    internal_input.vagueCenterPoint.yaw = input.VagueEmapGrid.vaguecenterPoint.yaw;
    
    // Precise center point
    internal_input.preciseCenterPoint.x = input.PreciseEmapGrid.PrecisecenterPoint.x;
    internal_input.preciseCenterPoint.y = input.PreciseEmapGrid.PrecisecenterPoint.y;
    internal_input.preciseCenterPoint.z = input.PreciseEmapGrid.PrecisecenterPoint.z;
    internal_input.preciseCenterPoint.yaw = input.PreciseEmapGrid.PrecisecenterPoint.yaw;
}

//-----------------------------------------------------------------------------
// Obstacle List Conversion
//-----------------------------------------------------------------------------

void PsdInputConverter::ConvertObstacles(
    const apa::fus::PkEmapObs& input,
    ObstacleListInput& internal_input)
{
    ConvertHeader(input.header, internal_input.header);

    for (size_t i = 0; i < Limits::kMaxObstacleCount; ++i) {
        const auto& src = input.pkEmapObs[i];
        auto& dst = internal_input.obstacles[i];

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
    PlanningDecisionInput& internal_input)
{
    ConvertHeader(input.header, internal_input.header);
    internal_input.frameId = input.frameID;
    internal_input.validSlotNum = input.validSlotNum;
    
    // Slot result
    internal_input.slotResult.slotId = input.emapSrchSlotsRes.SlotID;
    internal_input.slotResult.slotParkable = static_cast<SlotParkable>(input.emapSrchSlotsRes.slotParkable);
    internal_input.slotResult.slotScore = input.emapSrchSlotsRes.slotScore;
    
    // Parking space
    ConvertHeader(input.parkingSpace.header, internal_input.parkingSpace.header);
    internal_input.parkingSpace.p0 = ConvertPoint2f(input.parkingSpace.p0);
    internal_input.parkingSpace.p1 = ConvertPoint2f(input.parkingSpace.p1);
    internal_input.parkingSpace.p2 = ConvertPoint2f(input.parkingSpace.p2);
    internal_input.parkingSpace.p3 = ConvertPoint2f(input.parkingSpace.p3);
    internal_input.parkingSpace.p4 = ConvertPoint2f(input.parkingSpace.p4);
    internal_input.parkingSpace.p5 = ConvertPoint2f(input.parkingSpace.p5);
    internal_input.parkingSpace.p6 = ConvertPoint2f(input.parkingSpace.p6);
    internal_input.parkingSpace.p7 = ConvertPoint2f(input.parkingSpace.p7);
    internal_input.parkingSpace.chamferP0.aisleSidePoint = ConvertPoint2f(input.parkingSpace.chamferP0.aisleSidePoint);
    internal_input.parkingSpace.chamferP0.slotSidePoint = ConvertPoint2f(input.parkingSpace.chamferP0.slotSidePoint);
    internal_input.parkingSpace.chamferP5.aisleSidePoint = ConvertPoint2f(input.parkingSpace.chamferP5.aisleSidePoint);
    internal_input.parkingSpace.chamferP5.slotSidePoint = ConvertPoint2f(input.parkingSpace.chamferP5.slotSidePoint);
    
    // Target position
    internal_input.targetPos.finalTargetPos.x = input.targetPos.finalTargetPos.x;
    internal_input.targetPos.finalTargetPos.y = input.targetPos.finalTargetPos.y;
    internal_input.targetPos.finalTargetPos.yaw = input.targetPos.finalTargetPos.yaw;
    internal_input.targetPos.finalTargetPos.curv = input.targetPos.finalTargetPos.curv;
    internal_input.targetPos.stageTargetPos.x = input.targetPos.stageTargetPos.x;
    internal_input.targetPos.stageTargetPos.y = input.targetPos.stageTargetPos.y;
    internal_input.targetPos.stageTargetPos.yaw = input.targetPos.stageTargetPos.yaw;
    internal_input.targetPos.stageTargetPos.curv = input.targetPos.stageTargetPos.curv;
}

//-----------------------------------------------------------------------------
// Planning Status Conversion
//-----------------------------------------------------------------------------

void PsdInputConverter::ConvertPlanningStatus(
    const apa::planning::Plan2Status& input,
    PlanningStatusInput& internal_input)
{
    ConvertHeader(input.header, internal_input.header);
    internal_input.slotWidthStatus = input.soltWidthStatus;
    internal_input.pncStatus = input.PncStatus;
}

//-----------------------------------------------------------------------------
// Select Slot Conversion
//-----------------------------------------------------------------------------

void PsdInputConverter::ConvertSelectSlot(
    const apa::uic::SelectParkingSlot& input,
    SelectSlotInput& internal_input)
{
    internal_input.slotId = input.slot_id;
    internal_input.parkingDirection = static_cast<ParkingDirection>(input.parking_direction);
    internal_input.isValid = (input.slot_id != 0);
}

//-----------------------------------------------------------------------------
// Custom Slot Conversion
//-----------------------------------------------------------------------------

void PsdInputConverter::ConvertCustomSlot(
    const apa::uic::CustomizeParkingSlot& input,
    CustomSlotInput& internal_input)
{
    internal_input.pt = input.pt;
    internal_input.parkingSlotType = static_cast<uint8_t>(input.parking_slot_type);
    internal_input.slotSurfaceType = static_cast<uint8_t>(input.slot_surface_type);
    internal_input.targetSlotId = input.target_slot_id;
    internal_input.isValid = (input.target_slot_id != 0);
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
    PsdFusionInput& internal_input)
{
    internal_input.currentTimestamp = current_timestamp_us;
    
    ConvertStateMachine(state_machine, internal_input.stateMachine);
    ConvertVehiclePose(vehicle_pose, internal_input.vehiclePose);
    ConvertMapInfo(map_info, internal_input.mapInfo);
    ConvertUssSlots(uss_slots, internal_input.ussSlots);
    ConvertGridMap(gridmap, internal_input.gridMap);
    ConvertObstacles(emap_obs, internal_input.obstacles);
    ConvertPlanningDecision(parkable_slot, internal_input.planningDecision);
    ConvertPlanningStatus(pnc_status, internal_input.planningStatus);
}


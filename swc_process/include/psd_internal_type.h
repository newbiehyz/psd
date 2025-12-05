#pragma once

#include <cstdint>
#include <array>
#include <cmath>
#include <cstring>


//=============================================================================
// Compile-time Constants
//=============================================================================
struct Limits {
    static constexpr size_t kMaxSlotCount = 50;
    static constexpr size_t kMaxObstacleCount = 50;
    static constexpr size_t kMaxCornerPoints = 4;
    static constexpr size_t kMaxUssSlots = 4;
    static constexpr size_t kMaxStopperCount = 2;
    static constexpr size_t kMaxLaneLines = 50;
    static constexpr size_t kMaxStopLines = 50;
    static constexpr size_t kMaxBumps = 50;
    static constexpr size_t kMaxCrossWalks = 50;
    static constexpr size_t kMaxArrows = 50;
    static constexpr size_t kMaxTrajectoryPoints = 20;
    static constexpr size_t kMaxBoundingBoxPoints = 4;
};

struct VehicleParams {
    static constexpr float kVehicleLength = 5259.9f;         // mm
    static constexpr float kRearAxleToCenterRear = 1136.7f;  // mm
    static constexpr float kMmToMeter = 1000.0f;
};

//=============================================================================
// Basic Geometry Types
//=============================================================================
struct Point2f {
    float x = 0.0f;
    float y = 0.0f;

    Point2f() = default;
    Point2f(float px, float py) : x(px), y(py) {}
    
    float distanceTo(const Point2f& other) const {
        float dx = x - other.x;
        float dy = y - other.y;
        return std::sqrt(dx * dx + dy * dy);
    }
};

struct Point2i {
    int32_t x = 0;
    int32_t y = 0;

    Point2i() = default;
    Point2i(int32_t px, int32_t py) : x(px), y(py) {}
    
    Point2f toFloat() const { return Point2f(static_cast<float>(x), static_cast<float>(y)); }
};

struct Point3f {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;

    Point3f() = default;
    Point3f(float px, float py, float pz = 0.0f) : x(px), y(py), z(pz) {}
    
    Point2f toPoint2f() const { return Point2f(x, y); }
};

struct Pose4f {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    float yaw = 0.0f;

    Pose4f() = default;
    Pose4f(float px, float py, float pz, float pyaw) : x(px), y(py), z(pz), yaw(pyaw) {}
};

struct SlotCorners {
    Point2f corners[Limits::kMaxCornerPoints];  // A, B, C, D in order
    
    Point2f& operator[](size_t idx) { return corners[idx]; }
    const Point2f& operator[](size_t idx) const { return corners[idx]; }
};

struct SlotChamfer {
    Point2f aisleSidePoint;
    Point2f slotSidePoint;
};

//=============================================================================
// Common Header
//=============================================================================
struct Header {
    uint16_t flag = 0;
    uint32_t version = 0;
    uint16_t sequence = 0;
    uint64_t pubTimestampUs = 0;
};

//=============================================================================
// Enumeration Types - APA Status
//=============================================================================
enum class ApaStatus : uint8_t {
    Off = 0,
    Standby = 1,
    Search = 2,
    ParkOut = 3,
    Enabled = 4,
    Guidance = 5,
    Finished = 6,
    Error = 7
};

enum class LocStatus : uint8_t {
    Null = 0x00,
    Searching = 0x01,
    Parking = 0x02,
    Standby = 0x03
};

//=============================================================================
// Enumeration Types - Slot Related
//=============================================================================
enum class SlotType : uint8_t {
    Null = 0,
    Parallel = 1,
    Perpendicular = 2,
    Oblique = 3,
    ReverseOblique = 4,
    UserDefined = 5
};

enum class SlotSource : uint8_t {
    Null = 0,
    Vision = 1,
    Ultrasonic = 2,
    Fused = 3,
};

enum class SlotOccupancyStatus : uint8_t {
    Unknown = 0,
    Empty = 1,
    Occupied = 2,
    Locked = 3
};

enum class SlotBottomType : uint8_t {
    Empty = 0,
    Boundary = 1,
    Wall = 2,
    Other = 3
};

enum class SlotDisplayStatus : uint8_t {
    Unknown = 0,
    Searching = 1,
    Detected = 2,
    Available = 3,
    Occupied = 4,
    Selected = 5,
    Unavailable = 6,
    Recommended = 7
};

enum class InSlotObstacleType : uint8_t {
    None = 0,
    Vehicle = 1,
    ConeOrSign = 2,
    SlotLock = 3,
    Stopper = 4,
    Other = 5
};

enum class InSlotObstacleLocation : uint8_t {
    None = 0,
    Left = 1,
    Right = 2,
    BothSide = 3,
    Center = 4
};

enum class ParkingDirection : uint8_t {
    Unspecified = 0x00,
    Front = 0x01,
    Back = 0x02,
    Default = 0x03
};

//=============================================================================
// Enumeration Types - Obstacle Related
//=============================================================================
enum class ObstacleType : uint8_t {
    Null = 0x00,
    Vehicle = 0x01,
    Pedestrian = 0x02,
    Bike = 0x03,
    Column = 0x04,
    TrafficCone = 0x05,
    WheelStop = 0x06,
    SpeedBump = 0x07,
    SlotLock = 0x08,
    ParkLimiterPlate = 0x09,
    UnknownHigh = 0x0A,
    UnknownLow = 0x0B,
    GarageOn = 0x0C,
    GarageOff = 0x0D
};

enum class ObstacleBoundaryType : uint8_t {
    Cube = 0x00,
    Cylindrical = 0x01,
    Null = 0x02
};

enum class MotionStatus : uint8_t {
    Unknown = 0x00,
    Moving = 0x01,
    Still = 0x02
};

//=============================================================================
// Enumeration Types - Planning Related
//=============================================================================
enum class SlotParkable : uint8_t {
    Null = 0x00,
    Pass = 0x01,
    Fail = 0x02
};

//=============================================================================
// Input Data Structures
//=============================================================================

/// State machine output - from apa::status::Output2ALL
struct StateMachineInput {
    Header header;
    uint8_t apaStatusReq = 0;       // APA status request
    uint8_t hppStatusReq = 0;       // HPP status request
    uint8_t drivingOrParking1 = 0;  // Driving or parking mode 1
    uint8_t drivingOrParking2 = 0;  // Driving or parking mode 2
    uint8_t apaStartParkingReq = 0; // Start parking request
    uint8_t apaSrchInterrupt = 0;   // Search interrupt flag
    
    /// Get APA status as enum
    ApaStatus getApaStatus() const {
        return static_cast<ApaStatus>(apaStatusReq);
    }
    
    /// Check if in parking mode
    bool isInParkingMode() const {
        return apaStatusReq != static_cast<uint8_t>(ApaStatus::Off);
    }
    
    /// Check if searching
    bool isSearching() const {
        return apaStatusReq == static_cast<uint8_t>(ApaStatus::Search);
    }
    
    /// Check if in guidance phase
    bool isInGuidance() const {
        return apaStatusReq == static_cast<uint8_t>(ApaStatus::Guidance);
    }
    
    /// Check if park-out mode
    bool isParkOut() const {
        return apaStatusReq == static_cast<uint8_t>(ApaStatus::ParkOut);
    }
    
    /// Check if should clear data
    bool shouldClearData() const {
        auto status = static_cast<ApaStatus>(apaStatusReq);
        return status == ApaStatus::Off ||
               status == ApaStatus::Standby ||
               status == ApaStatus::Finished ||
               status == ApaStatus::Error;
    }
};

/// Vehicle pose - from apa::loc::Vehicle_Pose
struct VehiclePoseInput {
    Header header;
    LocStatus locStatus = LocStatus::Null;
    float posX = 0.0f;           // X position (mm)
    float posY = 0.0f;           // Y position (mm)
    float posZ = 0.0f;           // Z position (mm)
    float quaternionQx = 0.0f;
    float quaternionQy = 0.0f;
    float quaternionQz = 0.0f;
    float quaternionQw = 1.0f;
    float roll = 0.0f;           // Roll angle (rad)
    float pitch = 0.0f;          // Pitch angle (rad)
    float yaw = 0.0f;            // Yaw angle (rad)
    uint64_t timestampUs = 0;    // Timestamp in microseconds
    
    Point2f getPosition2D() const { return Point2f(posX, posY); }
    Point3f getPosition3D() const { return Point3f(posX, posY, posZ); }
};

/// Parking slot from map - from apa::loc::parkingSlots
struct MapParkingSlot {
    uint64_t id = 0;
    uint32_t psType = 0;
    float width = 0.0f;
    float length = 0.0f;
    Point3f center;
    Point3f wideDirection;
    Point3f longDirection;
    bool isOccupied = false;
    
    SlotType getSlotType() const {
        return static_cast<SlotType>(psType);
    }
};

/// Lane line from map
struct LaneLine {
    uint64_t id = 0;
    uint32_t lineType = 0;
    std::array<Point3f, 100> points;
    size_t pointCount = 0;
};

/// Stop line from map
struct StopLine {
    uint64_t id = 0;
    Point3f startPt;
    Point3f endPt;
};

/// Speed bump from map
struct Bump {
    uint64_t id = 0;
    float width = 0.0f;
    float length = 0.0f;
    Point3f endPt;
    Point3f wideDirection;
    Point3f longDirection;
};

/// Cross walk from map
struct CrossWalk {
    uint64_t id = 0;
    float width = 0.0f;
    float length = 0.0f;
    Point3f endPt;
    Point3f wideDirection;
    Point3f longDirection;
};

/// Arrow from map
struct Arrow {
    uint64_t id = 0;
    uint32_t arrowType = 0;
    std::array<Point3f, 20> points;
    size_t pointCount = 0;
};

/// Map info - from apa::loc::MapInfo
struct MapInfoInput {
    Header header;
    uint64_t timestamp = 0;
    uint64_t mapId = 0;
    
    std::array<MapParkingSlot, Limits::kMaxSlotCount> parkingSlots;
    size_t parkingSlotCount = 0;
    
    std::array<LaneLine, Limits::kMaxLaneLines> laneLines;
    size_t laneLineCount = 0;
    
    std::array<StopLine, Limits::kMaxStopLines> stopLines;
    size_t stopLineCount = 0;
    
    std::array<Bump, Limits::kMaxBumps> bumps;
    size_t bumpCount = 0;
    
    std::array<CrossWalk, Limits::kMaxCrossWalks> crossWalks;
    size_t crossWalkCount = 0;
    
    std::array<Arrow, Limits::kMaxArrows> arrows;
    size_t arrowCount = 0;
};

/// USS slot - from uss::UssSlot
struct UssSlot {
    std::array<Point2i, Limits::kMaxCornerPoints> pointABCD;
    SlotBottomType bottomType = SlotBottomType::Empty;
    InSlotObstacleLocation obsStatus = InSlotObstacleLocation::None;
    uint16_t leftSideObstaclePosition = 0;
    uint16_t rightSideObstaclePosition = 0;
    uint64_t timestamp = 0;
    
    /// Convert to float corners
    SlotCorners getCornersFloat() const {
        SlotCorners corners;
        for (size_t i = 0; i < Limits::kMaxCornerPoints; ++i) {
            corners[i] = pointABCD[i].toFloat();
        }
        return corners;
    }
};

/// USS slot list - from uss::UssSlotList
struct UssSlotListInput {
    std::array<UssSlot, Limits::kMaxUssSlots> slots;
    
    bool hasValidSlot(size_t idx) const {
        if (idx >= Limits::kMaxUssSlots) return false;
        // Check if slot has valid points (non-zero)
        const auto& slot = slots[idx];
        return (slot.pointABCD[0].x != 0 || slot.pointABCD[0].y != 0);
    }
};

/// Obstacle motion info
struct ObstacleMotion {
    bool isValid = false;
    Point3f center;
    Point3f centerUncertainty;
    Point3f velocity;
    Point3f velocityUncertainty;
    Point3f acceleration;
    Point3f accelerationUncertainty;
    float velocityHeading = 0.0f;
    float velocityHeadingUncertainty = 0.0f;
    float velocityHeadingRate = 0.0f;
    float velocityHeadingRateUncertainty = 0.0f;
};

/// Obstacle trajectory point
struct ObstacleTrajectoryPoint {
    uint64_t deltaTNs = 0;
    Point3f center;
    Point3f direction;
};

/// Obstacle trajectory
struct ObstacleTrajectory {
    MotionStatus motionStatus = MotionStatus::Unknown;
    float confidence = 0.0f;
    std::array<ObstacleTrajectoryPoint, Limits::kMaxTrajectoryPoints> points;
    size_t pointCount = 0;
};

/// Single obstacle - from apa::fus::EmapObs
struct Obstacle {
    uint64_t frameIndex = 0;
    uint8_t failSafe = 0;
    uint16_t obsId = 0;
    ObstacleType obsType = ObstacleType::Null;
    Point3f obsCenter;
    int32_t age = 0;
    Point3f obsDirection;
    ObstacleMotion motionInfo;
    ObstacleTrajectory trajectory;
    ObstacleBoundaryType boundaryType = ObstacleBoundaryType::Null;
    std::array<Point3f, Limits::kMaxBoundingBoxPoints> boundingBox;
    size_t boundingBoxCount = 0;
    uint8_t confidence = 0;
};

/// Obstacle list - from apa::fus::PkEmapObs
struct ObstacleListInput {
    Header header;
    std::array<Obstacle, Limits::kMaxObstacleCount> obstacles;
    
    size_t getValidCount() const {
        size_t count = 0;
        for (const auto& obs : obstacles) {
            if (obs.obsType != ObstacleType::Null) {
                ++count;
            }
        }
        return count;
    }
};

/// Grid map work mode
enum class GridMapWorkMode : uint8_t {
    Null = 0x00,
    APA = 0x01,
    Guidance = 0x02,
    Cruise = 0x03
};

/// Grid map - from apa::fus::EmapGridmap (simplified, full grid data not copied)
struct GridMapInput {
    Header header;
    uint64_t timestamp = 0;
    GridMapWorkMode workMode = GridMapWorkMode::Null;
    Pose4f vagueCenterPoint;
    Pose4f preciseCenterPoint;
};

/// Planning slot result - from apa::planning::EmapSlotsRes
struct PlanningSlotResult {
    uint32_t slotId = 0;
    SlotParkable slotParkable = SlotParkable::Null;
    uint8_t slotScore = 0;
};

/// Segment point for target position
struct SegmentPoint {
    float x = 0.0f;
    float y = 0.0f;
    float yaw = 0.0f;
    float curv = 0.0f;
};

/// Target position
struct TargetPosition {
    SegmentPoint finalTargetPos;
    SegmentPoint stageTargetPos;
};

/// Planning parking space - from apa::planning::ParkingSpace
struct PlanningParkingSpace {
    Header header;
    Point2f p0, p1, p2, p3, p4, p5, p6, p7;
    SlotChamfer chamferP0;
    SlotChamfer chamferP5;
};

/// Decision from planning to PSD - from apa::planning::DecPlan2Psd
struct PlanningDecisionInput {
    Header header;
    uint32_t frameId = 0;
    uint8_t validSlotNum = 0;
    PlanningSlotResult slotResult;
    PlanningParkingSpace parkingSpace;
    TargetPosition targetPos;
};

/// Planning to status - from apa::planning::Plan2Status
struct PlanningStatusInput {
    Header header;
    uint8_t slotWidthStatus = 0;
    uint8_t pncStatus = 0;
};

/// Select slot command - from apa::uic::SelectParkingSlot
struct SelectSlotInput {
    uint8_t slotId = 0;
    ParkingDirection parkingDirection = ParkingDirection::Unspecified;
    uint64_t timestamp = 0;
    bool isValid = false;
};

/// Custom slot command - from apa::uic::CustomizeParkingSlot
struct CustomSlotInput {
    float pt = 0.0f;
    uint8_t parkingSlotType = 0;
    uint8_t slotSurfaceType = 0;
    uint8_t targetSlotId = 0;
    uint64_t timestamp = 0;
    bool isValid = false;
};

//=============================================================================
// Aggregated Input Structure
//=============================================================================

/**
 * @brief Aggregated input data for PSD fusion processing
 * 
 * This structure contains all input data converted from external interfaces.
 * The conversion is done in the adapter layer (RunTask).
 */
struct PsdFusionInput {
    uint64_t currentTimestamp = 0;  // Current system time (us)
    
    StateMachineInput stateMachine;
    VehiclePoseInput vehiclePose;
    MapInfoInput mapInfo;
    UssSlotListInput ussSlots;
    ObstacleListInput obstacles;
    GridMapInput gridMap;
    PlanningDecisionInput planningDecision;
    PlanningStatusInput planningStatus;
    
    SelectSlotInput selectSlot;
    CustomSlotInput customSlot;
    
    /// Convenience accessors
    bool isInParkingMode() const { return stateMachine.isInParkingMode(); }
    bool isSearching() const { return stateMachine.isSearching(); }
    bool isInGuidance() const { return stateMachine.isInGuidance(); }
    bool isParkOut() const { return stateMachine.isParkOut(); }
    bool shouldClearData() const { return stateMachine.shouldClearData(); }
    
    Point2f getVehiclePosition() const { return vehiclePose.getPosition2D(); }
    float getVehicleYaw() const { return vehiclePose.yaw; }
};

//=============================================================================
// Output Data Structures
//=============================================================================

/// Single parking slot output - maps to apa::psd::parking_slot_t
struct ParkingSlotOutput {
    uint32_t slotId = 0;
    uint32_t slotDisplayId = 0;
    uint8_t slotDisplayStatus = 0;
    std::array<Point3f, Limits::kMaxCornerPoints> slotCorners;
    float slotEntryLength = 0.0f;
    float slotNonEntryLength = 0.0f;
    uint8_t slotType = 0;
    uint8_t slotSurfaceType = 0;
    uint8_t slotBottomType = 0;
    uint8_t slotObstacleType = 0; 
    uint8_t slotObstacleLocation = 0;
    float slotStopperDistance = 0.0f;
    float slotConfidence = 0.0f;
    uint8_t slotSource = 0; 
    bool slotOccupied = false;
    bool slotSelected = false;
    ParkingDirection parkInType = ParkingDirection::Unspecified;
    float ussSlotOppositeSpace = 0.0f;
    float ussSlotTransverseSpace = 0.0f;
    
    void clear() {
        *this = ParkingSlotOutput{};
    }
};

/// Parking slot list output - maps to apa::psd::ParkingSlotList
struct ParkingSlotListOutput {
    Header header;
    uint64_t slotNum = 0;
    std::array<ParkingSlotOutput, Limits::kMaxSlotCount> slots;
    
    void clear() {
        header = Header{};
        slotNum = 0;
        for (auto& slot : slots) {
            slot.clear();
        }
    }
    
    bool addSlot(const ParkingSlotOutput& slot) {
        if (slotNum >= Limits::kMaxSlotCount) return false;
        slots[slotNum++] = slot;
        return true;
    }
};

/// Parking slot stopper output - maps to apa::psd::ParkingSlotStopper
struct ParkingSlotStopperOutput {
    Header header;
    std::array<int16_t, Limits::kMaxStopperCount> limitBarX = {-20000, -20000};
    std::array<int16_t, Limits::kMaxStopperCount> limitBarY = {-20000, -20000};
    std::array<int16_t, Limits::kMaxStopperCount> limitBarYaw = {0, 0};
    std::array<uint16_t, Limits::kMaxStopperCount> limitBarId = {0, 0};
    
    void clear() {
        header = Header{};
        limitBarX.fill(-20000);
        limitBarY.fill(-20000);
        limitBarYaw.fill(0);
        limitBarId.fill(0);
    }
};

/// Target slot label output - maps to apa::psd::TargetSlotLabel
struct TargetSlotLabelOutput {
    Header header;
    uint16_t targetSlotId = 0;
    
    void clear() {
        header = Header{};
        targetSlotId = 0;
    }
};

//=============================================================================
// Aggregated Output Structure
//=============================================================================

/**
 * @brief Aggregated output data from PSD fusion processing
 * 
 * This structure contains all output data that will be converted to external
 * interface formats in the adapter layer (RunTask).
 */
struct PsdFusionOutput {
    ParkingSlotListOutput slotList;
    ParkingSlotStopperOutput slotStopper;
    TargetSlotLabelOutput targetLabel;
    
    void clear() {
        slotList.clear();
        slotStopper.clear();
        targetLabel.clear();
    }
};

//=============================================================================
// Internal Processing State (for algorithm use)
//=============================================================================

/// Internal slot representation with additional processing fields
struct InternalSlot {
    uint32_t slotId = 0;
    SlotType type = SlotType::Null;
    SlotSource source = SlotSource::Null;
    SlotOccupancyStatus occupancy = SlotOccupancyStatus::Unknown;
    SlotBottomType bottomType = SlotBottomType::Empty;
    
    SlotCorners localCorners;       // In vehicle coordinate frame
    SlotCorners worldCorners;       // In world coordinate frame
    
    float stopperDistance = 0.0f;
    std::array<Point2f, Limits::kMaxStopperCount> stopperPoints;
    
    // In-slot obstacle info
    bool hasStopperInSlot = false;
    bool hasLockInSlot = false;
    bool hasObstacleInSlot = false;
    InSlotObstacleType obstacleType = InSlotObstacleType::None;
    InSlotObstacleLocation obstacleLocation = InSlotObstacleLocation::None;
    
    // Slot dimensions
    float entryLength = 0.0f;
    float nonEntryLength = 0.0f;
    float oppositeSpace = 0.0f;
    float transverseSpace = 0.0f;
    
    // Processing flags
    bool isTargetSlot = false;
    bool isSelected = false;
    bool isRecommended = false;
    bool isParkable = false;
    bool needsRelease = true;
    
    // Quality metrics
    float confidence = 0.0f;
    uint8_t score = 0;
    int32_t age = 0;
    uint64_t firstDetectTime = 0;
    uint64_t lastUpdateTime = 0;
    
    // Display info
    uint32_t displayId = 0;
    SlotDisplayStatus displayStatus = SlotDisplayStatus::Unknown;
    ParkingDirection parkInDirection = ParkingDirection::Unspecified;
    
    void clear() {
        *this = InternalSlot{};
    }
};

/// Internal slot list for processing
struct InternalSlotList {
    std::array<InternalSlot, Limits::kMaxSlotCount> slots;
    size_t count = 0;
    
    uint32_t selectedSlotId = 0;
    uint32_t recommendedSlotId = 0;
    uint32_t targetSlotId = 0;
    
    void clear() {
        count = 0;
        selectedSlotId = 0;
        recommendedSlotId = 0;
        targetSlotId = 0;
        for (auto& slot : slots) {
            slot.clear();
        }
    }
    
    InternalSlot* findSlotById(uint32_t id) {
        for (size_t i = 0; i < count; ++i) {
            if (slots[i].slotId == id) {
                return &slots[i];
            }
        }
        return nullptr;
    }
    
    const InternalSlot* findSlotById(uint32_t id) const {
        for (size_t i = 0; i < count; ++i) {
            if (slots[i].slotId == id) {
                return &slots[i];
            }
        }
        return nullptr;
    }
    
    bool addSlot(const InternalSlot& slot) {
        if (count >= Limits::kMaxSlotCount) return false;
        slots[count++] = slot;
        return true;
    }
    
    void removeSlotAt(size_t idx) {
        if (idx >= count) return;
        for (size_t i = idx; i < count - 1; ++i) {
            slots[i] = slots[i + 1];
        }
        slots[count - 1].clear();
        --count;
    }
};
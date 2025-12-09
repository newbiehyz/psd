#include "psd_slot_fusion.h"
#include <cstring>
#include <algorithm>
#include <limits>
#include <vector>

PsdSlotFusion::PsdSlotFusion() {
    Reset();
}

void PsdSlotFusion::Reset() {
    current_slots_.clear();
    saved_target_slot_.clear();
    has_saved_target_slot_ = false;

    hmi_selected_id_ = 0;
    hmi_temp_id_ = 0;
    vcu_selected_id_ = 0;
    recommended_id_ = 0;
    final_selected_id_ = 0;

    is_park_out_mode_ = false;
    is_vehicle_still_ = true;
    previous_pose_ = VehiclePoseInput{};

    last_apa_status_ = 0;
    has_cleared_for_search_ = false;
    still_frame_counter_ = 0;
}

//=============================================================================
// Main Processing
//=============================================================================

void PsdSlotFusion::Process(
    const PsdFusionInput& input,
    PsdFusionOutput& output)
{

    // Update vehicle state
    uint8_t current_status = input.stateMachine.apaStatusReq;
    
    // Detect park-out mode
    if (current_status == static_cast<uint8_t>(ApaStatus::ParkOut)) {
        is_park_out_mode_ = true;
    } else if (current_status == static_cast<uint8_t>(ApaStatus::Search)) {
        is_park_out_mode_ = false;
    }

    // Detect vehicle still state
    float dx = input.vehiclePose.posX - previous_pose_.posX;
    float dy = input.vehiclePose.posY - previous_pose_.posY;
    float dyaw = input.vehiclePose.yaw - previous_pose_.yaw;
    bool has_changed = (std::fabs(dx) > kStillThreshold ||
                        std::fabs(dy) > kStillThreshold ||
                        std::fabs(dyaw) > 0.5f);  // ~0.5 degree

    if (!has_changed) {
        ++still_frame_counter_;
        if (still_frame_counter_ >= kStillFrameCount) {
            is_vehicle_still_ = true;
        }
    } else {
        still_frame_counter_ = 0;
        is_vehicle_still_ = false;
    }
    previous_pose_ = input.vehiclePose;

    // Initialize working slot list
    InternalSlotList slot_list;
    slot_list.clear();

    // Step 1: Clear data based on APA status
    ProcClearByStatus(input, slot_list);

    // Step 2: Convert mapinfo slots to internal format
    ProcConvertMapSlots(input, slot_list);

    // // Step 3: Convert and merge USS slots（ TODO: not available, and waiting for input)
    // ProcConvertUssSlots(input, slot_list);

    // Step 4: Process obstacles
    ProcObstacles(input, slot_list);

    // Step 5: Select target slot
    ProcSelectTargetSlot(input, slot_list);

    // Step 6: Update display status
    ProcUpdateDisplayStatus(input, slot_list);

    // Step 7: Convert to output format
    ProcConvertToOutput(input, slot_list, output);

    // Update status tracking
    last_apa_status_ = current_status;
    current_slots_ = slot_list;
}

//=============================================================================
// User Interaction
//=============================================================================

void PsdSlotFusion::UpdateSelectSlot(const SelectSlotInput& select_slot) {
    if (select_slot.isValid) {
        hmi_selected_id_ = select_slot.slotId;
    }
}

void PsdSlotFusion::UpdateCustomSlot(const CustomSlotInput& custom_slot) {

}

//=============================================================================
// Processing Steps Implementation
//=============================================================================

void PsdSlotFusion::ProcClearByStatus(const PsdFusionInput& input, InternalSlotList& slot_list)
{
    uint8_t status = input.stateMachine.apaStatusReq;
    
    if (status == static_cast<uint8_t>(ApaStatus::Off) ||
        status == static_cast<uint8_t>(ApaStatus::Standby) ||
        status == static_cast<uint8_t>(ApaStatus::Finished) ||
        status == static_cast<uint8_t>(ApaStatus::Error)) 
    {
        slot_list.clear();
        hmi_temp_id_ = 0;
        vcu_selected_id_ = 0;
        final_selected_id_ = 0;
        recommended_id_ = 0;
        has_cleared_for_search_ = false;
        has_saved_target_slot_ = false;
    }

    // Clear once on first entry to Search state
    else if (status == static_cast<uint8_t>(ApaStatus::Search) && !has_cleared_for_search_) {
        slot_list.clear();
        has_cleared_for_search_ = true;
    }
}

void PsdSlotFusion::ProcConvertMapSlots(const PsdFusionInput& input, InternalSlotList& slot_list)
{
    const auto& pose = input.vehiclePose;
    float theta = pose.yaw * static_cast<float>(M_PI) / 180.0f;
    float cos_theta = std::cos(theta);
    float sin_theta = std::sin(theta);

    for (size_t i = 0; i < input.mapInfo.parkingSlotCount; ++i) {
        const auto& map_slot = input.mapInfo.parkingSlots[i];
        
        InternalSlot slot;

        slot.slotId = static_cast<uint32_t>(map_slot.id);
        slot.displayId = slot.slotId;
        slot.displayStatus = map_slot.isOccupied ? SlotDisplayStatus::Occupied : SlotDisplayStatus::Available;

        // Calculate world and local corners
        Point3f c = map_slot.center;
        Point3f lon = map_slot.longDirection;
        Point3f wid = map_slot.wideDirection;
        float half_len = map_slot.length * 0.5f;
        float half_wid = map_slot.width * 0.5f;

        Point2f pt0_w(c.x + lon.x * half_len - wid.x * half_wid,
                      c.y + lon.y * half_len - wid.y * half_wid);
        Point2f pt1_w(c.x + lon.x * half_len + wid.x * half_wid,
                      c.y + lon.y * half_len + wid.y * half_wid);
        Point2f pt2_w(c.x - lon.x * half_len + wid.x * half_wid,
                      c.y - lon.y * half_len + wid.y * half_wid);
        Point2f pt3_w(c.x - lon.x * half_len - wid.x * half_wid,
                      c.y - lon.y * half_len - wid.y * half_wid);

        slot.worldCorners[0] = Point2f(pt0_w.x, pt0_w.y);
        slot.worldCorners[1] = Point2f(pt1_w.x, pt1_w.y);
        slot.worldCorners[2] = Point2f(pt2_w.x , pt2_w.y);
        slot.worldCorners[3] = Point2f(pt3_w.x, pt3_w.y);

        float pose_x_m = pose.posX;
        float pose_y_m = pose.posY;

        for (int j = 0; j < 4; ++j) {
            float wx = slot.worldCorners[j].x;
            float wy = slot.worldCorners[j].y;
            float dx = wx - pose_x_m;
            float dy = wy - pose_y_m;
            float bx = dx * cos_theta + dy * sin_theta;
            float by = -dx * sin_theta + dy * cos_theta;
            slot.localCorners[j] = Point2f(-by, bx);
        }
        AdjustCornerOrder(slot.localCorners);


        slot.entryLength = CalculateEntryLength(slot.localCorners);
        slot.nonEntryLength = CalculateNonEntryLength(slot.localCorners);

        slot.type = ConvertMapSlotType(map_slot.psType);
        slot.source = SlotSource::Vision;
        slot.occupancy = map_slot.isOccupied ? SlotOccupancyStatus::Occupied : SlotOccupancyStatus::Empty;

        slot_list.addSlot(slot);
    }
}

void PsdSlotFusion::ProcConvertUssSlots(const PsdFusionInput& input, InternalSlotList& slot_list)
{
    // Process USS slots - merge with existing vision/map slots
    for (size_t i = 0; i < Limits::kMaxUssSlots; ++i) {
        if (!input.ussSlots.hasValidSlot(i)) {
            continue;
        }

        const auto& uss_slot = input.ussSlots.slots[i];
        
        // USS slots get IDs starting from 10000
        uint32_t uss_slot_id = 10000 + static_cast<uint32_t>(i);

        // Check if this USS slot overlaps with any existing slot
        bool merged = false;
        for (size_t j = 0; j < slot_list.count; ++j) {
            // Simple overlap check: compare centers
            Point2f uss_center;
            for (int k = 0; k < 4; ++k) {
                uss_center.x += static_cast<float>(uss_slot.pointABCD[k].x);
                uss_center.y += static_cast<float>(uss_slot.pointABCD[k].y);
            }
            uss_center.x /= 4.0f;
            uss_center.y /= 4.0f;

            Point2f slot_center;
            for (int k = 0; k < 4; ++k) {
                slot_center.x += slot_list.slots[j].localCorners[k].x;
                slot_center.y += slot_list.slots[j].localCorners[k].y;
            }
            slot_center.x /= 4.0f;
            slot_center.y /= 4.0f;

            float dist = uss_center.distanceTo(slot_center);
            if (dist < 1500.0f) {  // 1.5m threshold for merge
                // Merge: update source to Fused
                slot_list.slots[j].source = SlotSource::Fused;
                slot_list.slots[j].bottomType = uss_slot.bottomType;
                merged = true;
                break;
            }
        }

        if (!merged) {
            // Add as new USS-only slot
            InternalSlot slot;
            slot.slotId = uss_slot_id;
            slot.type = SlotType::Perpendicular;  // Default for USS
            slot.source = SlotSource::Ultrasonic;
            slot.bottomType = uss_slot.bottomType;

            for (int k = 0; k < 4; ++k) {
                slot.localCorners[k] = Point2f(
                    static_cast<float>(uss_slot.pointABCD[k].x),
                    static_cast<float>(uss_slot.pointABCD[k].y));
            }

            AdjustCornerOrder(slot.localCorners);
            slot.entryLength = CalculateEntryLength(slot.localCorners);
            slot.nonEntryLength = CalculateNonEntryLength(slot.localCorners);
            slot.displayId = uss_slot_id;
            slot.displayStatus = SlotDisplayStatus::Available;
            slot.isParkable = true;

            slot_list.addSlot(slot);
        }
    }
}

void PsdSlotFusion::ProcObstacles(const PsdFusionInput& input, InternalSlotList& slot_list)
{
    if (slot_list.count == 0) {
        return;
    }

    // 遍历所有障碍物
    for (size_t i = 0; i < Limits::kMaxObstacleCount; ++i) {
        const auto& obs = input.obstacles.obstacles[i];
        
        if (obs.obsType == ObstacleType::Null) {
            continue;
        }

        Point2f obs_pt(obs.obsCenter.x, obs.obsCenter.y);

        // Step 1: 找到距离障碍物最近的车位
        float min_distance = std::numeric_limits<float>::max();
        int nearest_index = -1;

        for (size_t j = 0; j < slot_list.count; ++j) {
            auto& slot = slot_list.slots[j];
            
            // 计算车位中心
            Point2f slot_center;
            for (int k = 0; k < 4; ++k) {
                slot_center.x += slot.localCorners[k].x;
                slot_center.y += slot.localCorners[k].y;
            }
            slot_center.x /= 4.0f;
            slot_center.y /= 4.0f;

            // 计算障碍物到车位中心的距离
            float distance = std::hypot(obs_pt.x - slot_center.x, obs_pt.y - slot_center.y);

            if (distance < min_distance) {
                min_distance = distance;
                nearest_index = static_cast<int>(j);
            }
        }

        if (nearest_index == -1) {
            continue;
        }

        auto& nearest_slot = slot_list.slots[nearest_index];

        // Step 2: 使用向量投影法判断障碍物是否在最近车位内
        bool is_in_slot = PointInQuad(obs_pt, nearest_slot.localCorners);

        if (!is_in_slot) {
            continue;
        }

        // Step 3: 根据障碍物类型处理
        nearest_slot.hasObstacleInSlot = true;

        // 获取车位四个角点 (A=0, B=1, C=2, D=3)
        const Point2f& ptA = nearest_slot.localCorners[0];
        const Point2f& ptB = nearest_slot.localCorners[1];
        const Point2f& ptC = nearest_slot.localCorners[2];
        const Point2f& ptD = nearest_slot.localCorners[3];

        if (obs.obsType == ObstacleType::WheelStop) {
            // ============ 限位块处理 ============
            nearest_slot.hasStopperInSlot = true;
            nearest_slot.obstacleType = InSlotObstacleType::Stopper;
            
            // 存储限位块坐标
            nearest_slot.stopperPoints[0] = obs_pt;

            // 根据车位类型计算距离和位置
            if (nearest_slot.type == SlotType::Perpendicular || 
                nearest_slot.type == SlotType::Oblique ||
                nearest_slot.type == SlotType::ReverseOblique) {
                // 垂直/斜列车位：计算到AB边和CD边的距离
                float dist_to_AB = CalcPointToLineDistance(obs_pt, ptA, ptB);
                float dist_to_CD = CalcPointToLineDistance(obs_pt, ptC, ptD);
                
                nearest_slot.stopperDistance = std::max(dist_to_AB, dist_to_CD);
                
                // 判断限位块位置
                if (dist_to_AB > dist_to_CD) {
                    nearest_slot.obstacleLocation = InSlotObstacleLocation::Center;  // 靠近CD边（底部）
                    // 限位块在底部，不影响泊入
                } else if (dist_to_AB < dist_to_CD) {
                    nearest_slot.obstacleLocation = InSlotObstacleLocation::Center;  // 靠近AB边（入口）
                    // 限位块在入口附近，标记为占用
                    nearest_slot.occupancy = SlotOccupancyStatus::Occupied;
                }
            } else {
                // 水平车位：计算到BC边和AD边的距离
                float dist_to_BC = CalcPointToLineDistance(obs_pt, ptB, ptC);
                float dist_to_AD = CalcPointToLineDistance(obs_pt, ptA, ptD);
                
                nearest_slot.stopperDistance = dist_to_BC;
                
                // 判断限位块位置
                if (dist_to_BC > dist_to_AD) {
                    nearest_slot.obstacleLocation = InSlotObstacleLocation::Left;  // 靠近AD边
                } else if (dist_to_BC < dist_to_AD) {
                    nearest_slot.obstacleLocation = InSlotObstacleLocation::Right; // 靠近BC边
                } else {
                    nearest_slot.obstacleLocation = InSlotObstacleLocation::Center;
                }
            }
        }


        else if (obs.obsType == ObstacleType::SlotLock) {
            // ============ 地锁处理 ============
            
            // 垂直车位：距离AB边过远的地锁认为无效 (ADAS-3363)
            if (nearest_slot.type == SlotType::Perpendicular ||
                nearest_slot.type == SlotType::Oblique ||
                nearest_slot.type == SlotType::ReverseOblique) {
                float dist_to_AB = CalcPointToLineDistance(obs_pt, ptA, ptB);
                if (dist_to_AB >= kVehicleLength / 2.0f) {
                    // 距离过远，认为无效，不更新地锁状态
                    continue;
                }
            }
            
            // 有效地锁
            nearest_slot.hasLockInSlot = true;
            nearest_slot.obstacleType = InSlotObstacleType::SlotLock;
            nearest_slot.occupancy = SlotOccupancyStatus::Locked;
            nearest_slot.displayStatus = SlotDisplayStatus::Occupied;
            nearest_slot.isParkable = false;
        }


        else if (obs.obsType == ObstacleType::Vehicle) {
            // ============ 车辆障碍物处理 ============
            nearest_slot.obstacleType = InSlotObstacleType::Vehicle;
            nearest_slot.occupancy = SlotOccupancyStatus::Occupied;
            nearest_slot.displayStatus = SlotDisplayStatus::Occupied;
            nearest_slot.isParkable = false;
        }
        else if (obs.obsType == ObstacleType::TrafficCone ||
                 obs.obsType == ObstacleType::Column) {
            // ============ 锥桶/立柱处理 ============
            nearest_slot.obstacleType = InSlotObstacleType::ConeOrSign;
            nearest_slot.occupancy = SlotOccupancyStatus::Occupied;
            nearest_slot.displayStatus = SlotDisplayStatus::Unavailable;
            nearest_slot.isParkable = false;
        }
        else {
            // ============ 其他障碍物处理 ============
            nearest_slot.obstacleType = InSlotObstacleType::Other;
            nearest_slot.occupancy = SlotOccupancyStatus::Occupied;
            nearest_slot.displayStatus = SlotDisplayStatus::Occupied;
            nearest_slot.isParkable = false;
        }

        // 判断障碍物在车位内的左/右/中位置
        float slot_center_y = (ptA.y + ptB.y + ptC.y + ptD.y) / 4.0f;
        if (obs_pt.y < slot_center_y - 0.5f) {
            nearest_slot.obstacleLocation = InSlotObstacleLocation::Left;
        } else if (obs_pt.y > slot_center_y + 0.5f) {
            nearest_slot.obstacleLocation = InSlotObstacleLocation::Right;
        } else {
            if (nearest_slot.obstacleLocation == InSlotObstacleLocation::None) {
                nearest_slot.obstacleLocation = InSlotObstacleLocation::Center;
            }
        }
    }
}

void PsdSlotFusion::ProcSelectTargetSlot(
    const PsdFusionInput& input,
    InternalSlotList& slot_list)
{
    uint8_t status = input.stateMachine.apaStatusReq;

    // Handle HMI selection caching
    if (hmi_selected_id_ != 0) {
        hmi_temp_id_ = hmi_selected_id_;
        hmi_selected_id_ = 0;  // Clear after caching
    }

    // Determine final selection (HMI/VCU)
    if (vcu_selected_id_ != 0 && hmi_temp_id_ == 0) {
        final_selected_id_ = vcu_selected_id_;
    } else if (vcu_selected_id_ == 0 && hmi_temp_id_ != 0) {
        final_selected_id_ = hmi_temp_id_;
    } else if (vcu_selected_id_ != 0 && hmi_temp_id_ != 0) {
        // Both selected - prefer HMI if same, otherwise VCU
        if (vcu_selected_id_ == hmi_temp_id_) {
            final_selected_id_ = hmi_temp_id_;
        } else {
            final_selected_id_ = vcu_selected_id_;
        }
    } else {
        final_selected_id_ = 0;  // No user selection
    }

    //=========================================================================
    // 推荐车位逻辑
    //=========================================================================
    
    // 自车参考点位置 (后轴中心前方约4m处，单位：m)
    const Point2f kCarRefPoint(-4.0f, 0.0f);
    const int kMaxRecommendNum = 3;  // 最多显示3个推荐车位编号

    // Step 0: 清除旧的推荐信息
    for (size_t i = 0; i < slot_list.count; ++i) {
        slot_list.slots[i].isRecommended = false;
        slot_list.slots[i].displayId = slot_list.slots[i].slotId;  // 重置displayId
        if (slot_list.slots[i].displayStatus == SlotDisplayStatus::Recommended) {
            slot_list.slots[i].displayStatus = SlotDisplayStatus::Available;
        }
    }

    // Step 1: 找出所有available的车位，并计算到参考点的距离
    struct SlotDistance {
        size_t index;
        uint32_t slotId;
        float distance;
    };
    std::vector<SlotDistance> available_slots;

    for (size_t i = 0; i < slot_list.count; ++i) {
        auto& slot = slot_list.slots[i];
        
        // 只考虑可用且可泊入的车位
        if (slot.displayStatus != SlotDisplayStatus::Available &&
            slot.displayStatus != SlotDisplayStatus::Detected) {
            continue;
        }
        if (slot.occupancy == SlotOccupancyStatus::Occupied ||
            slot.occupancy == SlotOccupancyStatus::Locked) {
            continue;
        }
        if (!slot.isParkable && slot.occupancy != SlotOccupancyStatus::Empty) {
            continue;
        }

        // 计算车位中心到参考点的距离
        Point2f slot_center;
        for (int j = 0; j < 4; ++j) {
            slot_center.x += slot.localCorners[j].x;
            slot_center.y += slot.localCorners[j].y;
        }
        slot_center.x /= 4.0f;
        slot_center.y /= 4.0f;

        float dist = std::hypot(slot_center.x - kCarRefPoint.x, 
                                slot_center.y - kCarRefPoint.y);

        available_slots.push_back({i, slot.slotId, dist});
    }

    // Step 2: 按距离排序
    std::sort(available_slots.begin(), available_slots.end(),
              [](const SlotDistance& a, const SlotDistance& b) {
                  return a.distance < b.distance;
              });

    // Step 3: 根据状态决定是否使用推荐
    // 状态1：没有点选ID且静止 -> 使用推荐ID
    if (final_selected_id_ == 0 && is_vehicle_still_) {
        if (!available_slots.empty()) {
            // 最近的车位作为推荐车位
            recommended_id_ = available_slots[0].slotId;
            size_t rec_idx = available_slots[0].index;
            slot_list.slots[rec_idx].isRecommended = true;
            slot_list.slots[rec_idx].displayStatus = SlotDisplayStatus::Recommended;

            // 设置前几个车位的displayId为1,2,3...
            for (size_t i = 0; i < available_slots.size() && i < static_cast<size_t>(kMaxRecommendNum); ++i) {
                size_t idx = available_slots[i].index;
                slot_list.slots[idx].displayId = static_cast<uint32_t>(i + 1);
            }
        } else {
            recommended_id_ = 0;
        }
        // 使用推荐ID作为最终ID
        final_selected_id_ = recommended_id_;
    }
    // 状态2：没有点选ID且运动 -> 不推荐，清除
    else if (final_selected_id_ == 0 && !is_vehicle_still_) {
        recommended_id_ = 0;
        // 保持车位原状态，不做额外处理
    }
    // 状态3：有点选ID且静止 -> 使用点选ID，清除推荐
    else if (final_selected_id_ != 0 && is_vehicle_still_) {
        recommended_id_ = 0;
        // final_selected_id_ 保持不变
    }
    // 状态4：有点选ID且运动 -> 清除所有ID
    else if (final_selected_id_ != 0 && !is_vehicle_still_) {
        recommended_id_ = 0;
        hmi_temp_id_ = 0;
        vcu_selected_id_ = 0;
        final_selected_id_ = 0;
    }

    //=========================================================================
    // 更新车位选中状态
    //=========================================================================
    slot_list.selectedSlotId = final_selected_id_;
    slot_list.recommendedSlotId = recommended_id_;
    
    // 标记选中的车位
    for (size_t i = 0; i < slot_list.count; ++i) {
        auto& slot = slot_list.slots[i];
        
        // 如果是用户点选的车位（非推荐）
        if (slot.slotId == final_selected_id_ && final_selected_id_ != recommended_id_) {
            slot.isSelected = true;
            if (slot.occupancy != SlotOccupancyStatus::Occupied &&
                slot.occupancy != SlotOccupancyStatus::Locked) {
                slot.displayStatus = SlotDisplayStatus::Selected;
            }
        }
        // 如果是推荐车位（且被选中）
        else if (slot.slotId == recommended_id_ && recommended_id_ != 0) {
            slot.isRecommended = true;
            if (slot.occupancy != SlotOccupancyStatus::Occupied &&
                slot.occupancy != SlotOccupancyStatus::Locked) {
                slot.displayStatus = SlotDisplayStatus::Recommended;
            }
        }

        // In Enabled/Guidance phase, mark target slot
        if ((status == static_cast<uint8_t>(ApaStatus::Enabled) ||
             status == static_cast<uint8_t>(ApaStatus::Guidance)) &&
            slot.slotId == final_selected_id_ && final_selected_id_ != 0)
        {
            slot.isTargetSlot = true;
            slot_list.targetSlotId = slot.slotId;

            // Save target slot for guidance phase
            if (status == static_cast<uint8_t>(ApaStatus::Enabled) && !has_saved_target_slot_) {
                saved_target_slot_ = slot;
                has_saved_target_slot_ = true;
            }
        }
    }

    // In Guidance phase, use saved target slot
    if (status == static_cast<uint8_t>(ApaStatus::Guidance) && has_saved_target_slot_) {
        InternalSlot* target = slot_list.findSlotById(saved_target_slot_.slotId);
        if (target) {
            // Lock target slot data during guidance
            target->isTargetSlot = true;
            slot_list.targetSlotId = saved_target_slot_.slotId;
        }
    }
}

void PsdSlotFusion::ProcUpdateDisplayStatus(
    const PsdFusionInput& input,
    InternalSlotList& slot_list)
{
    for (size_t i = 0; i < slot_list.count; ++i) {
        auto& slot = slot_list.slots[i];

        // Skip already processed slots
        if (slot.displayStatus == SlotDisplayStatus::Selected ||
            slot.displayStatus == SlotDisplayStatus::Recommended) {
            continue;
        }

        // Determine display status based on occupancy and parkability
        if (slot.occupancy == SlotOccupancyStatus::Occupied ||
            slot.occupancy == SlotOccupancyStatus::Locked) {
            slot.displayStatus = SlotDisplayStatus::Occupied;
        } else if (!slot.isParkable) {
            slot.displayStatus = SlotDisplayStatus::Unavailable;
        } else {
            slot.displayStatus = SlotDisplayStatus::Available;
        }

        // Apply distance filter if enabled
        if (faraway_filter_enabled_) {
            Point2f center;
            for (int j = 0; j < 4; ++j) {
                center.x += slot.localCorners[j].x;
                center.y += slot.localCorners[j].y;
            }
            center.x /= 4.0f;
            center.y /= 4.0f;

            bool out_of_range = false;
            if (center.x < faraway_rear_ || center.x > faraway_front_) {
                out_of_range = true;
            }
            if (center.y < faraway_left_min_ || center.y > faraway_left_max_) {
                if (center.y < faraway_right_min_ || center.y > faraway_right_max_) {
                    out_of_range = true;
                }
            }

            if (out_of_range) {
                slot.displayStatus = SlotDisplayStatus::Unavailable;
                slot.needsRelease = false;
            }
        }
    }
}

void PsdSlotFusion::ProcConvertToOutput(
    const PsdFusionInput& input,
    const InternalSlotList& slot_list,
    PsdFusionOutput& output)
{
    output.clear();

    // Set header
    output.slotList.header.pubTimestampUs = input.currentTimestamp;
    output.slotList.header.sequence = static_cast<uint16_t>(input.currentTimestamp / 1000);

    // Convert each slot
    for (size_t i = 0; i < slot_list.count && i < Limits::kMaxSlotCount; ++i) {
        const auto& src = slot_list.slots[i];
        
        if (!src.needsRelease) {
            continue;  // Skip slots that shouldn't be released
        }

        ParkingSlotOutput dst;
        dst.slotId = src.slotId;
        dst.slotDisplayId = src.displayId;
        dst.slotDisplayStatus = static_cast<uint8_t>(src.displayStatus);

        // Convert corners to Point3f (z=0)
        for (int j = 0; j < 4; ++j) {
            dst.slotCorners[j] = Point3f(src.localCorners[j].x, src.localCorners[j].y, 0.0f);
        }

        dst.slotEntryLength = src.entryLength;
        dst.slotNonEntryLength = src.nonEntryLength;
        dst.slotType = static_cast<uint8_t>(src.type);
        dst.slotSurfaceType = 0;
        dst.slotBottomType = static_cast<uint8_t>(src.bottomType);
        dst.slotObstacleType = static_cast<uint8_t>(src.obstacleType);
        dst.slotObstacleLocation = static_cast<uint8_t>(src.obstacleLocation);
        dst.slotStopperDistance = src.stopperDistance;
        dst.slotConfidence = src.confidence;
        dst.slotSource = static_cast<uint8_t>(src.source);
        dst.slotOccupied = (src.occupancy == SlotOccupancyStatus::Occupied);
        dst.slotSelected = src.isSelected;
        dst.parkInType = src.parkInDirection;
        dst.ussSlotOppositeSpace = src.oppositeSpace;
        dst.ussSlotTransverseSpace = src.transverseSpace;

        output.slotList.addSlot(dst);
    }

    // Set target slot label
    output.targetLabel.header.pubTimestampUs = input.currentTimestamp;
    output.targetLabel.targetSlotId = static_cast<uint16_t>(slot_list.targetSlotId);

    // Set stopper info for target slot
    output.slotStopper.header.pubTimestampUs = input.currentTimestamp;
    output.slotStopper.limitBarX.fill(-20000);
    output.slotStopper.limitBarY.fill(-20000);

    const InternalSlot* target = slot_list.findSlotById(slot_list.targetSlotId);
    if (target && target->hasStopperInSlot) {
        for (size_t i = 0; i < Limits::kMaxStopperCount; ++i) {
            output.slotStopper.limitBarX[i] = static_cast<int16_t>(target->stopperPoints[i].x);
            output.slotStopper.limitBarY[i] = static_cast<int16_t>(target->stopperPoints[i].y);
        }
    }
}

//=============================================================================
// Helper Functions
//=============================================================================

SlotType PsdSlotFusion::ConvertMapSlotType(uint32_t map_type) {
    switch (map_type) {
        case 0: return SlotType::Perpendicular;
        case 1: return SlotType::Parallel;
        case 2: return SlotType::Oblique;
        case 3: return SlotType::ReverseOblique;
        case 4: return SlotType::UserDefined;
        default: return SlotType::Perpendicular;
    }
}

float PsdSlotFusion::CalculateEntryLength(const SlotCorners& corners) {
    float dx = corners[1].x - corners[0].x;
    float dy = corners[1].y - corners[0].y;
    return std::sqrt(dx * dx + dy * dy);
}

float PsdSlotFusion::CalculateNonEntryLength(const SlotCorners& corners) {
    float dx = corners[3].x - corners[0].x;
    float dy = corners[3].y - corners[0].y;
    return std::sqrt(dx * dx + dy * dy);
}

Point2f PsdSlotFusion::TransformBodyToWorld(
    const Point2f& pt_body,
    const VehiclePoseInput& pose)
{
    float theta = pose.yaw * static_cast<float>(M_PI) / 180.0f;
    float cos_t = std::cos(theta);
    float sin_t = std::sin(theta);
    
    Point2f pt_world;
    pt_world.x = pt_body.x * cos_t - pt_body.y * sin_t + pose.posX;
    pt_world.y = pt_body.x * sin_t + pt_body.y * cos_t + pose.posY;
    return pt_world;
}

Point2f PsdSlotFusion::TransformWorldToBody(
    const Point2f& pt_world,
    const VehiclePoseInput& pose)
{
    float theta = pose.yaw * static_cast<float>(M_PI) / 180.0f;
    float cos_t = std::cos(theta);
    float sin_t = std::sin(theta);
    
    float dx = pt_world.x - pose.posX;
    float dy = pt_world.y - pose.posY;
    
    Point2f pt_body;
    pt_body.x = dx * cos_t + dy * sin_t;
    pt_body.y = -dx * sin_t + dy * cos_t;
    return pt_body;
}

void PsdSlotFusion::AdjustCornerOrder(SlotCorners& corners) {
    // ------- 工具函数 --------
    auto signed_area = [&](int i0, int i1, int i2, int i3) -> float {
        const Point2f* p[4] = {
            &corners[i0],
            &corners[i1],
            &corners[i2],
            &corners[i3]
        };
        float area2 = 0.0f;
        for (int i = 0; i < 4; ++i) {
            const Point2f& a = *p[i];
            const Point2f& b = *p[(i + 1) & 3];
            area2 += a.x * b.y - b.x * a.y;
        }
        return area2; // >0 逆时针, <0 顺时针
    };

    auto cross = [](const Point2f& a, const Point2f& b, const Point2f& c) -> float {
        // (b - a) x (c - a)
        return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
    };

    auto segments_intersect = [&](const Point2f& p1,
                                  const Point2f& p2,
                                  const Point2f& p3,
                                  const Point2f& p4) -> bool {
        auto o1 = cross(p1, p2, p3);
        auto o2 = cross(p1, p2, p4);
        auto o3 = cross(p3, p4, p1);
        auto o4 = cross(p3, p4, p2);
        // 只关心“严格相交”（不是共线重叠那种）
        return (o1 * o2 < 0.0f) && (o3 * o4 < 0.0f);
    };

    auto is_hourglass = [&](int i0, int i1, int i2, int i3) -> bool {
        const Point2f& A = corners[i0];
        const Point2f& B = corners[i1];
        const Point2f& C = corners[i2];
        const Point2f& D = corners[i3];
        // 沙漏：非相邻边相交
        if (segments_intersect(A, B, C, D)) return true;
        if (segments_intersect(B, C, D, A)) return true;
        return false;
    };

    // ------- 判断车位在左还是右 -------
    float avg_y = 0.0f;
    for (int i = 0; i < 4; ++i) {
        avg_y += corners[i].y;
    }
    avg_y *= 0.25f;

    enum class SlotSide {
        UNKNOWN,
        LEFT,
        RIGHT
    };

    SlotSide side = SlotSide::UNKNOWN;
    const float kSideEps = 1e-3f;
    if (avg_y > kSideEps) {
        side = SlotSide::LEFT;   // 左侧车位 -> 逆时针
    } else if (avg_y < -kSideEps) {
        side = SlotSide::RIGHT;  // 右侧车位 -> 顺时针
    } else {
        side = SlotSide::UNKNOWN;
    }

    // ------- 枚举 4 种候选顺序 -------
    struct Candidate { int idx[4]; };

    Candidate candidates[4] = {
        {{0, 1, 2, 3}}, // A=0,B=1,C=2,D=3
        {{0, 1, 3, 2}}, // A=0,B=1,C=3,D=2
        {{1, 0, 2, 3}}, // A=1,B=0,C=2,D=3
        {{1, 0, 3, 2}}  // A=1,B=0,C=3,D=2
    };

    int best = 0;
    bool have_non_hourglass = false;
    bool have_dir_match = false;

    const float kAreaEps = 1e-5f;

    for (int i = 0; i < 4; ++i) {
        int i0 = candidates[i].idx[0];
        int i1 = candidates[i].idx[1];
        int i2 = candidates[i].idx[2];
        int i3 = candidates[i].idx[3];

        // 先剔除“沙漏”形状
        if (is_hourglass(i0, i1, i2, i3)) {
            continue;
        }

        if (!have_non_hourglass) {
            best = i;
            have_non_hourglass = true;
        }

        float area2 = signed_area(i0, i1, i2, i3);
        if (std::fabs(area2) < kAreaEps) {
            continue;
        }

        bool dir_ok = false;
        if (side == SlotSide::LEFT && area2 > 0.0f) {
            // 左侧车位 -> 逆时针
            dir_ok = true;
        } else if (side == SlotSide::RIGHT && area2 < 0.0f) {
            // 右侧车位 -> 顺时针
            dir_ok = true;
        } else if (side == SlotSide::UNKNOWN) {
            // 不知道左右，就不强制方向
            dir_ok = true;
        }

        if (dir_ok) {
            best = i;
            have_dir_match = true;
            if (side != SlotSide::UNKNOWN) {
                // 已经找到满足方向的合法四边形，可以提前结束
                break;
            }
        }
    }

    // 如果连一个非沙漏的都没有（极端退化情况），退回第一个候选
    const Candidate& chosen = candidates[best];

    // ------- 重新赋值 ABCD -------
    SlotCorners temp = corners;
    corners[0] = temp[chosen.idx[0]]; // A：开口边
    corners[1] = temp[chosen.idx[1]]; // B：开口边
    corners[2] = temp[chosen.idx[2]]; // C：闭口边
    corners[3] = temp[chosen.idx[3]]; // D：闭口边
}

bool PsdSlotFusion::PointInQuad(const Point2f& point, const SlotCorners& corners) {
    
    Point2f A = corners[0];
    Point2f B = corners[1];
    Point2f D = corners[3];
    
    float AB_x = B.x - A.x;
    float AB_y = B.y - A.y;
    float AD_x = D.x - A.x;
    float AD_y = D.y - A.y;
    
    float AP_x = point.x - A.x;
    float AP_y = point.y - A.y;
    
    float AB_dot_AB = AB_x * AB_x + AB_y * AB_y;
    float AD_dot_AD = AD_x * AD_x + AD_y * AD_y;
    
    if (AB_dot_AB < 1e-10f || AD_dot_AD < 1e-10f) {
        return false;
    }
    
    // 计算投影比例
    float AP_dot_AB = AP_x * AB_x + AP_y * AB_y;
    float AP_dot_AD = AP_x * AD_x + AP_y * AD_y;
    
    float dotAB = AP_dot_AB / AB_dot_AB;
    float dotAD = AP_dot_AD / AD_dot_AD;
    
    return (dotAB >= 0.0f && dotAB <= 1.0f && dotAD >= 0.0f && dotAD <= 1.0f);
}

float PsdSlotFusion::CalcPointToLineDistance(const Point2f& point, const Point2f& lineA, const Point2f& lineB) {
    
    float numerator = std::fabs(
        (lineB.y - lineA.y) * point.x + 
        (lineA.x - lineB.x) * point.y + 
        lineB.x * lineA.y - 
        lineA.x * lineB.y
    );
    
    float denominator = std::sqrt(
        (lineB.y - lineA.y) * (lineB.y - lineA.y) + 
        (lineA.x - lineB.x) * (lineA.x - lineB.x)
    );
    
    if (denominator < 1e-10f) {
        return 0.0f;
    }
    
    return numerator / denominator;
}

uint32_t PsdSlotFusion::GetFinalSelectedSlotId() const {
    return final_selected_id_;
}
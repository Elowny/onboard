#include "onboard/planner/decision/cautious_brake_decider.h"

#include <algorithm>
#include <string>
#include <utility>

#include "absl/container/flat_hash_map.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_format.h"
#include "absl/types/span.h"

#include "onboard/container/strong_int.h"
#include "onboard/maps/lane_path.h"
#include "onboard/maps/maps_common.h"
#include "onboard/maps/semantic_map_defs.h"
#include "onboard/math/geometry/polygon2d.h"
#include "onboard/math/util.h"
#include "onboard/math/vec.h"
#include "onboard/planner/decision/decision_util.h"
#include "onboard/planner/object/planner_object.h"
#include "onboard/planner/object/spacetime_object_trajectory.h"
#include "onboard/planner/planner_util.h"
#include "onboard/planner/second_order_trajectory_point.h"
#include "onboard/utils/status_macros.h"

namespace qcraft {
namespace planner {
namespace {
constexpr double kCrosswalkAssociationDistance = 2.0;
constexpr double kEpsilon = 0.1;
constexpr double kDrivePassageEndPointEpsilon = 0.1;
constexpr double kIntersectionSpeedReductionRatio = 0.75;
constexpr double kEmptyCrossWalkSpeedReductionRatio = 0.9;
constexpr double kVRUNearCrossWalkSpeedReductionRatio = 0.8;
constexpr double kVRUOnCrossWalkSpeedReductionRatio = 0.7;

constexpr double kMinCautiousSpeed = 3.0;  // m/s

bool IsObjectOnCrosswalk(const mapping::CrosswalkInfo& cw,
                         const SpacetimeObjectTrajectory& traj) {
  return cw.polygon_smooth.DistanceSquareTo(
             traj.planner_object().pose().pos()) < Sqr(kEpsilon);
}

bool IsObjectNearCrosswalk(const mapping::CrosswalkInfo& cw,
                           const SpacetimeObjectTrajectory& traj) {
  return cw.polygon_smooth.DistanceSquareTo(
             traj.planner_object().pose().pos()) <
         Sqr(kCrosswalkAssociationDistance);
}

bool IsAnyVruOnCrosswalk(const mapping::CrosswalkInfo& cw,
                         const SpacetimeTrajectoryManager& st_traj_mgr) {
  for (const auto& traj : st_traj_mgr.trajectories()) {
    if (IsVulnerableRoadUserType(traj.planner_object().type())) {
      if (IsObjectOnCrosswalk(cw, traj)) {
        return true;
      }
    }
  }
  return false;
}

bool IsAnyVruNearCrosswalk(const mapping::CrosswalkInfo& cw,
                           const SpacetimeTrajectoryManager& st_traj_mgr) {
  for (const auto& traj : st_traj_mgr.trajectories()) {
    if (IsVulnerableRoadUserType(traj.planner_object().type())) {
      if (IsObjectNearCrosswalk(cw, traj)) {
        return true;
      }
    }
  }
  return false;
}

absl::StatusOr<ConstraintProto::SpeedRegionProto> ProcessMapElement(
    const DrivePassage& passage, const mapping::LanePath& lane_path_from_start,
    const mapping::LanePath::LaneSegment& seg,
    const mapping::LaneInfo& lane_info,
    const std::pair<mapping::ElementId, Vec2d>& element, double s_offset,
    double lane_speed_limit, double speed_reduction_ratio) {
  if (seg.start_fraction > element.second.y()) {
    return absl::NotFoundError("Map Element not on current seg.");
  }

  const double passage_min_s = passage.front_s();
  const double passage_max_s = passage.end_s();
  const auto start_point = lane_info.LerpPointFromFraction(element.second.x());
  // End point.
  const auto end_point = lane_info.LerpPointFromFraction(element.second.y());
  ConstraintProto::SpeedRegionProto cautious_brake_constraint;
  start_point.ToProto(cautious_brake_constraint.mutable_start_point());
  end_point.ToProto(cautious_brake_constraint.mutable_end_point());

  const double start_point_s =
      lane_path_from_start.LaneIndexPointToArclength(
          /*lane_index_point=*/{seg.lane_index, element.second.x()}) +
      s_offset + passage.lane_path_start_s();
  const double end_point_s =
      lane_path_from_start.LaneIndexPointToArclength(
          /*lane_index_point=*/{seg.lane_index, element.second.y()}) +
      s_offset + passage.lane_path_start_s();

  if (start_point_s >= passage_max_s - kDrivePassageEndPointEpsilon) {
    return absl::NotFoundError(
        absl::StrFormat("Map Element beyond current drive passage. "
                        "start_point_s: %f, passage_max_s:%f",
                        start_point_s, passage_max_s));
  } else if (end_point_s <= passage_min_s + kDrivePassageEndPointEpsilon) {
    return absl::NotFoundError(
        absl::StrFormat("Map Element behind current drive passage. "
                        "end_point_s:%f, drive passage_min_s: %f",
                        end_point_s, passage_min_s));
  } else if (start_point_s + kDrivePassageEndPointEpsilon >= end_point_s) {
    return absl::NotFoundError(
        absl::StrFormat("Map Element range on drive passage is too short. "
                        "start_pont_s: %f, end_point_s: %f",
                        start_point_s, end_point_s));
  }
  cautious_brake_constraint.set_start_s(std::max(start_point_s, passage_min_s));
  cautious_brake_constraint.set_end_s(std::min(end_point_s, passage_max_s));
  const auto speed_limit =
      std::max(kMinCautiousSpeed, lane_speed_limit * speed_reduction_ratio);
  cautious_brake_constraint.set_max_speed(speed_limit);

  return cautious_brake_constraint;
}
}  // namespace

std::vector<ConstraintProto::SpeedRegionProto> BuildCautiousBrakeConstraints(
    const PlannerSemanticMapManager& planner_semantic_map_manager,
    const DrivePassage& passage, const mapping::LanePath& lane_path_from_start,
    double s_offset, const SpacetimeTrajectoryManager& st_mgr) {
  absl::flat_hash_map<std::string,
                      std::vector<ConstraintProto::SpeedRegionProto>>
      id_element_map;
  for (const auto& seg : lane_path_from_start) {
    const auto* lane_info_ptr =
        planner_semantic_map_manager.FindLaneInfoOrNull(seg.lane_id);
    if (lane_info_ptr == nullptr) continue;
    const double speed_limit =
        planner_semantic_map_manager.QueryLaneSpeedLimitByFraction(
            seg.lane_id,
            /*fraction=*/0.0);
    // Intersection.
    for (const auto& intersection : lane_info_ptr->Intersections()) {
      ASSIGN_OR_CONTINUE(
          auto cautious_brake_constraint,
          ProcessMapElement(passage, lane_path_from_start, seg, *lane_info_ptr,
                            intersection, s_offset, speed_limit,
                            kIntersectionSpeedReductionRatio));
      const auto id = intersection.first;
      cautious_brake_constraint.mutable_source()
          ->mutable_intersection()
          ->set_id(id.value());
      const std::string str_id =
          absl::StrFormat("cautious_brake_intersection_%d", id);
      cautious_brake_constraint.set_id(str_id);
      id_element_map[str_id].push_back(std::move(cautious_brake_constraint));
    }
    // Crosswalk.
    for (const auto& cw_idx : lane_info_ptr->Crosswalks()) {
      const auto* cw_ptr =
          planner_semantic_map_manager.FindCrosswalkByIdOrNull(cw_idx.first);
      if (cw_ptr == nullptr) continue;
      double reduction_ratio = kEmptyCrossWalkSpeedReductionRatio;
      if (IsAnyVruOnCrosswalk(*cw_ptr, st_mgr)) {
        reduction_ratio = kVRUOnCrossWalkSpeedReductionRatio;
      } else if (IsAnyVruNearCrosswalk(*cw_ptr, st_mgr)) {
        reduction_ratio = kVRUNearCrossWalkSpeedReductionRatio;
      }
      ASSIGN_OR_CONTINUE(
          auto cautious_brake_constraint,
          ProcessMapElement(passage, lane_path_from_start, seg, *lane_info_ptr,
                            cw_idx, s_offset, speed_limit, reduction_ratio));
      const auto id = cw_idx.first;
      cautious_brake_constraint.mutable_source()
          ->mutable_intersection()
          ->set_id(id.value());
      const std::string str_id =
          absl::StrFormat("cautious_brake_crosswalk_%d", id);
      cautious_brake_constraint.set_id(str_id);
      id_element_map[str_id].push_back(std::move(cautious_brake_constraint));
    }
  }
  std::vector<ConstraintProto::SpeedRegionProto> cautious_brake_constraints;
  cautious_brake_constraints.reserve(id_element_map.size());
  for (const auto& pair : id_element_map) {
    cautious_brake_constraints.push_back(MergeSameElement(pair.second));
  }
  return cautious_brake_constraints;
}

}  // namespace planner
}  // namespace qcraft

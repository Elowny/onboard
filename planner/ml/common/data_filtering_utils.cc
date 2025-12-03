#include "onboard/planner/ml/common/data_filtering_utils.h"

#include <cmath>
#include <ostream>
#include <utility>

#include "absl/status/statusor.h"
#include "glog/logging.h"

#include "onboard/container/strong_int.h"
#include "onboard/global/buffered_logger.h"
#include "onboard/lite/check.h"
#include "onboard/maps/maps_common.h"
#include "onboard/maps/semantic_map_defs.h"
#include "onboard/math/frenet_common.h"
#include "onboard/math/vec.h"

namespace qcraft::planner {

constexpr double kSearchLaneRadius = 10.0;           // m.
constexpr double kSearchLaneMaxDiffRadian = M_PI_4;  // radian.

bool RecursiveCheckOutgoingLane(const mapping::LaneInfo& incoming_lane_info,
                                const mapping::ElementId target_lane_id,
                                double search_range,
                                const PlannerSemanticMapManager& psmm) {
  if (incoming_lane_info.id == target_lane_id) {
    return true;
  }
  if (search_range < 0.0) {
    return false;
  }
  for (const auto id : incoming_lane_info.outgoing_lanes()) {
    const auto* outgoing_lane_info_ptr = psmm.FindLaneInfoOrNull(id);
    if (outgoing_lane_info_ptr == nullptr) continue;
    if (id == target_lane_id) {
      return true;
    } else {
      if (RecursiveCheckOutgoingLane(*outgoing_lane_info_ptr, target_lane_id,
                                     search_range - incoming_lane_info.length(),
                                     psmm)) {
        return true;
      }
    }
  }
  return false;
}

bool TwoPointsInConnectedLanes(const TrajectoryPoint& incoming_point,
                               const TrajectoryPoint& outgoing_point,
                               double search_range,
                               const PlannerSemanticMapManager& psmm) {
  constexpr int kSearchLevelMaxRange = 5;
  const auto& level_id = psmm.GetLevel();
  VLOG(3) << "Semantic map manager level is: " << level_id;

  auto* incoming_lane_info = psmm.GetNearestLaneInfoWithHeadingAtLevel(
      level_id, incoming_point.pos(), incoming_point.theta(), kSearchLaneRadius,
      kSearchLaneMaxDiffRadian);
  auto incoming_lane_level = level_id;
  if (incoming_lane_info == nullptr) {
    for (int level_diff = 1; level_diff < kSearchLevelMaxRange; ++level_diff) {
      const auto* upper_incoming_lane_info =
          psmm.GetNearestLaneInfoWithHeadingAtLevel(
              mapping::LevelId(level_id.value() + level_diff),
              incoming_point.pos(), incoming_point.theta(), kSearchLaneRadius,
              kSearchLaneMaxDiffRadian);
      const auto* lower_incoming_lane_info =
          psmm.GetNearestLaneInfoWithHeadingAtLevel(
              mapping::LevelId(level_id.value() - level_diff),
              incoming_point.pos(), incoming_point.theta(), kSearchLaneRadius,
              kSearchLaneMaxDiffRadian);

      if (upper_incoming_lane_info == nullptr &&
          lower_incoming_lane_info == nullptr) {
        continue;
      } else if (upper_incoming_lane_info != nullptr) {
        incoming_lane_info = upper_incoming_lane_info;
        incoming_lane_level = mapping::LevelId(level_id.value() + level_diff);
        break;
      } else if (lower_incoming_lane_info != nullptr) {
        incoming_lane_info = lower_incoming_lane_info;
        incoming_lane_level = mapping::LevelId(level_id.value() - level_diff);
        break;
      } else {
        QCHECK(false) << "Incoming lane should not be found in both level "
                      << level_id.value() + level_diff << " and "
                      << level_id.value() - level_diff;
      }
    }
    if (incoming_lane_info == nullptr) {
      QCHECK(false) << "Incoming lane can't be found in all +-"
                    << kSearchLevelMaxRange << " levels.";
    }
  }

  VLOG(3) << "Incoming lane level is: " << incoming_lane_level;
  VLOG(3) << "Incoming lane id is: " << incoming_lane_info->id;
  auto* outgoing_lane_info = psmm.GetNearestLaneInfoWithHeadingAtLevel(
      incoming_lane_level, outgoing_point.pos(), outgoing_point.theta(),
      kSearchLaneRadius, kSearchLaneMaxDiffRadian);
  if (outgoing_lane_info == nullptr) {
    for (int level_diff = 1; level_diff < kSearchLevelMaxRange; ++level_diff) {
      const auto* upper_outgoing_lane_info =
          psmm.GetNearestLaneInfoWithHeadingAtLevel(
              mapping::LevelId(incoming_lane_level.value() + level_diff),
              outgoing_point.pos(), outgoing_point.theta(), kSearchLaneRadius,
              kSearchLaneMaxDiffRadian);
      const auto* lower_outgoing_lane_info =
          psmm.GetNearestLaneInfoWithHeadingAtLevel(
              mapping::LevelId(incoming_lane_level.value() - level_diff),
              outgoing_point.pos(), outgoing_point.theta(), kSearchLaneRadius,
              kSearchLaneMaxDiffRadian);
      if (upper_outgoing_lane_info == nullptr &&
          lower_outgoing_lane_info == nullptr) {
        continue;
      } else if (upper_outgoing_lane_info != nullptr) {
        outgoing_lane_info = upper_outgoing_lane_info;
        VLOG(3) << "Outgoing lane level is: "
                << incoming_lane_level.value() + level_diff;
        break;
      } else if (lower_outgoing_lane_info != nullptr) {
        outgoing_lane_info = lower_outgoing_lane_info;
        VLOG(3) << "Outgoing lane level is: "
                << incoming_lane_level.value() - level_diff;
        break;
      } else {
        QCHECK(false) << "Outgoing lane should not be found in both level "
                      << incoming_lane_level.value() + level_diff << " and "
                      << incoming_lane_level.value() - level_diff;
      }
    }
    if (outgoing_lane_info == nullptr) {
      QCHECK(false) << "Outgoing lane can't be found in all +-"
                    << kSearchLevelMaxRange << " levels.";
    }
  }
  VLOG(3) << "Outgoing lane id is: " << outgoing_lane_info->id;
  return RecursiveCheckOutgoingLane(*incoming_lane_info, outgoing_lane_info->id,
                                    search_range, psmm);
}

double CalcDistBetweenTwoPoints(const TrajectoryPoint& first_point,
                                const TrajectoryPoint& second_point) {
  return std::sqrt((first_point.pos() - second_point.pos()).Sqr());
}

bool TrajIntentionSameAsExpert(
    const std::vector<TrajectoryPoint>& expert_traj_points,
    const std::vector<TrajectoryPoint>& candidate_traj_points,
    const DrivePassage& drive_passage, const PathSlBoundary& path_sl_boundary,
    const VehicleGeometryParamsProto& vehicle_geometry_params,
    const PlannerSemanticMapManager& psmm) {
  const auto& start_point = candidate_traj_points.front();
  const auto& candidate_end_point = candidate_traj_points.back();
  const auto& expert_end_point = expert_traj_points.back();

  double candidate_distance =
      CalcDistBetweenTwoPoints(start_point, candidate_end_point);
  double expert_distance =
      CalcDistBetweenTwoPoints(start_point, expert_end_point);

  constexpr double kSearchRangeForSelectorIntention = 300.0;  // meter

  if (candidate_distance < expert_distance) {
    VLOG(3) << "Incoming lane is candidate and outgoing lane is expert.";
    if (!TwoPointsInConnectedLanes(candidate_end_point, expert_end_point,
                                   kSearchRangeForSelectorIntention, psmm)) {
      VLOG(3) << "From candidate end point can't find expert end point in "
                 "connected lane.";
      return false;
    }
  } else {
    VLOG(3) << "Incoming lane is expert and outgoing lane is candidate.";
    if (!TwoPointsInConnectedLanes(expert_end_point, candidate_end_point,
                                   kSearchRangeForSelectorIntention, psmm)) {
      VLOG(3) << "From expert end point can't find candidate end point in "
                 "connected lane.";
      return false;
    }
  }

  constexpr double kPathBoundaryViolationLimit = 1.2;  // meter
  constexpr int kPathBoundaryViolationCheckHorizon = 50;
  for (int i = 0;
       i < expert_traj_points.size() && i < kPathBoundaryViolationCheckHorizon;
       ++i) {
    const auto& pt = expert_traj_points[i];
    const auto frenet_pt_or = drive_passage.QueryFrenetCoordinateAt(pt.pos());
    if (!frenet_pt_or.ok()) {
      VLOG(3) << "Query frenet coordinate by pos fail.";
      return false;
    }
    const auto l_pair = path_sl_boundary.QueryBoundaryL(frenet_pt_or->s);
    const double excced_left_boundary_dist =
        l_pair.second - frenet_pt_or->l - 0.5 * vehicle_geometry_params.width();
    if (excced_left_boundary_dist < -kPathBoundaryViolationLimit) {
      VLOG(3) << "Exceed left boundary.";
      return false;
    }
    const double excced_right_boundary_dist =
        l_pair.first - frenet_pt_or->l + 0.5 * vehicle_geometry_params.width();
    if (excced_right_boundary_dist > kPathBoundaryViolationLimit) {
      VLOG(3) << "Exceed right boundary.";
      return false;
    }
  }

  VLOG(3) << "Has same intention.";
  return true;
}

}  // namespace qcraft::planner

#include "onboard/prediction/util/lane_path_util.h"

#include <cmath>     // for M_PI
#include <optional>  // for optional
#include <set>       // for set, operator!=, _Rb_tree_const_iterator
#include <utility>
#include <vector>

#include "absl/status/statusor.h"

#include "onboard/container/strong_int.h"
#include "onboard/maps/lane_path.h"
#include "onboard/maps/lane_point.h"
#include "onboard/maps/maps_common.h"  // for LaneNeighborInfo, LaneInfo
#include "onboard/maps/semantic_map_defs.h"
#include "onboard/planner/util/spatial_search_util.h"
#include "onboard/prediction/predicted_trajectory.h"  // for PredictedTrajectoryPoint
#include "onboard/prediction/util/kinematic_model.h"  // for ObjectMotionStateToUniCycleState
#include "onboard/prediction/util/lane_path_finder.h"
#include "onboard/prediction/util/trajectory_developer.h"

namespace qcraft {
namespace prediction {
namespace {
// For associating agent's movement to lanes
constexpr double kCTRARollOutHorizon = 1.5;     // s.
constexpr double kMaxHeadingDiff = M_PI / 3.0;  // rad.
// Lane path generation related consts.
constexpr double kForwardLane = 50.0;           // m.
constexpr double kForwardExtendLength = 100.0;  // m.
constexpr double kBackwardLength = 10.0;        // m.
}  // namespace

std::vector<mapping::LanePath> FindPossibleLanePathsByCTRATPrediction(
    const ObjectMotionState& cur_state,
    const planner::PlannerSemanticMapManager& psmm) {
  const auto& pos = cur_state.pos;
  const auto probe_traj_pts = DevelopForwardCTRATrajectory(
      ObjectMotionStateToUniCycleState(cur_state), kPredictionTimeStep,
      kCTRARollOutHorizon, kCTRARollOutHorizon, kEmergencyGuardHorizon);
  auto associated_lane_ids =
      FindNearestLaneIdsByBBoxWithBoundaryDistLimitAndHeadingDiffLimit(
          psmm, cur_state.bbox,
          /*boundary_distance_limit=*/0.0, kMaxHeadingDiff);
  for (const auto& pt : probe_traj_pts) {
    const auto lane_id_or =
        FindNearestLaneIdWithBoundaryDistanceLimitAndHeadingDiffLimit(
            psmm, pt.pos(), pt.theta(),
            /*boundary_distance_limit=*/0.0, kMaxHeadingDiff);
    if (lane_id_or.has_value()) {
      associated_lane_ids.insert(*lane_id_or);
    }
  }
  // Insert neighbors
  for (const auto id : associated_lane_ids) {
    const auto* lane_info_ptr = psmm.FindLaneInfoOrNull(id);
    if (lane_info_ptr == nullptr) {
      continue;
    }
    for (const auto& neighbor_info : lane_info_ptr->lane_neighbors_on_left) {
      associated_lane_ids.insert(neighbor_info.other_id);
    }
    for (const auto& neighbor_info : lane_info_ptr->lane_neighbors_on_right) {
      associated_lane_ids.insert(neighbor_info.other_id);
    }
  }
  const auto filtered_ids = FilterLanesByConsecutiveRelationship(
      psmm, associated_lane_ids, /*filter_virtual=*/false);
  std::vector<mapping::LanePath> lane_paths;
  lane_paths.reserve(filtered_ids.size());
  for (const auto& id : filtered_ids) {
    const auto lps = SearchLanePath(pos, psmm, id, kForwardLane,
                                    /*is_reverse_driving=*/false);
    for (const auto& lp : lps) {
      // Extend the front part of lane path.
      auto extended_lp = ExtendMostStraightLanePath(lp, psmm, kBackwardLength,
                                                    /*is_reversed=*/true);
      extended_lp =
          ExtendMostStraightLanePath(extended_lp, psmm, kForwardExtendLength,
                                     /*is_reversed=*/false);
      lane_paths.push_back(std::move(extended_lp));
    }
  }

  return lane_paths;
}

mapping::LanePath BuildLanePathWithoutVirtualLaneAhead(
    const mapping::LanePath& lp, const planner::PlannerSemanticMapManager& psmm,
    const Vec2d& pos, double heading) {
  const auto closest_lane_point_or = planner::
      FindClosestLanePointToSmoothPointWithHeadingBoundAlongLanePathAtLevel(
          psmm.GetLevel(), psmm, pos, lp, heading);
  // mapping::ElementId closest_lane_id = lp.lane_ids()[0];
  if (!closest_lane_point_or.ok()) {
    return lp;
  }

  bool is_lane_forward = false;
  bool is_virtual = false;
  std::vector<mapping::ElementId> lane_ids;
  lane_ids.reserve(lp.lane_ids().size());

  for (const auto& lane_id : lp.lane_ids()) {
    bool is_current_lane = lane_id == closest_lane_point_or->lane_id();
    if (is_current_lane) {
      is_lane_forward = true;
    }

    if (is_lane_forward) {
      const auto lane_info = psmm.FindLaneInfoOrNull(lane_id);
      if (lane_info == nullptr) {
        return lp;
      }
      if (lane_info->IsVirtual()) {
        is_virtual = true;
      }
    }
    if (is_current_lane && is_virtual) {
      return lp;
    }
    if (!is_virtual || is_current_lane) {
      lane_ids.push_back(lane_id);
    }
  }
  if (is_virtual) {
    return mapping::LanePath(psmm.semantic_map_manager(), lane_ids, 0.0, 1.0);
  }
  return lp;
}

}  // namespace prediction
}  // namespace qcraft

#include "onboard/planner/speed/standstill_distance_decider.h"

#include <algorithm>
#include <optional>
#include <ostream>
#include <utility>
#include <vector>

#include "absl/strings/str_cat.h"
#include "absl/types/span.h"

#include "onboard/lite/check.h"
#include "onboard/lite/logging.h"
#include "onboard/maps/maps_common.h"
#include "onboard/math/geometry/box2d.h"
#include "onboard/math/geometry/polygon2d.h"
#include "onboard/math/vec.h"
#include "onboard/planner/decision/proto/constraint.pb.h"
#include "onboard/planner/object/spacetime_object_trajectory.h"
#include "onboard/planner/speed/proto/speed_finder.pb.h"
#include "onboard/planner/speed/proto/speed_finder_params.pb.h"
#include "onboard/planner/speed/st_boundary.h"
#include "onboard/planner/util/lane_path_util.h"
#include "onboard/utils/map_util.h"

namespace qcraft {
namespace planner {
namespace {

inline bool IsStBoundaryStalledObject(
    const StBoundary& st_boundary,
    const absl::flat_hash_set<std::string>& stalled_object_ids) {
  QCHECK(st_boundary.traj_id().has_value());
  const auto obj_id = SpacetimeObjectTrajectory::GetObjectIdFromTrajectoryId(
      *st_boundary.traj_id());
  return stalled_object_ids.contains(obj_id);
}

std::optional<Box2d> BuildBoomBarrierBoxOr(
    const PlannerSemanticMapManager* psmm, const mapping::LanePath& lane_path) {
  constexpr double kForwardDistance = 50.0;      // m.
  constexpr double kBoomBarrierBoxLength = 1.0;  // m.
  constexpr double kBoomBarrierBoxWidth = 2.0;   // m.
  const auto lanes_info = GetLanesInfoContinueIfNotFound(
      *psmm, lane_path.BeforeArclength(kForwardDistance));
  for (const auto* lane_info : lanes_info) {
    if (lane_info->endpoint_toll) {
      QCHECK_GE(lane_info->points_smooth.size(), 2);
      const auto& points = lane_info->points_smooth;
      const Vec2d end_vec(points.back() - points[points.size() - 2]);
      return Box2d(points.back(), end_vec.FastAngle(), kBoomBarrierBoxLength,
                   kBoomBarrierBoxWidth);
    }
  }
  return std::nullopt;
}

// BANDAID(ping): This is a hack to identify a gate boom barrier.
bool IsStaticStBoundaryBoomBarrier(
    const StBoundary& st_boundary, const PlannerSemanticMapManager* psmm,
    const mapping::LanePath& lane_path,
    const SpacetimeTrajectoryManager& st_traj_mgr) {
  const auto barrier_box = BuildBoomBarrierBoxOr(psmm, lane_path);
  if (!barrier_box.has_value()) return false;
  QCHECK(st_boundary.traj_id().has_value());
  const auto* obj =
      QCHECK_NOTNULL(st_traj_mgr.FindTrajectoryById(*st_boundary.traj_id()));
  return obj->contour().HasOverlap(*barrier_box);
}

// return: {follow_standstill_distance, lead_standstill_distance}.
std::pair<double, double> GetStBoundaryStandStillDistance(
    const StBoundary& st_boundary,
    const SpeedFinderParamsProto& speed_finder_params,
    const absl::flat_hash_set<std::string>& stalled_object_ids,
    const PlannerSemanticMapManager* psmm, const mapping::LanePath* lane_path,
    const SpacetimeTrajectoryManager& st_traj_mgr,
    const ConstraintManager& constraint_mgr,
    double extra_follow_standstill_for_large_vehicle) {
  double follow_standstill_distance = 0.0;
  double lead_standstill_distance = 0.0;
  switch (st_boundary.object_type()) {
    case StBoundaryProto::VEHICLE:
    case StBoundaryProto::CYCLIST:
    case StBoundaryProto::PEDESTRIAN: {
      if (st_boundary.is_stationary()) {
        if (IsStBoundaryStalledObject(st_boundary, stalled_object_ids)) {
          follow_standstill_distance =
              speed_finder_params.follow_standstill_distance_for_static_obj();
        } else {
          follow_standstill_distance =
              speed_finder_params.follow_standstill_distance();
        }
      } else {
        follow_standstill_distance =
            speed_finder_params.follow_standstill_distance();
      }
      if (st_boundary.is_large_vehicle()) {
        follow_standstill_distance += extra_follow_standstill_for_large_vehicle;
      }
      lead_standstill_distance = speed_finder_params.lead_standstill_distance();
      break;
    }
    case StBoundaryProto::STATIC: {
      if (lane_path != nullptr &&
          IsStaticStBoundaryBoomBarrier(st_boundary, psmm, *lane_path,
                                        st_traj_mgr)) {
        constexpr double kTollStandstillDist = 1.5;  // m.
        follow_standstill_distance = kTollStandstillDist;
      } else {
        follow_standstill_distance =
            speed_finder_params.follow_standstill_distance_for_static_obj();
      }
      lead_standstill_distance = speed_finder_params.lead_standstill_distance();
      break;
    }
    case StBoundaryProto::IMPASSABLE_BOUNDARY: {
      follow_standstill_distance =
          speed_finder_params.follow_standstill_distance_for_curb();
      lead_standstill_distance = 0.0;
      break;
    }
    case StBoundaryProto::PATH_BOUNDARY: {
      constexpr double kPathBoundaryStandstillDist = 1.0;
      follow_standstill_distance = kPathBoundaryStandstillDist;
      lead_standstill_distance = 0.0;
      break;
    }
    case StBoundaryProto::VIRTUAL: {
      const auto stop_line_it = std::find_if(
          constraint_mgr.StopLine().begin(), constraint_mgr.StopLine().end(),
          [&st_boundary](const ConstraintProto::StopLineProto& stopline) {
            return stopline.id() == st_boundary.id();
          });
      const auto path_stop_line_it = std::find_if(
          constraint_mgr.PathStopLine().begin(),
          constraint_mgr.PathStopLine().end(),
          [&st_boundary](const ConstraintProto::PathStopLineProto& stopline) {
            return stopline.id() == st_boundary.id();
          });
      QCHECK(stop_line_it != constraint_mgr.StopLine().end() ||
             path_stop_line_it != constraint_mgr.PathStopLine().end());
      follow_standstill_distance =
          stop_line_it != constraint_mgr.StopLine().end()
              ? stop_line_it->standoff()
              : path_stop_line_it->standoff();
      lead_standstill_distance = 0.0;
      break;
    }
    case StBoundaryProto::IGNORABLE:
    case StBoundaryProto::UNKNOWN_OBJECT: {
      QLOG(FATAL) << "Unpexted "
                  << StBoundaryProto::ObjectType_Name(st_boundary.object_type())
                  << " st-boundary " << st_boundary.id();
      break;
    }
  }

  return std::make_pair(follow_standstill_distance, lead_standstill_distance);
}

void ModifyStationaryObjectFollowDistanceForCongestionScene(
    const absl::flat_hash_set<std::string>& congested_cutin_object_ids,
    StBoundaryWithDecision* st_boundary_wd) {
  if (congested_cutin_object_ids.empty()) return;
  constexpr double kEps = 1e-2;
  const auto& st_boundary = *st_boundary_wd->raw_st_boundary();
  if (st_boundary.min_s() < kEps) return;
  if (st_boundary.source_type() != StBoundarySourceTypeProto::ST_OBJECT) {
    return;
  }
  // Only consider stationary vehicles.
  if (!st_boundary.is_stationary()) return;
  if (st_boundary.object_type() != StBoundaryProto::VEHICLE) return;
  if (st_boundary.is_large_vehicle()) return;
  if (!st_boundary.object_id().has_value()) return;
  if (ContainsKey(congested_cutin_object_ids, *st_boundary.object_id())) {
    // Do not modify follow distance if this object has cut-in intention.
    return;
  }

  constexpr double kCongestionSceneFollowDistanceBuffer = -0.5;  // m.
  const double follow_distance =
      std::clamp(st_boundary_wd->follow_standstill_distance() +
                     kCongestionSceneFollowDistanceBuffer,
                 0.0, st_boundary.min_s());
  st_boundary_wd->set_follow_standstill_distance(follow_distance);
  const std::string info =
      absl::StrCat("%s. Modify follow distance: (CS), %.3f. ",
                   st_boundary_wd->decision_info(), follow_distance);
  st_boundary_wd->set_decision_info(info);
}

}  // namespace

void DecideStandstillDistanceForStBoundary(
    const StandstillDistanceDeciderInput& input,
    StBoundaryWithDecision* st_boundary_wd) {
  QCHECK_NOTNULL(input.speed_finder_params);
  QCHECK_NOTNULL(input.stalled_object_ids);
  QCHECK_NOTNULL(input.st_traj_mgr);
  QCHECK_NOTNULL(input.constraint_mgr);
  QCHECK_NOTNULL(st_boundary_wd);
  const auto& st_boundary = *st_boundary_wd->raw_st_boundary();
  const auto standstill_distance = GetStBoundaryStandStillDistance(
      st_boundary, *input.speed_finder_params, *input.stalled_object_ids,
      input.planner_semantic_map_manager, input.lane_path, *input.st_traj_mgr,
      *input.constraint_mgr, input.extra_follow_standstill_for_large_vehicle);
  st_boundary_wd->set_follow_standstill_distance(standstill_distance.first);
  st_boundary_wd->set_lead_standstill_distance(standstill_distance.second);

  if (nullptr != input.congested_cutin_object_ids) {
    ModifyStationaryObjectFollowDistanceForCongestionScene(
        *input.congested_cutin_object_ids, st_boundary_wd);
  }
  return;
}

}  // namespace planner
}  // namespace qcraft

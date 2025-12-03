#include "onboard/planner/ml/captain_net/validation/trajectory_validation.h"

// IWYU pragma: no_include <ext/alloc_traits.h>
//              for __alloc_traits<>::value_type

#include <algorithm>  // for min, max
#include <cmath>      // for fabs, M_PI
#include <string>     // for string
#include <utility>    // for pair

#include "absl/container/flat_hash_map.h"  // for flat_hash_map, operator==, raw_hash_map
#include "absl/status/statusor.h"  // for StatusOr
#include "absl/types/span.h"       // for Span

#include "onboard/async/parallel_for.h"       // for ParallelFor
#include "onboard/global/trace.h"             // for SCOPED_QTRACE, ScopedTrace
#include "onboard/math/frenet_common.h"       // for FrenetCoordinate
#include "onboard/math/geometry/box2d.h"      // for Box2d
#include "onboard/math/geometry/polygon2d.h"  // for Polygon2d
#include "onboard/math/geometry/segment2d.h"  // for Segment2d
#include "onboard/math/util.h"                // for NormalizeAngle
#include "onboard/math/vec.h"  // for Vec2d, MatrixBase::operator-, MatrixBase::...
#include "onboard/planner/common/path_sl_boundary.h"  // for PathSlBoundary
#include "onboard/planner/router/drive_passage.h"     // for DrivePassage
#include "onboard/planner/trajectory_point.h"         // for TrajectoryPoint
#include "onboard/planner/trajectory_util.h"          // for ToTrajectoryPoint
#include "onboard/planner/trajectory_validation/trajectory_validation.h"  // for StObjectCollisionInfo, GetAvBoxFromTrajPoints
#include "onboard/planner/util/vehicle_geometry_util.h"  // for ComputeAvBoxWithBuffer, ComputeAvGeometryC...
#include "onboard/proto/trajectory_point.pb.h"  // for ApolloTrajectoryPointProto

namespace qcraft::planner::ml {

namespace {

bool ValidateTrajectoryCurbCollision(
    const PlannerSemanticMapManager& psmm,
    absl::Span<const TrajectoryPoint> traj_points,
    const VehicleGeometryParamsProto& vehicle_geometry_params) {
  constexpr double kCurbColCheckLengthHorizon = 50.0;
  constexpr int kCurbColCheckHorizon = 50;
  const double av_diagonal_length =
      Vec2d(vehicle_geometry_params.length(), vehicle_geometry_params.width())
          .norm();
  constexpr double kRadiusBuffer = 0.2;
  const double search_radius = 0.5 * av_diagonal_length + kRadiusBuffer;
  constexpr double kLatExtendBuffer = 0.1;
  for (int i = 0; i < traj_points.size() && i < kCurbColCheckHorizon; ++i) {
    if (traj_points[i].s() > kCurbColCheckLengthHorizon) break;

    // Extend the box laterally to account for lateral control error.
    const Box2d av_box = ComputeAvBoxWithBuffer(
        traj_points[i].pos(), traj_points[i].theta(), vehicle_geometry_params,
        /*length_buffer=*/0.0, /*width_buffer=*/kLatExtendBuffer);
    const Vec2d av_geo_center = ComputeAvGeometryCenter(
        traj_points[i].pos(), traj_points[i].theta(), vehicle_geometry_params);
    const auto level_id = psmm.GetLevel();
    const auto curb_segments = psmm.GetImpassableBoundariesAtLevel(
        level_id, av_geo_center, search_radius);
    for (const auto& curb_segment : curb_segments) {
      if (av_box.HasOverlap(curb_segment)) {
        return true;
      }
    }
  }
  return false;
}

bool ValidateTrajectoryObjectCollision(
    const std::vector<PartialSpacetimeObjectTrajectory>& considered_st_objects,
    const std::vector<TrajectoryPoint>& traj_points,
    const VehicleGeometryParamsProto& vehicle_geometry_params,
    ThreadPool* thread_pool) {
  constexpr int kObjectColCheckHorizon = 50;
  const auto av_box = GetAvBoxFromTrajPoints(
      traj_points, vehicle_geometry_params, kObjectColCheckHorizon);

  // Collision info: {string: planner_object_id, double: collision probability}.
  std::vector<StObjectCollisionInfo> collision_info(
      considered_st_objects.size());
  ParallelFor(0, considered_st_objects.size(), thread_pool, [&](int i) {
    GetStObjectCollisionInfo(considered_st_objects[i], traj_points, av_box,
                             /*skip_no_decision_object=*/false,
                             &collision_info[i]);
  });

  // Collect planner_object_id -> collision probability map. Check the result
  // and record collision result.
  constexpr double kCollisionProbThres = 0.8;
  absl::flat_hash_map<std::string, double> object_collision_prob_map;
  object_collision_prob_map.reserve(collision_info.size());
  for (int i = 0; i < collision_info.size(); ++i) {
    const auto& info = collision_info[i];
    if (object_collision_prob_map.find(info.object_id) ==
        object_collision_prob_map.end()) {
      object_collision_prob_map.emplace(info.object_id, info.probability);
    } else {
      object_collision_prob_map[info.object_id] += info.probability;
    }
    // Early exit if find an object whose cumulative collision probability is
    // beyond the threshold.
    const double cumulative_collision_prob =
        object_collision_prob_map[info.object_id];
    if (cumulative_collision_prob > kCollisionProbThres) {
      return true;
    }
  }

  return false;
}

bool ValidatePathBoundaryViolation(
    const std::vector<TrajectoryPoint>& traj_points,
    const DrivePassage& drive_passage, const PathSlBoundary& path_sl_boundary,
    const VehicleGeometryParamsProto& vehicle_geometry_params) {
  constexpr double kPathBoundaryViolationLimit = 1.2;  // meter
  constexpr int kPathBoundaryViolationCheckHorizon = 50;
  for (int i = 0;
       i < traj_points.size() && i < kPathBoundaryViolationCheckHorizon; ++i) {
    const auto& pt = traj_points[i];
    const auto frenet_pt_or = drive_passage.QueryFrenetCoordinateAt(pt.pos());
    if (!frenet_pt_or.ok()) {
      return true;
    }
    const auto l_pair = path_sl_boundary.QueryBoundaryL(frenet_pt_or->s);
    const double excced_left_boundary_dist =
        l_pair.second - frenet_pt_or->l - 0.5 * vehicle_geometry_params.width();
    if (excced_left_boundary_dist < -kPathBoundaryViolationLimit) {
      return true;
    }
    const double excced_right_boundary_dist =
        l_pair.first - frenet_pt_or->l + 0.5 * vehicle_geometry_params.width();
    if (excced_right_boundary_dist > kPathBoundaryViolationLimit) {
      return true;
    }
  }
  return false;
}

bool ValidateReverseDriving(const std::vector<TrajectoryPoint>& traj_points) {
  constexpr int kReverseDrivingCheckHorizon = 50;
  const double reverse_driving_threshold = M_PI / 2;
  const int reverse_driving_check_horizon = std::min(
      static_cast<int>(traj_points.size()), kReverseDrivingCheckHorizon);
  for (int i = 0; i + 1 < reverse_driving_check_horizon; ++i) {
    const Vec2d pos_vec = traj_points[i + 1].pos() - traj_points[i].pos();
    const double pos_vec_theta = pos_vec.FastAngle();
    const double reverse_driving_error =
        NormalizeAngle(pos_vec_theta - traj_points[i].theta());
    if (std::fabs(reverse_driving_error) > reverse_driving_threshold) {
      return true;
    }
  }
  return false;
}

bool ValidateSpeedLimitViolation(
    const SchedulerOutput& scheduler_output,
    const std::vector<TrajectoryPoint>& traj_points) {
  constexpr int kSpeedLimitViolationHorizon = 51;
  constexpr int kSpeedLimitViolationDelta = 10;
  const int horizon = std::min(static_cast<int>(traj_points.size()),
                               kSpeedLimitViolationHorizon);
  for (int i = 0; i < horizon; i += kSpeedLimitViolationDelta) {
    const double cur_v = traj_points[i].v();
    const auto speed_limit_or =
        scheduler_output.drive_passage.QuerySpeedLimitAt(traj_points[i].pos());
    if (!speed_limit_or.ok()) {
      continue;
    }
    if (std::fabs(cur_v) > *speed_limit_or) {
      return true;
    }
  }
  return false;
}
}  // namespace

std::vector<PartialSpacetimeObjectTrajectory> FakeConsideredStObjects(
    const SpacetimeTrajectoryManager& st_traj_mgr) {
  std::vector<PartialSpacetimeObjectTrajectory> all_trajs;
  const auto st_trajs = st_traj_mgr.trajectories();
  all_trajs.reserve(st_trajs.size());
  for (const auto& st_traj : st_trajs) {
    all_trajs.push_back(PartialSpacetimeObjectTrajectory(st_traj));
  }
  return all_trajs;
}

void CheckTrajectoryValidation(
    const PlannerSemanticMapManager& psmm,
    const std::vector<PartialSpacetimeObjectTrajectory>& considered_st_objects,
    bool full_stop, const SchedulerOutput& scheduler_output,
    const VehicleGeometryParamsProto& vehicle_geometry_params,
    captain_net::CaptainNetOutput* output, ThreadPool* thread_pool) {
  SCOPED_QTRACE("CaptainNet::CheckCaptainTrajectoryValidation");

  std::vector<TrajectoryPoint> traj_points =
      ToTrajectoryPoint(output->traj_points);

  output->validation.is_curb_collsion = ValidateTrajectoryCurbCollision(
      psmm, traj_points, vehicle_geometry_params);

  if (!full_stop) {
    output->validation.is_object_collision =
        ValidateTrajectoryObjectCollision(considered_st_objects, traj_points,
                                          vehicle_geometry_params, thread_pool);
  }

  output->validation.is_path_boundary_collision = ValidatePathBoundaryViolation(
      traj_points, scheduler_output.drive_passage, scheduler_output.sl_boundary,
      vehicle_geometry_params);

  output->validation.is_reverse_driving = ValidateReverseDriving(traj_points);

  output->validation.is_speed_limit_violation =
      ValidateSpeedLimitViolation(scheduler_output, traj_points);
}
}  // namespace qcraft::planner::ml

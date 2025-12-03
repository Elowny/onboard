#include "onboard/planner/plan/acc/acc_speed_finder.h"

#include <algorithm>  // IWYU pragma: keep
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "absl/time/clock.h"
#include "absl/types/span.h"

#include "gtest/gtest.h"

#include "onboard/global/buffered_logger.h"
#include "onboard/lite/check.h"
#include "onboard/planner/common/discretized_path.h"
#include "onboard/planner/object/spacetime_trajectory_manager.h"
#include "onboard/planner/speed/speed_finder_flags.h"  // IWYU pragma: keep
#include "onboard/planner/speed/st_boundary.h"
#include "onboard/planner/speed/st_boundary_with_decision.h"
#include "onboard/planner/speed/st_point.h"
#include "onboard/planner/speed/vt_point.h"
#include "onboard/planner/test_util/util.h"
#include "onboard/planner/util/path_util.h"
#include "onboard/proto/trajectory_point.pb.h"
#include "onboard/proto/vehicle.pb.h"

namespace qcraft {

namespace planner {
namespace {

PlannerParamsProto planner_params = DefaultPlannerParams();
VehicleGeometryParamsProto veh_geo_params = DefaultVehicleGeometry();

TEST(RunAccSpeedFinder, OptimizeAccSpeedTest) {
  double init_v = 0.0;
  double init_a = 0.0;
  StBoundaryPoints st_boundary_points;
  st_boundary_points.lower_points = {StPoint(5.0, 0.0), StPoint(5.0, 5.0)};
  st_boundary_points.upper_points = {StPoint(50.0, 0.0), StPoint(50.0, 5.0)};
  st_boundary_points.speed_points = {VtPoint(0.0, 0.0), VtPoint(0.0, 10.0)};
  StBoundaryRef st_boundary = StBoundary::CreateInstance(
      st_boundary_points, StBoundaryProto::VIRTUAL, "001", /*probability=*/1.0,
      /*is_stationary=*/true, StBoundaryProto::NON_PROTECTIVE,
      /*is_large_vehicle=*/false);
  std::vector<SpeedLimit::SpeedLimitRange> range;
  range.push_back({.start_s = 0.0, .end_s = 100.0, .speed_limit = 5.0});
  SpeedLimit lane_limit(range);

  std::vector<StBoundaryWithDecision> st_boundaries;
  st_boundaries.emplace_back(
      std::move(st_boundary), StBoundaryProto::FOLLOW,
      StBoundaryProto::UNKNOWN_REASON, /*decision_info=*/"",
      /*follow_standstill_distance=*/4.0, /*lead_standstill_distance=*/4.0,
      /*pass_time=*/0.0, /*yield_time=*/0.0);

  constexpr double kTotalTime = 10.0;  // s.
  const int knot_num =
      planner_params.speed_finder_params().speed_optimizer_params().knot_num();
  const double delta_t = kTotalTime / (knot_num - 1);  // s.
  const double allowed_max_speed =
      planner_params.motion_constraint_params().default_speed_limit();

  SpeedVector preliminary_speed;
  for (int i = 0; i < 100; ++i) {
    const double time = 0.1 * i;
    preliminary_speed.emplace_back(time, 5.0 * time, 5.0, 0.0, 0.0);
  }
  SpacetimeTrajectoryManager traj_mgr;
  SpeedOptimizerObjectManager opt_obj_mgr(st_boundaries, &preliminary_speed,
                                          traj_mgr, init_v, kTotalTime, delta_t,
                                          planner_params.speed_finder_params());
  const auto speed_bound_map =
      internal::EstimateDefaultSpeedBound(lane_limit, preliminary_speed, init_v,
                                          allowed_max_speed, knot_num, delta_t);

  const double default_v = 10.0;
  SpeedVector reference_speed;
  reference_speed.reserve(knot_num);
  for (int i = 0; i < knot_num; ++i) {
    reference_speed.emplace_back(
        /*t=*/i * 0.1, /*s=*/i * 0.1 * default_v,
        /*v=*/default_v,
        /*a=*/0.0,
        /*j=*/0.0);
  }

  SpeedVector optimized_speed;
  SpeedFinderDebugProto speed_finder_debug_proto;

  const absl::Status ret = internal::OptimizeAccSpeed(
      /*base_name=*/"", init_v, init_a, delta_t, opt_obj_mgr,
      /*path_length=*/100.0, speed_bound_map, reference_speed,
      /*time_aligned_prev_traj=*/nullptr,
      planner_params.motion_constraint_params(),
      planner_params.speed_finder_params(), &optimized_speed,
      &speed_finder_debug_proto);

  EXPECT_TRUE(ret.ok());
}

TEST(RunAccSpeedFinder, EstimateDefaultSpeedBoundTest) {
  std::vector<SpeedLimit::SpeedLimitRange> range;
  range.push_back({.start_s = 0.0, .end_s = 100.0, .speed_limit = 5.0});
  SpeedLimit lane_limit(range);
  double init_v = 0.0;
  const double allowed_max_speed = 22.2;  // m/s
  const int knot_num = 100;
  constexpr double kTotalTime = 10.0;                  // s.
  const double delta_t = kTotalTime / (knot_num - 1);  // s.
  SpeedVector preliminary_speed;
  for (int i = 0; i < 100; ++i) {
    const double time = 0.1 * i;
    preliminary_speed.emplace_back(time, 5.0 * time, 5.0, 0.0, 0.0);
  }
  const auto speed_bound_map =
      internal::EstimateDefaultSpeedBound(lane_limit, preliminary_speed, init_v,
                                          allowed_max_speed, knot_num, delta_t);
  QCHECK_EQ(speed_bound_map.size(), 2);
}

TEST(RunAccSpeedFinder, FindAccSpeedTest) {
  SpacetimeTrajectoryManager traj_mgr;

  constexpr int kPathCounts = 100;
  std::vector<PathPoint> path_points;
  path_points.reserve(kPathCounts);
  PathPoint start;
  start.set_x(0.0);
  start.set_y(0.0);
  start.set_z(0.0);
  start.set_theta(0.0);
  start.set_kappa(0.001);
  start.set_lambda(0.0);
  start.set_s(0.0);
  for (int k = 0; k < kPathCounts; ++k) {
    path_points.push_back(
        GetPathPointAlongCircle(start, static_cast<double>(k) * 1.0));
  }
  const DiscretizedPath path(path_points);

  SpeedLimit::SpeedLimitRange range;
  range.start_s = 0.0;
  range.end_s = 0.0;
  range.speed_limit = 10.0;
  range.info = "";

  std::vector<SpeedLimit::SpeedLimitRange> speed_limit_range;
  speed_limit_range.push_back(range);
  SpeedLimit speed_limit(speed_limit_range);

  std::set<std::string> crowded_side_objs;

  AccSpeedFinderInput acc_speed_finder_input{
      .base_name = "acc_speed",
      .traj_mgr = &traj_mgr,
      .path = &path,
      .speed_limit = &speed_limit,
      .plan_start_v = 0.0,
      .plan_start_a = 0.0,
      .user_speed_limit = 10.0,
      .plan_time = absl::Now(),
      .crowded_side_objs = &crowded_side_objs,
      .motion_constraint_params = &planner_params.motion_constraint_params(),
      .speed_finder_params = &planner_params.speed_finder_params(),
      .vehicle_geom = &veh_geo_params,
  };

  const absl::StatusOr<AccSpeedFinderOutput> res =
      FindAccSpeed(acc_speed_finder_input);
  QCHECK(res.ok());
}

}  // namespace
}  // namespace planner
}  // namespace qcraft

#include "onboard/planner/speed/speed_optimizer.h"

#include <map>
#include <optional>
#include <ostream>
#include <vector>

#include "absl/types/span.h"

#include "gtest/gtest.h"

#include "onboard/lite/check.h"
#include "onboard/lite/logging.h"
#include "onboard/planner/object/spacetime_trajectory_manager.h"
#include "onboard/planner/proto/planner_params.pb.h"
#include "onboard/planner/speed/proto/speed_finder.pb.h"
#include "onboard/planner/speed/speed_finder_util.h"
#include "onboard/planner/speed/speed_limit.h"
#include "onboard/planner/speed/speed_optimizer_object_manager.h"
#include "onboard/planner/speed/speed_point.h"
#include "onboard/planner/speed/st_boundary.h"
#include "onboard/planner/speed/st_boundary_with_decision.h"
#include "onboard/planner/speed/st_point.h"
#include "onboard/planner/speed/vt_point.h"
#include "onboard/planner/test_util/util.h"
#include "onboard/utils/map_util.h"

namespace qcraft::planner {
namespace {

SpeedBoundMapType EstimateDefaultSpeedBound(
    const SpeedLimit& speed_limit, const SpeedVector& preliminary_speed,
    double init_v, double allowed_max_speed, int knot_num, double delta_t) {
  const auto fill_speed_bound =
      [](const std::optional<SpeedLimit::SpeedLimitInfo>& speed_limit_info,
         SpeedBoundWithInfo* speed_bound, double allowed_max_speed) {
        QCHECK_NOTNULL(speed_bound);
        speed_bound->bound = speed_limit_info.has_value()
                                 ? speed_limit_info->speed_limit
                                 : allowed_max_speed;
        speed_bound->info =
            speed_limit_info.has_value() ? speed_limit_info->info : "";
      };

  SpeedBoundMapType speed_upper_bound_map;

  std::vector<double> estimated_s;
  estimated_s.reserve(knot_num);
  for (int i = 0; i < knot_num; ++i) {
    const double t = i * delta_t;
    const auto speed_point = preliminary_speed.EvaluateByTime(t);
    const double s = speed_point.has_value() ? speed_point->s() : init_v * t;
    estimated_s.push_back(s);
  }
  // Estimate speed bound for static speed limit.
  std::vector<SpeedBoundWithInfo> speed_bounds_with_info;
  speed_bounds_with_info.reserve(knot_num);
  for (int i = 0; i < knot_num; ++i) {
    const auto speed_limit_info =
        speed_limit.GetSpeedLimitInfoByS(estimated_s[i]);
    fill_speed_bound(speed_limit_info, &speed_bounds_with_info.emplace_back(),
                     allowed_max_speed);
  }
  speed_upper_bound_map.emplace(SpeedLimitTypeProto::DEFAULT,
                                speed_bounds_with_info);
  speed_upper_bound_map.emplace(SpeedLimitTypeProto::COMBINATION,
                                std::move(speed_bounds_with_info));
  return speed_upper_bound_map;
}

TEST(SpeedOptimizerTest, SimpleTest) {
  auto planner_params = DefaultPlannerParams();
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

  SpeedVector preliminary_speed;
  for (int i = 0; i < 100; ++i) {
    const double time = 0.1 * i;
    preliminary_speed.emplace_back(time, 5.0 * time, 5.0, 0.0, 0.0);
  }

  SpeedFinderDebugProto speed_finder_debug;
  SpeedVector speed_data;

  constexpr double kMaxPathLength = 100.0;
  constexpr double kTotalTime = 10.0;  // s.
  const int knot_num =
      planner_params.speed_finder_params().speed_optimizer_params().knot_num();
  const double delta_t = kTotalTime / (knot_num - 1);  // s.
  const double allowed_max_speed =
      planner_params.motion_constraint_params().default_speed_limit();
  SpeedOptimizer optimizer("UNITTEST", init_v, init_a,
                           &planner_params.motion_constraint_params(),
                           &planner_params.speed_finder_params(),
                           kMaxPathLength, allowed_max_speed, delta_t);

  SpacetimeTrajectoryManager traj_mgr;
  SpeedOptimizerObjectManager opt_obj_mgr(st_boundaries, &preliminary_speed,
                                          traj_mgr, init_v, kTotalTime, delta_t,
                                          planner_params.speed_finder_params());

  const auto speed_bound_map =
      EstimateDefaultSpeedBound(lane_limit, preliminary_speed, init_v,
                                allowed_max_speed, knot_num, delta_t);
  const auto reference_speed = GenerateReferenceSpeed(
      FindOrDie(speed_bound_map, SpeedLimitTypeProto::DEFAULT), init_v,
      planner_params.speed_finder_params()
          .speed_optimizer_params()
          .ref_speed_bias(),
      planner_params.speed_finder_params()
          .speed_optimizer_params()
          .ref_speed_static_limit_bias(),
      planner_params.motion_constraint_params().max_acceleration(),
      planner_params.motion_constraint_params().max_deceleration(), kTotalTime,
      delta_t);
  const absl::Status ret = optimizer.Optimize(
      opt_obj_mgr, speed_bound_map, reference_speed,
      /*time_aligned_prev_traj=*/nullptr, &speed_data, &speed_finder_debug);
  EXPECT_TRUE(ret.ok());

  for (const auto& res : speed_data) {
    QLOG(INFO) << "s[" << res.s() << "], t[" << res.t() << "], v[" << res.v()
               << "], a[" << res.a() << "].";
  }
}

}  // namespace
}  // namespace qcraft::planner

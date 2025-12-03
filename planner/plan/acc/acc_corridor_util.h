#ifndef ONBOARD_PLANNER_PLAN_ACC_CORRIDOR_UTIL_H_
#define ONBOARD_PLANNER_PLAN_ACC_CORRIDOR_UTIL_H_

#include <algorithm>
#include <cmath>

#include "absl/status/statusor.h"
#include "absl/types/span.h"

#include "onboard/math/cubic_spline.h"
#include "onboard/math/fast_math.h"
#include "onboard/math/piecewise_linear_function.h"
#include "onboard/math/util.h"
#include "onboard/math/vec.h"
#include "onboard/planner/common/discretized_path.h"
#include "onboard/planner/common/driving_map_topo.h"
#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/proto/positioning.pb.h"
#include "onboard/proto/trajectory_point.pb.h"

namespace qcraft {
namespace planner {

struct UniCycleState {
  double x = 0.0;
  double y = 0.0;
  double heading = 0.0;
  double kappa = 0.0;
};

inline UniCycleState SimulateUniCycleState(const UniCycleState& state, double s,
                                           double kappa_decay) {
  double heading_cos_sin[2];
  fast_math::CosAndSin<7>(state.heading, heading_cos_sin);
  return UniCycleState{
      .x = state.x + s * heading_cos_sin[0],
      .y = state.y + s * heading_cos_sin[1],
      .heading = state.heading + s * state.kappa * kappa_decay,
      .kappa = state.kappa * kappa_decay,
  };
}

inline UniCycleState SimulateUniCycleStateWithHeadingRestrict(
    const UniCycleState& state, double s, double kappa_decay,
    double init_heading, double max_delta_heading) {
  double heading_cos_sin[2];
  fast_math::CosAndSin<7>(state.heading, heading_cos_sin);
  return UniCycleState{
      .x = state.x + s * heading_cos_sin[0],
      .y = state.y + s * heading_cos_sin[1],
      .heading = std::clamp(state.heading + s * state.kappa * kappa_decay,
                            init_heading - max_delta_heading,
                            init_heading + max_delta_heading),
      .kappa = state.kappa * kappa_decay,
  };
}

inline Vec2d LatPoint(const Vec2d& pos, const Vec2d& tangent, double l) {
  return pos + tangent.Perp() * l;
}

DiscretizedPath ComputeExtendedSmoothPath(absl::Span<const Vec2d> ref_center_xy,
                                          absl::Span<const double> ref_s_vec,
                                          const Vec2d& heading, double step_s,
                                          double resample_step_s,
                                          double extended_length);

PiecewiseLinearFunction<double> ComputeSmoothedCurvatureProfile(
    absl::Span<const Vec2d> xy);

PathPoint EvaluatePathPointOnCubicSplines(const CubicSpline& x_s,
                                          const CubicSpline& y_s, double cur_s);

inline double ComputeCurvature(double dx, double dy, double ddx, double ddy) {
  const double nominator = dx * ddy - dy * ddx;
  const double denominator = PowerThreeHalves(dx * dx + dy * dy);
  constexpr double kEpsilon = 1e-8;
  return std::fabs(denominator) < kEpsilon ? 0.0 : nominator / denominator;
}

absl::StatusOr<DrivingMapTopo> BuildDrivingMapTopo(
    const PoseProto& pose, const PlannerSemanticMapManager& psmm,
    double plan_start_v);

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_PLAN_ACC_CORRIDOR_UTIL_H_

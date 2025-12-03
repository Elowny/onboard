
#include "onboard/prediction/util/pole_placement_util.h"

// IWYU pragma: no_include <ext/alloc_traits.h>
//              for __alloc_traits<>::value_type
#include <stddef.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <ostream>
#include <utility>

#include "absl/status/status.h"
#include "absl/status/statusor.h"

#include "onboard/global/logging.h"
#include "onboard/lite/check.h"
#include "onboard/math/cubic_spline.h"
#include "onboard/math/fast_math.h"
#include "onboard/math/frenet_common.h"
#include "onboard/math/polynomial_fitter.h"
#include "onboard/math/polynomialxd.h"
#include "onboard/math/util.h"
#include "onboard/math/vec.h"
#include "onboard/planner/router/drive_passage.h"
#include "onboard/prediction/prediction_defs.h"
#include "onboard/prediction/util/trajectory_developer.h"
#include "onboard/utils/status_macros.h"
namespace qcraft {
namespace prediction {
namespace {
constexpr double kMinPathSpeed = 3.0;  // m/s.
const PiecewiseLinearFunction<double, double> kPathSpeedLimitPlf(
    std::vector<double>{0.0, 10.0, 15.0, 30.0},
    std::vector<double>{5.0, 5.0, 8.0, 15.0});

// Pole placement parameters
constexpr double kDesirePolePlacementDs =
    8.0;  // Sample interval of drive passage.
constexpr int kMinPolePlacementDpSampleNum =
    5;  // Minimal samples of drive passage.

constexpr double kPolePlacementReferencePointDs = 0.2;  // m.
constexpr double kPoleMinSpeed = 0.6;
constexpr double kPoleMinAcc = -9.8;  // Min acc for pole placement tracking
constexpr double kPoleMaxAcc = 4.0;   // Max acc for pole placement tracking
constexpr double kMaxLatErr = 2.0;
constexpr double kMaxHeadingErr = 0.175;
constexpr double kMaxLateralAcceleration = 3.0;
constexpr double kNominalDv = 5.0;
constexpr double kMaxDistanceSquare = 100.0;            // 10 meters.
constexpr double kEndOfReferenceDistanceSquare = 0.25;  // 0.5 meter.
constexpr int kLookAheadIndex = 20;
const PiecewiseLinearFunction<double, double> kVelToPolesPlf(
    std::vector<double>{0.0, 0.5, 1.8, 6.6, 10.5, 11.5, 35.0},
    std::vector<double>{0.01, 0.2, 0.5, 0.8, 1.0, 1.3, 3.0});
const PiecewiseLinearFunction<double, double> kReferencePointMoveRatio(
    std::vector<double>{2.0, 4.0}, std::vector<double>{1.0, 0.0});
BicycleModelState UpdateBicycleStatebySteer(const BicycleModelState& state,
                                            double velocity, double steer_cmd,
                                            double wheelbase, double length,
                                            double max_steer, double dt) {
  // KSteerInertia is for imitating steering delay.
  const double max_curvature =
      kMaxLateralAcceleration / Sqr(std::max(kPoleMinSpeed, velocity));
  max_steer = std::min(max_curvature * length, max_steer);
  BicycleModelState update_bicycle_state = state;

  // Assume max steer rate is from 0 to max steer in 0.5s.
  update_bicycle_state.front_wheel_angle =
      std::clamp(steer_cmd, state.front_wheel_angle - max_steer * dt * 2.0,
                 state.front_wheel_angle + max_steer * dt * 2.0);

  update_bicycle_state.front_wheel_angle =
      std::clamp(update_bicycle_state.front_wheel_angle, -max_steer, max_steer);

  update_bicycle_state.x =
      state.x + velocity * fast_math::Cos(state.heading) * dt;
  update_bicycle_state.y =
      state.y + velocity * fast_math::Sin(state.heading) * dt;
  update_bicycle_state.heading =
      NormalizeAngle(state.heading +
                     velocity / wheelbase *
                         std::tan(update_bicycle_state.front_wheel_angle) * dt);
  update_bicycle_state.v = velocity;
  return update_bicycle_state;
}

double FeedbackSteer(double lateral_error, double heading_error, double speed,
                     double wheelbase, double ts) {
  // No feedback when speed is too low.
  if (std::fabs(speed) < kEpsilon) {
    return 0.0;
  }
  const double pole = -1.0 * kVelToPolesPlf(std::fabs(speed));
  double v_limit = std::max(std::fabs(speed), kPoleMinSpeed);
  if (speed < 0.0) {
    v_limit = std::min(std::fabs(speed), -kPoleMinSpeed);
  }
  const double pole_exp = std::exp(pole * ts);

  const double lat_gain = ((1.0 - 2.0 * pole_exp + Sqr(pole_exp)) * wheelbase) /
                          (Sqr(v_limit) * Sqr(ts));
  const double heading_gain =
      ((3.0 - 2.0 * pole_exp - Sqr(pole_exp)) * wheelbase) /
      (2.0 * v_limit * ts);
  const double lateral_error_limit =
      std::clamp(lateral_error, -kMaxLatErr, kMaxLatErr);
  const double heading_error_limit =
      std::clamp(heading_error, -kMaxHeadingErr, kMaxHeadingErr);

  return (lateral_error_limit * lat_gain + heading_error_limit * heading_gain);
}

int FindClosestPointIndex(absl::Span<const Vec2d> reference_points,
                          int start_idx, double x, double y,
                          int look_ahead_index) {
  unsigned int index_min = 0;
  double dist_min_sqr = std::numeric_limits<double>::infinity();
  for (int i = start_idx;
       i < std::min<int>(start_idx + look_ahead_index + 1,
                         static_cast<int>(reference_points.size()));
       ++i) {
    const double dist_temp_sqr =
        Sqr((reference_points[i].x() - x)) + Sqr((reference_points[i].y() - y));
    if (dist_temp_sqr <= dist_min_sqr) {
      index_min = i;
      dist_min_sqr = dist_temp_sqr;
    }
  }
  return index_min;
}

double ComputeFeedBackSteerCommand(const BicycleModelState& rear_pt,
                                   const Vec2d& ref_pt, double heading_ref,
                                   double wheelbase, double dt) {
  const double heading_error = NormalizeAngle(heading_ref - rear_pt.heading);
  const double xb = ref_pt.x();
  const double yb = ref_pt.y();
  const double xe = xb + fast_math::Cos(heading_ref);
  const double ye = yb + fast_math::Sin(heading_ref);
  const double lateral_error =
      -((xe - xb) * (rear_pt.y - yb) - (ye - yb) * (rear_pt.x - xb)) /
      Hypot(xe - xb, ye - yb);
  return FeedbackSteer(lateral_error, heading_error, rear_pt.v, wheelbase, dt);
}

struct PolePlacementReference {
  std::vector<Vec2d> points;
  std::vector<double> kappa;
  std::vector<double> heading;
};

absl::StatusOr<PolePlacementReference> CalculatePolePlacementReferencePoints(
    const BicycleModelState& state, const planner::DrivePassage& dp,
    const FrenetCoordinate& state_sl,
    const PiecewiseLinearFunction<double, double>& s_offset_plf,
    double sample_dist, bool is_reverse_driving) {
  constexpr double kMaxBackwardExtensionLengthFromCurrentState = 10.0;  // m.
  const double front_accum_s = is_reverse_driving ? dp.end_s() : dp.front_s();
  const double backward_extension_length =
      std::min<double>(kMaxBackwardExtensionLengthFromCurrentState,
                       std::fabs(state_sl.s - front_accum_s));
  sample_dist += backward_extension_length;  // Total sample dist.
  int target_num =
      std::max(static_cast<int>(sample_dist / kDesirePolePlacementDs) + 1,
               kMinPolePlacementDpSampleNum);
  const double desire_ds = sample_dist / target_num;
  std::vector<double> vec_x, vec_y, vec_s;
  vec_x.reserve(target_num);
  vec_y.reserve(target_num);
  vec_s.reserve(target_num);
  std::vector<Vec2d> vec_xs, vec_ys;
  vec_xs.reserve(target_num);
  vec_ys.reserve(target_num);
  double last_s = 0.0;
  const double start_sample_s = is_reverse_driving
                                    ? state_sl.s + backward_extension_length
                                    : state_sl.s - backward_extension_length;
  for (int idx = 0; idx < target_num && desire_ds * idx <= sample_dist; ++idx) {
    double cur_s = start_sample_s;
    double relative_s = desire_ds * idx;
    if (is_reverse_driving) {
      cur_s -= relative_s;
    } else {
      cur_s += relative_s;
    }
    cur_s = std::clamp(cur_s, dp.front_s(), dp.end_s());
    relative_s = std::fabs(cur_s - start_sample_s);
    // Relative to start sample s, as x-axis for poly fitter.
    if (!vec_s.empty() && relative_s <= vec_s.back()) {
      break;
    }
    // Plf's is s relative to state current s.
    const double relative_s_state =
        is_reverse_driving ? cur_s - state_sl.s : state_sl.s - cur_s;
    const double lat_offset = s_offset_plf(relative_s_state);
    auto pt_or = dp.QueryPointXYAtSL(cur_s, lat_offset);
    if (!pt_or.ok()) {
      break;
    }
    vec_x.push_back(pt_or->x());
    vec_y.push_back(pt_or->y());
    vec_s.push_back(relative_s);
    vec_xs.push_back(Vec2d(relative_s, pt_or->x()));
    vec_ys.push_back(Vec2d(relative_s, pt_or->y()));
    last_s = relative_s;
    cur_s += desire_ds;
  }
  if (vec_x.size() < 3) {
    return absl::NotFoundError("Reference path not long enough");
  }
  CubicSpline x_fitter(vec_s, vec_x);
  CubicSpline y_fitter(vec_s, vec_y);
  constexpr int kPolynomialDegree = 5;
  const auto x_polynomial_or = FitPolynomialXdToData<kPolynomialDegree>(vec_xs);
  const auto y_polynomial_or = FitPolynomialXdToData<kPolynomialDegree>(vec_ys);
  const bool has_poly = x_polynomial_or.ok() && y_polynomial_or.ok();

  PolePlacementReference refs;
  auto& reference_points = refs.points;
  auto& ks = refs.kappa;
  auto& heading_ref = refs.heading;
  double ref_point_ds =
      std::max(kPolePlacementReferencePointDs, state.v * kPredictionTimeStep);
  const int ref_num = static_cast<int>(std::floor(last_s / ref_point_ds)) + 1;
  reference_points.reserve(ref_num);
  ks.reserve(ref_num);
  heading_ref.reserve(ref_num);
  constexpr double kKappaLimit = 0.2;
  for (int i = 0; i < ref_num; ++i) {
    const double t = i * ref_point_ds;

    double x = x_fitter.Evaluate(t);
    double y = y_fitter.Evaluate(t);
    double dx = 0.0, dy = 0.0;
    double d2x = 0.0, d2y = 0.0;
    if (has_poly) {
      dx = x_polynomial_or->EvaluateDerivative</*order=*/1>(t);
      d2x = x_polynomial_or->EvaluateDerivative</*order=*/2>(t);
      dy = y_polynomial_or->EvaluateDerivative</*order=*/1>(t);
      d2y = y_polynomial_or->EvaluateDerivative</*order=*/2>(t);
    } else {
      dx = x_fitter.EvaluateDerivative</*order=*/1>(t);
      d2x = x_fitter.EvaluateDerivative</*order=*/2>(t);
      dy = y_fitter.EvaluateDerivative</*order=*/1>(t);
      d2y = y_fitter.EvaluateDerivative</*order=*/2>(t);
    }
    constexpr double kLen2Epsilon = 1e-6;
    const double d_len2 = dx * dx + dy * dy + kLen2Epsilon;
    reference_points.push_back(Vec2d(x, y));
    ks.push_back(
        std::clamp((dx * d2y - dy * d2x) / (d_len2 * std::sqrt(d_len2)),
                   -kKappaLimit, kKappaLimit));
    heading_ref.push_back(fast_math::Atan2(dy, dx));
  }
  return refs;
}
}  // namespace
std::vector<PredictedTrajectoryPoint> DevelopPolePlacementTrajectory(
    absl::Span<const Vec2d> ref_pts, absl::Span<const double> ref_ks,
    absl::Span<const double> ref_headings, absl::Span<const double> ref_v,
    const BicycleModelState& state, double dt, double obj_len, double max_steer,
    double wheelbase, PolePlacementFeedForwardType ff_type) {
  const int desired_num_pts = ref_v.size();
  std::vector<PredictedTrajectoryPoint> points;
  points.reserve(desired_num_pts);
  QCHECK_GE(state.v, 0.0);
  PredictedTrajectoryPoint init_pt;
  init_pt.set_t(0.0);
  init_pt.set_s(0.0);
  init_pt.set_pos(Vec2d(state.x, state.y));
  init_pt.set_theta(state.heading);
  init_pt.set_kappa(0.0);
  init_pt.set_v(std::fabs(state.v));
  init_pt.set_a(0.0);
  points.push_back(std::move(init_pt));

  // obstacle info to bicycle state
  BicycleModelState cur_bicycle_state = state;
  BicycleModelState new_bicycle_state = state;
  cur_bicycle_state.acc = 0.0;
  // compute pseudo control input
  BicycleModelState rear_pt = cur_bicycle_state;
  rear_pt.x = cur_bicycle_state.x -
              fast_math::Cos(cur_bicycle_state.heading) * 0.5 * wheelbase;
  rear_pt.y = cur_bicycle_state.y -
              fast_math::Sin(cur_bicycle_state.heading) * 0.5 * wheelbase;

  // forward simulation of the object's movement using pure pursuit controller
  double s = 0.0;
  int index = 0;
  for (int i = 1; i < desired_num_pts; ++i) {
    index = FindClosestPointIndex(ref_pts, index, rear_pt.x, rear_pt.y,
                                  kLookAheadIndex);
    const double dist_sqr = Sqr((ref_pts[index].x() - rear_pt.x)) +
                            Sqr((ref_pts[index].y() - rear_pt.y));
    if ((index + 1 >= static_cast<int>(ref_pts.size()) &&
         dist_sqr > kEndOfReferenceDistanceSquare) ||
        dist_sqr > kMaxDistanceSquare) {
      break;
    }

    double cur_speed = ref_v[i];
    cur_speed = std::max(cur_speed, rear_pt.v + kPoleMinAcc * dt);
    cur_speed = std::min(cur_speed, rear_pt.v + kPoleMaxAcc * dt);

    // Compute feedforward cmd.
    double feed_forward_cmd = 0.0;
    switch (ff_type) {
      case PolePlacementFeedForwardType::CURVATURE:
        feed_forward_cmd = std::atan(ref_ks[index] * wheelbase);
        break;
      case PolePlacementFeedForwardType::HEADING_DIFF:
        if (index != 0) {
          feed_forward_cmd = fast_math::Atan2(
              NormalizeAngle(ref_headings[index] - ref_headings[index - 1]) *
                  wheelbase,
              dt * std::max(cur_speed, 0.5));
        }
        break;
    }

    // Compute feedback cmd.
    const double feed_back_steer_cmd = ComputeFeedBackSteerCommand(
        rear_pt, ref_pts[index], ref_headings[index], wheelbase, dt);
    const double steer_cmd = feed_forward_cmd + feed_back_steer_cmd;

    // evolve the system
    new_bicycle_state = UpdateBicycleStatebySteer(
        rear_pt, cur_speed, steer_cmd, wheelbase, obj_len, max_steer, dt);
    Vec2d next_pos(new_bicycle_state.x, new_bicycle_state.y);
    next_pos +=
        0.5 * wheelbase * Vec2d::UnitFromAngle(new_bicycle_state.heading);
    s += (next_pos - points.back().pos()).norm();
    PredictedTrajectoryPoint next_pt;
    next_pt.set_t(dt * i);
    next_pt.set_pos(next_pos);
    next_pt.set_s(s);
    next_pt.set_theta(new_bicycle_state.heading);
    next_pt.set_kappa(
        fast_math::Sin(new_bicycle_state.front_wheel_angle) /
        (fast_math::Cos(new_bicycle_state.front_wheel_angle) * wheelbase));
    next_pt.set_v(new_bicycle_state.v);
    next_pt.set_a(0.0);
    points.push_back(std::move(next_pt));
    rear_pt = new_bicycle_state;
  }
  for (int i = 0; i < static_cast<int>(points.size()) - 1; ++i) {
    points[i].set_a((points[i + 1].v() - points[i].v()) / dt);
  }
  return points;
}

absl::StatusOr<std::vector<PredictedTrajectoryPoint>>
DevelopConstVelocityPolePlacementTrajectory(
    const BicycleModelState& state, const planner::DrivePassage& dp,
    const PiecewiseLinearFunction<double, double>& s_offset_plf, double dt,
    double horizon, double obj_len, double max_steer, double wheelbase,
    bool is_reverse_driving) {
  ASSIGN_OR_RETURN(const auto frenet_pt,
                   dp.QueryFrenetCoordinateAt(Vec2d(state.x, state.y)));
  const double sample_dist = horizon * (state.v + kNominalDv);
  ASSIGN_OR_RETURN(const auto refs, CalculatePolePlacementReferencePoints(
                                        state, dp, frenet_pt, s_offset_plf,
                                        sample_dist, is_reverse_driving));
  const int num_pts = static_cast<int>(horizon / dt) + 1;
  std::vector<double> ref_vs(num_pts, state.v);
  return DevelopPolePlacementTrajectory(
      refs.points, refs.kappa, refs.heading, ref_vs, state, dt, obj_len,
      max_steer, wheelbase, PolePlacementFeedForwardType::CURVATURE);
}

absl::StatusOr<planner::DiscretizedPath> DevelopPolePlacementPath(
    const BicycleModelState& state, const planner::DrivePassage& dp, double dt,
    double horizon, double lane_offset, double obj_len, double max_steer,
    double wheelbase) {
  const double cur_v = state.v;
  const double nominal_v = kPathSpeedLimitPlf(cur_v);
  auto cur_state = state;
  cur_state.v = nominal_v;

  const PiecewiseLinearFunction<double, double> s_offset_plf(
      {0.0, horizon * nominal_v}, {lane_offset, lane_offset});
  ASSIGN_OR_RETURN(const auto traj_pts,
                   DevelopConstVelocityPolePlacementTrajectory(
                       cur_state, dp, s_offset_plf, dt, horizon, obj_len,
                       max_steer, wheelbase,
                       /*is_reverse_driving=*/false));
  if (traj_pts.size() <= 1) {
    return absl::NotFoundError(
        "Generated path should contain at least two points.");
  }
  return PredictedTrajectoryPointsToDiscretizedPath(traj_pts);
}

absl::StatusOr<planner::DiscretizedPath>
DevelopPolePlacementPathWithSmoothConvergence(
    const BicycleModelState& state, const planner::DrivePassage& dp,
    const PiecewiseLinearFunction<double, double>& t_offset_plf, double dt,
    double horizon, double obj_len, double max_steer, double wheelbase) {
  const double cur_v = state.v;
  std::vector<double> s;
  s.reserve(t_offset_plf.x().size());
  const double estimated_v_for_offset = std::max(kMinPathSpeed, cur_v);
  for (const auto& t : t_offset_plf.x()) {
    s.push_back(t * estimated_v_for_offset);
  }
  const PiecewiseLinearFunction<double, double> s_offset_plf{s,
                                                             t_offset_plf.y()};

  const double nominal_v = kPathSpeedLimitPlf(cur_v);
  auto cur_state = state;
  cur_state.v = nominal_v;
  ASSIGN_OR_RETURN(const auto traj_pts,
                   DevelopConstVelocityPolePlacementTrajectory(
                       cur_state, dp, s_offset_plf, dt, horizon, obj_len,
                       max_steer, wheelbase,
                       /*is_reverse_driving=*/false));
  if (traj_pts.size() <= 1) {
    return absl::NotFoundError(
        "Generated path should contain at least two points.");
  }
  return PredictedTrajectoryPointsToDiscretizedPath(traj_pts);
}

std::vector<PredictedTrajectoryPoint> TrackTrajectoryByPolePlacement(
    absl::Span<const PredictedTrajectoryPoint> ref_pts,
    const BicycleModelState& state, double obj_len, double max_steer,
    double wheelbase, double dt, double horizon) {
  const int num_pts = std::min(static_cast<int>(horizon / dt) + 1,
                               static_cast<int>(ref_pts.size()));
  std::vector<Vec2d> ref_coords;
  ref_coords.reserve(num_pts);
  std::vector<double> ref_ks;
  ref_ks.reserve(num_pts);
  std::vector<double> ref_headings;
  ref_headings.reserve(num_pts);
  std::vector<double> ref_vs;
  ref_vs.reserve(num_pts);
  // If object is moving at low speed, the coords of the reference points are
  // the future rear axis center. If object is moving at high speed, then the
  // coords of the reference points are the future center points.
  const double ref_ratio = kReferencePointMoveRatio(state.v);
  for (size_t i = 0; i < ref_pts.size(); ++i) {
    ref_coords.push_back(
        ref_pts[i].pos() -
        ref_ratio * 0.5 * Vec2d::UnitFromAngle(ref_pts[i].theta()) * wheelbase);
    ref_ks.push_back(ref_pts[i].kappa());
    ref_headings.push_back(ref_pts[i].theta());
    ref_vs.push_back(ref_pts[i].v());
  }
  return DevelopPolePlacementTrajectory(
      ref_coords, ref_ks, ref_headings, ref_vs, state, dt, obj_len, max_steer,
      wheelbase, PolePlacementFeedForwardType::HEADING_DIFF);
}

}  // namespace prediction
}  // namespace qcraft

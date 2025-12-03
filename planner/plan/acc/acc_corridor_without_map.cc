// IWYU pragma: no_include <ext/alloc_traits.h>
//              for __alloc_traits<>::value_type
#include "onboard/planner/plan/acc/acc_corridor_without_map.h"

#include <algorithm>
#include <cmath>
#include <ostream>
#include <utility>
#include <vector>

#include "absl/status/status.h"
#include "absl/types/span.h"

#include "onboard/control/steering_converter.h"
#include "onboard/global/trace.h"
#include "onboard/math/frenet_frame.h"
#include "onboard/math/geometry/proto/affine_transformation.pb.h"
#include "onboard/math/piecewise_linear_function.h"
#include "onboard/math/util.h"
#include "onboard/math/vec.h"
#include "onboard/planner/common/discretized_path.h"
#include "onboard/planner/common/path_sl_boundary.h"
#include "onboard/planner/plan/acc/acc_corridor.h"
#include "onboard/planner/plan/acc/acc_corridor_util.h"
#include "onboard/planner/plan/acc/acc_defs.h"
#include "onboard/planner/planner_defs.h"

namespace qcraft {
namespace planner {
namespace {
constexpr double kMaxAllowedAccKappa = 0.2;  // m^-1.
constexpr double kKappaEpsilon = 1e-5;
constexpr double kCredibleCorridorRadian = 0.33 * M_PI;  // ~60°.
constexpr double kKappaCredibleMinSpeed = Kph2Mps(20.0);
constexpr double kMinPathLength = 50.0;         // m.
constexpr double kExpandedBoundaryMinS = 9.0;   // m.
constexpr double kExpandedBoundaryMaxS = 22.0;  // m.

const PiecewiseLinearFunction<double, double> kVelocityKphKappaDeadZonePLF(
    std::vector<double>{30.0, 50.0},
    std::vector<double>{0.005, 0.002});  // 0.005 ~= 1° of front wheel angle.

const PiecewiseLinearFunction<double, double> kVelocityKphCredibleLengthPLF(
    std::vector<double>{0.0, 36.0, 72.0, 100.0, 145.0},
    std::vector<double>{10.0, 50.0, 100.0, 140.0, 150.0});

const PiecewiseLinearFunction<double, double> kInnerBoundaryHalfWidthPlf(
    std::vector<double> /*s_ratio=*/{0.5, 0.75, 1.0},
    std::vector<double> /*inner_boundary_half_width=*/{1.2, 1.0, 0.8});
const PiecewiseLinearFunction<double, double> kOuterBoundaryHalfWidthPlf(
    std::vector<double> /*s_ratio=*/{0.5, 0.75, 1.0},
    std::vector<double> /*outer_boundary_half_width=*/{1.5, 1.3, 1.1});
const PiecewiseLinearFunction<double, double> kExpandedBoundaryHalfWidthPlf(
    std::vector<double> /*steering angle(degree)=*/{20.0, 30.0, 100.0},
    std::vector<double> /*expanded_boundary_half_width=*/{1.5, 2.2, 2.8});
}  // namespace

absl::StatusOr<AccCorridor> BuildAccCorridorFromPlanStartPoint(
    const PoseProto& pose, const ApolloTrajectoryPointProto& plan_start_point,
    const std::optional<double>& steering_percentage,
    const VehicleGeometryParamsProto& vehicle_geom_params,
    const VehicleDriveParamsProto& vehicle_drive_params, double corridor_step_s,
    double total_preview_time, double av_kappa_cache_average) {
  FUNC_QTRACE();
  // In low speed, use steering wheel angle and in high speed, use yaw rate
  // which is more stable when speed up, kappa = yaw_rate / speed.
  double av_kappa = pose.curvature();
  control::SteeringConverter steering_converter(vehicle_geom_params,
                                                vehicle_drive_params);
  if (pose.vel_body().x() < kKappaCredibleMinSpeed &&
      steering_percentage.has_value()) {
    av_kappa = steering_converter.SteerPctToKappa(*steering_percentage);
  }
  // Add kappa dead zone to avoid corridor swing left and right.
  const double kappa_deadzone =
      kVelocityKphKappaDeadZonePLF(Mps2Kph(pose.vel_body().x()));
  av_kappa = std::fabs(av_kappa) < kappa_deadzone &&
                     std::fabs(av_kappa_cache_average) < 0.5 * kappa_deadzone
                 ? 0.0
                 : av_kappa;
  av_kappa = std::clamp(av_kappa, -kMaxAllowedAccKappa, kMaxAllowedAccKappa);
  double credible_length =
      kVelocityKphCredibleLengthPLF(Mps2Kph(pose.vel_body().x()));
  credible_length = std::min(
      kCredibleCorridorRadian * 1.0 / (std::fabs(av_kappa) + kKappaEpsilon),
      credible_length);
  const int n = static_cast<int>(credible_length / corridor_step_s) + 1;

  std::vector<double> s_vec, ref_center_l, right_l, left_l, target_right_l,
      target_left_l;
  s_vec.reserve(n);
  ref_center_l.reserve(n);
  right_l.reserve(n);
  left_l.reserve(n);
  target_right_l.reserve(n);
  target_left_l.reserve(n);
  std::vector<Vec2d> inner_right_xy, inner_left_xy, outer_right_xy,
      outer_left_xy, ref_center_xy;
  inner_right_xy.reserve(n);
  inner_left_xy.reserve(n);
  outer_right_xy.reserve(n);
  outer_left_xy.reserve(n);
  ref_center_xy.reserve(n);
  UniCycleState cur_state{
      .x = plan_start_point.path_point().x(),
      .y = plan_start_point.path_point().y(),
      .heading = plan_start_point.path_point().theta(),
      .kappa = av_kappa,
  };

  const double init_heading = cur_state.heading;
  const double front_edge_to_center =
      vehicle_geom_params.front_edge_to_center();
  const double av_half_width = 0.5 * vehicle_geom_params.width();
  const PiecewiseLinearFunction<double, double>
      near_av_inner_boundary_half_width_plf(
          std::vector<double>{0.0, front_edge_to_center},
          std::vector<double>{av_half_width, 2.5});
  const PiecewiseLinearFunction<double, double>
      near_av_outer_boundary_half_width_plf(
          std::vector<double>{0.0, front_edge_to_center},
          std::vector<double>{av_half_width, 3.0});
  for (int i = 0; i < n; ++i) {
    const double cur_s = i * corridor_step_s;
    double inner_boundary_half_width =
        kInnerBoundaryHalfWidthPlf(cur_s / credible_length);
    double outer_boundary_half_width =
        kOuterBoundaryHalfWidthPlf(cur_s / credible_length);
    const double steering_angle =
        r2d(steering_converter.SteerPctToSteerAngle(*steering_percentage));
    if (cur_s < kExpandedBoundaryMinS) {
      inner_boundary_half_width = near_av_inner_boundary_half_width_plf(cur_s);
      outer_boundary_half_width = near_av_outer_boundary_half_width_plf(cur_s);
    } else if (cur_s > kExpandedBoundaryMinS && cur_s < kExpandedBoundaryMaxS &&
               std::fabs(steering_angle) >
                   kExpandedBoundaryHalfWidthPlf.x().front()) {
      inner_boundary_half_width = kExpandedBoundaryHalfWidthPlf(steering_angle);
      outer_boundary_half_width = kExpandedBoundaryHalfWidthPlf(steering_angle);
    }
    s_vec.push_back(cur_s);
    ref_center_l.push_back(0.0);
    right_l.push_back(-outer_boundary_half_width);
    left_l.push_back(outer_boundary_half_width);
    target_right_l.push_back(-inner_boundary_half_width);
    target_left_l.push_back(inner_boundary_half_width);

    Vec2d pos(cur_state.x, cur_state.y);
    const auto tangent = Vec2d::FastUnitFromAngle(cur_state.heading);
    inner_left_xy.emplace_back(
        LatPoint(pos, tangent, inner_boundary_half_width));
    inner_right_xy.emplace_back(
        LatPoint(pos, tangent, -inner_boundary_half_width));
    outer_left_xy.emplace_back(
        LatPoint(pos, tangent, outer_boundary_half_width));
    outer_right_xy.emplace_back(
        LatPoint(pos, tangent, -outer_boundary_half_width));
    ref_center_xy.push_back(std::move(pos));

    cur_state = SimulateUniCycleStateWithHeadingRestrict(
        cur_state, corridor_step_s, /*kappa_decay=*/1.0,
        /*init_heading=*/init_heading, /*max_delta_heading=*/M_PI_2);
  }

  // Extent path points to extended length.
  const double extend_length =
      std::max(pose.vel_body().x() *
                   (total_preview_time + kCorridorExtendLengthTimeHorizon),
               kMinPathLength);
  const auto heading_vec = Vec2d::UnitFromAngle(cur_state.heading);
  auto path = ComputeExtendedSmoothPath(
      absl::MakeSpan(ref_center_xy), absl::MakeSpan(s_vec), heading_vec,
      corridor_step_s, kPathSampleInterval, extend_length);

  auto frenet_frame_or = BuildQtfmEnhancedKdTreeFrenetFrame(
      ref_center_xy, /*down_sample_raw_points=*/true);
  if (!frenet_frame_or.ok()) {
    return absl::NotFoundError("Frenet frame construction failed");
  }
  auto path_boundary = PathSlBoundary(
      std::move(s_vec), std::move(ref_center_l), std::move(right_l),
      std::move(left_l), std::move(target_right_l), std::move(target_left_l),
      std::move(ref_center_xy), std::move(outer_right_xy),
      std::move(outer_left_xy), std::move(inner_right_xy),
      std::move(inner_left_xy));
  PiecewiseLinearFunction<double> kappa_s(
      std::vector<double>{0.0, path.length()},
      std::vector<double>{av_kappa, av_kappa});
  return AccCorridor{.boundary = std::move(path_boundary),
                     .frenet_frame = std::move(frenet_frame_or).value(),
                     .path = std::move(path),
                     .credible_length = credible_length,
                     .kappa_s = std::move(kappa_s)};
}

}  // namespace planner
}  // namespace qcraft

#include "onboard/planner/decision/beyond_length_along_route.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <utility>

#include "absl/status/status.h"

#include "onboard/global/buffered_logger.h"
#include "onboard/lite/check.h"
#include "onboard/math/proto/piecewise_linear_function.pb.h"
#include "onboard/planner/planner_defs.h"
#include "onboard/utils/status_macros.h"

namespace qcraft::planner {

namespace {
constexpr double kMaxLCSpeedOnBorrow = 15.0 / 3.6;  // m/s.
constexpr double kComfortableDeceletation = -1.6;   // m/s^2.
constexpr double kSpeedProfileTimeInterval = 1.0;   // seconds.
constexpr int kSpeedProfileSteps = static_cast<int>(kTrajectoryTimeHorizon) + 1;
constexpr double kEpsilon = 1e-3;

PiecewiseLinearFunctionDoubleProto CreateConstantSpeedProfile(
    double speed_limit, double start_time = 0.0 /*default start time*/) {
  PiecewiseLinearFunctionDoubleProto speed_profile;
  speed_profile.mutable_x()->Reserve(kSpeedProfileSteps);
  speed_profile.mutable_y()->Reserve(kSpeedProfileSteps);

  for (double t = 0.0; t < kTrajectoryTimeHorizon;
       t += kSpeedProfileTimeInterval) {
    speed_profile.add_x(t);
    if (t < start_time) {
      speed_profile.add_y(std::numeric_limits<double>::max());
    } else {
      speed_profile.add_y(speed_limit);
    }
  }

  return speed_profile;
}

PiecewiseLinearFunctionDoubleProto CreateDeceleratedSpeedProfile(
    double speed_limit, double v_begin, double dec,
    double start_time = 0.0 /*default start time*/) {
  PiecewiseLinearFunctionDoubleProto speed_profile;
  speed_profile.mutable_x()->Reserve(kSpeedProfileSteps);
  speed_profile.mutable_y()->Reserve(kSpeedProfileSteps);

  for (double t = 0.0; t < kTrajectoryTimeHorizon;
       t += kSpeedProfileTimeInterval) {
    speed_profile.add_x(t);
    if (t < start_time) {
      speed_profile.add_y(std::numeric_limits<double>::max());
    } else {
      speed_profile.add_y(
          std::max(speed_limit, v_begin + dec * (t - start_time)));
    }
  }
  return speed_profile;
}

PiecewiseLinearFunctionDoubleProto GetUpperSpeedProfile(
    const PiecewiseLinearFunctionDoubleProto& sp1,
    const PiecewiseLinearFunctionDoubleProto& sp2) {
  QCHECK_EQ(sp1.x_size(), sp1.y_size());
  QCHECK_EQ(sp1.x_size(), sp2.x_size());
  QCHECK_EQ(sp1.y_size(), sp2.y_size());

  const auto size = sp1.x_size();
  PiecewiseLinearFunctionDoubleProto new_speed_profile;
  new_speed_profile.mutable_x()->Reserve(size);
  new_speed_profile.mutable_y()->Reserve(size);

  for (int i = 0; i < size; ++i) {
    QCHECK_EQ(sp1.x(i), sp2.x(i));
    new_speed_profile.add_x(sp1.x(i));
    new_speed_profile.add_y(std::max(sp1.y(i), sp2.y(i)));
  }
  return new_speed_profile;
}

double CalculateDeceleratedDistance(double v_begin, double v_end, double dec) {
  QCHECK_GT(v_begin, v_end);
  QCHECK_LT(dec, -kEpsilon);
  return 0.5 * (v_end * v_end - v_begin * v_begin) / dec;
}

double CalculateVBegin(double dist, double v_end, double dec) {
  QCHECK_GT(dist, kEpsilon);
  QCHECK_LT(dec, -kEpsilon);
  return std::sqrt(v_end * v_end - 2 * dec * dist);
}

absl::StatusOr<PiecewiseLinearFunctionDoubleProto> FindSpeedProfile(
    double length_along_route_on_dp, double ego_v, double speed_limit,
    double max_deceleration) {
  ego_v = std::max(ego_v, kEpsilon);

  if (length_along_route_on_dp <= kEpsilon) {
    return GetUpperSpeedProfile(
        CreateConstantSpeedProfile(speed_limit),
        CreateDeceleratedSpeedProfile(speed_limit, ego_v, max_deceleration));
  }

  if (ego_v < speed_limit + kEpsilon) {
    return CreateConstantSpeedProfile(speed_limit,
                                      length_along_route_on_dp / ego_v);
  }

  const double decelerated_motion_dist = CalculateDeceleratedDistance(
      ego_v, speed_limit, kComfortableDeceletation);

  if (decelerated_motion_dist < length_along_route_on_dp) {
    /*
    There is enough distance to decelerate comfortabely, so av may speed up or
    move at constant speed before decelerate. VT - speed profile like below,
    "xxx" means max value.

    xxxxxxxxxxx
               \
                \
                 \
                  \___________
    */
    const double uniform_motion_duration =
        (length_along_route_on_dp - decelerated_motion_dist) / ego_v;
    return CreateDeceleratedSpeedProfile(
        speed_limit, ego_v, kComfortableDeceletation, uniform_motion_duration);
  } else {
    /*
    There is no enough distance to decelerate comfortabely. VT - speed profile
    like below.

     \
      \
       \
        \___________
    */
    const double v_begin_limit = CalculateVBegin(
        length_along_route_on_dp, speed_limit, kComfortableDeceletation);
    return GetUpperSpeedProfile(
        CreateDeceleratedSpeedProfile(speed_limit, v_begin_limit,
                                      kComfortableDeceletation),
        CreateDeceleratedSpeedProfile(speed_limit, ego_v, max_deceleration));
  }

  return absl::NotFoundError("no feasible solution");
}

}  // namespace

absl::StatusOr<ConstraintProto::SpeedProfileProto>
BuildBeyondLengthAlongRouteConstraint(
    const DrivePassage& dp,
    const MotionConstraintParamsProto& motion_constraint_params,
    double length_along_route, bool borrow_boundary, double ego_v) {
  if (length_along_route == std::numeric_limits<double>::max()) {
    return absl::NotFoundError("Length along route not available.");
  }

  const double length_along_route_on_dp =
      length_along_route + dp.lane_path_start_s();

  constexpr double kReachDestinationThreshold = 1.0;  // m.
  if (length_along_route_on_dp > dp.end_s() - kReachDestinationThreshold) {
    return absl::NotFoundError("Target lane path reaches destination.");
  }

  const double speed_limit =
      borrow_boundary ? kMaxLCSpeedOnBorrow : kMinLCSpeed;

  ASSIGN_OR_RETURN(
      auto speed_profile,
      FindSpeedProfile(length_along_route_on_dp, ego_v, speed_limit,
                       motion_constraint_params.max_deceleration()));

  ConstraintProto::SpeedProfileProto proto;
  *proto.mutable_vt_upper_constraint() = std::move(speed_profile);
  const std::string id = "beyond_length_along_route";
  proto.mutable_source()->mutable_beyond_length_along_route()->set_id(id);

  return proto;
}

}  // namespace qcraft::planner

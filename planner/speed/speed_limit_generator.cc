#include "onboard/planner/speed/speed_limit_generator.h"

#include <algorithm>
#include <cmath>
#include <deque>
#include <functional>
#include <limits>
#include <map>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "absl/status/statusor.h"
#include "absl/strings/str_format.h"
#include "absl/types/span.h"

#include "onboard/global/buffered_logger.h"
#include "onboard/global/trace.h"
#include "onboard/lite/check.h"
#include "onboard/math/frenet_common.h"
#include "onboard/math/frenet_frame.h"
#include "onboard/math/geometry/util.h"
#include "onboard/math/piecewise_linear_function.h"
#include "onboard/math/util.h"
#include "onboard/math/vec.h"
#include "onboard/planner/decision/constraint_manager.h"
#include "onboard/planner/decision/proto/constraint.pb.h"
#include "onboard/planner/planner_defs.h"
#include "onboard/planner/speed/proto/speed_finder_params.pb.h"
#include "onboard/planner/util/path_util.h"
#include "onboard/planner/util/vehicle_geometry_util.h"
#include "onboard/utils/status_macros.h"

namespace qcraft::planner {
namespace {
using SpeedLimitProto = SpeedFinderParamsProto::SpeedLimitParamsProto;
using SpeedLimitRange = SpeedLimit::SpeedLimitRange;
// Speed limit diff less than this value will be ignored.
constexpr double kApproxSpeedLimitEps = 1.0;

constexpr double kEps = 0.001;

std::vector<double> GetMaxValuesNearbyPathPoint(
    const DiscretizedPath& discretized_points, double forward_radius,
    double backward_radius,
    const std::function<double(const PathPoint&)>& get_value) {
  const auto num_of_points = discretized_points.size();
  // Deque element: [s, v].
  std::deque<std::pair<double, double>> dq;
  std::vector<double> res;
  res.reserve(num_of_points);
  int k = 0;
  for (int i = 0; i < num_of_points; ++i) {
    const auto& path_point = discretized_points[i];
    const double start_s = path_point.s() - backward_radius;
    const double end_s = path_point.s() + forward_radius;
    while (!dq.empty() && dq.front().first < start_s) {
      dq.pop_front();
    }
    while (k < num_of_points && discretized_points[k].s() <= end_s) {
      const double value = get_value(discretized_points[k]);
      while (!dq.empty() && dq.back().second < value) {
        dq.pop_back();
      }
      dq.emplace_back(discretized_points[k++].s(), value);
    }
    res.push_back(dq.front().second);
  }
  return res;
}

SpeedLimit GenerateCurvatureSpeedLimit(
    const DiscretizedPath& path_points,
    const VehicleDriveParamsProto& veh_drive_params,
    const VehicleGeometryParamsProto& veh_geo_params, double max_speed_limit,
    double av_speed, const SpeedLimitProto& speed_limit_config) {
  FUNC_QTRACE();
  const std::function<double(const PathPoint&)> get_kappa =
      [](const PathPoint& pt) { return std::fabs(pt.kappa()); };

  const double max_allowed_kappa =
      ComputeCenterMaxCurvature(veh_geo_params, veh_drive_params);
  const double av_speed_sqr = Sqr(av_speed);

  // The new curvature speed limit equation is
  // \frac{a}{\kappa^{b} + c} + d.
  // The derivative of lat acc is
  // -\frac{2ab\kappa^{b}(a+d(\kappa^{b}+c))}{(\kappa^{b}+c)^{3}}+(\frac{a}{\kappa^{b}+c})^{2}
  // Use the tangent at kSmallKappaThres to calc speed limit for the points
  // whose kappa is less than kSmallKappaThres to avoid hard brake.
  constexpr double kSmallKappaThres = 0.003;  // m^-1.
  const double a = speed_limit_config.curvature_numerator();
  const double b = speed_limit_config.curvature_power();
  const double c = speed_limit_config.curvature_bias1();
  const double d = speed_limit_config.curvature_bias2();
  const double k_power_b = std::pow(kSmallKappaThres, b);
  const double k_power_b_plus_c = k_power_b + c;
  const double small_kappa_speed_limit_sqr = Sqr(a / k_power_b_plus_c + d);
  const double small_kappa_lat_acc_derivative =
      std::max(0.0, -(2.0 * a * b * k_power_b * (a + d * k_power_b_plus_c)) /
                            Cube(k_power_b_plus_c) +
                        small_kappa_speed_limit_sqr);
  const double intercept =
      std::max(0.0, -kSmallKappaThres * small_kappa_lat_acc_derivative +
                        small_kappa_speed_limit_sqr * kSmallKappaThres);
  const auto get_speed_limit = [&speed_limit_config, max_allowed_kappa,
                                max_speed_limit, small_kappa_lat_acc_derivative,
                                intercept,
                                av_speed_sqr](double max_kappa, double s) {
    double speed_limit = 0.0;
    if (speed_limit_config.enable_new_curvature_speed_limit()) {
      if (max_kappa >= kSmallKappaThres) {
        speed_limit =
            speed_limit_config.curvature_numerator() /
                (std::pow(max_kappa, speed_limit_config.curvature_power()) +
                 speed_limit_config.curvature_bias1()) +
            speed_limit_config.curvature_bias2();
      } else {
        const double lat_acc =
            max_kappa * small_kappa_lat_acc_derivative + intercept;
        constexpr double kSmallEps = 1e-6;
        speed_limit = std::sqrt(lat_acc / (max_kappa + kSmallEps));
      }
    } else {
      speed_limit =
          speed_limit_config.curvature_bias() +
          1.0 / (max_kappa + kEps) * speed_limit_config.curvature_gain();
    }

    if (max_kappa > max_allowed_kappa) {
      speed_limit = std::min(
          speed_limit, speed_limit_config.speed_limit_for_large_curvature());
    }
    constexpr double kComfortableBrakeAcc = -1.2;  // m/s^2.
    if (const double comfortable_brake_speed_sqr =
            av_speed_sqr + 2.0 * kComfortableBrakeAcc * s;
        comfortable_brake_speed_sqr > Sqr(speed_limit)) {
      speed_limit = std::sqrt(comfortable_brake_speed_sqr);
    }
    return std::min(speed_limit, max_speed_limit);
  };

  std::vector<SpeedLimitRange> speed_limit_ranges;
  speed_limit_ranges.reserve(path_points.size());
  const double radius = speed_limit_config.max_curvature_consider_radius();
  const auto max_kappas =
      GetMaxValuesNearbyPathPoint(path_points, radius, radius, get_kappa);
  const double init_limit = get_speed_limit(max_kappas[0], path_points[0].s());
  std::pair<double, double> prev_speed_limit_point =
      std::make_pair(path_points[0].s(), init_limit);
  for (int i = 1; i < path_points.size(); ++i) {
    const double curr_speed_limit =
        get_speed_limit(max_kappas[i], path_points[i].s());
    if (std::fabs(curr_speed_limit - prev_speed_limit_point.second) >
            kApproxSpeedLimitEps ||
        i == path_points.size() - 1) {
      speed_limit_ranges.push_back(
          {.start_s = prev_speed_limit_point.first,
           .end_s = path_points[i].s(),
           .speed_limit = prev_speed_limit_point.second,
           .info = ""});
      prev_speed_limit_point =
          std::make_pair(path_points[i].s(), curr_speed_limit);
    }
  }
  QCHECK(!speed_limit_ranges.empty());
  return SpeedLimit(speed_limit_ranges);
}

SpeedLimit GenerateSteerRateSpeedLimit(
    const std::vector<PathPoint>& path_points,
    const VehicleDriveParamsProto& veh_drive_params,
    const VehicleGeometryParamsProto& veh_geo_params, double max_speed_limit,
    const SpeedLimitProto& speed_limit_config) {
  const double wheel_base = veh_geo_params.wheel_base();
  QCHECK_GT(wheel_base, 0.0);
  QCHECK(!path_points.empty());

  const std::function<double(const PathPoint&)> get_front_wheel_omega_by_v =
      [&wheel_base](const PathPoint& p) {
        const auto lambda = std::fabs(p.lambda());
        const auto kappa = p.kappa();
        constexpr double kKappaThreshold = 0.05;  // for 20m radius.
        if (std::fabs(kappa) < kKappaThreshold) return 0.0;
        // tan(delta) = l * kappa;
        // d(delta)/dt = l / (1 + sqr(l * kappa)) * d(kappa) /ds * v;
        return wheel_base * lambda / (1.0 + Sqr(wheel_base * kappa));
      };

  const auto get_speed_limit = [&speed_limit_config, &veh_drive_params,
                                &max_speed_limit](
                                   double max_front_wheel_omega_by_v) {
    double speed_limit =
        speed_limit_config.max_steer_rate() /
        (veh_drive_params.steer_ratio() * max_front_wheel_omega_by_v + kEps);
    constexpr double kLowSpeedThreshold = 1.0;  // m/s.
    if (speed_limit < kLowSpeedThreshold) {
      speed_limit = speed_limit_config.min_steer_rate_speed_limit();
    }
    return std::min(speed_limit, max_speed_limit);
  };

  const DiscretizedPath discretized_path_points(path_points);
  std::vector<PathPoint> resampled_points;
  resampled_points.reserve(
      CeilToInt(discretized_path_points.length() / kPathSampleInterval) +
      discretized_path_points.size());
  auto iter = discretized_path_points.begin();
  double prev_s = iter->s();
  resampled_points.push_back(*iter);
  while (iter != discretized_path_points.end()) {
    const auto next_iter = iter + 1;
    if (next_iter == discretized_path_points.end()) {
      break;
    }
    // The comparison shall be as the same logic as the operation in the "else"
    // statement because a <= b + c may be not equal to a - b <= c due to
    // numeric error.
    if (next_iter->s() <= prev_s + kPathSampleInterval) {
      resampled_points.push_back(*next_iter);
      prev_s = next_iter->s();
      ++iter;
    } else {
      prev_s += kPathSampleInterval;
      resampled_points.push_back(discretized_path_points.Evaluate(prev_s));
    }
  }

  // Remove outlying lambda.
  constexpr double kOutlierThreshold = 5.0;  // times.
  for (int i = 1; i < resampled_points.size() - 1; ++i) {
    const double cur_lambda = resampled_points[i].lambda();
    const double prev_lambda = resampled_points[i - 1].lambda();
    const double next_lambda = resampled_points[i + 1].lambda();
    const double avg_lambda = 0.5 * (prev_lambda + next_lambda);
    if (resampled_points[i + 1].s() - resampled_points[i - 1].s() <
            kPathSampleInterval &&
        prev_lambda * next_lambda > -kEps &&
        std::fabs(cur_lambda) > kOutlierThreshold * std::fabs(avg_lambda)) {
      resampled_points[i].set_lambda(avg_lambda);
    }
  }

  std::vector<SpeedLimitRange> speed_limit_ranges;
  speed_limit_ranges.reserve(resampled_points.size());
  const auto front_wheel_omega_by_vs = GetMaxValuesNearbyPathPoint(
      DiscretizedPath(resampled_points),
      speed_limit_config.min_steer_rate_forward_consider_radius(),
      speed_limit_config.min_steer_rate_backward_consider_radius(),
      get_front_wheel_omega_by_v);
  const double init_limit = get_speed_limit(front_wheel_omega_by_vs[0]);
  std::pair<double, double> prev_speed_limit_point =
      std::make_pair(resampled_points[0].s(), init_limit);
  for (int i = 1; i < resampled_points.size(); ++i) {
    const double curr_speed_limit = get_speed_limit(front_wheel_omega_by_vs[i]);
    if (std::fabs(curr_speed_limit - prev_speed_limit_point.second) >
            kApproxSpeedLimitEps ||
        i == resampled_points.size() - 1) {
      speed_limit_ranges.push_back(
          {.start_s = prev_speed_limit_point.first,
           .end_s = resampled_points[i].s(),
           .speed_limit = prev_speed_limit_point.second,
           .info = ""});
      prev_speed_limit_point =
          std::make_pair(resampled_points[i].s(), curr_speed_limit);
    }
  }
  QCHECK(!speed_limit_ranges.empty());
  return SpeedLimit(speed_limit_ranges);
}

SpeedLimit GenerateLaneSpeedLimit(const DiscretizedPath& path_points,
                                  double max_speed_limit, double av_speed,
                                  const DrivePassage& drive_passage) {
  FUNC_QTRACE();
  constexpr double kExceedLimitThreshold = 1.0;  // m/s.
  const double av_speed_sqr = Sqr(av_speed - kExceedLimitThreshold);
  const auto get_speed_limit = [&drive_passage, max_speed_limit,
                                av_speed_sqr](const PathPoint& path_point) {
    constexpr double kComfortableBrakeAcc = -0.7;  // m/s^2
    const auto speed_limit =
        drive_passage.QuerySpeedLimitAt(ToVec2d(path_point));
    if (!speed_limit.ok()) {
      return max_speed_limit;
    } else if (const double comfortable_brake_speed_sqr =
                   av_speed_sqr + 2.0 * kComfortableBrakeAcc * path_point.s();
               comfortable_brake_speed_sqr > Sqr(*speed_limit)) {
      return std::sqrt(comfortable_brake_speed_sqr);
    } else {
      return *speed_limit;
    }
  };

  std::vector<SpeedLimitRange> speed_limit_ranges;
  const int num_points = path_points.size();
  speed_limit_ranges.reserve(num_points);
  // first: s second: v
  std::pair<double, double> prev_speed_limit_point =
      std::make_pair(path_points[0].s(), get_speed_limit(path_points[0]));
  double last_sample_s = 0.0;
  for (int i = 1; i < num_points; ++i) {
    // Only check the lane speed limit at every meter to save computation.
    constexpr double kSpeedLimitSampleRange = 1.0;  // Meters.
    if (path_points[i].s() - last_sample_s > kSpeedLimitSampleRange ||
        i == num_points - 1) {
      last_sample_s = path_points[i].s();
      if (const double curr_speed_limit = get_speed_limit(path_points[i]);
          std::fabs(curr_speed_limit - prev_speed_limit_point.second) >
              kApproxSpeedLimitEps ||
          i == num_points - 1) {
        QCHECK_GT(path_points[i].s(), prev_speed_limit_point.first);
        double end_s = path_points[i].s();
        if (i != num_points - 1 &&
            curr_speed_limit >
                prev_speed_limit_point.second + kApproxSpeedLimitEps) {
          constexpr double kAccPreviewTime = 6.0;
          const double acc_preview_s = kAccPreviewTime * av_speed;
          end_s = std::max(end_s - acc_preview_s, prev_speed_limit_point.first);
        }
        if (end_s > prev_speed_limit_point.first) {
          speed_limit_ranges.push_back(
              {.start_s = prev_speed_limit_point.first,
               .end_s = end_s,
               .speed_limit = prev_speed_limit_point.second,
               .info = ""});
        }
        prev_speed_limit_point = std::make_pair(end_s, curr_speed_limit);
      }
    }
  }
  QCHECK(!speed_limit_ranges.empty());
  return SpeedLimit(speed_limit_ranges);
}

std::optional<SpeedLimit> GenerateCloseCurbSpeedLimit(
    const DiscretizedPath& path_points, double max_speed_limit,
    const std::vector<DistanceInfo>&
        distance_info_to_impassable_path_boundaries,
    const PiecewiseLinearFunction<double, double>&
        hard_curb_clearance_rel_speed_plf) {
  struct SpeedLimitPoint {
    double s = 0.0;
    double v = 0.0;
    std::string info;
  };
  const auto get_speed_limit_point =
      [&path_points, &distance_info_to_impassable_path_boundaries,
       &hard_curb_clearance_rel_speed_plf,
       max_speed_limit](int index) -> SpeedLimitPoint {
    const double max_hard_curb_clearance =
        hard_curb_clearance_rel_speed_plf.x().back();

    const auto& distance_info =
        distance_info_to_impassable_path_boundaries[index];
    if (distance_info.dist > max_hard_curb_clearance) {
      return {.s = path_points[index].s(), .v = max_speed_limit, .info = ""};
    }
    return {.s = path_points[index].s(),
            .v = std::min(hard_curb_clearance_rel_speed_plf(distance_info.dist),
                          max_speed_limit),
            .info = distance_info.info};
  };

  const auto path_size_before_collision =
      distance_info_to_impassable_path_boundaries.size();
  std::vector<SpeedLimitRange> speed_limit_ranges;
  speed_limit_ranges.reserve(path_size_before_collision);
  // first: s second: v
  auto prev_speed_limit_point = get_speed_limit_point(0);
  for (int i = 1; i < path_size_before_collision; ++i) {
    auto curr_speed_limit_point = get_speed_limit_point(i);
    if (std::fabs(curr_speed_limit_point.v - prev_speed_limit_point.v) >
            kApproxSpeedLimitEps ||
        i == path_size_before_collision - 1) {
      QCHECK_GT(path_points[i].s(), prev_speed_limit_point.s);
      speed_limit_ranges.push_back({.start_s = prev_speed_limit_point.s,
                                    .end_s = path_points[i].s(),
                                    .speed_limit = prev_speed_limit_point.v,
                                    .info = prev_speed_limit_point.info});
      prev_speed_limit_point = curr_speed_limit_point;
    }
  }
  if (speed_limit_ranges.empty()) return std::nullopt;
  return SpeedLimit(speed_limit_ranges);
}

std::string TypeCaseToString(const SourceProto::TypeCase type) {
  switch (type) {
    case SourceProto::TypeCase::kCloseObject:
      return "CLOSE_OBJECT";
    case SourceProto::TypeCase::kSpeedBump:
      return "SPEED_BUMP";
    case SourceProto::TypeCase::kIntersection:
      return "INTERSECTION";
    case SourceProto::TypeCase::kLcEndOfCurrentLane:
      return "LC_END_OF_CURRENT_LANE";
    case SourceProto::TypeCase::kBeyondLengthAlongRoute:
      return "BEYOND_LENGTH_ALONE_ROUTE";
    case SourceProto::TypeCase::kPedestrianObject:
      return "PEDESTRIAN_OBJECT";
    case SourceProto::TypeCase::kCrosswalk:
      return "CROSSWALK";
    case SourceProto::TypeCase::kToll:
      return "TOLL";
    case SourceProto::TypeCase::kTrafficLight:
      return "TRAFFIC_LIGHT";
    case SourceProto::TypeCase::kNoBlock:
      return "NO_BLOCK";
    case SourceProto::TypeCase::kEndOfPathBoundary:
      return "END_OF_PATH_BOUNDARY";
    case SourceProto::TypeCase::kEndOfCurrentLanePath:
      return "END_OF_CURRENT_LANE_PATH";
    case SourceProto::TypeCase::kRouteDestination:
      return "ROUTE_DESTINATION";
    case SourceProto::TypeCase::kParkingBrakeRelease:
      return "PARKING_BRAKERELEASE";
    case SourceProto::TypeCase::kBlockingStaticObject:
      return "BLOCKING_STATIC_OBJECT";
    case SourceProto::TypeCase::kStandby:
      return "STANDBY";
    case SourceProto::TypeCase::kStopSign:
      return "STOP_SIGN";
    case SourceProto::TypeCase::kStandstill:
      return "STANDSTILL";
    case SourceProto::TypeCase::kPullOver:
      return "PULL_OVER";
    case SourceProto::TypeCase::kBrakeToStop:
      return "BRAKE_TO_STOP";
    case SourceProto::TypeCase::kEndOfLocalPath:
      return "END_OF_LOCAL_PATH";
    case SourceProto::TypeCase::kSolidLineWithinBoundary:
      return "SOLID_LINE_WITHIN_BOUNDARY";
    case SourceProto::TypeCase::kOccludedObject:
      return "OCCLUDED_OBJECT";
    case SourceProto::TypeCase::kDenseTrafficFlow:
      return "DENSE_TRAFFIC_FLOW";
    case SourceProto::TypeCase::kStopPolyline:
      return "STOP_POLYLINE";
    case SourceProto::TypeCase::TYPE_NOT_SET:
      return "TYPE_NOT_SET";
  }
}

std::optional<SpeedLimit> GenerateExternalSpeedLimit(
    const DiscretizedPath& path_points, const ConstraintManager& constraint_mgr,
    const VehicleGeometryParamsProto& veh_geo_params, double max_speed_limit) {
  std::vector<Vec2d> points;
  points.reserve(path_points.size());
  for (const auto& path_point : path_points) {
    points.emplace_back(path_point.x(), path_point.y());
  }
  ASSIGN_OR_DIE(
      const auto frenet_path,
      BuildBruteForceFrenetFrame(points, /*down_sample_raw_points=*/true));
  constexpr double kSpeedRegionBuffer = 0.5;
  std::vector<SpeedLimitRange> speed_limit_ranges;
  speed_limit_ranges.reserve(constraint_mgr.SpeedRegion().size() +
                             constraint_mgr.PathSpeedRegion().size());

  // Add drive passage speed regions.
  for (const ConstraintProto::SpeedRegionProto& speed_region :
       constraint_mgr.SpeedRegion()) {
    const auto type_case = speed_region.source().type_case();
    // Only consider upper bound speed limit.
    if (type_case == SourceProto::TypeCase::kNoBlock) continue;
    const double start_s =
        frenet_path.XYToSL(Vec2dFromProto(speed_region.start_point())).s -
        veh_geo_params.front_edge_to_center() - kSpeedRegionBuffer;
    const double end_s =
        frenet_path.XYToSL(Vec2dFromProto(speed_region.end_point())).s +
        veh_geo_params.back_edge_to_center() + kSpeedRegionBuffer;
    if (start_s >= end_s) continue;
    speed_limit_ranges.push_back(
        {.start_s = start_s,
         .end_s = end_s,
         .speed_limit = std::min(speed_region.max_speed(), max_speed_limit),
         .info = TypeCaseToString(speed_region.source().type_case())});
  }

  // Add path speed regions.
  for (const ConstraintProto::PathSpeedRegionProto& path_speed_region :
       constraint_mgr.PathSpeedRegion()) {
    const double start_s = path_speed_region.start_s() -
                           veh_geo_params.front_edge_to_center() -
                           kSpeedRegionBuffer;
    const double end_s = path_speed_region.end_s() +
                         veh_geo_params.back_edge_to_center() +
                         kSpeedRegionBuffer;
    if (start_s >= end_s) continue;
    speed_limit_ranges.push_back(
        {.start_s = start_s,
         .end_s = end_s,
         .speed_limit =
             std::min(path_speed_region.max_speed(), max_speed_limit),
         .info = absl::StrFormat(
             "%s Id: %s",
             TypeCaseToString(path_speed_region.source().type_case()),
             path_speed_region.id())});
  }
  if (speed_limit_ranges.empty()) return std::nullopt;
  return SpeedLimit(speed_limit_ranges);
}

SpeedLimit GenerateCombinationSpeedLimit(
    const std::map<SpeedLimitTypeProto::Type, SpeedLimit>& speed_limit_map,
    double /*max_speed_limit*/) {
  FUNC_QTRACE();
  std::vector<SpeedLimitRange> speed_limit_ranges;
  int cnt = 0;
  for (const auto& [_, speed_limit] : speed_limit_map) {
    cnt += speed_limit.speed_limit_ranges().size();
  }
  speed_limit_ranges.reserve(cnt);
  for (const auto& [_, speed_limit] : speed_limit_map) {
    for (const auto& range : speed_limit.speed_limit_ranges()) {
      speed_limit_ranges.push_back(range);
    }
  }
  QCHECK(!speed_limit_ranges.empty());
  return SpeedLimit(speed_limit_ranges);
}

}  // namespace

std::map<SpeedLimitTypeProto::Type, SpeedLimit> GetSpeedLimitMap(
    const DiscretizedPath& discretized_points,
    const std::vector<PathPoint>& st_path_points, double max_speed_limit,
    double av_speed, const VehicleGeometryParamsProto& veh_geo_params,
    const VehicleDriveParamsProto& veh_drive_params,
    const DrivePassage* drive_passage, const ConstraintManager& constraint_mgr,
    const SpeedLimitProto& speed_limit_config,
    const std::vector<DistanceInfo>&
        distance_info_to_impassable_path_boundaries) {
  SCOPED_QTRACE("GetSpeedLimitMap");

  std::map<SpeedLimitTypeProto::Type, SpeedLimit> speed_limit_map;
  if (drive_passage != nullptr) {
    SpeedLimit lane_speed_limit = GenerateLaneSpeedLimit(
        discretized_points, max_speed_limit, av_speed, *drive_passage);
    speed_limit_map.emplace(SpeedLimitTypeProto::LANE,
                            std::move(lane_speed_limit));
  }

  SpeedLimit curvature_speed_limit = GenerateCurvatureSpeedLimit(
      discretized_points, veh_drive_params, veh_geo_params, max_speed_limit,
      av_speed, speed_limit_config);
  speed_limit_map.emplace(SpeedLimitTypeProto::CURVATURE,
                          std::move(curvature_speed_limit));

  SpeedLimit steer_rate_speed_limit = GenerateSteerRateSpeedLimit(
      st_path_points, veh_drive_params, veh_geo_params, max_speed_limit,
      speed_limit_config);
  speed_limit_map.emplace(SpeedLimitTypeProto::STEER_RATE,
                          std::move(steer_rate_speed_limit));

  auto close_curb_speed_limit = GenerateCloseCurbSpeedLimit(
      discretized_points, max_speed_limit,
      distance_info_to_impassable_path_boundaries,
      PiecewiseLinearFunctionFromProto(
          speed_limit_config.hard_curb_clearance_rel_speed_plf()));
  if (close_curb_speed_limit.has_value()) {
    speed_limit_map.emplace(SpeedLimitTypeProto::CLOSE_CURB,
                            std::move(*close_curb_speed_limit));
  }

  if (auto external_speed_limit = GenerateExternalSpeedLimit(
          discretized_points, constraint_mgr, veh_geo_params, max_speed_limit);
      external_speed_limit.has_value()) {
    speed_limit_map.emplace(SpeedLimitTypeProto::EXTERNAL,
                            std::move(*external_speed_limit));
  }

  SpeedLimit combination_speed_limit =
      GenerateCombinationSpeedLimit(speed_limit_map, max_speed_limit);
  speed_limit_map.emplace(SpeedLimitTypeProto::COMBINATION,
                          std::move(combination_speed_limit));

  return speed_limit_map;
}

VtSpeedLimit GetExternalVtSpeedLimit(const ConstraintManager& constraint_mgr,
                                     int traj_steps, double time_step) {
  FUNC_QTRACE();
  VtSpeedLimit vt_speed_limit;
  const auto& vt_speed_profiles = constraint_mgr.SpeedProfiles();
  if (vt_speed_profiles.empty()) return vt_speed_limit;
  vt_speed_limit.reserve(traj_steps + 1);
  // Convert speed profiles to map of piecewise linear func.
  std::map<std::string, PiecewiseLinearFunction<double>> vt_speed_map;
  for (const auto& speed_profile : vt_speed_profiles) {
    vt_speed_map.emplace(
        TypeCaseToString(speed_profile.source().type_case()),
        PiecewiseLinearFunctionFromProto(speed_profile.vt_upper_constraint()));
  }
  for (double t = 0.0; t < traj_steps * kTrajectoryTimeStep; t += time_step) {
    double min_speed_upper_bound = std::numeric_limits<double>::max();
    std::string min_type;
    for (const auto& [type, speed_profile] : vt_speed_map) {
      if (speed_profile(t) < min_speed_upper_bound) {
        min_speed_upper_bound = speed_profile(t);
        min_type = type;
      }
    }
    vt_speed_limit.emplace_back(min_speed_upper_bound, std::move(min_type));
  }
  return vt_speed_limit;
}

}  // namespace qcraft::planner

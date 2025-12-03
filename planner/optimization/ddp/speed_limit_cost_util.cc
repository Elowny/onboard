#include "onboard/planner/optimization/ddp/speed_limit_cost_util.h"

// IWYU pragma: no_include <ext/alloc_traits.h>
//              for __alloc_traits<>::value_type

#include <algorithm>
#include <cmath>
#include <iterator>
#include <limits>
#include <map>
#include <memory>
#include <ostream>
#include <utility>
#include <vector>

#include "absl/status/statusor.h"
#include "absl/types/span.h"
#include "glog/logging.h"

#include "onboard/global/buffered_logger.h"
#include "onboard/lite/check.h"
#include "onboard/math/frenet_common.h"
#include "onboard/math/geometry/segment2d.h"
#include "onboard/math/util.h"
#include "onboard/planner/decision/proto/constraint.pb.h"
#include "onboard/planner/planner_defs.h"

namespace qcraft {
namespace planner {
namespace optimizer {
namespace speedlimit {
// NOLINTNEXTLINE
void MergeSpeedLimitWithFirstStopLine(
    double first_stop_line_s, const Vec2d& first_stop_point,
    const std::vector<double>& station_points_s,
    const std::vector<Vec2d>& speed_limit_x,
    std::vector<double>* station_speed_limits,
    std::vector<std::vector<SpeedlimitInfoPoint>>* additional_speed_limits) {
  QCHECK_EQ(station_points_s.size(), additional_speed_limits->size() + 1);
  constexpr double kEps = 1e-10;
  constexpr double kMergeStopPointS = 0.1;
  const int speed_limit_x_size = speed_limit_x.size();

  const auto fill_zero_speed_limit = [&additional_speed_limits,
                                      &station_speed_limits](
                                         int segment_index,
                                         int internal_index) {
    if (segment_index == station_speed_limits->size() - 1) {
      (*station_speed_limits)[segment_index] = 0.0;
      return;
    }
    auto& speed_limit_info = (*additional_speed_limits)[segment_index];
    if (speed_limit_info.empty() && internal_index == 0) {
      (*station_speed_limits)[segment_index] = 0.0;
    } else {
      QCHECK_LE(internal_index, speed_limit_info.size() - 1);
      QCHECK_GE(internal_index, 0);
      speed_limit_info[internal_index].speed_limit = 0.0;
      speed_limit_info.erase(speed_limit_info.begin() + internal_index + 1,
                             speed_limit_info.end());
    }
    for (int k = segment_index + 1; k < additional_speed_limits->size(); ++k) {
      (*additional_speed_limits)[k].clear();
      (*station_speed_limits)[k] = 0.0;
    }
    station_speed_limits->back() = 0.0;
  };

  if ((first_stop_line_s - station_points_s.back()) > kEps) {
    return;
  } else if (first_stop_line_s - station_points_s.front() < kMergeStopPointS) {
    // First stop point very close or even ahead of first speed limit point
    // (could happen when AV rac point passed first stop point), merge them
    fill_zero_speed_limit(0, 0);
  } else {
    const auto min_dis_index =
        std::min_element(speed_limit_x.begin(), speed_limit_x.end(),
                         [&first_stop_point](const Vec2d& p1, const Vec2d& p2) {
                           return (p1 - first_stop_point).squaredNorm() <
                                  (p2 - first_stop_point).squaredNorm();
                         }) -
        speed_limit_x.begin();
    if ((first_stop_point - speed_limit_x[min_dis_index]).norm() <
        kMergeStopPointS) {
      if (min_dis_index < speed_limit_x.size() - 1) {
        (*additional_speed_limits)[min_dis_index].clear();
      }
      fill_zero_speed_limit(min_dis_index, 0);
      return;
    }

    int last_index_front_stop_point = 0;
    int first_index_after_stop_point = 1;

    if (min_dis_index == 0) {
      last_index_front_stop_point = 0;
      first_index_after_stop_point = 1;
    } else if (min_dis_index == (speed_limit_x.size() - 1)) {
      last_index_front_stop_point = speed_limit_x_size - 2;
      first_index_after_stop_point = speed_limit_x_size - 1;
    } else {
      const Segment2d line(speed_limit_x[min_dis_index - 1],
                           speed_limit_x[min_dis_index]);
      if (line.ProjectOntoUnit(first_stop_point) > line.length()) {
        last_index_front_stop_point = min_dis_index;
        first_index_after_stop_point = min_dis_index + 1;
      } else {
        last_index_front_stop_point = min_dis_index - 1;
        first_index_after_stop_point = min_dis_index;
      }
    }

    const Segment2d segment(speed_limit_x[last_index_front_stop_point],
                            speed_limit_x[first_index_after_stop_point]);
    const double alpha = std::clamp(
        segment.ProjectOntoUnit(first_stop_point) / segment.length(), 0.0, 1.0);
    auto& speed_limit_info =
        (*additional_speed_limits)[last_index_front_stop_point];
    int first_index_upper_stop_line = static_cast<int>(speed_limit_info.size());
    for (int idx = 0; idx < speed_limit_info.size(); ++idx) {
      if (speed_limit_info[idx].alpha > alpha) {
        first_index_upper_stop_line = idx;
        break;
      }
    }
    speed_limit_info.insert(
        speed_limit_info.begin() + first_index_upper_stop_line, {alpha, 0.0});
    fill_zero_speed_limit(last_index_front_stop_point,
                          first_index_upper_stop_line);
  }
}

// NOLINTNEXTLINE
void MergeSpeedLimitWithSpeedZones(
    const std::vector<SpeedZoneInfo>& speed_zones,
    const std::vector<Vec2d>& station_points,
    std::vector<double>* station_speed_limits,
    std::vector<std::vector<SpeedlimitInfoPoint>>* additional_speed_limits) {
  constexpr double kEps = 1e-4;
  additional_speed_limits->reserve(station_points.size() - 1);
  int speed_zone_idx = 0;
  double cur_speed_limit = std::numeric_limits<double>::infinity();
  for (int station_i = 0; station_i < station_points.size() - 1; ++station_i) {
    std::vector<SpeedlimitInfoPoint> speed_limit_info;
    const double station_speed_limit_cur = (*station_speed_limits)[station_i];
    (*station_speed_limits)[station_i] =
        std::min(station_speed_limit_cur, cur_speed_limit);

    const int next_station_i = station_i + 1;
    const Segment2d segment(station_points[station_i],
                            station_points[next_station_i]);
    while (speed_zone_idx < speed_zones.size()) {
      const auto& speed_zone_info = speed_zones[speed_zone_idx];
      const double speed_zone_start_alpha =
          segment.ProjectOntoUnit(speed_zone_info.x_start) / segment.length();
      const double speed_zone_end_alpha =
          segment.ProjectOntoUnit(speed_zone_info.x_end) / segment.length();
      if (speed_zone_end_alpha < 0.0) {
        ++speed_zone_idx;
        continue;
      }

      if (speed_limit_info.empty() && speed_zone_start_alpha <= kEps) {
        cur_speed_limit = speed_zone_info.target_speed;
        (*station_speed_limits)[station_i] =
            std::min(cur_speed_limit, station_speed_limit_cur);
      } else if (speed_zone_start_alpha >= -kEps &&
                 speed_zone_start_alpha < 1.0) {
        cur_speed_limit = speed_zone_info.target_speed;
        if (speed_limit_info.empty() ||
            speed_zone_start_alpha > (speed_limit_info.back().alpha + kEps)) {
          speed_limit_info.push_back(
              {std::max(0.0, speed_zone_start_alpha),
               std::min(cur_speed_limit, station_speed_limit_cur)});
        } else {
          speed_limit_info.back().speed_limit =
              std::min(cur_speed_limit, station_speed_limit_cur);
        }
      }

      if (speed_zone_end_alpha > -kEps && speed_zone_end_alpha < 1.0) {
        if (speed_limit_info.empty() ||
            speed_zone_end_alpha > (speed_limit_info.back().alpha + kEps)) {
          speed_limit_info.push_back(
              {std::max(0.0, speed_zone_end_alpha), station_speed_limit_cur});
        } else {
          speed_limit_info.back().speed_limit = station_speed_limit_cur;
        }
        cur_speed_limit = std::numeric_limits<double>::infinity();
        ++speed_zone_idx;
      } else {
        break;
      }
    }
    additional_speed_limits->push_back(std::move(speed_limit_info));
  }
  station_speed_limits->back() =
      std::min(station_speed_limits->back(), cur_speed_limit);
}

inline double SpeedAfterDeltaLength(double start_v, double a,
                                    double delta_length) {
  const double speed_square = Sqr(start_v) + a * 2.0 * delta_length;
  QCHECK_GE(speed_square, 0.0);
  return std::sqrt(speed_square);
}

inline double DeltaT(double delta_length, double v1, double v2) {
  QCHECK_GE(v1 + v2, 1e-15);
  return delta_length * 2.0 / (v1 + v2);
}

void CreateSpatialRefWithPointsSpeedLimit(
    double trajectory_time_step, const std::vector<double>& points_length,
    const std::vector<double>& points_speed_limits, int start_index,
    int start_k, double start_v, double start_s, double max_a, double min_a,
    int step_count, std::vector<double>* s_ref) {
  CHECK_GE(start_index, 0);
  CHECK_LT(start_index, points_length.size());
  CHECK_GE(s_ref->size(), start_k);
  QCHECK_GT(trajectory_time_step, 0.0);
  constexpr double kEps = 1e-6;
  const double start_delta_length = points_length[start_index] - start_s;

  std::vector<double> v_ref_on_points;
  v_ref_on_points.reserve(points_length.size());
  v_ref_on_points.push_back(start_v);
  v_ref_on_points.push_back(
      std::min(points_speed_limits[start_index],
               SpeedAfterDeltaLength(v_ref_on_points.back(), max_a,
                                     start_delta_length)));

  // Forward and backward pass along s to get v_ref at points. We assume that av
  // moves based on constant acc/decel model.
  for (int idx = start_index + 1; idx < points_length.size(); ++idx) {
    const double delta_length = points_length[idx] - points_length[idx - 1];
    v_ref_on_points.push_back(std::min(
        points_speed_limits[idx],
        SpeedAfterDeltaLength(v_ref_on_points.back(), max_a, delta_length)));
  }

  for (int idx = points_length.size() - 2; idx >= start_index; --idx) {
    const double delta_length = points_length[idx + 1] - points_length[idx];
    const int v_ref_on_points_index = idx - start_index;

    v_ref_on_points[v_ref_on_points_index] =
        std::min(v_ref_on_points[v_ref_on_points_index],
                 std::max(0.0, SpeedAfterDeltaLength(
                                   v_ref_on_points[v_ref_on_points_index + 1],
                                   -min_a, delta_length)));
  }

  // Generate time base on v_ref_on_points at evert points.
  std::vector<double> ts;
  ts.reserve(v_ref_on_points.size());
  ts.push_back(static_cast<double>(start_k) * trajectory_time_step);
  if (v_ref_on_points[0] < kEps && v_ref_on_points[1] < kEps) {
    ts.push_back(ts.back());
  } else {
    ts.push_back(ts.back() + DeltaT(start_delta_length, v_ref_on_points[0],
                                    v_ref_on_points[1]));
  }
  for (int idx = start_index; idx < points_length.size() - 1; ++idx) {
    const int v_ref_on_points_index = idx - start_index;
    const double delta_length = points_length[idx + 1] - points_length[idx];
    if (v_ref_on_points[v_ref_on_points_index + 1] < kEps &&
        v_ref_on_points[v_ref_on_points_index + 2] < kEps) {
      ts.push_back(ts.back());
    } else {
      ts.push_back(ts.back() +
                   DeltaT(delta_length,
                          v_ref_on_points[v_ref_on_points_index + 1],
                          v_ref_on_points[v_ref_on_points_index + 2]));
    }
  }

  // Interpolation on v_ref_on_points and ts at trajectory time point to get
  // s_ref and v_ref.
  (*s_ref)[start_k] = start_s;
  for (int k = start_k + 1; k < step_count; ++k) {
    const double t = trajectory_time_step * static_cast<double>(k);
    const int index =
        std::distance(ts.begin(), std::upper_bound(ts.begin(), ts.end(), t));
    if (index == ts.size() || std::abs(ts[index] - ts[index - 1]) < kEps) {
      (*s_ref)[k] = (*s_ref)[k - 1];
    } else {
      const double alpha = LerpFactor(ts[index - 1], ts[index], t);
      const double delta_t = t - ts[index - 1];
      const double s_base =
          (index == 1) ? start_s : points_length[start_index + index - 2];
      if (delta_t < kEps) {
        (*s_ref)[k] = s_base;
      } else {
        const double v_ref =
            Lerp(v_ref_on_points[index - 1], v_ref_on_points[index], alpha);
        const double a = (v_ref - v_ref_on_points[index - 1]) / delta_t;
        (*s_ref)[k] = s_base + v_ref_on_points[index - 1] * delta_t +
                      0.5 * a * Sqr(delta_t);
      }
    }
  }
}

// https://qcraft.feishu.cn/docx/ONqqdugNZoZkPGxMXiecsb9VncH
// NOLINTNEXTLINE
std::vector<double> CreateSpatialRef(
    double trajectory_time_step, const TrajectoryPoint& plan_start_point,
    const std::vector<Vec2d>& station_points,
    const std::vector<double>& station_speed_limits,
    const std::vector<std::vector<SpeedlimitInfoPoint>>&
        additional_speed_limits,
    const std::vector<LeadingInfo>& leading_min_s, int step_count, double max_a,
    double min_a, double leading_s_offset) {
  QCHECK_GT(trajectory_time_step, 0.0);
  // Merge station_speed_limits and additional_speed_limits to a
  // vector and generate points length squence.
  std::vector<double> points_length;
  std::vector<double> points_speed_limits;
  points_length.reserve(station_speed_limits.size());
  points_speed_limits.reserve(station_speed_limits.size());
  points_length.push_back(0.0);
  points_speed_limits.push_back(station_speed_limits.front());
  for (int idx = 0; idx < station_speed_limits.size() - 1; ++idx) {
    const auto& additional_speed_limit = additional_speed_limits[idx];
    const double segment_length =
        (station_points[idx + 1] - station_points[idx]).norm();
    if (additional_speed_limit.empty()) {
      points_length.push_back(points_length.back() + segment_length);
      points_speed_limits.push_back(station_speed_limits[idx + 1]);
    } else {
      const double delta_length =
          additional_speed_limit.front().alpha * segment_length;
      points_length.push_back(points_length.back() + delta_length);
      points_speed_limits.push_back(additional_speed_limit.front().speed_limit);
      for (int i = 1; i < additional_speed_limit.size(); ++i) {
        const double delta_length = (additional_speed_limit[i].alpha -
                                     additional_speed_limit[i - 1].alpha) *
                                    segment_length;
        points_length.push_back(points_length.back() + delta_length);
        points_speed_limits.push_back(additional_speed_limit[i].speed_limit);
      }
      const double delta_length_back =
          (1.0 - additional_speed_limit.back().alpha) * segment_length;
      points_length.push_back(points_length.back() + delta_length_back);
      points_speed_limits.push_back(station_speed_limits[idx + 1]);
    }
  }

  // Find points first in front of plan start point all points.
  int start_index = 0;
  double min_square_dist = std::numeric_limits<double>::infinity();
  for (int idx = 0; idx < station_points.size(); ++idx) {
    const double square_dist =
        (plan_start_point.pos() - station_points[idx]).squaredNorm();
    if (square_dist < min_square_dist) {
      start_index = idx;
      min_square_dist = square_dist;
    }
  }
  const Segment2d line(station_points[start_index],
                       station_points[start_index + 1]);
  double alpha = line.ProjectOntoUnit(plan_start_point.pos());
  if (alpha < 0.0 && start_index > 0) {
    const Segment2d line_prev(station_points[start_index - 1],
                              station_points[start_index]);
    alpha = line_prev.ProjectOntoUnit(plan_start_point.pos());
    start_index -= 1;
  }

  const double start_point_length = points_length[start_index] + alpha;

  for (int idx = start_index; idx < points_length.size(); ++idx) {
    if (points_length[idx] > start_point_length) {
      start_index = idx;
      break;
    }
  }

  // Preprocessing leading min s: Leading_min_s should not be diminishing along
  // time sequence.
  std::vector<LeadingInfo> amend_leading_min_s = leading_min_s;
  amend_leading_min_s.reserve(step_count);
  for (int i = amend_leading_min_s.size(); i < step_count; ++i) {
    amend_leading_min_s.push_back(
        LeadingInfo{.s = std::numeric_limits<double>::infinity(),
                    .v = std::numeric_limits<double>::infinity()});
  }
  for (int k = amend_leading_min_s.size() - 1; k > 0; --k) {
    if (amend_leading_min_s[k].s < amend_leading_min_s[k - 1].s) {
      amend_leading_min_s[k - 1] = amend_leading_min_s[k];
      amend_leading_min_s[k - 1].v = 0.0;
    }
  }

  for (int k = 0; k < amend_leading_min_s.size(); ++k) {
    amend_leading_min_s[k].s += leading_s_offset;
    amend_leading_min_s[k].v = std::max(amend_leading_min_s[k].v, 0.0);
  }

  // Generate s_ref firstly based on points speed limits with constant
  // acceleration and deceleration.
  std::vector<double> s_ref(step_count, 0.0);
  CreateSpatialRefWithPointsSpeedLimit(
      trajectory_time_step, points_length, points_speed_limits, start_index,
      /*start_k=*/0, plan_start_point.v(), start_point_length, max_a, min_a,
      step_count, &s_ref);

  // Generate s_ref considering leading. Detail in document
  // https://qcraft.feishu.cn/docx/ONqqdugNZoZkPGxMXiecsb9VncH.
  constexpr double kEps = 1e-6;
  const int size = s_ref.size();
  int idx = 0;
  while (idx < size) {
    if (s_ref[idx] > amend_leading_min_s[idx].s) {
      if (idx == (size - 1)) {
        s_ref[idx] = amend_leading_min_s[idx].s;
      } else {
        for (int k = idx + 1; k < size; ++k) {
          if (s_ref[k] <= amend_leading_min_s[k].s &&
              std::isfinite(amend_leading_min_s[k].s)) {
            idx = k;
            break;
          } else if ((k != 0 && amend_leading_min_s[k].v >
                                    amend_leading_min_s[k - 1].v + kEps) ||
                     (k == (size - 1) &&
                      std::isfinite(amend_leading_min_s[k].s))) {
            for (int i = idx; i < k; ++i) {
              s_ref[i] = amend_leading_min_s[i].s;
            }
            if (k < (size - 1)) {
              const int start_index = std::distance(
                  points_length.begin(),
                  std::upper_bound(points_length.begin(), points_length.end(),
                                   amend_leading_min_s[k - 1].s));
              CreateSpatialRefWithPointsSpeedLimit(
                  trajectory_time_step, points_length, points_speed_limits,
                  start_index,
                  /*start_k=*/k - 1, amend_leading_min_s[k - 1].v,
                  amend_leading_min_s[k - 1].s, max_a, min_a, step_count,
                  &s_ref);
            }
            idx = k;
            break;
          }
        }
      }
    } else {
      ++idx;
    }
  }
  return s_ref;
}

}  // namespace speedlimit

namespace {

// Copied from empty_road_decider.
// NOLINTNEXTLINE
std::vector<SpeedZoneInfo> CreateExclusiveSpeedZones(
    const std::vector<SpeedZoneInfo>& speed_zones,
    std::vector<int>* exclusive_speed_zone_original_indices) {
  struct SpeedZoneEndpointInfo {
    int zone_index;
    bool start;
    double s;
    double speed;
  };

  std::vector<SpeedZoneEndpointInfo> endpoints;
  endpoints.reserve(speed_zones.size() * 2);
  for (int i = 0; i < speed_zones.size(); ++i) {
    const auto& speed_zone = speed_zones[i];
    endpoints.push_back({i, true, speed_zone.s_start, speed_zone.target_speed});
    endpoints.push_back({i, false, speed_zone.s_end, speed_zone.target_speed});
  }
  std::stable_sort(endpoints.begin(), endpoints.end(),
                   [](const SpeedZoneEndpointInfo& p0,
                      const SpeedZoneEndpointInfo& p1) { return p0.s < p1.s; });

  // map: from speed to <zone index, start s>.
  std::multimap<double, std::pair<int, double>> active_zones;
  std::vector<SpeedZoneInfo> exclusive_speed_zones;
  if (exclusive_speed_zone_original_indices != nullptr) {
    exclusive_speed_zone_original_indices->clear();
  }
  for (int i = 0; i < endpoints.size(); ++i) {
    const SpeedZoneEndpointInfo& endpoint = endpoints[i];
    VLOG(3) << "endpoint " << i << ": " << endpoint.zone_index << " "
            << endpoint.start << " " << endpoint.s << " " << endpoint.speed;
    if (endpoint.start) {
      if (!active_zones.empty() &&
          endpoint.speed < active_zones.begin()->first) {
        exclusive_speed_zones.push_back(
            {.s_start = active_zones.begin()->second.second,
             .s_end = endpoint.s,
             .target_speed = active_zones.begin()->first});
        if (exclusive_speed_zone_original_indices != nullptr) {
          exclusive_speed_zone_original_indices->push_back(
              active_zones.begin()->second.first);
        }
      }
      active_zones.insert({endpoint.speed, {endpoint.zone_index, endpoint.s}});
    } else {
      QCHECK(!active_zones.empty());
      auto start_it = active_zones.end();
      const auto range = active_zones.equal_range(endpoint.speed);
      for (auto it = range.first; it != range.second; ++it) {
        if (endpoint.zone_index == it->second.first) {
          start_it = it;
          break;
        }
      }
      QCHECK(start_it != active_zones.end());
      QCHECK_EQ(endpoint.zone_index, start_it->second.first);

      if (start_it == active_zones.begin()) {  // Active.
        exclusive_speed_zones.push_back({.s_start = start_it->second.second,
                                         .s_end = endpoint.s,
                                         .target_speed = endpoint.speed});
        if (exclusive_speed_zone_original_indices != nullptr) {
          exclusive_speed_zone_original_indices->push_back(endpoint.zone_index);
        }
        const auto next = std::next(start_it);
        if (next != active_zones.end()) {
          next->second.second = endpoint.s;
        }
      }
      active_zones.erase(start_it);
    }
  }

  for (int i = 0; i + 1 < exclusive_speed_zones.size(); ++i) {
    QCHECK_LE(exclusive_speed_zones[i].s_end,
              exclusive_speed_zones[i + 1].s_start);
  }
  if (exclusive_speed_zone_original_indices != nullptr) {
    QCHECK_EQ(exclusive_speed_zones.size(),
              exclusive_speed_zone_original_indices->size());
  }
  return exclusive_speed_zones;
}

double GetUniformDecelerationS(
    const TrajectoryPoint& plan_start_point, const DrivePassage& drive_passage,
    const MotionConstraintParamsProto& motion_constraint_params) {
  const double plan_start_point_v = plan_start_point.v();
  const auto plan_start_point_sl_or = drive_passage.QueryFrenetCoordinateAt(
      Vec2d(plan_start_point.pos().x(), plan_start_point.pos().y()));
  if (plan_start_point_sl_or.ok()) {
    const double plan_start_point_s_on_drive_passage =
        plan_start_point_sl_or->s;
    const double max_deceleration = motion_constraint_params.max_deceleration();
    const double slow_down_zero_time =
        std::max(0.0, plan_start_point_v / max_deceleration);
    return plan_start_point_s_on_drive_passage +
           plan_start_point_v * slow_down_zero_time +
           0.5 * max_deceleration * Sqr(slow_down_zero_time);
  } else {
    return std::numeric_limits<double>::infinity();
  }
}

void ProcessSpeedLimitWithSpeedRegions(
    double min_passage_s, double max_passage_s,
    const DrivePassage& drive_passage,
    absl::Span<const ConstraintProto::SpeedRegionProto> speed_regions,
    double front_edge_to_center, double back_edge_to_center,
    const std::vector<Vec2d>& station_points,
    std::vector<double>* station_speed_limits,
    std::vector<std::vector<SpeedlimitInfoPoint>>* additional_speed_limits) {
  QCHECK_NOTNULL(additional_speed_limits);
  QCHECK(additional_speed_limits->empty());

  // Speed zones from constraints. Note that speed zone should not exceed the
  // range of drive passage.
  std::vector<SpeedZoneInfo> speed_zones;
  for (const auto& speed_region : speed_regions) {
    // Only consider speed bumps now.
    if (speed_region.source().type_case() ==
        SourceProto::TypeCase::kSpeedBump) {
      auto& speed_zone = speed_zones.emplace_back();
      // Make sure that speed_zone.s_end is larger than speed_zone.s_start.
      speed_zone.s_start =
          std::clamp(speed_region.start_s() - front_edge_to_center,
                     min_passage_s, max_passage_s);
      speed_zone.s_end = std::clamp(speed_region.end_s() + back_edge_to_center,
                                    min_passage_s, max_passage_s);
      speed_zone.target_speed = speed_region.max_speed();
    }
  }

  std::vector<int> speed_zone_indices;
  speed_zones = CreateExclusiveSpeedZones(speed_zones, &speed_zone_indices);
  for (int i = 0; i + 1 < speed_zones.size(); ++i) {
    QCHECK_LE(speed_zones[i].s_end, speed_zones[i + 1].s_start);
  }
  for (auto& speed_zone : speed_zones) {
    speed_zone.x_start =
        drive_passage.QueryPointXYAtS(speed_zone.s_start).value();
    speed_zone.x_end = drive_passage.QueryPointXYAtS(speed_zone.s_end).value();
  }
  speedlimit::MergeSpeedLimitWithSpeedZones(speed_zones, station_points,
                                            station_speed_limits,
                                            additional_speed_limits);
}

double ClampStopLine(
    const TrajectoryPoint& plan_start_point, const DrivePassage& drive_passage,
    double first_stop_line_s, double min_passage_s, double max_passage_s,
    const MotionConstraintParamsProto& motion_constraint_params) {
  first_stop_line_s =
      std::clamp(first_stop_line_s, min_passage_s, max_passage_s);
  return std::clamp(GetUniformDecelerationS(plan_start_point, drive_passage,
                                            motion_constraint_params),
                    first_stop_line_s, max_passage_s);
}

void ProcessSpeedLimitWithFirstStopLine(
    const DrivePassage& drive_passage, double first_stop_line_s,
    const std::vector<Vec2d>& station_points,
    const std::vector<double>& station_points_s,
    std::vector<double>* station_speed_limits,
    std::vector<std::vector<SpeedlimitInfoPoint>>* additional_speed_limits) {
  const auto first_stop_point_status =
      drive_passage.QueryPointXYAtS(first_stop_line_s);
  QCHECK_OK(first_stop_point_status.status());
  const Vec2d& first_stop_point = *first_stop_point_status;
  speedlimit::MergeSpeedLimitWithFirstStopLine(
      first_stop_line_s, first_stop_point, station_points_s, station_points,
      station_speed_limits, additional_speed_limits);
}

}  // namespace

void AddSpeedLimitCost(
    int trajectory_steps, double trajectory_time_step,
    const TrajectoryPoint& plan_start_point, const DrivePassage& drive_passage,
    const ConstraintManager& constraint_manager,
    const TrajectoryOptimizerCostWeightParamsProto& cost_weight_params,
    const MotionConstraintParamsProto& motion_constraint_params,
    const VehicleGeometryParamsProto& veh_geo_params,
    const std::unique_ptr<CenterLineQueryHelper<Mfob>>& stations_query_helper,
    const std::vector<LeadingInfo>& leading_min_s, double* ref_end_state_s,
    std::vector<std::unique_ptr<Cost<Mfob>>>* costs) {
  const auto& stop_lines = constraint_manager.StopLine();
  const auto& speed_regions = constraint_manager.SpeedRegion();

  QCHECK_GT(trajectory_steps, 0);
  QCHECK_GT(trajectory_time_step, 0.0);
  const int free_index = static_cast<int>(
      (kTrajectorySteps - 1) * kTrajectoryTimeStep / trajectory_time_step);

  // TODO(renjie): See if needs resampling other than use passage stations
  // directly.
  QCHECK_EQ(stations_query_helper->points().size(), drive_passage.size());
  const std::vector<Vec2d>& station_points = stations_query_helper->points();
  std::vector<double> station_points_s;      // From drive passage start.
  std::vector<double> station_speed_limits;  // m/s.
  std::vector<double> station_thetas;
  std::vector<double> free_station_speed_limits;  // m/s.
  station_points_s.reserve(drive_passage.size());
  station_speed_limits.reserve(drive_passage.size());
  station_thetas.reserve(drive_passage.size());
  for (const auto& station : drive_passage.stations()) {
    station_points_s.push_back(station.accumulated_s());
    station_speed_limits.push_back(station.speed_limit());
    station_thetas.push_back(station.tangent().FastAngle());
  }
  // Update lane speed limits considering previewed average passage curvature.
  // Four params below should a little larger than params in speed_limit_params
  // in speed_finder_default_params.pb.txt.
  // TODO(yuhang): Refactor the logic below to a common api.
  const auto& speed_limit_cost_params =
      cost_weight_params.speed_limit_cost_params();
  const double curvature_power = speed_limit_cost_params.curvature_power();
  const double curvature_numerator =
      speed_limit_cost_params.curvature_numerator();
  const double curvature_bias1 = speed_limit_cost_params.curvature_bias1();
  const double curvature_bias2 = speed_limit_cost_params.curvature_bias2();
  constexpr double kSmallKappaThres = 0.003;           // m^-1.
  constexpr double kKappaSpeedLimitCuffThres = 0.002;  // m^-1.
  const double k_power = std::pow(kSmallKappaThres, curvature_power);
  const double k_power_plus_bias1 = k_power + curvature_bias1;
  const double small_kappa_speed_limit_sqr =
      Sqr(curvature_numerator / k_power_plus_bias1 + curvature_bias2);
  const double small_kappa_lat_acc_derivative = std::max(
      0.0, -(2.0 * curvature_numerator * curvature_power * k_power *
             (curvature_numerator + curvature_bias2 * k_power_plus_bias1)) /
                   Cube(k_power_plus_bias1) +
               small_kappa_speed_limit_sqr);
  const double intercept =
      std::max(0.0, -kSmallKappaThres * small_kappa_lat_acc_derivative +
                        small_kappa_speed_limit_sqr * kSmallKappaThres);
  constexpr double kMinPreviewDistance = 10.0;         // m
  constexpr double kKappaEstimateLookAheadTime = 1.0;  // s
  for (int i = 0; i < station_points.size(); ++i) {
    const double s_now = station_points_s[i];
    const double theta_now = station_thetas[i];
    const double preview_distance =
        std::max(kMinPreviewDistance,
                 station_speed_limits[i] * kKappaEstimateLookAheadTime);
    const double s_preview = s_now + preview_distance;
    const auto& station_preview =
        drive_passage.FindNearestStationAtS(s_preview);
    const double theta_preview = station_preview.tangent().FastAngle();
    const double kappa_fd =
        std::abs(NormalizeAngle(theta_preview - theta_now) / preview_distance);
    double v_kappa_fd = 0.0;
    if (kappa_fd < kKappaSpeedLimitCuffThres) {
      continue;
    } else if (kappa_fd >= kSmallKappaThres) {
      v_kappa_fd = curvature_numerator /
                       (std::pow(kappa_fd, curvature_power) + curvature_bias1) +
                   curvature_bias2;
    } else {
      const double lat_acc =
          kappa_fd * small_kappa_lat_acc_derivative + intercept;
      constexpr double kSmallEps = 1e-6;
      v_kappa_fd = std::sqrt(lat_acc / (kappa_fd + kSmallEps));
    }
    station_speed_limits[i] = std::min(station_speed_limits[i], v_kappa_fd);
  }
  // Filter curvature speed limit with LPF.
  constexpr double kLpfAlpha = 0.4;
  const int station_points_size = static_cast<int>(station_points.size());
  for (int i = 1; i < station_points_size; ++i) {
    station_speed_limits[i] =
        Lerp(station_speed_limits[i - 1], station_speed_limits[i], kLpfAlpha);
  }
  for (int i = station_points_size - 1; i > 0; --i) {
    station_speed_limits[i - i] =
        Lerp(station_speed_limits[i], station_speed_limits[i - 1], kLpfAlpha);
  }
  free_station_speed_limits = station_speed_limits;

  // Get drive passage s range.
  constexpr double kPassageEps = 0.1;  // m.
  const double min_passage_s = drive_passage.front_s() + kPassageEps;
  const double max_passage_s = drive_passage.end_s() - kPassageEps;

  // TODO(runbing): Create virtual stop lines for static objects.
  double first_stop_line_s = std::numeric_limits<double>::infinity();
  double path_boundary_stop_line_s = std::numeric_limits<double>::infinity();
  for (const auto& stop_line : stop_lines) {
    first_stop_line_s =
        std::min(first_stop_line_s, stop_line.s() - stop_line.standoff() -
                                        veh_geo_params.front_edge_to_center());
    if (stop_line.source().type_case() ==
        SourceProto::TypeCase::kEndOfPathBoundary) {
      path_boundary_stop_line_s = stop_line.s() - stop_line.standoff() -
                                  veh_geo_params.front_edge_to_center();
    }
  }
  const double first_stop_line_s_clamped =
      ClampStopLine(plan_start_point, drive_passage, first_stop_line_s,
                    min_passage_s, max_passage_s, motion_constraint_params);
  const double path_boundary_stop_line_s_clamped =
      ClampStopLine(plan_start_point, drive_passage, path_boundary_stop_line_s,
                    min_passage_s, max_passage_s, motion_constraint_params);

  // Process speed limit info.
  std::vector<double> s_ref;
  std::vector<std::vector<SpeedlimitInfoPoint>> additional_speed_limits;

  ProcessSpeedLimitWithSpeedRegions(
      min_passage_s, max_passage_s, drive_passage, speed_regions,
      veh_geo_params.front_edge_to_center(),
      veh_geo_params.back_edge_to_center(), station_points,
      &station_speed_limits, &additional_speed_limits);

  // Use speed limit info before merging stop line to get reference s
  // sequence.
  s_ref = speedlimit::CreateSpatialRef(
      trajectory_time_step, plan_start_point, station_points,
      station_speed_limits, additional_speed_limits, leading_min_s, free_index,
      /*max_a=*/0.7, /*min_a=*/-1.0,
      /*leading_s_offset=*/-veh_geo_params.front_edge_to_center() -
          drive_passage.front_s());

  ProcessSpeedLimitWithFirstStopLine(
      drive_passage, first_stop_line_s_clamped, station_points,
      station_points_s, &station_speed_limits, &additional_speed_limits);

  if (VLOG_IS_ON(4)) {
    for (int i = 0; i < additional_speed_limits.size(); ++i) {
      VLOG(4) << i << " " << additional_speed_limits[i].size() << " "
              << station_points_s[i] << " " << station_speed_limits[i];
      const auto& speed_limit_info = additional_speed_limits[i];
      for (int k = 0; k < speed_limit_info.size(); ++k) {
        VLOG(4) << k << " " << speed_limit_info[k].alpha << " "
                << speed_limit_info[k].speed_limit;
      }
    }
  }

  // Process free speed limit info.
  // For trajectory point beyond normal planning horizon, we use a
  // separate speed limit which ignores all stop lines and speed regions
  // except one stop line at drive passage end to encourage progress.
  std::vector<double> free_s_ref;
  std::vector<std::vector<SpeedlimitInfoPoint>> additional_free_speed_limits;
  additional_free_speed_limits.resize(station_points.size() - 1, {});

  // Use end state of s_ref of normal planning horizon as start point to
  // continue extending reference s based on free stage speed limit, so that get
  // end of s_ref with reasonable length.
  TrajectoryPoint free_start_point;
  const double s_ref_length = s_ref.back() - s_ref.front();
  // To ensure reference s not exceeeding stop line, use closest stop line to
  // cut off s_ref_length to ensure s_ref_length.
  const double speed_ref_s_virtual = std::min(
      drive_passage.end_s(), std::min(first_stop_line_s_clamped, s_ref_length));
  if (speed_ref_s_virtual < s_ref_length) {
    const auto speed_ref_s_pos =
        drive_passage.QueryPointXYAtS(speed_ref_s_virtual);
    QCHECK(speed_ref_s_pos.ok());
    // If s_ref_length is cut off by stop line, speed at end of s_ref should
    // be zero.
    const double speed_end = 0.0;
    free_start_point.set_pos(*speed_ref_s_pos);
    free_start_point.set_v(speed_end);
  } else {
    const auto speed_ref_s_pos = drive_passage.QueryPointXYAtS(s_ref_length);
    QCHECK(speed_ref_s_pos.ok());
    const double speed_end =
        (s_ref[s_ref.size() - 1] - s_ref[s_ref.size() - 2]) /
        trajectory_time_step;
    free_start_point.set_pos(*speed_ref_s_pos);
    free_start_point.set_v(speed_end);
  }

  // Compute SpatialRef first to avoid effect from path boundary stop line: If
  // AV stops before path boundary stop line with constant deceleration (we
  // don't want this.), AV needs to start slow down early, and ref s may be too
  // low.
  free_s_ref = speedlimit::CreateSpatialRef(
      trajectory_time_step, free_start_point, station_points,
      free_station_speed_limits, additional_free_speed_limits,
      /*leading_min_s=*/std::vector<LeadingInfo>(),
      trajectory_steps - free_index, /*max_a=*/1.0,
      /*min_a=*/-1.0,
      /*leading_s_offset=*/-veh_geo_params.front_edge_to_center() -
          drive_passage.front_s());

  ProcessSpeedLimitWithFirstStopLine(
      drive_passage, path_boundary_stop_line_s_clamped, station_points,
      station_points_s, &free_station_speed_limits,
      &additional_free_speed_limits);
  if (VLOG_IS_ON(4)) {
    for (int i = 0; i < additional_free_speed_limits.size(); ++i) {
      VLOG(4) << i << " " << additional_free_speed_limits[i].size() << " "
              << station_points_s[i] << " " << station_speed_limits[i];
      const auto& speed_limit_info = additional_free_speed_limits[i];
      for (int k = 0; k < speed_limit_info.size(); ++k) {
        VLOG(4) << k << " " << speed_limit_info[k].alpha << " "
                << speed_limit_info[k].speed_limit;
      }
    }
  }

  constexpr double kStopSpeedCostGain = 800.0;
  constexpr double kOverSpeedCostGain = 100.0;
  constexpr double kUnderSpeedCostGain = 0.0;
  using SpeedLimitInfo = SegmentedSpeedLimitCostV2<Mfob>::SpeedLimitInfo;

  costs->emplace_back(std::make_unique<SegmentedSpeedLimitCostV2<Mfob>>(
      trajectory_steps, station_points, stations_query_helper.get(),
      SpeedLimitInfo(station_speed_limits, additional_speed_limits),
      SpeedLimitInfo(free_station_speed_limits, additional_free_speed_limits),
      free_index, "MfobSpeedLimitCost",
      speed_limit_cost_params.speed_limit_cost_weight(),
      /*cost_type=*/Cost<Mfob>::CostType::MUST_HAVE,
      /*stop_speed_gain=*/kStopSpeedCostGain,
      /*over_speed_gain=*/kOverSpeedCostGain,
      /*under_speed_gain=*/kUnderSpeedCostGain,
      /*use_qtfm=*/true));

  // Finally use path boundary stop line to cut off s_ref value.
  *ref_end_state_s =
      std::min(drive_passage.end_s() - veh_geo_params.front_edge_to_center(),
               std::min(path_boundary_stop_line_s_clamped,
                        free_s_ref.back() - s_ref.front()));
}

}  // namespace optimizer
}  // namespace planner
}  // namespace qcraft

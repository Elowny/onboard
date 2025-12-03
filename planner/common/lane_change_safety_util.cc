#include "onboard/planner/common/lane_change_safety_util.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <numeric>
#include <vector>

#include "onboard/math/util.h"
#include "onboard/planner/common/lane_change_safety_params.h"
#include "onboard/planner/planner_defs.h"
#include "onboard/planner/planner_flags.h"
#include "onboard/planner/second_order_trajectory_point.h"
#include "onboard/prediction/object_prediction.h"

namespace qcraft::planner {
namespace {

inline double GetLaneChangeStyleFactor(LaneChangeStyle lc_style) {
  switch (lc_style) {
    case LC_STYLE_NORMAL:
      return 1.0;
    case LC_STYLE_RADICAL:
      return 0.8;
    case LC_STYLE_CONSERVATIVE:
      return 1.5;
  }
}

double ComputeSimilarSpeedFactor(double lead_v, double follow_v, double ref_v) {
  constexpr double kMaxSpeedDiffThres = 3.0;  // m/s.
  constexpr double kSimilarSpeedThresRatio = 1.3 - 1.0;

  return std::clamp(
      (follow_v - lead_v) /
          std::min(kMaxSpeedDiffThres, kSimilarSpeedThresRatio * ref_v),
      0.0, 1.0);
}

}  // namespace

double ComputeEgoLeadTime(double speed_limit, double ego_v, double obj_v) {
  constexpr double kMinSimilarSpeedFactor = 0.3;
  const double similar_speed_factor = std::max(
      ComputeSimilarSpeedFactor(ego_v, obj_v, ego_v), kMinSimilarSpeedFactor);

  constexpr double kExceedSpeedLimitRatio = 0.95;
  constexpr double kExceedSpeedLimitSlope = 8.0;
  const double speed_limit_factor =
      1.0 +
      kExceedSpeedLimitSlope *
          Sqr(std::max(0.0, obj_v / speed_limit - kExceedSpeedLimitRatio));

  return kEgoLeadTimeBuffer * similar_speed_factor * speed_limit_factor;
}

double ComputeEgoFollowTime(double obj_v, double ego_v) {
  constexpr double kHigherSpeedThresRatio = 0.95;

  const double min_follow_time = kMinLonBufferToFront / ego_v;
  if (obj_v > ego_v * kHigherSpeedThresRatio) return min_follow_time;

  return std::max(
      min_follow_time,
      kEgoFollowTimeBuffer * ComputeSimilarSpeedFactor(obj_v, ego_v, ego_v));
}

double ComputeLcConservFactor(const FrenetBox& av_frenet_box,
                              double av_half_width, bool lc_left,
                              LaneChangeStyle lc_style) {
  const double lat_offset = lc_left
                                ? std::abs(std::min(0.0, av_frenet_box.l_max))
                                : std::abs(std::max(0.0, av_frenet_box.l_min));
  return (FLAGS_planner_enable_lc_style_params
              ? GetLaneChangeStyleFactor(lc_style)
              : 1.0) *
         std::min(1.0,
                  Sqr(std::max(0.0, lat_offset - kEnterTargetLateralThreshold) /
                      (kDefaultLaneWidth - av_half_width -
                       kEnterTargetLateralThreshold)));
}

double ComputeSafeResponseInterval(double v_lead, double v_follow,
                                   double response_time, double lead_time) {
  const double v_diff = std::max(0.0, v_follow - v_lead);
  const double buffer_dist =
      v_diff * response_time +
      std::max(std::min(v_lead, v_follow) * lead_time, kMinLonBufferToFront);
  return buffer_dist;
}

double ComputeSafeDecelerationInterval(double v_lead, double v_follow,
                                       double max_allowed_deceleration) {
  const double v_diff = std::max(0.0, v_follow - v_lead);
  return 0.5 * Sqr(v_diff) / (std::fabs(max_allowed_deceleration));
}

double EstimateObjectSpeed(const PlannerObject& object, double preview_time) {
  double obj_v = object.pose().v();
  const auto& accel_hist = object.long_term_behavior().accel_history;
  if (!accel_hist.empty()) {
    constexpr int kMaxConsideredAccelHistoryItem = 5;  // One record per second.
    constexpr std::array<double, 5> kAccelWeights{8.0, 6.0, 4.0, 2.0, 1.0};

    int idx = 0;
    double avg_accel = 0.0;
    for (auto it = accel_hist.rbegin(); it != accel_hist.rend(); ++it) {
      avg_accel += *it * kAccelWeights[idx++];
      if (idx >= kMaxConsideredAccelHistoryItem) break;
    }
    avg_accel /= std::accumulate(kAccelWeights.begin(),
                                 kAccelWeights.begin() + idx, 0.0);
    obj_v += avg_accel * preview_time;
  }
  return obj_v;
}

}  // namespace qcraft::planner

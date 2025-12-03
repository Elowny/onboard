#include "onboard/planner/initializer/ref_speed_table.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <ostream>

#include "absl/status/statusor.h"
#include "absl/types/span.h"
#include "glog/logging.h"

#include "onboard/math/frenet_common.h"
#include "onboard/math/util.h"
#include "onboard/planner/object/spacetime_object_state.h"
#include "onboard/planner/object/spacetime_object_trajectory.h"
#include "onboard/prediction/predicted_trajectory.h"
#include "onboard/utils/status_macros.h"

namespace qcraft::planner {

namespace {
constexpr int kSampleTimeSteps = 11;
constexpr double kPredictionTimeSampleStep = 0.1;  // s
constexpr std::array<double, kSampleTimeSteps> kTimeSamples{
    0.0, 0.9, 1.9, 2.9, 3.9, 4.9, 5.9, 6.9, 7.9, 8.9, 9.9};

constexpr double kSpanDiscretizationStep = 0.5;  // m

constexpr double kSafeLeadingDist = 4.5;              // m
constexpr double kSafeLeadingTime = 1.0;              // s
constexpr double kMaxComfortableDeceleration = -1.0;  // m/s2

}  // namespace

// #################### RefSpeedVec ####################
RefSpeedVec::RefSpeedVec(const DrivePassage& drive_passage,
                         const std::vector<std::pair<double, double>>& obj_info,
                         double stop_s) {
  std::vector<double> station_accum_s, ref_speed_vec;
  station_accum_s.reserve(drive_passage.size());
  ref_speed_vec.reserve(drive_passage.size());

  for (const auto& station : drive_passage.stations()) {
    double current_s = station.accumulated_s();
    if (current_s > stop_s) break;

    const double limit_from_stop =
        std::sqrt(2 * (stop_s - current_s) * (-kMaxComfortableDeceleration));

    double limit_from_obj = std::numeric_limits<double>::max();
    for (const auto& [obj_s, obj_v] : obj_info) {
      const double obj_ref_v =
          current_s >= obj_s
              ? obj_v
              : std::sqrt(Sqr(obj_v) - 2 * (obj_s - current_s) *
                                           kMaxComfortableDeceleration);
      limit_from_obj = std::min(limit_from_obj, obj_ref_v);
    }

    station_accum_s.push_back(current_s);
    ref_speed_vec.push_back(std::min(limit_from_stop, limit_from_obj));
  }

  start_s_ = station_accum_s.front();
  end_s_ = station_accum_s.back();
  const int discretized_num =
      CeilToInt((end_s_ - start_s_) / kSpanDiscretizationStep);
  discretized_ref_speed_by_s_.resize(discretized_num + 1, 0.0);
  for (int i = 0, s_idx = 0; i <= discretized_num; ++i) {
    const double sample_s = i * kSpanDiscretizationStep + start_s_;
    if (sample_s <= start_s_ || sample_s >= end_s_) continue;

    if (sample_s > station_accum_s[s_idx]) ++s_idx;
    discretized_ref_speed_by_s_[i] =
        Lerp(ref_speed_vec[s_idx - 1], ref_speed_vec[s_idx],
             (sample_s - station_accum_s[s_idx - 1]) /
                 (station_accum_s[s_idx] - station_accum_s[s_idx - 1]));
  }
}

double RefSpeedVec::FastComputeRefSpeed(double s) const {
  if (s <= start_s_ || s >= end_s_) return 0.0;

  return discretized_ref_speed_by_s_[RoundToInt((s - start_s_) /
                                                kSpanDiscretizationStep)];
}

// #################### RefSpeedTable ####################
RefSpeedTable::RefSpeedTable(const SpacetimeTrajectoryManager& st_traj_mgr,
                             const std::vector<std::string>& leading_trajs,
                             const DrivePassage& drive_passage,
                             const std::vector<double>& stop_s) {
  // limits from stop constraints
  const double nearest_stop_s =
      stop_s.empty() ? std::numeric_limits<double>::max()
                     : *std::min_element(stop_s.begin(), stop_s.end());

  // limits from station speed limits
  station_accum_s_.reserve(drive_passage.size());
  station_speed_limits_.reserve(drive_passage.size());
  for (const auto& station : drive_passage.stations()) {
    station_accum_s_.push_back(station.accumulated_s());
    station_speed_limits_.push_back(station.speed_limit());
  }

  // limits from leading object predictions
  ref_speed_table_.reserve(kSampleTimeSteps);
  for (const double time_sample : kTimeSamples) {
    const int obj_state_idx =
        RoundToInt(time_sample / kPredictionTimeSampleStep);

    std::vector<std::pair<double, double>> obj_info;
    obj_info.reserve(leading_trajs.size());
    for (const auto& traj_id : leading_trajs) {
      const auto* traj_ptr = st_traj_mgr.FindTrajectoryById(traj_id);
      if (traj_ptr == nullptr) {
        LOG(WARNING) << "Leading trajectory " << traj_id << " cannot be found.";
        continue;
      }
      const auto& states = traj_ptr->states();
      if (states.size() <= obj_state_idx) continue;

      ASSIGN_OR_CONTINUE(const auto obj_aabbox, drive_passage.QueryFrenetBoxAt(
                                                    states[obj_state_idx].box));
      const double obj_v = states[obj_state_idx].traj_point->v();
      const double obj_s = std::max(
          0.0, obj_aabbox.s_min - kSafeLeadingDist - obj_v * kSafeLeadingTime);
      obj_info.push_back({obj_s, obj_v});
    }
    ref_speed_table_.emplace_back(drive_passage, obj_info, nearest_stop_s);
  }
}

std::pair<double, double> RefSpeedTable::LookUpRefSpeed(double time,
                                                        double span) const {
  CHECK_GE(time, 0.0);  // No look-up into the past.

  const int station_idx =
      std::upper_bound(station_accum_s_.begin(), station_accum_s_.end(), span) -
      station_accum_s_.begin();
  const double speed_limit = station_idx == station_accum_s_.size()
                                 ? station_speed_limits_.back()
                                 : station_speed_limits_[station_idx];

  double ref_speed;
  const int next_index =
      std::upper_bound(kTimeSamples.begin(), kTimeSamples.end(), time) -
      kTimeSamples.begin();
  if (next_index == kTimeSamples.size()) {
    ref_speed = ref_speed_table_.back().FastComputeRefSpeed(span);
  } else {
    const double prev_ref_speed =
        ref_speed_table_[next_index - 1].FastComputeRefSpeed(span);
    const double succ_ref_speed =
        ref_speed_table_[next_index].FastComputeRefSpeed(span);
    const double interp_t =
        (time - kTimeSamples[next_index - 1]) /
        (kTimeSamples[next_index] - kTimeSamples[next_index - 1]);

    ref_speed = Lerp(prev_ref_speed, succ_ref_speed, interp_t);
  }

  return {speed_limit, std::min(speed_limit, ref_speed)};
}

}  // namespace qcraft::planner

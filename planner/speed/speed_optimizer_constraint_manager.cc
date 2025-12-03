#include "onboard/planner/speed/speed_optimizer_constraint_manager.h"

#include <algorithm>
#include <iterator>
#include <utility>

#include "onboard/global/buffered_logger.h"
#include "onboard/lite/check.h"
#include "onboard/planner/speed/proto/speed_finder_params.pb.h"

namespace qcraft::planner {
namespace {
using Sfp = SpeedFinderParamsProto;
using BoundStrType = SpeedOptimizerConstraintManager::BoundStrType;
using BoundDirType = SpeedOptimizerConstraintManager::BoundDirType;
using SoftConstraint = SpeedOptimizerConstraintManager::SoftConstraint;
using SoftConstraintInput =
    SpeedOptimizerConstraintManager::SoftConstraintInput;
using SSoftConstraintData =
    SpeedOptimizerConstraintManager::SSoftConstraintData;
using SpeedSoftConstraintData =
    SpeedOptimizerConstraintManager::SpeedSoftConstraintData;
using SlackWeight = SpeedOptimizerConstraintManager::SlackWeight;

constexpr int kSoftSConstraintTypeNum = 3;
constexpr int kSoftSpeedConstraintTypeNum = 3;

int ReturnAndUpdateSSlackIndex(
    int time_range_idx, BoundStrType strength_type,
    std::map<std::pair<int, BoundStrType>, int>* slack_idx_map,
    int* slack_idx) {
  QCHECK_NOTNULL(slack_idx_map);
  QCHECK_NOTNULL(slack_idx);
  const auto key = std::make_pair(time_range_idx, strength_type);
  const auto slack_idx_it = slack_idx_map->find(key);
  if (slack_idx_it == slack_idx_map->end()) {
    (*slack_idx_map)[key] = (*slack_idx)++;
    return (*slack_idx) - 1;
  }
  return slack_idx_it->second;
}

int ReturnAndUpdateSpeedSlackIndex(
    SpeedLimitLevelProto::Level speed_limit_level,
    std::map<SpeedLimitLevelProto::Level, int>* slack_idx_map, int* slack_idx) {
  QCHECK_NOTNULL(slack_idx_map);
  QCHECK_NOTNULL(slack_idx);
  const auto slack_idx_it = slack_idx_map->find(speed_limit_level);
  if (slack_idx_it == slack_idx_map->end()) {
    (*slack_idx_map)[speed_limit_level] = (*slack_idx)++;
    return (*slack_idx) - 1;
  }
  return slack_idx_it->second;
}

void UpdateSoftConstraints(
    const SoftConstraintInput& constraint_input, int constraint_level,
    int slack_idx, BoundDirType dir_type, int knot_num,
    std::vector<std::vector<std::optional<SoftConstraint>>>* soft_constraints,
    std::map<int, std::map<int, double>>* slack_weight) {
  QCHECK_NOTNULL(soft_constraints);
  QCHECK_NOTNULL(slack_weight);

  const int knot_idx = constraint_input.knot_idx;
  auto& raw_soft_constraint = (*soft_constraints)[knot_idx][constraint_level];
  if (!raw_soft_constraint.has_value() ||
      (dir_type == BoundDirType::UPPER
           ? constraint_input.bound < raw_soft_constraint->bound
           : constraint_input.bound > raw_soft_constraint->bound)) {
    raw_soft_constraint = {.knot_idx = knot_idx,
                           .slack_idx = slack_idx,
                           .bound = constraint_input.bound};
    (*slack_weight)[slack_idx][knot_idx] = constraint_input.weight * knot_num;
  }
}

std::vector<SoftConstraint> AssembleSoftConstraint(
    const std::vector<std::vector<std::optional<SoftConstraint>>>&
        soft_constraints) {
  std::vector<SoftConstraint> constraints;
  for (const auto& constraints_at_idx : soft_constraints) {
    for (const auto& constraint : constraints_at_idx) {
      if (constraint.has_value()) {
        constraints.push_back(*constraint);
      }
    }
  }
  return constraints;
}

std::vector<SlackWeight> AssembleSlackWeight(
    const std::map<int, std::map<int, double>>& slack_weight_map) {
  std::vector<SlackWeight> slack_weights;
  slack_weights.reserve(slack_weight_map.size());
  for (const auto& [slack_idx, weights] : slack_weight_map) {
    if (weights.empty()) continue;
    double sum_weight = 0.0;
    for (const auto& [_, weight] : weights) {
      sum_weight += weight;
    }
    slack_weights.push_back(
        {.slack_idx = slack_idx, .weight = sum_weight / weights.size()});
  }
  return slack_weights;
}

void FilterSoftConstraints(
    int knot_idx, BoundDirType dir,
    std::vector<std::vector<std::optional<SoftConstraint>>>* soft_constraints,
    std::map<int, std::map<int, double>>* slack_weight_map) {
  QCHECK_NOTNULL(soft_constraints);
  QCHECK_NOTNULL(slack_weight_map);
  auto& constraints_at_idx = (*soft_constraints)[knot_idx];
  for (int i = 0; i < constraints_at_idx.size(); ++i) {
    if (!constraints_at_idx[i].has_value()) continue;
    for (int j = 0; j < constraints_at_idx.size(); ++j) {
      if (i == j || !constraints_at_idx[j].has_value()) continue;
      const auto& ci = *constraints_at_idx[i];
      const auto& cj = *constraints_at_idx[j];
      const double wi = (*slack_weight_map)[ci.slack_idx][knot_idx];
      const double wj = (*slack_weight_map)[cj.slack_idx][knot_idx];
      if (dir == BoundDirType::UPPER) {
        if (ci.bound <= cj.bound && wi >= wj) {
          constraints_at_idx[j].reset();
          (*slack_weight_map)[cj.slack_idx].erase(knot_idx);
        }
      } else {
        if (ci.bound >= cj.bound && wi >= wj) {
          constraints_at_idx[j].reset();
          (*slack_weight_map)[cj.slack_idx].erase(knot_idx);
        }
      }
    }
  }
}

}  // namespace

SpeedOptimizerConstraintManager::SpeedOptimizerConstraintManager(
    std::vector<double> piecewise_time_range, double delta_t,
    int total_knot_num)
    : piecewise_time_range_(std::move(piecewise_time_range)),
      delta_t_(delta_t),
      total_knot_num_(total_knot_num) {
  QCHECK_GE(piecewise_time_range_.size(), 2);

  s_upper_data_.soft_constraints.resize(
      total_knot_num,
      std::vector<std::optional<SoftConstraint>>(kSoftSConstraintTypeNum));
  s_lower_data_.soft_constraints.resize(
      total_knot_num,
      std::vector<std::optional<SoftConstraint>>(kSoftSConstraintTypeNum));
  speed_upper_data_.soft_constraints.resize(
      total_knot_num,
      std::vector<std::optional<SoftConstraint>>(kSoftSpeedConstraintTypeNum));
  speed_lower_data_.soft_constraints.resize(
      total_knot_num,
      std::vector<std::optional<SoftConstraint>>(kSoftSpeedConstraintTypeNum));
}

void SpeedOptimizerConstraintManager::FilterObjectSConstraint(int knot_idx) {
  FilterSoftConstraints(knot_idx, BoundDirType::UPPER,
                        &s_upper_data_.soft_constraints,
                        &s_upper_data_.slack_weight_map);
  FilterSoftConstraints(knot_idx, BoundDirType::LOWER,
                        &s_lower_data_.soft_constraints,
                        &s_lower_data_.slack_weight_map);
}

void SpeedOptimizerConstraintManager::FilterObjectSpeedConstraint(
    int knot_idx) {
  FilterSoftConstraints(knot_idx, BoundDirType::UPPER,
                        &speed_upper_data_.soft_constraints,
                        &speed_upper_data_.slack_weight_map);
  FilterSoftConstraints(knot_idx, BoundDirType::LOWER,
                        &speed_lower_data_.soft_constraints,
                        &speed_lower_data_.slack_weight_map);
}

void SpeedOptimizerConstraintManager::AddSoftSUpperConstraint(
    BoundStrType strength_type, const SoftConstraintInput& constraint_input) {
  const int time_range_idx = GetTimeRangeIndex(constraint_input.knot_idx);
  const int knot_num = GetKnotNum(time_range_idx);
  const int slack_idx = ReturnAndUpdateSSlackIndex(
      time_range_idx, strength_type, &s_upper_data_.slack_idx_map,
      &s_upper_slack_idx_);
  UpdateSoftConstraints(constraint_input, static_cast<int>(strength_type),
                        slack_idx, BoundDirType::UPPER, knot_num,
                        &s_upper_data_.soft_constraints,
                        &s_upper_data_.slack_weight_map);
}

void SpeedOptimizerConstraintManager::AddSoftSLowerConstraint(
    BoundStrType strength_type, const SoftConstraintInput& constraint_input) {
  const int time_range_idx = GetTimeRangeIndex(constraint_input.knot_idx);
  const int knot_num = GetKnotNum(time_range_idx);
  const int slack_idx = ReturnAndUpdateSSlackIndex(
      time_range_idx, strength_type, &s_lower_data_.slack_idx_map,
      &s_lower_slack_idx_);
  UpdateSoftConstraints(constraint_input, static_cast<int>(strength_type),
                        slack_idx, BoundDirType::LOWER, knot_num,
                        &s_lower_data_.soft_constraints,
                        &s_lower_data_.slack_weight_map);
}

void SpeedOptimizerConstraintManager::AddHardSConstraint(int knot_idx,
                                                         double lower_bound,
                                                         double upper_bound) {
  hard_s_constraint_.push_back({.knot_idx = knot_idx,
                                .lower_bound = lower_bound,
                                .upper_bound = upper_bound});
}

void SpeedOptimizerConstraintManager::AddSoftSpeedLowerConstraints(
    SpeedLimitLevelProto::Level speed_limit_level,
    const SoftConstraintInput& constraint_input) {
  const int slack_idx = ReturnAndUpdateSpeedSlackIndex(
      speed_limit_level, &speed_lower_data_.slack_idx_map, &v_lower_slack_idx_);
  UpdateSoftConstraints(constraint_input, static_cast<int>(speed_limit_level),
                        slack_idx, BoundDirType::LOWER, total_knot_num_,
                        &speed_lower_data_.soft_constraints,
                        &speed_lower_data_.slack_weight_map);
}

void SpeedOptimizerConstraintManager::AddSoftSpeedUpperConstraints(
    SpeedLimitLevelProto::Level speed_limit_level,
    const SoftConstraintInput& constraint_input) {
  const int slack_idx = ReturnAndUpdateSpeedSlackIndex(
      speed_limit_level, &speed_upper_data_.slack_idx_map, &v_upper_slack_idx_);
  UpdateSoftConstraints(constraint_input, static_cast<int>(speed_limit_level),
                        slack_idx, BoundDirType::UPPER, total_knot_num_,
                        &speed_upper_data_.soft_constraints,
                        &speed_upper_data_.slack_weight_map);
}

void SpeedOptimizerConstraintManager::AddHardSpeedConstraint(
    int knot_idx, double lower_bound, double upper_bound) {
  hard_v_constraint_.push_back({.knot_idx = knot_idx,
                                .lower_bound = lower_bound,
                                .upper_bound = upper_bound});
}

void SpeedOptimizerConstraintManager::AddAccelConstraint(int knot_idx,
                                                         double lower_bound,
                                                         double upper_bound) {
  hard_a_constraint_.push_back({.knot_idx = knot_idx,
                                .lower_bound = lower_bound,
                                .upper_bound = upper_bound});
}

void SpeedOptimizerConstraintManager::AddAccelSoftUpperConstraint(
    const SoftConstraintInput& accel_constraint) {
  accel_upper_data_.soft_constraints.push_back(
      {.knot_idx = accel_constraint.knot_idx,
       .slack_idx = 0,
       .bound = accel_constraint.bound});
  accel_upper_data_.slack_weight_map[0] += accel_constraint.weight;
}

void SpeedOptimizerConstraintManager::AddAccelSoftLowerConstraint(
    const SoftConstraintInput& accel_constraint) {
  accel_lower_data_.soft_constraints.push_back(
      {.knot_idx = accel_constraint.knot_idx,
       .slack_idx = 0,
       .bound = accel_constraint.bound});
  accel_lower_data_.slack_weight_map[0] += accel_constraint.weight;
}

void SpeedOptimizerConstraintManager::AddJerkSoftUpperConstraint(
    const SoftConstraintInput& jerk_constraint) {
  jerk_upper_data_.soft_constraints.push_back(
      {.knot_idx = jerk_constraint.knot_idx,
       .slack_idx = 0,
       .bound = jerk_constraint.bound});
  jerk_upper_data_.slack_weight_map[0] += jerk_constraint.weight;
}

void SpeedOptimizerConstraintManager::AddJerkSoftLowerConstraint(
    const SoftConstraintInput& jerk_constraint) {
  jerk_lower_data_.soft_constraints.push_back(
      {.knot_idx = jerk_constraint.knot_idx,
       .slack_idx = 0,
       .bound = jerk_constraint.bound});
  jerk_lower_data_.slack_weight_map[0] += jerk_constraint.weight;
}

std::vector<SoftConstraint>
SpeedOptimizerConstraintManager::GetLowerConstraintOfSoftS() const {
  return AssembleSoftConstraint(s_lower_data_.soft_constraints);
}

std::vector<SoftConstraint>
SpeedOptimizerConstraintManager::GetUpperConstraintOfSoftS() const {
  return AssembleSoftConstraint(s_upper_data_.soft_constraints);
}

std::vector<SoftConstraint>
SpeedOptimizerConstraintManager::GetLowerConstraintOfSpeed() const {
  return AssembleSoftConstraint(speed_lower_data_.soft_constraints);
}

std::vector<SoftConstraint>
SpeedOptimizerConstraintManager::GetUpperConstraintOfSpeed() const {
  return AssembleSoftConstraint(speed_upper_data_.soft_constraints);
}

std::vector<SlackWeight>
SpeedOptimizerConstraintManager::GetLowerSlackWeightOfSoftS() const {
  return AssembleSlackWeight(s_lower_data_.slack_weight_map);
}

std::vector<SlackWeight>
SpeedOptimizerConstraintManager::GetUpperSlackWeightOfSoftS() const {
  return AssembleSlackWeight(s_upper_data_.slack_weight_map);
}

std::vector<SlackWeight>
SpeedOptimizerConstraintManager::GetLowerSlackWeightOfSpeed() const {
  return AssembleSlackWeight(speed_lower_data_.slack_weight_map);
}

std::vector<SlackWeight>
SpeedOptimizerConstraintManager::GetUpperSlackWeightOfSpeed() const {
  return AssembleSlackWeight(speed_upper_data_.slack_weight_map);
}

int SpeedOptimizerConstraintManager::GetTimeRangeIndex(int knot_idx) const {
  const double time = knot_idx * delta_t_;
  const int dist =
      std::distance(piecewise_time_range_.begin(),
                    std::lower_bound(piecewise_time_range_.begin(),
                                     piecewise_time_range_.end(), time));
  return std::clamp(dist - 1, 0,
                    static_cast<int>(piecewise_time_range_.size() - 2));
}

int SpeedOptimizerConstraintManager::GetKnotNum(int time_range_idx) const {
  return (piecewise_time_range_[time_range_idx + 1] -
          piecewise_time_range_[time_range_idx]) /
         delta_t_;
}

}  // namespace qcraft::planner

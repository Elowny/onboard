#include "onboard/planner/speed/speed_optimizer_object_manager.h"

// IWYU pragma: no_include <ext/alloc_traits.h>
//              for __alloc_traits<>::value_type
#include <algorithm>
#include <limits>
#include <map>
#include <optional>
#include <ostream>
#include <string>
#include <utility>

#include "onboard/global/buffered_logger.h"
#include "onboard/lite/check.h"
#include "onboard/math/piecewise_linear_function.h"
#include "onboard/math/util.h"
#include "onboard/planner/object/planner_object.h"
#include "onboard/planner/second_order_trajectory_point.h"
#include "onboard/planner/speed/proto/speed_finder.pb.h"
#include "onboard/planner/speed/proto/speed_finder_params.pb.h"
#include "onboard/planner/speed/speed_point.h"
#include "onboard/planner/speed/st_boundary.h"

namespace qcraft::planner {
namespace {
using ClassifiedObjects = std::vector<std::optional<SpeedOptimizerObject>>;

using ClassifiedOverlapState = std::vector<std::optional<ObjectOverlapState>>;

constexpr int kObjectTypeNum = 3;

const PiecewiseLinearFunction<double, double>
    kFollowDistanceLowestGainRelMinTPlf = {{2.0, 5.0}, {0.25, 0.1}};
const PiecewiseLinearFunction<double, double>
    kAvSpeedLeadDistanceLowestGainPlf = {{0.0, 6.0, 10.0}, {0.9, 0.6, 0.4}};
const PiecewiseLinearFunction<double, double> kDecayFollowTime = {
    {0.0, 5.0, 10.0}, {0.6, 0.4, 0.0}};
const PiecewiseLinearFunction<double, double>
    kFollowLonBufferTimeLowestGainPlf = {{5.0, 10.0}, {1.0, 0.8}};

void FillOverlapStateLonBuffer(
    const SpeedVector* preliminary_speed, double time,
    const std::vector<std::optional<double>>& standstills,
    double follow_time_headway,
    const PiecewiseLinearFunction<double>& follow_rel_speed_gain_plf,
    const PiecewiseLinearFunction<double, double>&
        follow_lon_buffer_time_gain_plf,
    double lead_time_headway,
    const PiecewiseLinearFunction<double>& lead_rel_speed_gain_plf,
    double av_speed, StBoundaryProto::ProtectionType protection_type,
    ClassifiedOverlapState* overlap_states) {
  QCHECK_NOTNULL(overlap_states);
  double predicted_av_speed = av_speed;
  const bool is_preliminary_speed_valid =
      preliminary_speed != nullptr && !preliminary_speed->empty();
  if (is_preliminary_speed_valid) {
    predicted_av_speed = preliminary_speed->EvaluateByTime(time)
                             .value_or(preliminary_speed->back())
                             .v();
  }
  const double decay_follow_time = kDecayFollowTime(predicted_av_speed);
  constexpr double kMinTimeHeadway = 0.5;  // s.
  const double final_follow_time_headway =
      std::max(kMinTimeHeadway, follow_time_headway - decay_follow_time);
  const double final_lead_time_headway =
      std::max(kMinTimeHeadway, lead_time_headway - decay_follow_time);
  for (int type = 0; type < overlap_states->size(); ++type) {
    auto& state = (*overlap_states)[type];
    if (!state.has_value()) {
      continue;
    }
    const double obj_speed = std::max(0.0, state->speed);
    const double follow_speed =
        is_preliminary_speed_valid ? predicted_av_speed : obj_speed;
    switch (type) {
      case SpeedOptimizerObjectType::MOVING_FOLLOW:
        QCHECK(
            standstills[SpeedOptimizerObjectType::MOVING_FOLLOW].has_value());
        state->lon_buffer =
            protection_type == StBoundaryProto::LARGE_VEHICLE_BLIND_SPOT
                ? *standstills[SpeedOptimizerObjectType::MOVING_FOLLOW]
                : (follow_speed * final_follow_time_headway *
                       follow_lon_buffer_time_gain_plf(time) +
                   *standstills[SpeedOptimizerObjectType::MOVING_FOLLOW]) *
                      follow_rel_speed_gain_plf(obj_speed - predicted_av_speed);
        break;
      case SpeedOptimizerObjectType::MOVING_LEAD:
        QCHECK(standstills[SpeedOptimizerObjectType::MOVING_LEAD].has_value());
        state->lon_buffer =
            protection_type == StBoundaryProto::LARGE_VEHICLE_BLIND_SPOT
                ? *standstills[SpeedOptimizerObjectType::MOVING_LEAD]
                : (obj_speed * final_lead_time_headway +
                   *standstills[SpeedOptimizerObjectType::MOVING_LEAD]) *
                      lead_rel_speed_gain_plf(predicted_av_speed - obj_speed);
        break;
      case SpeedOptimizerObjectType::STATIONARY:
        QCHECK(standstills[SpeedOptimizerObjectType::STATIONARY].has_value());
        constexpr double kSpeedMargin = 3.0;                   // m/s.
        constexpr double kFollowTimeHeadwayForStionary = 1.0;  // s.
        state->lon_buffer = *standstills[SpeedOptimizerObjectType::STATIONARY] +
                            std::max(0.0, follow_speed - kSpeedMargin) *
                                kFollowTimeHeadwayForStionary;
        break;
    }
  }
}

ClassifiedOverlapState IntegrateAndClassifyOverlapState(
    const std::vector<const StBoundaryWithDecision*>& st_boundaries,
    double time, double prediction_impact_factor) {
  // The lambda function to integrate overlap state.
  const auto integrate_overlap_state =
      [](const ObjectOverlapState& new_state,
         std::optional<ObjectOverlapState>* raw_state) {
        QCHECK_NOTNULL(raw_state);
        if (!raw_state->has_value()) *raw_state = ObjectOverlapState{};
        (*raw_state)->bound += new_state.bound * new_state.prob;
        (*raw_state)->speed += new_state.speed * new_state.prob;
        (*raw_state)->prob += new_state.prob;
      };

  ClassifiedOverlapState classified_overlap_states(kObjectTypeNum);
  double min_lower_bound = std::numeric_limits<double>::max();
  double max_obj_speed = 1e-6;  // Small positive eps to aviod nan.
  double min_obj_speed = std::numeric_limits<double>::max();
  for (const StBoundaryWithDecision* stb_wd : st_boundaries) {
    const StBoundary* stb = stb_wd->st_boundary();
    const auto s_range = stb->GetBoundarySRange(time);
    if (!s_range.has_value()) continue;
    double bound = 0.0;
    const auto decision = stb_wd->decision_type();
    if (decision == StBoundaryProto::FOLLOW) {
      bound = s_range->second;
      min_lower_bound = std::min(min_lower_bound, s_range->second);
    } else {
      bound = s_range->first;
    }
    const auto obj_speed = stb->GetStBoundarySpeedAtT(time);
    QCHECK(obj_speed.has_value());
    min_obj_speed = std::min(min_obj_speed, *obj_speed);
    max_obj_speed = std::max(max_obj_speed, *obj_speed);

    const ObjectOverlapState tmp_overlap_state = {
        .bound = bound, .speed = *obj_speed, .prob = stb->probability()};
    if (stb->is_stationary()) {
      integrate_overlap_state(
          tmp_overlap_state,
          &classified_overlap_states[SpeedOptimizerObjectType::STATIONARY]);
    } else if (decision == StBoundaryProto::FOLLOW) {
      integrate_overlap_state(
          tmp_overlap_state,
          &classified_overlap_states[SpeedOptimizerObjectType::MOVING_FOLLOW]);
    } else if (decision == StBoundaryProto::LEAD) {
      integrate_overlap_state(
          tmp_overlap_state,
          &classified_overlap_states[SpeedOptimizerObjectType::MOVING_LEAD]);
    }
  }

  // Normalize and adjust by `prediction_impact_factor`.
  for (int type = 0; type < classified_overlap_states.size(); ++type) {
    auto& state = classified_overlap_states[type];
    if (!state.has_value()) continue;
    const double prob = state->prob;
    QCHECK_GT(prob, 0.0);
    state->bound /= prob;
    state->speed /= prob;
    if (type == SpeedOptimizerObjectType::MOVING_FOLLOW ||
        type == SpeedOptimizerObjectType::STATIONARY) {
      const PiecewiseLinearFunction<double, double>
          prediction_impact_factor_plf = {{0.0, 1.0},
                                          {prediction_impact_factor, 1.0}};
      const double divergence_factor =
          (max_obj_speed - min_obj_speed) / max_obj_speed;
      const double lerp_factor =
          prediction_impact_factor_plf(divergence_factor);
      state->bound = Lerp(state->bound, min_lower_bound, lerp_factor);
    }
  }
  return classified_overlap_states;
}

ClassifiedObjects GenerateIntegratedClassifiedObjects(
    const std::vector<const StBoundaryWithDecision*>& st_boundaries_wd,
    const SpeedVector* preliminary_speed,
    const SpacetimeTrajectoryManager& traj_mgr, const std::string& id,
    double delta_t, double plan_total_time, double av_speed,
    const SpeedFinderParamsProto& speed_finder_params) {
  QCHECK(!st_boundaries_wd.empty());

  const auto protection_type =
      st_boundaries_wd[0]->st_boundary()->protection_type();

  double min_t = std::numeric_limits<double>::max();
  double max_t = -std::numeric_limits<double>::max();
  std::vector<std::optional<double>> standstills(kObjectTypeNum);
  for (const StBoundaryWithDecision* stb_wd : st_boundaries_wd) {
    const StBoundary* st_boundary = stb_wd->st_boundary();
    // Each protection type of st-boundary is handled individually.
    QCHECK_EQ(st_boundary->protection_type(), protection_type);
    min_t = std::min(min_t, st_boundary->min_t());
    max_t = std::max(max_t, st_boundary->max_t());
    if (st_boundary->is_stationary()) {
      standstills[SpeedOptimizerObjectType::STATIONARY] =
          stb_wd->follow_standstill_distance();
    } else if (stb_wd->decision_type() == StBoundaryProto::FOLLOW) {
      standstills[SpeedOptimizerObjectType::MOVING_FOLLOW] =
          stb_wd->follow_standstill_distance();
    } else if (stb_wd->decision_type() == StBoundaryProto::LEAD) {
      standstills[SpeedOptimizerObjectType::MOVING_LEAD] =
          stb_wd->lead_standstill_distance();
    }
  }

  // Generate rel_speed_gain_plf {rel_v, ratio}.
  PiecewiseLinearFunction<double> follow_rel_speed_gain_plf = {{-10.0, 10.0},
                                                               {1.0, 1.0}};
  {
    std::optional<double> obj_v = std::nullopt;
    double min_t = std::numeric_limits<double>::infinity();
    for (const StBoundaryWithDecision* stb_wd : st_boundaries_wd) {
      const StBoundary* st_boundary = stb_wd->st_boundary();
      if (stb_wd->decision_type() == StBoundaryProto::LEAD ||
          st_boundary->is_stationary()) {
        continue;
      }
      if (!obj_v.has_value()) {
        const auto& object_id = st_boundary->object_id();
        QCHECK(object_id.has_value());
        obj_v = QCHECK_NOTNULL(traj_mgr.FindObjectByObjectId(*object_id))
                    ->pose()
                    .v();
      }
      min_t = std::min(min_t, st_boundary->min_t());
    }
    if (obj_v.has_value()) {
      const PiecewiseLinearFunction<double, double>
          follow_rel_speed_lowest_gain_plf = {
              {0.0, 6.0, 10.0},
              {0.9, 0.6, kFollowDistanceLowestGainRelMinTPlf(min_t)}};
      follow_rel_speed_gain_plf = {
          /*x=*/{1.5, 4.0},
          /*y=*/{1.0, follow_rel_speed_lowest_gain_plf(*obj_v)}};
    }
  }
  PiecewiseLinearFunction<double> lead_rel_speed_gain_plf = {{-10.0, 10.0},
                                                             {1.0, 1.0}};
  for (const StBoundaryWithDecision* stb_wd : st_boundaries_wd) {
    const StBoundary* st_boundary = stb_wd->st_boundary();
    if (stb_wd->decision_type() != StBoundaryProto::LEAD) {
      continue;
    }
    const auto& object_id = st_boundary->object_id();
    QCHECK(object_id.has_value());
    const double obj_v =
        QCHECK_NOTNULL(traj_mgr.FindObjectByObjectId(*object_id))->pose().v();
    lead_rel_speed_gain_plf = {
        /*x=*/{1.5, 4.0},
        /*y=*/{1.0, kAvSpeedLeadDistanceLowestGainPlf(obj_v)}};
    break;
  }

  const auto& opt_params = speed_finder_params.speed_optimizer_params();
  const double lead_time_headway = speed_finder_params.lead_time_headway();
  const int knot_num = opt_params.knot_num();
  const double prediction_impact_factor = opt_params.prediction_impact_factor();

  std::vector<std::vector<std::optional<ObjectOverlapState>>>
      classified_entire_time_overlap_states(kObjectTypeNum);
  max_t = std::min(max_t, plan_total_time);
  const int start_idx = FloorToInt(min_t / delta_t);
  const int end_idx = FloorToInt(max_t / delta_t);
  const double follow_time_headway =
      st_boundaries_wd[0]->st_boundary()->is_large_vehicle()
          ? speed_finder_params.large_vehicle_follow_time_headway()
          : speed_finder_params.follow_time_headway();
  const PiecewiseLinearFunction<double, double>
      prediction_impact_factor_rel_time_plf = {{5.0, 10.0},  // time: s.
                                               {prediction_impact_factor, 0.5}};
  const PiecewiseLinearFunction<double, double>
      follow_lon_buffer_time_gain_plf = {
          {0.0, 3.0}, {kFollowLonBufferTimeLowestGainPlf(av_speed), 1.0}};
  for (int idx = start_idx; idx <= end_idx; ++idx) {
    const double time = idx * delta_t;
    const double final_prediction_impact_factor =
        opt_params.enable_prediction_impact_factor_decay()
            ? prediction_impact_factor_rel_time_plf(time)
            : prediction_impact_factor;
    auto overlap_states_at_t = IntegrateAndClassifyOverlapState(
        st_boundaries_wd, time, final_prediction_impact_factor);
    // Fill lon buffer.
    FillOverlapStateLonBuffer(preliminary_speed, time, standstills,
                              follow_time_headway, follow_rel_speed_gain_plf,
                              follow_lon_buffer_time_gain_plf,
                              lead_time_headway, lead_rel_speed_gain_plf,
                              av_speed, protection_type, &overlap_states_at_t);
    QCHECK_LT(idx, knot_num);
    for (int type = 0; type < overlap_states_at_t.size(); ++type) {
      auto state = overlap_states_at_t[type];
      if (state.has_value()) {
        auto& entire_time_states = classified_entire_time_overlap_states[type];
        if (entire_time_states.empty()) entire_time_states.resize(knot_num);
        entire_time_states[idx] = state;
      }
    }
  }

  // Construct speed optimizer objects.
  const auto* stb = st_boundaries_wd.front()->st_boundary();
  const auto object_type = stb->object_type();
  const auto source_type = stb->source_type();
  ClassifiedObjects classified_objects(kObjectTypeNum);
  for (int type = 0; type < classified_objects.size(); ++type) {
    auto& states = classified_entire_time_overlap_states[type];
    if (!states.empty()) {
      const auto standstill = standstills[type];
      QCHECK(standstill.has_value());
      classified_objects[type] =
          SpeedOptimizerObject(std::move(states), delta_t, id, *standstill,
                               object_type, source_type, protection_type);
    }
  }
  return classified_objects;
}

}  // namespace

SpeedOptimizerObjectManager::SpeedOptimizerObjectManager(
    absl::Span<const StBoundaryWithDecision> st_boundaries_with_decision,
    const SpeedVector* preliminary_speed,
    const SpacetimeTrajectoryManager& traj_mgr, double av_speed,
    double plan_total_time, double plan_time_interval,
    const SpeedFinderParamsProto& speed_finder_params) {
  std::map<std::string, std::vector<const StBoundaryWithDecision*>>
      st_boundary_map;
  for (const StBoundaryWithDecision& stb_wd : st_boundaries_with_decision) {
    if (stb_wd.decision_type() == StBoundaryProto::UNKNOWN ||
        stb_wd.decision_type() == StBoundaryProto::IGNORE) {
      continue;
    }
    const auto id = GetStBoundaryIntegrationId(*stb_wd.st_boundary());
    st_boundary_map[id].push_back(&stb_wd);
  }

  objects_.resize(kObjectTypeNum);
  for (const auto& [id, st_boundaries] : st_boundary_map) {
    auto classified_objects = GenerateIntegratedClassifiedObjects(
        st_boundaries, preliminary_speed, traj_mgr, id, plan_time_interval,
        plan_total_time, av_speed, speed_finder_params);
    for (int type = 0; type < classified_objects.size(); ++type) {
      auto object = classified_objects[type];
      if (object.has_value()) {
        objects_[type].push_back(std::move(*object));
      }
    }
  }
}

}  // namespace qcraft::planner

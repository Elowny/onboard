#include "onboard/planner/decision/traffic_light/traffic_light_info_collector.h"

#include <algorithm>
#include <map>
#include <optional>
#include <ostream>
#include <utility>
#include <vector>

#include "absl/meta/type_traits.h"

#include "onboard/global/logging.h"
#include "onboard/lite/proto/lite_common.pb.h"
#include "onboard/maps/map_or_die_macros.h"
#include "onboard/maps/maps_common.h"
#include "onboard/maps/proto/semantic_map.pb.h"
#include "onboard/math/piecewise_linear_function.h"

namespace qcraft::planner {

namespace {

constexpr double kMaxDistanceForCollectTrafficLightInfo = 200.0;  // Meter.

// Extract traffic light data from perception.
struct TrafficLightData {
  TrafficLightState state = TrafficLightState::TL_STATE_UNKNOWN;
  std::optional<double> countdown = std::nullopt;
};

using TrafficLightDataMap = std::map<mapping::ElementId, TrafficLightData>;

TrafficLightState TrafficLightStateProto2TlState(
    const TrafficLightStateProto& proto) {
  if (proto.color() == TL_UNKNOWN) return TrafficLightState::TL_STATE_UNKNOWN;
  if (proto.color() == TL_RED) return TrafficLightState::TL_STATE_RED;
  if (proto.color() == TL_GREEN && !proto.flashing()) {
    return TrafficLightState::TL_STATE_GREEN;
  }
  if (proto.color() == TL_GREEN && proto.flashing()) {
    return TrafficLightState::TL_STATE_GREEN_FLASHING;
  }
  if (proto.color() == TL_YELLOW && !proto.flashing()) {
    return TrafficLightState::TL_STATE_YELLOW;
  }
  if (proto.color() == TL_YELLOW && proto.flashing()) {
    return TrafficLightState::TL_STATE_YELLOW_FLASHING;
  }
  return TrafficLightState::TL_STATE_UNKNOWN;
}

std::optional<double> GetTrafficLightCountDown(
    const TrafficLightStateProto& proto) {
  if (proto.has_countdown_int() == false) return std::nullopt;
  return static_cast<double>(proto.countdown_int());
}

// Key: traffic light id. Value: traffic light data.
TrafficLightDataMap CollectTrafficLightDataFromPerception(
    const TrafficLightStatesProto& tl_states_from_perception,
    bool tl_is_fresh) {
  TrafficLightDataMap tl_data_map;
  for (const auto& tl_state_perception : tl_states_from_perception.states()) {
    const auto tl_id =
        mapping::ElementId(tl_state_perception.traffic_light_id());
    if (tl_data_map.find(tl_id) != tl_data_map.end()) {
      continue;
    }

    const auto tl_state = TrafficLightStateProto2TlState(tl_state_perception);
    const auto tl_countdown = GetTrafficLightCountDown(tl_state_perception);

    // Regard all tl state is UNKNOWN when perception tl state is stale.
    if (tl_is_fresh) {
      tl_data_map.emplace(tl_id, TrafficLightData{tl_state, tl_countdown});
    } else {
      tl_data_map.emplace(
          tl_id,
          TrafficLightData{TrafficLightState::TL_STATE_UNKNOWN, std::nullopt});
    }
  }
  return tl_data_map;
}

// Update yellow light observations.
void UpdateYellowLightObservation(const TrafficLightDataMap& tl_data_map,
                                  absl::Time plan_time,
                                  YellowLightObservationsNew* observations) {
  for (const auto [tl_id, data] : tl_data_map) {
    const auto tl_state = data.state;
    if (tl_state != TrafficLightState::TL_STATE_YELLOW &&
        tl_state != TrafficLightState::TL_STATE_GREEN_FLASHING) {
      observations->erase(tl_id);
      continue;
    }
    auto iter = observations->find(tl_id);
    if (iter == observations->end()) {
      YellowLightObservationNew observation;
      if (tl_state == TrafficLightState::TL_STATE_YELLOW) {
        observation.first_flashing_green_observation = plan_time;
        observation.latest_flashing_green_observation = plan_time;
        observation.first_yellow_observation = plan_time;
        observation.latest_yellow_observation = plan_time;
      } else {
        observation.first_flashing_green_observation = plan_time;
        observation.latest_flashing_green_observation = plan_time;
        observation.first_yellow_observation = kMaxTime;
        observation.latest_yellow_observation = kMaxTime;
      }
      observations->emplace(tl_id, observation);
    } else {
      if (tl_state == TrafficLightState::TL_STATE_YELLOW) {
        iter->second.first_yellow_observation =
            std::min(iter->second.first_yellow_observation, plan_time);
        iter->second.latest_yellow_observation =
            std::min(iter->second.latest_yellow_observation, plan_time);
      }
      constexpr absl::Duration kYellowLightObservationResetThreshold =
          absl::Seconds(30);
      if (iter->second.first_flashing_green_observation <
              plan_time - kYellowLightObservationResetThreshold ||
          iter->second.first_yellow_observation <
              plan_time - kYellowLightObservationResetThreshold) {
        // First observation too old, could be from the previous
        // encounter with this light. Reset.

        if (tl_state == TrafficLightState::TL_STATE_YELLOW) {
          iter->second.first_yellow_observation = plan_time;
        } else {
          iter->second.first_flashing_green_observation = plan_time;
        }
      }
      if (tl_state == TrafficLightState::TL_STATE_YELLOW) {
        iter->second.latest_yellow_observation = plan_time;
      } else {
        iter->second.latest_flashing_green_observation = plan_time;
      }
    }
  }
}

// TODO(jiayu): Calculate traffic light turn red time left.
double CalculateTurnRedTImeLeft(
    absl::Time plan_time, mapping::ElementId tl_id, double speed_limit_mph,
    TrafficLightState tl_state,
    const YellowLightObservationsNew& observations) {
  if (tl_state != TrafficLightState::TL_STATE_YELLOW &&
      tl_state != TrafficLightState::TL_STATE_GREEN_FLASHING) {
    return 0.0;
  }

  auto iter = observations.find(tl_id);
  if (iter == observations.end()) return 0.0;

  // CVC 21455.7.
  // CA Manual on Uniform Traffic Control Devices 2014 rev 4.
  // https://dot.ca.gov/programs/traffic-operations/camutcd/camutcd-rev4
  // Section 4D-26 and Table 4D-102b.
  //
  // Road speed (mph) to minimum yellow light duration in seconds.
  // Determination of the speed to use according to the CA MUTCD is
  // non-trivial and relies on traffic data or heuristics. The estimation
  // below uses speed limit which is conservative when traffic data is not
  // available.
  const PiecewiseLinearFunction<double>
      min_yellow_light_duration_plf_without_flashing_green(
          {15.0, 20.0, 25.0, 30.0, 35.0, 40.0, 45.0, 50.0, 55.0, 60.0},
          {3.0, 3.2, 3.6, 3.7, 4.1, 4.4, 4.8, 5.2, 5.5, 5.9});
  const PiecewiseLinearFunction<double>
      min_yellow_light_duration_plf_with_flashing_green(
          {15.0, 20.0, 25.0, 30.0, 35.0, 40.0, 45.0, 50.0, 55.0, 60.0},
          {5.0, 5.3, 5.6, 6.0, 6.0, 6.0, 6.0, 6.0, 6.0, 6.0});
  // A correction on the yellow light duration based on our observation of
  // shorter-than-regulation yellow lights in reality. E.g. Issue #428.
  const PiecewiseLinearFunction<double> yellow_light_duration_correction_plf(
      {15.0, 20.0, 25.0, 30.0, 35.0, 40.0, 45.0, 50.0, 55.0, 60.0},
      {0.0, 0.2, 0.2, 0.2, 0.4, 0.4, 0.4, 0.4, 0.4, 0.4});
  // A delay corresponding to camera triggering frequency and TL
  // classification temporal voting latency.
  constexpr double kTlDetectionDelay = 0.5;  // s.

  // Estimate the time till red using the flashing green timing.
  const double flashing_green_observed_duration = absl::ToDoubleSeconds(
      plan_time - iter->second.first_flashing_green_observation);
  const double expected_yellow_light_duration_with_flashing_green =
      min_yellow_light_duration_plf_with_flashing_green(speed_limit_mph) -
      yellow_light_duration_correction_plf(speed_limit_mph) - kTlDetectionDelay;
  const double estimated_turn_red_time_left_from_flashing_green =
      expected_yellow_light_duration_with_flashing_green -
      flashing_green_observed_duration;

  // Estimate the time till red using the yellow timing.
  const double yellow_observed_duration =
      absl::ToDoubleSeconds(plan_time - iter->second.first_yellow_observation);
  const double expected_yellow_light_duration_without_flashing_green =
      min_yellow_light_duration_plf_without_flashing_green(speed_limit_mph) -
      yellow_light_duration_correction_plf(speed_limit_mph) - kTlDetectionDelay;
  const double estimated_turn_red_time_left_from_yellow =
      expected_yellow_light_duration_without_flashing_green -
      yellow_observed_duration;

  // Take the smaller of the two estimated time till red values (the
  // more conservative estimate).

  return std::max(0.0,
                  std::min(estimated_turn_red_time_left_from_flashing_green,
                           estimated_turn_red_time_left_from_yellow));
}

SingleTlInfo CreateSingleTlInfo(const std::vector<mapping::ElementId>& tl_ids,
                                const TrafficLightDataMap& tl_data_map,
                                const YellowLightObservationsNew& observations,
                                absl::Time plan_time, double speed_limit_mph) {
  for (const auto tl_id : tl_ids) {
    auto iter = tl_data_map.find(tl_id);

    if (iter == tl_data_map.end()) continue;

    return SingleTlInfo{
        .tl_id = tl_id,
        .tl_state = iter->second.state,
        .estimated_turn_red_time_left =
            CalculateTurnRedTImeLeft(plan_time, tl_id, speed_limit_mph,
                                     iter->second.state, observations),
        .countdown = iter->second.countdown};
  }

  // There is no valid tl state find from perception, just return UNKNOWN
  // state.
  return SingleTlInfo{.tl_id = tl_ids.front(),
                      .tl_state = TrafficLightState::TL_STATE_UNKNOWN,
                      .estimated_turn_red_time_left = 0.0,
                      .countdown = std ::nullopt};
}

std::vector<mapping::ElementId> CollectAllOutgoingLaneIds(
    const PlannerSemanticMapManager& psmm,
    const std::vector<mapping::ElementId>& lane_ids) {
  std::vector<mapping::ElementId> outgoing_lane_ids;
  for (const auto id : lane_ids) {
    const auto* lane_info_ptr = psmm.FindLaneInfoOrNull(id);
    if (lane_info_ptr == nullptr) continue;
    for (const auto id : lane_info_ptr->outgoing_lanes()) {
      outgoing_lane_ids.push_back(id);
    }
  }
  return outgoing_lane_ids;
}

// Generate TlInfo and associate with related lane.
std::map<mapping::ElementId, TlInfo> CollectTrafficLightInfo(
    const PlannerSemanticMapManager& psmm,
    const TrafficLightDataMap& tl_data_map, const RouteSections& route_sections,
    const YellowLightObservationsNew& observations, bool tl_is_fresh,
    absl::Time plan_time) {
  std::map<mapping::ElementId, TlInfo> tl_info_map;

  const int n = route_sections.size();
  double accumulate_s = 0.0;
  for (int i = 0; i < n; ++i) {
    if (accumulate_s > kMaxDistanceForCollectTrafficLightInfo) {
      break;
    }

    // Generate tl info foreach lane which controlled by tl.
    const auto& section_segment = route_sections.route_section_segment(i);
    SMM_ASSIGN_SECTION_OR_CONTINUE(section_info, psmm,
                                   mapping::SectionId(section_segment.id));
    std::vector<mapping::ElementId> queried_lane_ids;
    if (i == 0) {
      queried_lane_ids = section_info.lane_ids;
    } else {
      SMM_ASSIGN_SECTION_OR_CONTINUE(
          last_section, psmm,
          mapping::SectionId(route_sections.section_id(i - 1)));
      queried_lane_ids = CollectAllOutgoingLaneIds(psmm, last_section.lane_ids);
    }
    for (const auto lane_id : queried_lane_ids) {
      const auto* lane_proto = psmm.FindLaneByIdOrNull(lane_id);
      if (lane_proto == nullptr) continue;

      const auto& start_point_control_tl =
          lane_proto->startpoint_associated_traffic_lights();
      const auto& multi_tl_control_points =
          lane_proto->multi_traffic_light_control_points();

      // This lane isn't controlled by traffic light.
      if (start_point_control_tl.empty() && multi_tl_control_points.empty()) {
        continue;
      }

      const bool can_go_on_red =
          lane_proto->has_can_go_on_red() && lane_proto->can_go_on_red();

      const auto* lane_info_ptr = psmm.FindLaneInfoOrNull(lane_id);
      if (lane_info_ptr == nullptr) continue;
      if (!start_point_control_tl.empty()) {
        // Create single_tl_info and add to tl_direction_map.
        std::vector<mapping::ElementId> tl_ids;
        for (const auto tl_id : start_point_control_tl) {
          tl_ids.push_back(mapping::ElementId(tl_id));
        }
        absl::flat_hash_map<TrafficLightDirection, SingleTlInfo>
            tl_direction_map;
        tl_direction_map.emplace(
            TrafficLightDirection::UNMARKED,
            CreateSingleTlInfo(tl_ids, tl_data_map, observations, plan_time,
                               lane_proto->speed_limit_mph()));

        // Add tl_info to tl_info_map
        const double start_point_control_s =
            accumulate_s +
            lane_info_ptr->length() * (0.0 - section_segment.start_fraction);
        tl_info_map.emplace(
            lane_id, TlInfo(lane_id, {start_point_control_s}, can_go_on_red,
                            std::move(tl_direction_map), tl_is_fresh,
                            /*last_error_msg*/ "", lane_proto->direction()));

      } else {
        absl::flat_hash_map<TrafficLightDirection, SingleTlInfo>
            tl_direction_map;
        std::vector<double> control_points_relative_s;

        for (const auto& multi_tl_control_point : multi_tl_control_points) {
          const double control_point_relative_s =
              accumulate_s + lane_info_ptr->length() *
                                 (multi_tl_control_point.lane_fraction() -
                                  section_segment.start_fraction);
          control_points_relative_s.push_back(control_point_relative_s);

          // Create straight direction traffic light info.
          if (tl_direction_map.find(TrafficLightDirection::STRAIGHT) ==
              tl_direction_map.end()) {
            std::vector<mapping::ElementId> straight_tl_ids;
            for (const auto tl_id : multi_tl_control_point.straight_tls()) {
              straight_tl_ids.push_back(mapping::ElementId(tl_id));
            }
            tl_direction_map.emplace(
                TrafficLightDirection::STRAIGHT,
                CreateSingleTlInfo(straight_tl_ids, tl_data_map, observations,
                                   plan_time, lane_proto->speed_limit_mph()));
          }

          // Create Left direction traffic light info.
          if (tl_direction_map.find(TrafficLightDirection::LEFT) ==
              tl_direction_map.end()) {
            std::vector<mapping::ElementId> left_tl_ids;
            for (const auto tl_id : multi_tl_control_point.left_tls()) {
              left_tl_ids.push_back(mapping::ElementId(tl_id));
            }
            tl_direction_map.emplace(
                TrafficLightDirection::LEFT,
                CreateSingleTlInfo(left_tl_ids, tl_data_map, observations,
                                   plan_time, lane_proto->speed_limit_mph()));
          }
        }
        std::sort(control_points_relative_s.begin(),
                  control_points_relative_s.end());

        tl_info_map.emplace(
            lane_id, TlInfo(lane_id, control_points_relative_s, can_go_on_red,
                            tl_direction_map, tl_is_fresh,
                            /*last_error_msg*/ "", lane_proto->direction()));
      }
    }
    accumulate_s +=
        (section_segment.end_fraction - section_segment.start_fraction) *
        section_info.average_length;
  }

  return tl_info_map;
}

}  // namespace

void YellowLightObservationNew::FromProto(
    const YellowLightObservationProto& proto) {
  first_flashing_green_observation =
      qcraft::FromProto(proto.first_flashing_green_observation());
  latest_flashing_green_observation =
      qcraft::FromProto(proto.latest_flashing_green_observation());
  first_yellow_observation =
      qcraft::FromProto(proto.first_yellow_observation());
  latest_yellow_observation =
      qcraft::FromProto(proto.latest_yellow_observation());
}

void YellowLightObservationNew::ToProto(
    YellowLightObservationProto* proto) const {
  qcraft::ToProto(first_flashing_green_observation,
                  proto->mutable_first_flashing_green_observation());
  qcraft::ToProto(latest_flashing_green_observation,
                  proto->mutable_latest_flashing_green_observation());
  qcraft::ToProto(first_yellow_observation,
                  proto->mutable_first_yellow_observation());
  qcraft::ToProto(latest_yellow_observation,
                  proto->mutable_latest_yellow_observation());
}

bool YellowLightObservationNew::operator==(
    const YellowLightObservationNew& o) const {
  return this->first_flashing_green_observation ==
             o.first_flashing_green_observation &&
         this->latest_flashing_green_observation ==
             o.latest_flashing_green_observation &&
         this->first_yellow_observation == o.first_yellow_observation &&
         this->latest_yellow_observation == o.latest_yellow_observation;
}

absl::StatusOr<TrafficLightInfoCollectorOutput> CollectTrafficLightInfo(
    const TrafficLightInfoCollectorInput& input,
    YellowLightObservationsNew yellow_light_observations) {
  const auto& psmm = *input.psmm;
  const auto& tl_states_from_perception = *input.traffic_light_states;
  const auto& route_sections = *input.route_sections;

  // Check whether perception traffic light message is fresh.
  constexpr absl::Duration kTrafficLightStatesStaleThreshold = absl::Seconds(1);
  const absl::Time tl_states_time =
      absl::FromUnixMicros(tl_states_from_perception.header().timestamp());
  const bool tl_is_fresh =
      input.plan_time - tl_states_time < kTrafficLightStatesStaleThreshold;

  // Collect traffic light state and count down from perception.
  const auto tl_data_map = CollectTrafficLightDataFromPerception(
      tl_states_from_perception, tl_is_fresh);

  // Refresh YellowLightObservation cache.
  UpdateYellowLightObservation(tl_data_map, input.plan_time,
                               &yellow_light_observations);

  // Generate tl_info_map based on tl_data_map and route_sections.
  auto tl_info_map = CollectTrafficLightInfo(psmm, tl_data_map, route_sections,
                                             yellow_light_observations,
                                             tl_is_fresh, input.plan_time);

  return TrafficLightInfoCollectorOutput{
      .tl_info_map = std::move(tl_info_map),
      .yellow_light_observations = std::move(yellow_light_observations)};
}
}  // namespace qcraft::planner

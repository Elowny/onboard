#ifndef ONBOARD_PLANNER_DECISION_TRAFFIC_LIGHT_INFO_COLLECTOR_H_
#define ONBOARD_PLANNER_DECISION_TRAFFIC_LIGHT_INFO_COLLECTOR_H_

#include "absl/container/flat_hash_map.h"
#include "absl/hash/hash.h"
#include "absl/status/statusor.h"
#include "absl/time/time.h"

#include "onboard/maps/semantic_map_defs.h"
#include "onboard/planner/decision/proto/traffic_light_info.pb.h"
#include "onboard/planner/decision/traffic_light/tl_info.h"
#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/planner/router/route_sections.h"
#include "onboard/proto/perception.pb.h"
#include "onboard/utils/time_util.h"

namespace qcraft::planner {
// TODO(jiayu): Rename this struct after delete traffic light state voter.
struct YellowLightObservationNew {
  absl::Time first_flashing_green_observation = kMaxTime;
  absl::Time latest_flashing_green_observation = kMaxTime;
  absl::Time first_yellow_observation = kMaxTime;
  absl::Time latest_yellow_observation = kMaxTime;

  void FromProto(const YellowLightObservationProto& proto);
  void ToProto(YellowLightObservationProto* proto) const;
  bool operator==(const YellowLightObservationNew& observation) const;
};

// TODO(jiayu): Rename this abbreviation after delete traffic light state voter.
using YellowLightObservationsNew =
    absl::flat_hash_map<mapping::ElementId, YellowLightObservationNew>;

struct TrafficLightInfoCollectorInput {
  const PlannerSemanticMapManager* psmm = nullptr;
  const TrafficLightStatesProto* traffic_light_states = nullptr;
  // Rout sections from av current start point.
  const RouteSections* route_sections = nullptr;
  absl::Time plan_time;
};

struct TrafficLightInfoCollectorOutput {
  TrafficLightInfoMap tl_info_map;
  YellowLightObservationsNew yellow_light_observations;
};

absl::StatusOr<TrafficLightInfoCollectorOutput> CollectTrafficLightInfo(
    const TrafficLightInfoCollectorInput& input,
    YellowLightObservationsNew yellow_light_observations);
}  // namespace qcraft::planner

#endif

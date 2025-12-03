#ifndef ONBOARD_PLANNER_DECISION_TRAFFIC_LIGHT_DECIDER_H_
#define ONBOARD_PLANNER_DECISION_TRAFFIC_LIGHT_DECIDER_H_

#include <vector>

#include "absl/status/statusor.h"

#include "onboard/maps/lane_path.h"
#include "onboard/planner/common/speed_profile.h"
#include "onboard/planner/decision/proto/constraint.pb.h"
#include "onboard/planner/decision/traffic_light/tl_info.h"
#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/planner/router/drive_passage.h"
#include "onboard/proto/trajectory_point.pb.h"
#include "onboard/proto/vehicle.pb.h"

namespace qcraft {
namespace planner {

struct TrafficLightDeciderOutput {
  std::vector<ConstraintProto::StopLineProto> stop_lines;
  std::vector<ConstraintProto::SpeedProfileProto> speed_profiles;
  TrafficLightDeciderStateProto traffic_light_decider_state;
};

absl::StatusOr<TrafficLightDeciderOutput> BuildTrafficLightConstraints(
    const PlannerSemanticMapManager& psmm,
    const qcraft::VehicleGeometryParamsProto& vehicle_geometry_params,
    const ApolloTrajectoryPointProto& plan_start_point,
    const DrivePassage& passage, const mapping::LanePath& lane_path_from_start,
    double s_offset, const TrafficLightInfoMap& tl_info_map,
    const SpeedProfile& preliminary_speed_profile,
    const TrafficLightDeciderStateProto& decider_state);

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_DECISION_TRAFFIC_LIGHT_DECIDER_H_

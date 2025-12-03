#ifndef ONBOARD_PLANNER_DECISION_CROSSWALK_DECIDER_H_
#define ONBOARD_PLANNER_DECISION_CROSSWALK_DECIDER_H_

#include <vector>

#include "absl/status/statusor.h"
#include "absl/types/span.h"
// IWYU pragma: no_include <google/protobuf/repeated_ptr_field.h>

#include "onboard/maps/lane_path.h"
#include "onboard/maps/proto/semantic_map.pb.h"
#include "onboard/planner/decision/proto/constraint.pb.h"
#include "onboard/planner/decision/proto/crosswalk_state.pb.h"
#include "onboard/planner/object/planner_object_manager.h"
#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/planner/router/drive_passage.h"
#include "onboard/proto/trajectory_point.pb.h"
#include "onboard/proto/vehicle.pb.h"

namespace qcraft {
namespace planner {

struct CrosswalkDeciderOutput {
  std::vector<ConstraintProto::StopLineProto> stop_lines;
  std::vector<ConstraintProto::SpeedRegionProto> speed_regions;
  std::vector<CrosswalkStateProto> crosswalk_states;
};

struct CrosswalkDeciderInput {
  const qcraft::VehicleGeometryParamsProto* vehicle_geometry_params = nullptr;
  const PlannerSemanticMapManager* psmm = nullptr;
  const ApolloTrajectoryPointProto* plan_start_point = nullptr;
  const DrivePassage* passage = nullptr;
  const mapping::LanePath* lane_path_from_start = nullptr;
  const PlannerObjectManager* obj_mgr = nullptr;
  const ::google::protobuf::RepeatedPtrField<CrosswalkStateProto>*
      last_crosswalk_states = nullptr;
  absl::Span<const mapping::CrosswalkProto::Type> valid_cw_types;
  double now_in_seconds;
  double s_offset;
};

absl::StatusOr<CrosswalkDeciderOutput> BuildCrosswalkConstraints(
    const CrosswalkDeciderInput& input);
}  // namespace planner
}  // namespace qcraft
#endif

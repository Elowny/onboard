
#ifndef ONBOARD_PLANNER_DECISION_STOP_POLYLINE_DECIDER_H_
#define ONBOARD_PLANNER_DECISION_STOP_POLYLINE_DECIDER_H_

#include <vector>

#include "absl/status/statusor.h"

#include "onboard/math/frenet_common.h"
#include "onboard/planner/decision/proto/constraint.pb.h"
#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/planner/router/drive_passage.h"

namespace qcraft::planner {

struct StopPolylineDeciderOutput {
  std::vector<ConstraintProto::StopLineProto> stop_lines;
  StopPolylineDeciderStateProto stop_polyline_state;
};

absl::StatusOr<StopPolylineDeciderOutput> BuildStopPolylineConstraints(
    const PlannerSemanticMapManager& psmm, const DrivePassage& passage,
    const FrenetBox& ego_frenet_box,
    const StopPolylineDeciderStateProto& decider_state,
    bool is_pass_nearest_stop_line);
}  // namespace qcraft::planner
#endif

#ifndef ONBOARD_PLANNER_PLANNER_STATE_UTIL_H_
#define ONBOARD_PLANNER_PLANNER_STATE_UTIL_H_

#include "absl/status/statusor.h"

#include "onboard/maps/v2/semantic_map_manager.h"
#include "onboard/planner/assist/proto/assist_plan_state.pb.h"
#include "onboard/planner/planner_input.h"
#include "onboard/planner/planner_state.h"
#include "onboard/proto/route.pb.h"

namespace qcraft {
namespace planner {

absl::StatusOr<PlannerState> RecoverPlannerStateFromProto(
    const PlannerInput& input, bool recover_async_state);

void ResetAlccAssistPlanState(AssistPlanStateProto* assist_plan_state);

void ResetAccAssistPlanState(AssistPlanStateProto* assist_plan_state);

PlannerState::HdMapState ObtainHdMapState(
    const mapping::v2::SemanticMapManager& smm,
    const RouteSectionSequenceProto& section_seq_proto);

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_PLANNER_STATE_UTIL_H_

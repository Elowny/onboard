#ifndef ONBOARD_PLANNER_DECISION_DECISION_UTIL_H_
#define ONBOARD_PLANNER_DECISION_DECISION_UTIL_H_

#include <utility>

#include "absl/types/span.h"

#include "onboard/maps/proto/semantic_map.pb.h"
#include "onboard/math/frenet_common.h"
#include "onboard/planner/common/path_sl_boundary.h"
#include "onboard/planner/common/speed_profile.h"
#include "onboard/planner/decision/constraint_manager.h"
#include "onboard/planner/decision/proto/constraint.pb.h"
#include "onboard/planner/object/spacetime_object_trajectory.h"
#include "onboard/planner/router/drive_passage.h"
#include "onboard/proto/perception/fusion/object.pb.h"

namespace qcraft {
namespace planner {

SpeedProfile CreateSpeedProfile(
    double v_now, const DrivePassage& passage,
    const absl::Span<const ConstraintProto::SpeedRegionProto>& speed_zones,
    const absl::Span<const ConstraintProto::StopLineProto>& stop_points);

ConstraintProto::SpeedRegionProto MergeSameElement(
    absl::Span<const ConstraintProto::SpeedRegionProto> elements);

void FillDecisionConstraintDebugInfo(const ConstraintManager& constraint_mgr,
                                     ConstraintProto* constraint);

bool IsLeadingObjectType(ObjectType type);

std::pair<double, double> CalcSlBoundaries(const PathSlBoundary& sl_boundary,
                                           const FrenetBox& frenet_box);

ConstraintProto::LeadingObjectProto CreateLeadingObject(
    const SpacetimeObjectTrajectory& traj, const DrivePassage& passage,
    ConstraintProto::LeadingObjectProto::Reason reason);

bool IsTrafficLightControlledLane(const mapping::LaneProto& lane);
// TODO(jiayu): implement this function:
// ConstraintProto::LeadingObjectProto ProjectTrajectorySTOnDrivePassage()

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_DECISION_DECISION_UTIL_H_

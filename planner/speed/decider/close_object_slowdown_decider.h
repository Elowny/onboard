#ifndef ONBOARD_PLANNER_SPEED_DECIDER_CLOSE_OBJECT_SLOWDOWN_DECIDER_H_
#define ONBOARD_PLANNER_SPEED_DECIDER_CLOSE_OBJECT_SLOWDOWN_DECIDER_H_

#include <vector>

#include "onboard/planner/common/discretized_path.h"
#include "onboard/planner/common/path_sl_boundary.h"
#include "onboard/planner/decision/proto/constraint.pb.h"
#include "onboard/planner/router/drive_passage.h"
#include "onboard/planner/speed/st_graph_defs.h"

namespace qcraft {
namespace planner {

std::vector<ConstraintProto::PathSpeedRegionProto>
MakeCloseObjectSlowdownDecision(
    const std::vector<CloseSpaceTimeObject>& close_space_time_objects,
    const DrivePassage& drive_passage, const DiscretizedPath& path_points,
    double av_speed, const PathSlBoundary& path_sl_boundary);

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_SPEED_DECIDER_CLOSE_OBJECT_SLOWDOWN_DECIDER_H_

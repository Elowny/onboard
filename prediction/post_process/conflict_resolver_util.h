#ifndef ONBOARD_PREDICTION_POST_PROCESS_CONFLICT_RESOLVER_UTIL_H_
#define ONBOARD_PREDICTION_POST_PROCESS_CONFLICT_RESOLVER_UTIL_H_

#include <string>
#include <utility>
#include <vector>

#include "absl/types/span.h"

#include "onboard/planner/common/discretized_path.h"
#include "onboard/planner/speed/speed_point.h"
#include "onboard/planner/speed/speed_vector.h"
#include "onboard/prediction/post_process/object_svt_sample.h"
#include "onboard/prediction/predicted_trajectory.h"
#include "onboard/prediction/proto/conflict_resolver.pb.h"
#include "onboard/prediction/proto/prediction_common.pb.h"

namespace qcraft {
namespace prediction {
std::pair<planner::DiscretizedPath, planner::SpeedVector>
PredictedTrajectoryToPurePathAndSpeedVector(
    const PredictedTrajectory& trajectory);

std::pair<planner::DiscretizedPath, planner::SpeedVector>
PredictedTrajectoryPointProtoToPurePathAndSpeedVector(
    const std::vector<PredictedTrajectoryPointProto>& points);

ConflictResolverDebugProto::SimpleSpeedProfile
EdgeConnectionToSimpleSpeedProfile(
    absl::Span<const std::string> cost_names, const std::vector<SvtEdge>& edges,
    const SvtEdgeVector<SvtEdgeCost>& search_costs,
    const std::vector<SvtEdgeIndex>& edge_idxes);

ConflictResolverDebugProto::SpeedProfile SpeedVectorToSpeedProfile(
    const planner::SpeedVector& speed_vector,
    const std::vector<SvtEdgeIndex>& edge_idxes);

planner::SpeedPoint SvtStateToSpeedPoint(const SvtState& state, double a);

}  // namespace prediction
}  // namespace qcraft

#endif  // ONBOARD_PREDICTION_POST_PROCESS_CONFLICT_RESOLVER_UTIL_H_

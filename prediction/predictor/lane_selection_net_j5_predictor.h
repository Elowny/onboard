#ifndef ONBOARD_PREDICTION_PREDICTOR_LANE_SELECTION_NET_J5_PREDICTOR_H_
#define ONBOARD_PREDICTION_PREDICTOR_LANE_SELECTION_NET_J5_PREDICTOR_H_

#include <algorithm>  // IWYU pragma: keep
#include <map>        // for map
#include <vector>     // for vector

#include "absl/types/span.h"  // for Span

#include "onboard/async/thread_pool.h"                    // for ThreadPool
#include "onboard/math/geometry/box2d.h"                  // for Box2d
#include "onboard/planner/router/drive_passage.h"         // for DrivePassage
#include "onboard/prediction/container/object_history.h"  // for ObjectHistory
#include "onboard/prediction/container/prediction_context.h"  // for PredictionContext
#include "onboard/prediction/feature_extractor/object_history_sampler.h"  // for ObjectHistorySampler
#include "onboard/prediction/net/horizon/lane_selection_net_j5_inferencer.h"  // for LaneSelectionNetJ5Inferencer
#include "onboard/prediction/predicted_trajectory.h"  // for PredictedTrajectory
#include "onboard/prediction/prediction_defs.h"  // for ObjectIDType, AgentDrivePassagesMap
#include "onboard/prediction/proto/prediction_common.pb.h"  // for ObjectPredictionScenario

namespace qcraft {
namespace prediction {
struct ObjectLaneSelectionNetPred {
  std::vector<PredictedTrajectory> pred_trajs;
};
using ObjectsLaneSelectionNetPredMap =
    std::map<ObjectIDType, ObjectLaneSelectionNetPred>;

ObjectsLaneSelectionNetPredMap MakeLaneSelectionNetJ5Prediction(
    const PredictionContext& prediction_context,
    absl::Span<const ObjectHistory* const> objects_history,
    const lane_selection_net::LaneSelectionNetJ5Inferencer&
        lane_selection_net_j5_inferencer,
    const std::map<ObjectIDType, ObjectPredictionScenario>& object_scenarios,
    const ObjectHistorySampler& obj_sampler, ThreadPool* thread_pool);

ObjectsLaneSelectionNetPredMap
GenerateLaneSelectionTrajsByModelAndHeuristicInfo(
    const AgentDrivePassagesMap& agent_dps_map,
    const LaneSelectionInferencerOutputMap& infer_result_map,
    const ObjectHistorySampler& obj_sampler,
    const PredictionContext& prediction_context, bool is_mapless,
    ThreadPool* thread_pool);

AgentDrivePassagesMap SelectLaneSelectionJ5PredictedObjectsWithDps(
    const Box2d& ego_box,
    absl::Span<const ObjectHistory* const> objs_to_predict,
    const ObjectHistorySampler& obj_sampler,
    const PredictionContext& prediction_context,
    const std::map<ObjectIDType, ObjectPredictionScenario>& object_scenarios,
    ThreadPool* thread_pool);

std::vector<planner::DrivePassage> BuildLaneSelectionObjectDrivePassages(
    const ObjectMotionHistory& obj_hist, const PredictionContext& context,
    bool build_intersection = true);
}  // namespace prediction
}  // namespace qcraft

#endif  // ONBOARD_PREDICTION_PREDICTOR_LANE_SELECTION_NET_J5_PREDICTOR_H_

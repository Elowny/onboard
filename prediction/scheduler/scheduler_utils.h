#ifndef ONBOARD_PREDICTION_SCHEDULER_SCHEDULER_UTILS_H_
#define ONBOARD_PREDICTION_SCHEDULER_SCHEDULER_UTILS_H_

#include <map>
#include <string>
#include <vector>

#include "absl/types/span.h"

#include "onboard/async/thread_pool.h"
#include "onboard/prediction/container/model_pool.h"
#include "onboard/prediction/container/object_history.h"
#include "onboard/prediction/container/object_prediction_result.h"
#include "onboard/prediction/container/prediction_context.h"
#include "onboard/prediction/feature_extractor/object_history_sampler.h"
#include "onboard/prediction/predicted_trajectory.h"
#include "onboard/prediction/prediction_defs.h"
#include "onboard/prediction/proto/prediction_common.pb.h"
#include "onboard/prediction/scheduler/priority_analyzer.h"
#include "onboard/proto/prediction.pb.h"
namespace qcraft {
namespace prediction {

bool IsHighOrEqualPriority(PredictionType cur, PredictionType in);

void TryInsertTrajectories(
    std::map<std::string, std::vector<PredictedTrajectory>>* obj_traj_map,
    std::vector<PredictedTrajectory> trajs, const ObjectIDType& obj_id);

void RecordPredSpeeGapQevent(const ObjectIDType& obj_id, double obj_v,
                             absl::Span<const PredictedTrajectory> obj_trajs);

std::vector<PredictedTrajectory> FilterOutStaticAndLowProbTrajectory(
    absl::Span<const PredictedTrajectory> trajs, double startup_prob);

void PredictStationaryAndReversed(
    absl::Span<const ObjectHistory* const> objs_to_predict,
    const ObjectHistorySampler& obj_sampler,
    std::map<ObjectIDType, std::vector<PredictedTrajectory>>* object_trajs,
    const std::map<ObjectIDType, ObjectPredictionScenario>& object_scenarios,
    bool ignore_off_road);

void PredictIgnoredObjects(
    absl::Span<const ObjectHistory* const> objs_to_predict,
    const ObjectHistorySampler& obj_sampler,
    const std::map<ObjectIDType, ObjectPredictionPriorityInfo>&
        object_priorities,
    std::map<ObjectIDType, std::vector<PredictedTrajectory>>* object_trajs);

void PredictObjectsByHeuristic(
    const PredictionContext& prediction_context,
    absl::Span<const ObjectHistory* const> objs_to_predict,
    const std::map<ObjectIDType, ObjectPredictionScenario>& object_scenarios,
    const ObjectHistorySampler& obj_sampler, ThreadPool* thread_pool,
    std::map<ObjectIDType, std::vector<PredictedTrajectory>>* object_trajs,
    bool ignore_off_road);

void PredictObjectsByLaneSelection(
    const PredictionContext& prediction_context,
    absl::Span<const ObjectHistory* const> objs_to_predict,
    const std::map<ObjectIDType, ObjectPredictionScenario>& object_scenarios,
    const ObjectHistorySampler& obj_sampler, ThreadPool* thread_pool,
    std::map<ObjectIDType, std::vector<PredictedTrajectory>>* object_trajs,
    const ModelPool& model_pool, bool ignore_off_road);

void PredictUnderParkingScenario(
    absl::Span<const ObjectHistory* const> objs_to_predict,
    const ObjectHistorySampler& obj_sampler, ThreadPool* thread_pool,
    std::map<ObjectIDType, std::vector<PredictedTrajectory>>* object_trajs);

void AssemblePredictionResult(
    const PredictionContext& prediction_context,
    absl::Span<const ObjectHistory* const> objs_to_predict,
    const ObjectHistorySampler& obj_sampler,
    const std::map<ObjectIDType, ObjectPredictionScenario>& object_scenarios,
    const std::map<ObjectIDType, ObjectPredictionPriorityInfo>&
        object_priorities,
    std::map<ObjectIDType, std::vector<PredictedTrajectory>>* object_trajs,
    std::map<ObjectIDType, ObjectPredictionResult>* results);

}  // namespace prediction
}  // namespace qcraft

#endif  // ONBOARD_PREDICTION_SCHEDULER_SCHEDULER_UTILS_H_

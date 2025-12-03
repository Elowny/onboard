#ifndef ONBOARD_PREDICTION_CONTAINER_PREDICTION_RUNNER_H_
#define ONBOARD_PREDICTION_CONTAINER_PREDICTION_RUNNER_H_

#include <memory>

#include "absl/status/statusor.h"

#include "onboard/async/thread_pool.h"
#include "onboard/prediction/container/model_pool.h"
#include "onboard/prediction/container/prediction_input.h"
#include "onboard/proto/prediction.pb.h"

namespace qcraft::prediction {
absl::StatusOr<std::unique_ptr<ObjectsPredictionProto>> ComputePrediction(
    const PredictionInput& input, ModelPool* model_pool,
    ThreadPool* thread_pool, PredictionDebugProto* debug);

}  // namespace qcraft::prediction

#endif  // ONBOARD_PREDICTION_CONTAINER_PREDICTION_RUNNER_H_

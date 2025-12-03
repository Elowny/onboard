#ifndef ONBOARD_PREDICTION_POST_PROCESS_OBJECT_CONFLICT_RESOLVER_H_
#define ONBOARD_PREDICTION_POST_PROCESS_OBJECT_CONFLICT_RESOLVER_H_

#include "absl/status/statusor.h"

#include "onboard/async/thread_pool.h"
#include "onboard/prediction/post_process/conflict_resolver_input.h"
#include "onboard/prediction/predicted_trajectory.h"
#include "onboard/prediction/proto/conflict_resolver.pb.h"

namespace qcraft::prediction {
absl::StatusOr<PredictedTrajectory> ResolveObjectConflict(
    const ObjectConflictResolverInput& input, ThreadPool* thread_pool,
    ConflictResolverDebugProto* debug_proto);
}  // namespace qcraft::prediction
#endif  // ONBOARD_PREDICTION_POST_PROCESS_OBJECT_CONFLICT_RESOLVER_H_

#include "onboard/prediction/scheduler/scheduler.h"

#include "onboard/global/run_context.h"
#include "onboard/prediction/scheduler/fsd_scheduler.h"
#include "onboard/prediction/scheduler/noa_scheduler.h"

namespace qcraft {
namespace prediction {

std::map<ObjectIDType, ObjectPredictionResult> SchedulePrediction(
    const PredictionContext& prediction_context, const ModelPool& model_pool,
    const ObjectHistorySampler& obj_sampler,
    absl::Span<const ObjectHistory* const> objs_to_predict,
    ThreadPool* thread_pool, PredictionDebugProto* debug) {
#if defined(__J5__)
  bool is_j5 = true;
#else
  bool is_j5 = false;
#endif

  if (!IsRunModeL4() || is_j5) {
    return ScheduleNoaPrediction(prediction_context, model_pool, obj_sampler,
                                 objs_to_predict, thread_pool);
  }
  return ScheduleFsdPrediction(prediction_context, model_pool, obj_sampler,
                               objs_to_predict, thread_pool, debug);
}

}  // namespace prediction
}  // namespace qcraft

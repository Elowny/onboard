#ifndef ONBOARD_PREDICTION_FEATURE_EXTRACTOR_OBJECT_HISTORY_SAMPLER_H_
#define ONBOARD_PREDICTION_FEATURE_EXTRACTOR_OBJECT_HISTORY_SAMPLER_H_

#include <memory>
#include <unordered_map>
#include <vector>

#include "absl/status/statusor.h"
#include "absl/types/span.h"

#include "onboard/math/geometry/box2d.h"
#include "onboard/prediction/container/object_history.h"
#include "onboard/prediction/container/object_history_span.h"
#include "onboard/prediction/prediction_defs.h"
#include "onboard/proto/perception.pb.h"

namespace qcraft {
namespace prediction {

std::vector<ObjectProto> ResampleObjectHistorySpan(
    const ObjectHistorySpan& obj_history, double current_ts, double time_step,
    int max_steps, bool enable_smoothing);

ObjectMotionHistory ResampleObjectHistorySpanToMotionHistory(
    const ObjectHistorySpan& obj_history, double current_ts, double time_step,
    int max_steps, bool enable_smoothing);

absl::StatusOr<ObjectMotionHistory>
ResampleObjectMotionHistoryFromTrackerHistory(const ObjectProto& obj,
                                              double current_ts,
                                              double time_step, int max_steps);

ObjectMotionState LerpObjectMotionState(const ObjectMotionState& a,
                                        const ObjectMotionState& b,
                                        double alpha);

bool CheckHistoryConsistency(const ObjectProto& obj,
                             const ObjectProto& next_obj, double ts);

class ObjectHistorySampler {
 public:
  explicit ObjectHistorySampler(
      absl::Span<const ObjectHistory* const> objs_to_predict,
      const ObjectHistory& av_history, double current_time, double time_step,
      int max_steps, bool enable_smoothing, bool use_tracker_history);

  const ObjectMotionHistory& GetResampledMotionHistoryById(
      const ObjectIDType& obj_id) const;

  std::vector<const ObjectMotionHistory*>
  GetResampledMotionHistoryWithAVInBox2d(const ObjectIDType& agent_id,
                                         const Box2d& region_box,
                                         int num_other_objs) const;
  // Query history pointer, might be nullptr !
  const ObjectMotionHistory* GetResampledMotionHistoryPtrById(
      const ObjectIDType& obj_id) const;

  inline const ObjectMotionHistory* GetResampledAVMotionHistory() const {
    return resampled_histories_.at(kAvObjectId).get();
  }

  double current_time() const { return current_time_; }
  double time_step() const { return time_step_; }
  int max_steps() const { return max_steps_; }
  int objects_size() const { return resampled_histories_.size(); }

 private:
  std::unordered_map<ObjectIDType, std::unique_ptr<ObjectMotionHistory>>
      resampled_histories_;
  double current_time_;
  double time_step_;
  int max_steps_;
};

}  // namespace prediction
}  // namespace qcraft

#endif  // ONBOARD_PREDICTION_FEATURE_EXTRACTOR_OBJECT_HISTORY_SAMPLER_H_

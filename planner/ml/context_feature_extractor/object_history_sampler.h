#ifndef ONBOARD_PLANNER_ML_CONTEXT_FEATURE_EXTRACTOR_OBJECT_HISTORY_SAMPLER_H_  // NOLINT
#define ONBOARD_PLANNER_ML_CONTEXT_FEATURE_EXTRACTOR_OBJECT_HISTORY_SAMPLER_H_  // NOLINT

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/status/statusor.h"

#include "onboard/math/geometry/box2d.h"
#include "onboard/planner/object/planner_object_manager.h"
#include "onboard/planner/object/spacetime_trajectory_manager.h"
#include "onboard/prediction/container/object_history.h"
#include "onboard/prediction/predicted_trajectory.h"
#include "onboard/prediction/prediction_defs.h"
#include "onboard/proto/perception.pb.h"
#include "onboard/utils/map_util.h"

namespace qcraft {
namespace planner {
namespace ml {

// Predicted trajectory is already synced by planner using the latest perception
// result (which is updated in planner object timestamp).
absl::StatusOr<prediction::ObjectMotionHistory>
ResampleObjectMotionHistoryFromTrackerHistoryWithLookAhead(
    const ObjectProto& obj,
    const prediction::PredictedTrajectory& predicted_trajectory,
    double pred_traj_ts, double predicted_plan_time, double time_step,
    int max_steps);

class ObjectHistorySampler {
 public:
  explicit ObjectHistorySampler(const PlannerObjectManager& objs_mgr,
                                const prediction::ObjectHistory& av_history,
                                double current_time, double time_step,
                                int max_steps, bool enable_smoothing);

  explicit ObjectHistorySampler(
      const std::shared_ptr<const ObjectsProto>& real_objects,
      const std::shared_ptr<const ObjectsProto>& virtual_objects,
      const prediction::ObjectHistory& av_history, double current_time,
      double time_step, int max_steps, bool enable_smoothing);

  const prediction::ObjectMotionHistory& GetResampledMotionHistoryByObjectId(
      const prediction::ObjectIDType& obj_id) const;

  std::vector<const prediction::ObjectMotionHistory*>
  GetResampledMotionHistoryInBox2d(const prediction::ObjectIDType& agent_id,
                                   const Box2d& region_box, int num_objs) const;
  const prediction::ObjectMotionHistory& GetResampledAVMotionHistory() const {
    return *resampled_histories_.at(prediction::kAvObjectId);
  }

  // Only use tracker history, with look ahead time on synced predicted
  // trajectory. Only sample trajs needed.
  explicit ObjectHistorySampler(
      const std::map<std::string, std::vector<int>>& trajs_to_sample,
      const ObjectsProto* real_objects, const ObjectsProto* virtual_objects,
      const SpacetimeTrajectoryManager& st_traj_mgr,
      const prediction::ObjectHistory& av_history, double predicted_plan_time,
      double time_step, int max_steps);

  const prediction::ObjectMotionHistory*
  GetResampledMotionHistoryPtrByTrajectoryId(const std::string& id) const {
    // Id is trajectory id.
    const auto* hist_ptr = FindOrNull(resampled_histories_, id);
    return (hist_ptr == nullptr) ? nullptr : (*hist_ptr).get();
  }
  double current_time() const { return current_time_; }
  double time_step() const { return time_step_; }
  int max_steps() const { return max_steps_; }
  int objects_size() const { return resampled_histories_.size(); }

 private:
  absl::flat_hash_map<prediction::ObjectIDType,
                      std::unique_ptr<prediction::ObjectMotionHistory>>
      resampled_histories_;
  double current_time_;
  double time_step_;
  int max_steps_;
};

}  // namespace ml
}  // namespace planner
}  // namespace qcraft

// NOLINTNEXTLINE
#endif  // ONBOARD_PLANNER_ML_CONTEXT_FEATURE_EXTRACTOR_OBJECT_HISTORY_SAMPLER_H_

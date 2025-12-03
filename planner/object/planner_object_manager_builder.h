#ifndef ONBOARD_PLANNER_OBJECT_PLANNER_OBJECT_MANAGER_BUILDER_H_
#define ONBOARD_PLANNER_OBJECT_PLANNER_OBJECT_MANAGER_BUILDER_H_

// IWYU pragma: no_include <algorithm>
#include <optional>
#include <utility>
#include <vector>

#include "absl/status/statusor.h"

#include "onboard/async/thread_pool.h"
#include "onboard/planner/object/object_vector.h"
#include "onboard/planner/object/planner_object.h"
#include "onboard/planner/object/planner_object_manager.h"
#include "onboard/planner/object/proto/planner_object.pb.h"
#include "onboard/planner/object/trajectory_filter.h"
#include "onboard/prediction/object_prediction.h"
#include "onboard/proto/perception.pb.h"
#include "onboard/proto/prediction.pb.h"

namespace qcraft {
namespace planner {

// A helper class used to build the PlannerObjectManager.
class PlannerObjectManagerBuilder {
 public:
  PlannerObjectManagerBuilder& SetAlignTime(double time) {
    aligned_time_ = time;
    return *this;
  }

  PlannerObjectManagerBuilder& set_planner_objects(
      ObjectVector<PlannerObject> planner_objects) {
    planner_objects_ = std::move(planner_objects);
    return *this;
  }

  PlannerObjectManagerBuilder& set_filters(
      std::vector<const TrajectoryFilter*> filters) {
    filters_ = std::move(filters);
    return *this;
  }

  absl::StatusOr<PlannerObjectManager> Build(
      FilteredTrajectories* filtered_trajs = nullptr,
      ThreadPool* thread_pool = nullptr);

 private:
  // A negative value means do not do time alignment.
  double aligned_time_ = -1.0;

  std::vector<const TrajectoryFilter*> filters_;

  ObjectVector<PlannerObject> planner_objects_;
};

// This function builds planner objects from perception objects and prediction.
// One of `perception` or `prediction` could be nullptr. If `align_time` is
// set, tt aligns the objects' and predictions' time.
ObjectVector<PlannerObject> BuildPlannerObjects(
    const ObjectsProto* perception, const ObjectsPredictionProto* prediction,
    std::optional<double> align_time, ThreadPool* thread_pool = nullptr);

// To replace the above function.
ObjectVector<PlannerObject> BuildPlannerObjectsFromObjectPrediction(
    const ObjectsProto* perception,
    const prediction::ObjectsPrediction* prediction,
    std::optional<double> align_time, ThreadPool* thread_pool = nullptr);

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_OBJECT_PLANNER_OBJECT_MANAGER_BUILDER_H_

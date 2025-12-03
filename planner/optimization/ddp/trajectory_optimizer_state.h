#ifndef ONBOARD_PLANNER_OPTIMIZATION_DDP_TRAJECTORY_OPTIMIZER_STATE_H_
#define ONBOARD_PLANNER_OPTIMIZATION_DDP_TRAJECTORY_OPTIMIZER_STATE_H_

#include <vector>

#include "absl/time/time.h"

#include "onboard/planner/optimization/proto/optimizer.pb.h"
#include "onboard/planner/trajectory_point.h"

namespace qcraft {
namespace planner {

struct TrajectoryOptimizerState {
  TrajectoryOptimizerState() = default;
  explicit TrajectoryOptimizerState(
      const TrajectoryOptimizerStateProto& proto) {
    FromProto(proto);
  }

  void FromProto(const TrajectoryOptimizerStateProto& proto);

  TrajectoryOptimizerStateProto ToProto() const;

  absl::Time last_plan_start_time;
  std::vector<TrajectoryPoint> last_optimized_trajectory;
};

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_OPTIMIZATION_DDP_TRAJECTORY_OPTIMIZER_STATE_H_

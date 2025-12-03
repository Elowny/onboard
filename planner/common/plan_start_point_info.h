#ifndef ONBOARD_PLANNER_COMMON_PLAN_START_POINT_INFO_H_
#define ONBOARD_PLANNER_COMMON_PLAN_START_POINT_INFO_H_

#include "absl/time/time.h"

#include "onboard/proto/planner.pb.h"
#include "onboard/proto/trajectory_point.pb.h"

namespace qcraft {
namespace planner {

struct PlanStartPointInfo {
  bool reset;
  std::optional<int> start_index_on_prev_traj;
  ApolloTrajectoryPointProto start_point;
  double path_s_increment_from_previous_frame;
  absl::Time plan_time;
  bool full_stop;
  ResetReasonProto::Reason reset_reason;
};

struct StPathPlanStartPointInfo {
  bool reset;
  int relative_index_from_plan_start_point;
  ApolloTrajectoryPointProto start_point;
  absl::Time plan_time;
};

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_COMMON_PLAN_START_POINT_INFO_H_

#ifndef ONBOARD_PLANNER_PLAN_ACC_ACC_TASK_INPUT_H_
#define ONBOARD_PLANNER_PLAN_ACC_ACC_TASK_INPUT_H_

#include <optional>
#include <set>
#include <string>
#include <vector>

#include "absl/time/time.h"
#include "absl/types/span.h"

#include "common/proto/qacc.pb.h"

#include "onboard/params/vehicle_param_api.h"
#include "onboard/planner/common/plan_start_point_info.h"
#include "onboard/planner/object/planner_object_manager.h"
#include "onboard/planner/object/spacetime_trajectory_manager.h"
#include "onboard/planner/plan/acc/acc_corridor.h"
#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/planner/proto/planner_params.pb.h"
#include "onboard/prediction/container/av_context.h"
#include "onboard/proto/positioning.pb.h"
#include "onboard/proto/trajectory.pb.h"

namespace qcraft::planner {

struct AccTaskInput {
  const PlannerSemanticMapManager* planner_semantic_map_manager = nullptr;
  const PoseProto* pose = nullptr;
  std::optional<double> steering_percentage = std::nullopt;
  const AccTaskParamsProto* acc_params = nullptr;
  const VehicleGeometryParamsProto* vehicle_geometry_params = nullptr;
  const VehicleDriveParamsProto* vehicle_drive_params = nullptr;
  const PlanStartPointInfo* plan_start_point_info = nullptr;
  absl::Time plan_time;
  const SpacetimeTrajectoryManager* st_traj_mgr = nullptr;
  const TrajectoryProto* prev_trajectory = nullptr;
  const std::vector<ApolloTrajectoryPointProto>* time_aligned_prev_traj =
      nullptr;
  bool is_acc_standwait = false;
  bool prev_collision_warning_request = false;
  double average_kappa = 0.0;
  const QACCTaskProto* acc_task_proto = nullptr;
  std::optional<double> lcc_cruising_speed_limit = std::nullopt;
  QRunEvent::LccFollowingDistanceLevel following_distance_level =
      QRunEvent::LCC_FOLLOWING_DISTANCE_LEVEL_MEDIUM;
};

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_PLAN_ACC_ACC_TASK_INPUT_H_

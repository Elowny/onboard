#ifndef ONBOARD_PLANNER_PLAN_UTURN_TASK_H_
#define ONBOARD_PLANNER_PLAN_UTURN_TASK_H_

#include "absl/status/status.h"
#include "absl/time/time.h"

#include "onboard/async/thread_pool.h"
#include "onboard/maps/lane_path.h"
#include "onboard/math/coordinate_converter.h"
#include "onboard/planner/common/global_pose.h"
#include "onboard/planner/common/plan_start_point_info.h"
#include "onboard/planner/freespace/proto/freespace_planner.pb.h"
#include "onboard/planner/object/planner_cluster_object_manager.h"
#include "onboard/planner/object/planner_object_manager.h"
#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/planner/proto/planner_params.pb.h"
#include "onboard/proto/autonomy_state.pb.h"
#include "onboard/proto/charts.pb.h"
#include "onboard/proto/chassis.pb.h"
#include "onboard/proto/planner.pb.h"
#include "onboard/proto/positioning.pb.h"
#include "onboard/proto/trajectory.pb.h"
#include "onboard/proto/vehicle.pb.h"

namespace qcraft::planner {

struct UTurnTaskOutput {
  TrajectoryProto trajectory_info;
  FreespacePlannerDebugProto debug_proto;
  vis::vantage::ChartsDataProto chart_data;
  PlannerType planner_type;
  bool reset = false;
  ResetReasonProto::Reason reset_reason;
};

struct UTurnTaskInput {
  bool reset;
  const AutonomyStateProto* autonomy_state;
  const PlannerSemanticMapManager* psmm;
  const CoordinateConverter* coordinate_converter;
  const GlobalPose* goal;
  const mapping::LanePath* lane_path;
  const PoseProto* pose;
  const Chassis* chassis;
  const PlanStartPointInfo* plan_start_point_info;
  absl::Time plan_time;
  const FreespaceParamsProto* freespace_params;
  const PlannerVehicleModelParamsProto* vehicle_models_params;
  const VehicleGeometryParamsProto* veh_geo_params;
  const VehicleDriveParamsProto* veh_drive_params;
  const TrajectoryProto* prev_trajectory_proto;
  const PlannerObjectManager* object_manager;
  const PlannerClusterObjectManager* cluster_object_manager;
  double time_interval;
};

absl::Status RunUTurnTask(const UTurnTaskInput& input,
                          FreespacePlannerStateProto* state,
                          UTurnTaskOutput* result, ThreadPool* thread_pool);

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_PLAN_UTURN_TASK_H_

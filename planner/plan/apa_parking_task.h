#ifndef ONBOARD_PLANNER_PLAN_APA_PARKING_TASK_H_
#define ONBOARD_PLANNER_PLAN_APA_PARKING_TASK_H_

#include "absl/status/status.h"
#include "absl/time/time.h"

// IWYU pragma: no_include <google/protobuf/repeated_ptr_field.h>

#include "onboard/async/thread_pool.h"
#include "onboard/maps/maps_common.h"
#include "onboard/planner/common/plan_start_point_info.h"
#include "onboard/planner/freespace/proto/freespace_planner.pb.h"
#include "onboard/planner/object/planner_cluster_object_manager.h"
#include "onboard/planner/object/planner_object_manager.h"
#include "onboard/planner/proto/planner_params.pb.h"
#include "onboard/proto/autonomy_state.pb.h"
#include "onboard/proto/charts.pb.h"
#include "onboard/proto/chassis.pb.h"
#include "onboard/proto/planner.pb.h"
#include "onboard/proto/positioning.pb.h"
#include "onboard/proto/trajectory.pb.h"
#include "onboard/proto/vehicle.pb.h"

namespace qcraft::planner {

struct ApaParkingTaskOutput {
  TrajectoryProto trajectory_info;
  FreespacePlannerDebugProto debug_proto;
  vis::vantage::ChartsDataProto chart_data;
  PlannerType planner_type;
  bool reset = false;
  ResetReasonProto::Reason reset_reason;
};

struct ApaParkingTaskInput {
  bool reset;
  const AutonomyStateProto* autonomy_state;
  const mapping::ParkingSpotInfo parking_spot_info;
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

// TODO(zhuang): Need a ComputeApaPlanStartPoint() function and a parking spot
// info converter.
absl::Status RunApaParkingTask(const ApaParkingTaskInput& input,
                               FreespacePlannerStateProto* state,
                               ApaParkingTaskOutput* result,
                               ThreadPool* thread_pool);

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_PLAN_APA_PARKING_TASK_H_

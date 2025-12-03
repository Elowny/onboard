#ifndef ONBOARD_PLANNER_FREESPACE_FREESPACE_PLANNER_H_
#define ONBOARD_PLANNER_FREESPACE_FREESPACE_PLANNER_H_

#include <string>
#include <vector>

#include "absl/container/flat_hash_set.h"
#include "absl/status/statusor.h"
#include "absl/time/time.h"

#include "onboard/async/thread_pool.h"
#include "onboard/maps/maps_common.h"
#include "onboard/math/coordinate_converter.h"
#include "onboard/planner/freespace/directional_path.h"
#include "onboard/planner/freespace/freespace_planner_defs.h"
#include "onboard/planner/freespace/proto/freespace_planner.pb.h"
#include "onboard/planner/object/planner_cluster_object.h"
#include "onboard/planner/object/planner_cluster_object_manager.h"
#include "onboard/planner/object/planner_object_manager.h"
#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/planner/proto/planner_params.pb.h"
#include "onboard/proto/autonomy_state.pb.h"
#include "onboard/proto/charts.pb.h"
#include "onboard/proto/chassis.pb.h"
#include "onboard/proto/planner.pb.h"
#include "onboard/proto/positioning.pb.h"
#include "onboard/proto/trajectory_point.pb.h"
#include "onboard/proto/vehicle.pb.h"

namespace qcraft::planner {

struct FreespacePlannerOutput {
  std::vector<ApolloTrajectoryPointProto> traj_points;
  Chassis::GearPosition gear_position;
  bool low_speed_freespace = false;
  bool enable_stationary_steering = true;
  bool reset = false;
  ResetReasonProto::Reason reset_reason;
  DirectionalPath smooth_directional_path;
  double stop_s = 0.0;
  bool is_path_blocked = false;
};

struct FreespacePlannerInput {
  bool new_task;
  bool force_stop;
  bool safe_stop;
  const AutonomyStateProto* autonomy_state;
  const PoseProto* ego_pose;
  const CoordinateConverter* coordinate_converter;
  const Chassis* chassis;
  const PlannerObjectManager* obj_mgr;
  const PlannerClusterObjectManager* cluster_obj_mgr;
  const PlannerSemanticMapManager* psmm;
  const absl::flat_hash_set<std::string>* stalled_object_ids;
  const absl::flat_hash_set<PlannerClusterObject::Id>*
      stalled_cluster_object_ids;
  const ApolloTrajectoryPointProto* plan_start_point;
  bool start_point_reset;
  ResetReasonProto::Reason reset_reason;
  absl::Time plan_time;
  const FreespaceMap* freespace_map;
  const mapping::ParkingSpotInfo* parking_spot_info;
  // params
  const FreespaceParamsProto* freespace_params;
  const PlannerVehicleModelParamsProto* vehicle_models_params;
  const VehicleGeometryParamsProto* veh_geo_params;
  const VehicleDriveParamsProto* veh_drive_params;
  double time_interval;
};

absl::StatusOr<FreespacePlannerOutput> RunFreespacePlanner(
    const FreespacePlannerInput& input, FreespacePlannerStateProto* state,
    FreespacePlannerDebugProto* debug_info,
    vis::vantage::ChartsDataProto* charts_data, ThreadPool* thread_pool);

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_FREESPACE_FREESPACE_PLANNER_H_

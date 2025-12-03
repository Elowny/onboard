#ifndef ONBOARD_PLANNER_DECISION_DECIDER_INPUT_H_
#define ONBOARD_PLANNER_DECISION_DECIDER_INPUT_H_

#include <limits>
#include <string>

#include "onboard/math/frenet_common.h"
#include "onboard/params/v2/proto/vehicle/common.pb.h"
#include "onboard/planner/common/path_sl_boundary.h"
#include "onboard/planner/decision/traffic_light/tl_info.h"
#include "onboard/planner/object/planner_object_manager.h"
#include "onboard/planner/object/spacetime_trajectory_manager.h"
#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/planner/proto/planner_params.pb.h"
#include "onboard/planner/router/drive_passage.h"
#include "onboard/planner/scene/proto/scene_understanding.pb.h"
#include "onboard/planner/scheduler/proto/lane_change.pb.h"
#include "onboard/planner/scheduler/scheduler_output.h"
#include "onboard/proto/driving_style.pb.h"
#include "onboard/proto/perception.pb.h"

namespace qcraft::planner {

struct DeciderInput {
  const qcraft::VehicleGeometryParamsProto* vehicle_geometry_params = nullptr;
  const MotionConstraintParamsProto* motion_constraint_params = nullptr;
  const DecisionConstraintConfigProto* config = nullptr;
  const PlannerSemanticMapManager* planner_semantic_map_manager = nullptr;
  const LaneChangeStateProto* lc_state;
  const ApolloTrajectoryPointProto* plan_start_point = nullptr;
  double target_offset_from_start = 0.0;
  const mapping::LanePath* lane_path_before_lc = nullptr;
  const DrivePassage* passage = nullptr;
  const PathSlBoundary* sl_boundary = nullptr;
  const FrenetBox* ego_frenet_box = nullptr;
  bool borrow_lane_boundary = false;
  const PlannerObjectManager* obj_mgr = nullptr;
  const SpacetimeTrajectoryManager* st_traj_mgr = nullptr;
  const TrafficLightInfoMap* tl_info_map = nullptr;
  const DeciderStateProto* pre_decider_state = nullptr;
  absl::Time parking_brake_release_time;
  bool teleop_enable_traffic_light_stop;
  bool enable_pull_over;
  std::optional<double> brake_to_stop = std::nullopt;
  double max_reach_length = std::numeric_limits<double>::max();
  // TODO(PNC-501): This is a hack, remove later.
  VehicleModel vehicle_model;
  absl::Time plan_time;
  // Null if no routed lane change is detected.
  const SceneOutputProto* scene_reasoning = nullptr;
  bool enable_stop_polyline_stopping = false;
  bool is_engage_steer_only = false;
  bool enable_force_stop = false;
};

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_DECISION_DECIDER_INPUT_H_

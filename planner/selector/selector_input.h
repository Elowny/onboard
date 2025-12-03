#ifndef ONBOARD_PLANNER_SELECTOR_SELECTOR_INPUT_H_
#define ONBOARD_PLANNER_SELECTOR_SELECTOR_INPUT_H_

#include <string>
#include <vector>

#include "absl/container/flat_hash_set.h"

#include "onboard/planner/ml/context_feature_extractor/context_feature.h"
#include "onboard/planner/ml/model_pool.h"
#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/planner/proto/planner_params.pb.h"
#include "onboard/planner/router/navi/route_navi_info.h"
#include "onboard/planner/router/route_sections_info.h"
#include "onboard/planner/selector/selector_state.h"
#include "onboard/proto/driving_style.pb.h"
#include "onboard/proto/prediction.pb.h"

namespace qcraft::planner {

struct SelectorFlags {
  int planner_begin_lane_change_frame = 1;
  int planner_begin_signal_frame = 1;
  bool planner_enable_lane_change_in_intersection = true;
  bool planner_enable_cross_solid_boundary = true;
  LaneChangeStyle planner_lane_change_style = LaneChangeStyle::LC_STYLE_NORMAL;
  bool planner_need_to_lane_change_confirmation = false;
  bool planner_is_bus_model = false;
  bool planner_is_l4_mode = true;
  bool planner_enable_obstacle_lane_change = true;
  bool planner_enable_lc_request_in_tricky_scenario = false;
  int planner_begin_radical_lane_change_frame = 1;
  double planner_allow_lc_time_after_activate_selector = 2.0;
  double planner_max_allow_lc_time_before_give_up = 15.0;
  double planner_allow_lc_time_after_give_up_lc = 5.0;
  int planner_lc_begin_request_frame_in_tricky_scenario = 1;
  bool planner_enable_prefilter_for_selector = false;
  double planner_allow_opposite_lc_time_after_paddle_lc = 10.0;
  double planner_alc_request_reject_cool_down_time = 60.0;
};

struct SelectorInput {
  const PlannerSemanticMapManager* psmm;
  const RouteSectionsInfo* sections_info;
  const mapping::LanePath* prev_lane_path_from_current;
  const std::vector<ApolloTrajectoryPointProto>* prev_traj;
  const MotionConstraintParamsProto* motion_constraints;
  const VehicleGeometryParamsProto* vehicle_geom;
  const ApolloTrajectoryPointProto* plan_start_point;
  const absl::flat_hash_set<std::string>* stalled_objects;
  const RouteNaviInfo* route_navi_info;
  const absl::flat_hash_set<mapping::ElementId>* avoid_lanes;
  const absl::Time plan_time;
  const std::optional<bool> alc_confirmation;
  const mapping::LanePath* preferred_lane_path;
  const PredictionDebugProto* prediction_debug;
  const ml::ContextFeature* context_feature;
  const ModelPool* planner_model_pool;
  const SelectorState* selector_state;
  const SelectorFlags* selector_flags;
  const SelectorParamsProto* config;
};

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_SELECTOR_SELECTOR_INPUT_H_

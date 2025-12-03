#ifndef ONBOARD_PLANNER_SELECTOR_CANDIDATE_STATS_H_
#define ONBOARD_PLANNER_SELECTOR_CANDIDATE_STATS_H_

#include <float.h>
#include <limits.h>

#include <optional>
#include <string>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/container/flat_hash_set.h"

#include "onboard/maps/semantic_map_defs.h"
#include "onboard/planner/common/planner_status.h"
#include "onboard/planner/est_planner_output.h"
#include "onboard/planner/object/spacetime_trajectory_manager.h"
#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/planner/router/route_sections_info.h"
#include "onboard/planner/scheduler/scheduler_output.h"
#include "onboard/planner/selector/common_feature.h"
#include "onboard/proto/trajectory_point.pb.h"
#include "onboard/proto/vehicle.pb.h"

namespace qcraft::planner {

struct LaneSpeedInfo {
  double init_leader_dist;
  double init_leader_speed;
  double max_leader_speed;
  double lane_speed_limit_by_leader;
  double lane_speed_limit;
  std::optional<std::string> block_obj;
};

struct StalledObjInfo {
  std::string stalled_obj_id;
  double stalled_obj_s;
  double punish_factor;
};

struct ProgressStats {
  ProgressStats(const SelectorCommonFeature& common_feature,
                const ApolloTrajectoryPointProto& plan_start_point,
                const VehicleGeometryParamsProto& vehicle_geom,
                const std::vector<PlannerStatus>& est_status,
                const std::vector<SpacetimeTrajectoryManager>& st_traj_mgr_list,
                const std::vector<EstPlannerOutput>& results);

  double ego_v;
  absl::flat_hash_map<SchedulerOutput::HashType, LaneSpeedInfo> lane_speed_map;
  double max_lane_speed = 0.0;
  double min_leader_dist = DBL_MAX;
};

struct RouteLookAheadStats {
  RouteLookAheadStats(
      const SelectorCommonFeature& common_feature,
      const PlannerSemanticMapManager& psmm,
      const RouteSectionsInfo& sections_info,
      const ApolloTrajectoryPointProto& plan_start_point,
      const absl::flat_hash_set<std::string>& stalled_objects,
      const std::vector<PlannerStatus>& est_status,
      const std::vector<SpacetimeTrajectoryManager>& st_traj_mgr_list,
      const std::vector<EstPlannerOutput>& results);

  absl::flat_hash_map<mapping::ElementId, double> driving_dist_map,
      len_before_intersection_map, len_before_merge_lane_map;
  absl::flat_hash_map<mapping::ElementId, int> lc_num_to_targets_map,
      lc_num_within_driving_dist_map;
  absl::flat_hash_map<mapping::ElementId, bool> is_right_most_lane_map,
      is_valid_merge_lane_map;
  absl::flat_hash_map<SchedulerOutput::HashType, std::optional<StalledObjInfo>>
      front_stalled_obj_map;
  absl::flat_hash_map<SchedulerOutput::HashType, double> len_along_route_map,
      raw_len_along_route_map;

  bool enable_discourage_right_most_cost = false;
  bool enable_encourage_right_most_cost = false;
  double max_length_along_route = 0.0;
  double min_length_along_route = DBL_MAX;
  double traffic_congestion_factor = 0.0;
  int min_lc_num = INT_MAX;
  int max_lc_num = 0;
  bool is_left_turn = false;
  bool is_right_turn = false;
  bool is_non_occluding_leader = false;
  double ego_v = 0.0;
};
// ----------------------------------------------------------------

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_SELECTOR_CANDIDATE_STATS_H_

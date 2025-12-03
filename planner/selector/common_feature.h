#ifndef ONBOARD_PLANNER_SELECTOR_COMMON_FEATURE_H_
#define ONBOARD_PLANNER_SELECTOR_COMMON_FEATURE_H_

#include <string>

#include "absl/container/flat_hash_map.h"
#include "absl/container/flat_hash_set.h"

#include "onboard/planner/scheduler/scheduler_output.h"
#include "onboard/proto/perception.pb.h"

namespace qcraft::planner {

struct LeaderInfo {
  std::string obj_id;
  double obj_s;
  double obj_v;
  ObjectType obj_type;
  bool is_stationary;
  bool is_stalled;
};

struct LaneFeatureInfo {
  // Route info.
  double speed_limit = 0.0;
  int lc_num_to_targets = INT_MAX;
  int lc_num_within_driving_dist = INT_MAX;
  double driving_dist = 0.0;
  double len_before_merge_lane = 0.0;
  absl::flat_hash_set<mapping::ElementId> merge_targets;
  // Obstacle info
  absl::flat_hash_set<std::string> block_obj_ids;
  absl::flat_hash_set<std::string> front_non_block_obj_ids;
  std::optional<LeaderInfo> nearest_leader = std::nullopt;
};

struct SelectorCommonFeature {
  // Traffic info.
  bool in_high_way = false;
  // Intersection info.
  double length_before_intersection = 0.0;
  bool is_left_turn = false;
  bool is_right_turn = false;
  // Lane feature info.
  absl::flat_hash_map<SchedulerOutput::HashType, LaneFeatureInfo>
      lane_feature_infos;
};

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_SELECTOR_COMMON_FEATURE_H_

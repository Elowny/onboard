#ifndef ONBOARD_PLANNER_ROUTER_NAVI_ROUTE_NAVI_INFO_H_
#define ONBOARD_PLANNER_ROUTER_NAVI_ROUTE_NAVI_INFO_H_

#include <limits>
#include <string>

#include "absl/container/flat_hash_map.h"
#include "absl/container/flat_hash_set.h"
#include "absl/hash/hash.h"
#include "absl/strings/str_format.h"
#include "absl/strings/str_join.h"

#include "onboard/maps/semantic_map_defs.h"
#include "onboard/planner/router/route_sections.h"
#include "onboard/proto/route.pb.h"

namespace qcraft {
namespace planner {
struct RouteNaviInfo {
  // Distance is beginning from the lane, and ignore the start fraction.
  // Need to align when use this info.
  struct RouteLaneInfo {
    double max_driving_distance = 0.0;
    double max_reach_length = 0.0;
    double recommend_reach_length = 0.0;
    int min_lc_num_to_target = std::numeric_limits<int>::max();
    int lc_num_within_driving_dist = std::numeric_limits<int>::max();
    double len_before_merge_lane = 0.0;
    absl::flat_hash_set<mapping::ElementId> merge_targets;
    std::string DebugString() const {
      return absl::StrFormat(
          "{max_driving_distance: %f, max_reach_length: %f, "
          "recommend_reach_length: %f, min_lc_num_to_target: %d, "
          "lc_num_within_driving_dist: %d, len_before_merge_lane: %f}",
          max_driving_distance, max_reach_length, recommend_reach_length,
          min_lc_num_to_target, lc_num_within_driving_dist,
          len_before_merge_lane);
    }
  };

  struct NaviSectionInfo {
    double length_before_intersection = 0.0;
    NaviSectionInfoProto::Direction intersection_direction =
        NaviSectionInfoProto::STRAIGHT;
  };

  absl::flat_hash_map<mapping::ElementId, RouteLaneInfo> route_lane_info_map;
  absl::flat_hash_map<mapping::SectionId, NaviSectionInfo>
      navi_section_info_map;
  RouteSections back_extend_sections;

  std::string DebugString() const {
    return absl::StrJoin(
        route_lane_info_map, "\n", [](std::string* out, auto i) {
          out->append(absl::StrFormat("id: %d, %s", i.first.value(),
                                      i.second.DebugString()));
        });
  }
  bool in_highway = false;

  void FromProto(const RouteNaviInfoProto& navi_info_proto);
  void ToProto(RouteNaviInfoProto* navi_info_proto) const;
};

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_ROUTER_NAVI_ROUTE_NAVI_INFO_H_

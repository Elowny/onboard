#include "onboard/planner/router/navi/route_navi_info.h"

#include <algorithm>
#include <utility>
#include <vector>

#include "google/protobuf/repeated_ptr_field.h"

#include "onboard/container/strong_int.h"

namespace qcraft {
namespace planner {

void RouteNaviInfo::FromProto(const RouteNaviInfoProto& navi_info_proto) {
  route_lane_info_map.clear();
  navi_section_info_map.clear();
  const auto& route_lane_info_proto = navi_info_proto.route_lane_info();
  const auto& navi_section_info_proto = navi_info_proto.navi_section_info();
  for (int i = 0; i < route_lane_info_proto.size(); ++i) {
    const auto& cur_navi_info = route_lane_info_proto[i];
    absl::flat_hash_set<mapping::ElementId> merge_targets;
    merge_targets.reserve(cur_navi_info.merge_targets().size());
    for (const auto lane_id : cur_navi_info.merge_targets()) {
      merge_targets.insert(mapping::ElementId(lane_id));
    }
    route_lane_info_map[mapping::ElementId(cur_navi_info.lane_id())] =
        RouteLaneInfo{
            .max_driving_distance = cur_navi_info.max_driving_distance(),
            .max_reach_length = cur_navi_info.max_driving_distance_before_lc(),
            .recommend_reach_length =
                cur_navi_info.recommend_driving_distance(),
            .min_lc_num_to_target = cur_navi_info.min_lc_num_to_target(),
            .lc_num_within_driving_dist =
                cur_navi_info.lc_num_within_driving_dist(),
            .len_before_merge_lane = cur_navi_info.len_before_merge_lane(),
            .merge_targets = std::move(merge_targets)};
  }

  for (int i = 0; i < navi_section_info_proto.size(); ++i) {
    const auto& cur_navi_info = navi_section_info_proto[i];
    navi_section_info_map[mapping::SectionId(cur_navi_info.section_id())] =
        NaviSectionInfo{
            .length_before_intersection =
                cur_navi_info.length_before_intersection(),
            .intersection_direction = cur_navi_info.intersection_direction()};
  }

  if (navi_info_proto.has_back_extend_sections()) {
    back_extend_sections =
        RouteSections::BuildFromProto(navi_info_proto.back_extend_sections());
  }

  in_highway =
      navi_info_proto.has_in_highway() ? navi_info_proto.in_highway() : false;
}

void RouteNaviInfo::ToProto(RouteNaviInfoProto* navi_info_proto) const {
  navi_info_proto->Clear();

  const auto cmp = [](const auto& pair1, const auto& pair2) {
    return pair1.first < pair2.first;
  };
  navi_info_proto->mutable_route_lane_info()->Reserve(
      route_lane_info_map.size());
  std::vector<std::pair<mapping::ElementId, RouteLaneInfo>> ordered_lane_info;
  ordered_lane_info.reserve(route_lane_info_map.size());
  for (const auto& [lane_id, lane_navi_info] : route_lane_info_map) {
    ordered_lane_info.push_back({lane_id, lane_navi_info});
  }
  std::sort(ordered_lane_info.begin(), ordered_lane_info.end(), cmp);
  for (const auto& [lane_id, lane_navi_info] : ordered_lane_info) {
    RouteLaneInfoProto lane_info_proto;
    lane_info_proto.set_lane_id(lane_id.value());
    lane_info_proto.set_max_driving_distance(
        lane_navi_info.max_driving_distance);
    lane_info_proto.set_max_driving_distance_before_lc(
        lane_navi_info.max_reach_length);
    lane_info_proto.set_recommend_driving_distance(
        lane_navi_info.recommend_reach_length);
    lane_info_proto.set_min_lc_num_to_target(
        lane_navi_info.min_lc_num_to_target);
    lane_info_proto.set_lc_num_within_driving_dist(
        lane_navi_info.lc_num_within_driving_dist);
    lane_info_proto.set_len_before_merge_lane(
        lane_navi_info.len_before_merge_lane);
    for (const auto lane_id : lane_navi_info.merge_targets) {
      lane_info_proto.add_merge_targets(lane_id.value());
    }
    navi_info_proto->mutable_route_lane_info()->Add(std::move(lane_info_proto));
  }

  navi_info_proto->mutable_navi_section_info()->Reserve(
      navi_section_info_map.size());
  std::vector<std::pair<mapping::SectionId, NaviSectionInfo>>
      ordered_section_info;
  ordered_section_info.reserve(navi_section_info_map.size());
  for (const auto& [section_id, section_navi_info] : navi_section_info_map) {
    ordered_section_info.push_back({section_id, section_navi_info});
  }
  std::sort(ordered_section_info.begin(), ordered_section_info.end(), cmp);
  for (const auto& [section_id, navi_section_info] : ordered_section_info) {
    NaviSectionInfoProto section_info_proto;
    section_info_proto.set_section_id(section_id.value());
    section_info_proto.set_length_before_intersection(
        navi_section_info.length_before_intersection);
    section_info_proto.set_intersection_direction(
        navi_section_info.intersection_direction);
    navi_info_proto->mutable_navi_section_info()->Add(
        std::move(section_info_proto));
  }

  back_extend_sections.ToProto(navi_info_proto->mutable_back_extend_sections());
  navi_info_proto->set_in_highway(in_highway);
}

}  // namespace planner
}  // namespace qcraft

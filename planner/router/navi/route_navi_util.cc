#include "onboard/planner/router/navi/route_navi_util.h"

// IWYU pragma: no_include <ext/alloc_traits.h>
//              for __alloc_traits<>::value_type

#include <float.h>
#include <limits.h>

#include <algorithm>
#include <cmath>
#include <iterator>
#include <memory>
#include <optional>
#include <ostream>
#include <utility>
#include <vector>

#include "absl/hash/hash.h"
#include "absl/types/span.h"

#include "onboard/container/strong_int.h"
#include "onboard/global/trace.h"
#include "onboard/lite/logging.h"
#include "onboard/maps/lane_point.h"
#include "onboard/maps/map_or_die_macros.h"
#include "onboard/maps/proto/semantic_map.pb.h"
#include "onboard/maps/semantic_map_util.h"
#include "onboard/maps/v2/semantic_map_object.h"
#include "onboard/utils/map_util.h"
#include "onboard/utils/source_location.h"

namespace qcraft {
namespace planner {
namespace {
absl::flat_hash_set<mapping::ElementId> FindMergeTargets(
    const mapping::v2::SemanticMapManager& v2smm,
    const RouteSectionsInfo& sections_info, mapping::ElementId merge_lane_id,
    int curr_section_idx) {
  // Find lane after merge.
  std::optional<mapping::ElementId> lane_id_after_merge;
  mapping::ElementId curr_lane_id = merge_lane_id;
  int sec_idx = curr_section_idx;
  while (lane_id_after_merge == std::nullopt &&
         sec_idx < sections_info.size()) {
    SMM2_ASSIGN_LANE_OR_BREAK(lane_info, v2smm, curr_lane_id);
    if (!lane_info.proto().is_merging()) {
      lane_id_after_merge = curr_lane_id;
      break;
    }
    if (lane_info.proto().outgoing_lanes_size() == 0) {
      break;
    }
    // Suppose there is only one outgoing lane for merge lane.
    curr_lane_id = mapping::ElementId(lane_info.proto().outgoing_lanes(0));
    ++sec_idx;
  }
  if (lane_id_after_merge == std::nullopt) return {};

  // Find which neighbor lane can reach the lane after merge.
  absl::flat_hash_set<mapping::ElementId> lane_ids_to_visit;
  lane_ids_to_visit.insert(*lane_id_after_merge);
  while (sec_idx > curr_section_idx) {
    if (lane_ids_to_visit.empty()) {
      QLOG(WARNING) << "Can not find path from " << *lane_id_after_merge
                    << " to " << merge_lane_id;
      return {};
    }
    const auto& incoming_sec_seg = sections_info.section_segment(--sec_idx);
    absl::flat_hash_set<mapping::ElementId> incoming_lane_ids_to_visit;
    for (const auto& lane_id : lane_ids_to_visit) {
      SMM2_ASSIGN_LANE_OR_CONTINUE(lane_info, v2smm, lane_id);
      for (const auto& incoming_lane_id : lane_info.proto().incoming_lanes()) {
        if (incoming_lane_id != merge_lane_id.value() &&
            incoming_sec_seg.id_idx_map.contains(
                mapping::ElementId(incoming_lane_id))) {
          incoming_lane_ids_to_visit.insert(
              mapping::ElementId(incoming_lane_id));
        }
      }
    }
    lane_ids_to_visit = std::move(incoming_lane_ids_to_visit);
  }

  return lane_ids_to_visit;
}

}  // namespace

absl::flat_hash_map<mapping::ElementId, int> FindLcNumToTargets(
    const mapping::v2::SemanticMapManager& v2smm,
    const RouteSectionsInfo& sections_info,
    const absl::flat_hash_set<mapping::ElementId>& avoid_lanes,
    double preview_length, int start_section_idx) {
  SCOPED_QTRACE("CalcNaviInfo::FindLcNumToTargets");
  int preview_idx = start_section_idx;
  for (; preview_idx + 1 < sections_info.size(); ++preview_idx) {
    if (sections_info.section_segment(preview_idx).lane_ids.empty()) {
      preview_idx = std::max(preview_idx - 1, 0);
      break;
    }
    preview_length -= sections_info.section_segment(preview_idx).length();
    if (preview_length <= 0.0) break;
  }
  const auto& target_ids = sections_info.section_segment(preview_idx).lane_ids;
  std::optional<int> destination_id_index = std::nullopt;
  if (preview_idx == sections_info.size() - 1) {
    destination_id_index =
        FindWithDefault(sections_info.section_segment(preview_idx).id_idx_map,
                        sections_info.destination().lane_id(), INT_MAX);
  }

  absl::flat_hash_map<mapping::ElementId, int> lc_num_map;
  for (const auto target_id : target_ids) {
    // Consider destination target when preview reach destination.
    if (destination_id_index.has_value() && *destination_id_index != INT_MAX) {
      int target_id_index =
          FindWithDefault(sections_info.section_segment(preview_idx).id_idx_map,
                          target_id, INT_MAX);
      lc_num_map[target_id] = std::abs(target_id_index - *destination_id_index);
    } else {
      lc_num_map[target_id] = 0;
    }
  }
  if (preview_idx == start_section_idx) return lc_num_map;

  int cur_sec_idx = preview_idx, prev_sec_idx = preview_idx - 1;
  while (cur_sec_idx != start_section_idx) {
    const auto& prev_lane_ids =
        sections_info.section_segment(prev_sec_idx).lane_ids;
    bool is_connected = false;
    for (const auto prev_id : prev_lane_ids) {
      SMM2_ASSIGN_LANE_OR_CONTINUE(lane_info, v2smm,
                                   mapping::ElementId(prev_id.value()));
      int min_lc_num = INT_MAX;
      if (avoid_lanes.contains(prev_id)) {
        lc_num_map[prev_id] = min_lc_num;
        continue;
      }
      is_connected =
          is_connected || lane_info.proto().outgoing_lanes_size() != 0;
      for (const auto out_lane_id : lane_info.proto().outgoing_lanes()) {
        auto next_lane_id = mapping::ElementId(out_lane_id);
        if (avoid_lanes.contains(next_lane_id)) continue;
        if (lc_num_map.contains(next_lane_id)) {
          min_lc_num =
              std::min(min_lc_num, FindOrDie(lc_num_map, next_lane_id));
        }
      }
      lc_num_map[prev_id] = min_lc_num;
    }
    if (!is_connected) {
      QLOG(ERROR) << sections_info.section_segment(prev_sec_idx).id
                  << " is not connected to destination lane";
    }

    for (int i = 0; i < prev_lane_ids.size(); ++i) {
      auto& min_lc_num = lc_num_map[prev_lane_ids[i]];
      for (int j = 0; j < prev_lane_ids.size(); ++j) {
        const int other_lc_num = lc_num_map.at(prev_lane_ids[j]);
        if (other_lc_num == INT_MAX) continue;

        min_lc_num = std::min(min_lc_num, other_lc_num + std::abs(i - j));
      }
    }
    cur_sec_idx = prev_sec_idx--;
  }

  return lc_num_map;
}

absl::flat_hash_map<mapping::ElementId, double> CalculateMaxDrivingDistance(
    const mapping::v2::SemanticMapManager& v2smm,
    const RouteSectionsInfo& sections_info,
    const absl::flat_hash_set<mapping::ElementId>& avoid_lanes,
    bool from_lane_beginning) {
  SCOPED_QTRACE("CalcNaviInfo::CalculateMaxDrivingDistance");
  const auto& sections = sections_info.section_segments();
  const auto& back_sec = sections.back();
  absl::flat_hash_map<mapping::ElementId, double> driving_dist_map;

  constexpr double kBonusForDestination = 10.0;
  for (int i = 0; i < back_sec.lane_ids.size(); ++i) {
    driving_dist_map[back_sec.lane_ids[i]] = back_sec.length();
    if (back_sec.lane_ids[i] == sections_info.destination().lane_id()) {
      driving_dist_map[back_sec.lane_ids[i]] += kBonusForDestination;
    }
  }

  for (auto iter = sections.rbegin() + 1; iter != sections.rend(); ++iter) {
    const auto& this_section = *iter;
    const auto& next_section = *(iter - 1);
    for (const auto& this_lane_id : this_section.lane_ids) {
      SMM2_ASSIGN_LANE_OR_CONTINUE(lane_info, v2smm, this_lane_id);
      if (avoid_lanes.contains(this_lane_id) ||
          mapping::IsPassengerVehicleAvoidLaneType(lane_info.proto().type())) {
        driving_dist_map[this_lane_id] = 0.0;
        continue;
      }

      auto& this_driving_dist = driving_dist_map[this_lane_id];
      const double section_length = from_lane_beginning
                                        ? this_section.average_length
                                        : this_section.length();
      this_driving_dist = section_length;
      for (const auto& next_lane_id : next_section.lane_ids) {
        if (mapping::IsOutgoingLane(v2smm, lane_info.Proto(), next_lane_id)) {
          this_driving_dist =
              std::max(this_driving_dist,
                       driving_dist_map[next_lane_id] + section_length);
        }
      }
    }
  }
  return driving_dist_map;
}

absl::flat_hash_map<mapping::ElementId, MergeLaneInfo>
CalculateLengthBeforeMergeLane(
    const mapping::v2::SemanticMapManager& v2smm,
    const RouteSectionsInfo& sections_info,
    const absl::flat_hash_set<mapping::ElementId>& avoid_lanes,
    double preview_length) {
  SCOPED_QTRACE("CalcNaviInfo::CalculateLengthBeforeMergeLane");
  absl::flat_hash_map<mapping::ElementId, MergeLaneInfo> merge_lane_info_map;
  int preview_idx = 0;
  for (; preview_idx + 1 < sections_info.size(); ++preview_idx) {
    if (sections_info.section_segment(preview_idx).lane_ids.empty()) {
      preview_idx = std::max(preview_idx - 1, 0);
      break;
    }
    preview_length -= sections_info.section_segment(preview_idx).length();
    if (preview_length <= 0.0) break;
  }
  int cur_sec_idx = preview_idx;
  while (cur_sec_idx >= 0) {
    const auto& cur_section_seg = sections_info.section_segment(cur_sec_idx);
    const double cur_section_length = cur_section_seg.average_length;
    for (const auto& lane_id : cur_section_seg.lane_ids) {
      SMM2_ASSIGN_LANE_OR_CONTINUE(lane_info, v2smm, lane_id);
      MergeLaneInfo merge_lane_info;
      if (avoid_lanes.contains(lane_id)) {
        merge_lane_info.len_before_merge_lane = cur_section_length;
        merge_lane_info.merge_targets = {};
      } else if (lane_info.proto().is_merging()) {
        merge_lane_info.len_before_merge_lane = cur_section_length;
        merge_lane_info.merge_targets =
            FindMergeTargets(v2smm, sections_info, lane_id, cur_sec_idx);
      } else {
        for (const auto out_lane_id : lane_info.proto().outgoing_lanes()) {
          auto next_lane_id = mapping::ElementId(out_lane_id);
          if (!merge_lane_info_map.contains(next_lane_id)) {
            if (cur_sec_idx != preview_idx) {
              // Out of route section.
              merge_lane_info.len_before_merge_lane = std::max(
                  cur_section_length, merge_lane_info.len_before_merge_lane);
            } else {
              // End of preview distance.
              merge_lane_info.len_before_merge_lane = DBL_MAX;
              merge_lane_info.merge_targets = {};
              break;
            }
          } else {
            const auto& next_merge_lane_info =
                merge_lane_info_map.at(next_lane_id);
            const double length_before_merge =
                std::isinf(next_merge_lane_info.len_before_merge_lane)
                    ? next_merge_lane_info.len_before_merge_lane
                    : next_merge_lane_info.len_before_merge_lane +
                          cur_section_length;
            if (length_before_merge > merge_lane_info.len_before_merge_lane) {
              merge_lane_info.len_before_merge_lane = length_before_merge;

              absl::flat_hash_set<mapping::ElementId> new_merge_targets;
              for (const auto& next_merge_target_id :
                   next_merge_lane_info.merge_targets) {
                SMM2_ASSIGN_LANE_OR_CONTINUE(next_merge_target_info, v2smm,
                                             next_merge_target_id);
                for (const auto& incoming_lane_id :
                     next_merge_target_info.proto().incoming_lanes()) {
                  if (incoming_lane_id != lane_id.value() &&
                      cur_section_seg.id_idx_map.contains(
                          mapping::ElementId(incoming_lane_id))) {
                    new_merge_targets.insert(
                        mapping::ElementId(incoming_lane_id));
                  }
                }
              }
              merge_lane_info.merge_targets = std::move(new_merge_targets);
            }
          }
        }
      }
      merge_lane_info_map[lane_id] = merge_lane_info;
    }
    cur_sec_idx--;
  }
  return merge_lane_info_map;
}

}  // namespace planner
}  // namespace qcraft

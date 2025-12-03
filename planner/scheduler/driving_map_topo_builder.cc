#include "onboard/planner/scheduler/driving_map_topo_builder.h"

#include <algorithm>
#include <queue>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_set.h"
#include "absl/hash/hash.h"
#include "absl/status/status.h"
#include "absl/strings/str_cat.h"
#include "absl/types/span.h"

#include "onboard/container/strong_int.h"
#include "onboard/global/trace.h"
#include "onboard/maps/maps_common.h"
#include "onboard/maps/semantic_map_defs.h"
#include "onboard/math/frenet_common.h"
#include "onboard/planner/util/online_semantic_map_util.h"
#include "onboard/planner/util/planner_semantic_map_util.h"

namespace qcraft::planner {

namespace {

bool ContainsLane(const mapping::SectionInfo& sec, mapping::ElementId lane_id) {
  return std::find(sec.lane_ids.begin(), sec.lane_ids.end(), lane_id) !=
         sec.lane_ids.end();
}

std::vector<mapping::ElementId> FilterVirtualLane(
    const PlannerSemanticMapManager& psmm,
    const std::vector<mapping::ElementId>& ids, bool allow_virtual) {
  if (allow_virtual) {
    return ids;
  }
  std::vector<mapping::ElementId> filtered;
  filtered.reserve(ids.size());
  for (const auto& id : ids) {
    const auto* lane_info = psmm.FindLaneInfoOrNull(id);
    if (lane_info == nullptr || lane_info->IsVirtual()) continue;
    filtered.push_back(id);
  }
  return filtered;
}

struct LaneNode {
  mapping::ElementId id;
  double accum_length;
};

}  // namespace

absl::StatusOr<DrivingMapTopo> BuildDrivingMapByRouteOnOfflineMap(
    const PlannerSemanticMapManager& psmm,
    const RouteSections& sections_from_start) {
  SCOPED_QTRACE("BuildDrivingMapByRouteOnOfflineMap");

  std::vector<const mapping::SectionInfo*> sections_info;
  sections_info.reserve(sections_from_start.size());
  int tot_lanes = 0;

  for (int i = 0; i < sections_from_start.size(); ++i) {
    const auto& sec_seg = sections_from_start.route_section_segment(i);

    const auto* sec_info = psmm.FindSectionInfoOrNull(sec_seg.id);

    if (sec_info == nullptr) {
      return absl::NotFoundError(
          absl::StrCat("Can not find section ", sec_seg.id));
    }

    tot_lanes += sec_info->lane_ids.size();
    sections_info.push_back(sec_info);
  }

  std::vector<DrivingMapTopo::Lane> lanes;
  lanes.reserve(tot_lanes);

  for (int i = 0; i < sections_info.size(); ++i) {
    const auto& sec_seg = sections_from_start.route_section_segment(i);

    const auto* next_sec =
        i + 1 == sections_info.size() ? nullptr : sections_info[i + 1];

    const auto* prev_sec = i == 0 ? nullptr : sections_info[i - 1];

    for (mapping::ElementId lane_id : sections_info[i]->lane_ids) {
      const auto* lane_info = psmm.FindLaneInfoOrNull(lane_id);

      if (lane_info == nullptr) {
        return absl::NotFoundError(absl::StrCat("Can not find lane ", lane_id));
      }

      std::vector<mapping::ElementId> outgoing_lane_ids;
      if (next_sec != nullptr) {
        for (const auto& out_id : lane_info->outgoing_lanes()) {
          if (ContainsLane(*next_sec, out_id)) {
            outgoing_lane_ids.push_back(out_id);
          }
        }
      }

      std::vector<mapping::ElementId> incoming_lane_ids;
      if (prev_sec != nullptr) {
        for (const auto& in_id : lane_info->incoming_lanes()) {
          if (ContainsLane(*prev_sec, in_id)) {
            incoming_lane_ids.push_back(in_id);
          }
        }
      }
      // TODO(weijun): Sample lane boundary.
      lanes.push_back(DrivingMapTopo::Lane{
          .id = lane_id,
          .start_fraction = sec_seg.start_fraction,
          .end_fraction = sec_seg.end_fraction,
          .outgoing_lane_ids = std::move(outgoing_lane_ids),
          .incoming_lane_ids = std::move(incoming_lane_ids)});
    }
  }

  return DrivingMapTopo(std::move(lanes), sections_info.front()->lane_ids);
}

// TODO(weijun): Add unit test once online semantic map converter is done.
absl::StatusOr<DrivingMapTopo> BuildDrivingMapByOnlineMap(
    const PlannerSemanticMapManager& psmm,
    const mapping::OnlineSemanticMapProto& online_map, const Vec2d& ego_pos) {
  std::vector<DrivingMapTopo::Lane> lanes;
  lanes.reserve(online_map.lanes_size());
  std::queue<mapping::ElementId> search_queue;
  absl::flat_hash_set<mapping::ElementId> visited_id;

  constexpr double kProjEpsilon = 1.0;         // m.
  constexpr double kMaxBackProjEpsilon = 8.0;  // m.
  const auto starting_lane_ids = ComputeStartLanesByPos(
      psmm, online_map, ego_pos, kProjEpsilon, kMaxBackProjEpsilon);

  std::vector<std::pair<mapping::ElementId, double>> start_lane_proj;
  start_lane_proj.reserve(starting_lane_ids.size());
  for (const auto lane_id : starting_lane_ids) {
    const auto* lane_info = psmm.FindLaneInfoOrNull(lane_id);
    if (lane_info == nullptr) {
      return absl::NotFoundError(absl::StrCat("Can not find lane ", lane_id));
    }

    const auto ego_sl = lane_info->SmoothXYToSL(ego_pos);
    const double start_fraction =
        std::clamp(ego_sl.s / lane_info->length(), 0.0, 1.0);

    start_lane_proj.emplace_back(lane_id, ego_sl.l);
    visited_id.insert(lane_id);
    search_queue.push(lane_id);
    lanes.push_back(DrivingMapTopo::Lane{
        .id = lane_id,
        .start_fraction = start_fraction,
        .end_fraction = 1.0,
        .outgoing_lane_ids = lane_info->outgoing_lanes(),
        .incoming_lane_ids = lane_info->incoming_lanes(),
    });
  }

  visited_id.clear();

  // BFS
  while (!search_queue.empty()) {
    const auto current_id = search_queue.front();
    visited_id.insert(current_id);
    search_queue.pop();
    const auto* lane_info = psmm.FindLaneInfoOrNull(current_id);
    if (lane_info == nullptr) {
      return absl::NotFoundError(
          absl::StrCat("Can not find lane ", current_id));
    }
    for (const auto& out_lane_id : lane_info->outgoing_lanes()) {
      if (visited_id.contains(out_lane_id)) {
        continue;
      }
      visited_id.insert(out_lane_id);
      search_queue.push(out_lane_id);
      const auto* out_lane_info = psmm.FindLaneInfoOrNull(out_lane_id);
      if (out_lane_info == nullptr) {
        return absl::NotFoundError(
            absl::StrCat("Can not find lane ", out_lane_id));
      }
      lanes.push_back(DrivingMapTopo::Lane{
          .id = out_lane_id,
          .start_fraction = 0.0,
          .end_fraction = 1.0,
          .outgoing_lane_ids = out_lane_info->outgoing_lanes(),
          .incoming_lane_ids = out_lane_info->incoming_lanes(),
      });
    }
  }

  // Sort start lane ids
  std::stable_sort(
      start_lane_proj.begin(), start_lane_proj.end(),
      [](const auto& lhs, const auto& rhs) { return lhs.second < rhs.second; });

  std::vector<mapping::ElementId> sorted_start_lanes;
  sorted_start_lanes.reserve(start_lane_proj.size());
  for (const auto& [id, _] : start_lane_proj) {
    sorted_start_lanes.push_back(id);
  }

  return DrivingMapTopo(std::move(lanes), std::move(sorted_start_lanes));
}

absl::StatusOr<DrivingMapTopo>
BuildDrivingMapAtPosAndHeadingInRadiusWithMaxLength(
    const PlannerSemanticMapManager& psmm, const Vec2d& pos, double heading,
    double radius, double max_forward_length, double heading_diff,
    bool allow_virtual) {
  FUNC_QTRACE();
  const auto lanes_in_radius = psmm.GetLanesInfoWithHeadingAtLevel(
      psmm.GetLevel(), pos, heading, radius, heading_diff);
  // Filter lanes in radius if do not allow virtual lanes.
  std::vector<const mapping::LaneInfo*> filtered_lanes_in_radius;
  filtered_lanes_in_radius.reserve(lanes_in_radius.size());
  for (const auto* lane_info : lanes_in_radius) {
    if (!allow_virtual && lane_info->IsVirtual()) continue;
    filtered_lanes_in_radius.push_back(lane_info);
  }
  // Collect direct children.
  absl::flat_hash_set<mapping::ElementId> childrens;
  for (const auto* lane_info : filtered_lanes_in_radius) {
    if (lane_info == nullptr) continue;
    for (const auto& out_id : lane_info->outgoing_lanes()) {
      childrens.insert(out_id);
    }
  }
  // Collect start search lane ids.
  std::vector<mapping::ElementId> start_search_lane_ids;
  start_search_lane_ids.reserve(filtered_lanes_in_radius.size());
  for (const auto* lane_info : filtered_lanes_in_radius) {
    if (lane_info == nullptr || childrens.contains(lane_info->id)) continue;
    start_search_lane_ids.push_back(lane_info->id);
  }

  // Set projection threshold.
  constexpr double kProjThres = 10.0;
  constexpr double kMaxBackwardProjThres = 15.0;
  const auto start_lane_ids = RecomputeStartLanesByPosWithLaneIds(
      psmm, absl::MakeSpan(start_search_lane_ids), pos, kProjThres,
      kMaxBackwardProjThres);

  absl::flat_hash_set<mapping::ElementId> collected_ids;
  std::vector<LaneNode> search_seq;
  std::vector<DrivingMapTopo::Lane> lanes_topo;
  std::vector<mapping::ElementId> filtered_start_lane_ids;
  filtered_start_lane_ids.reserve(start_lane_ids.size());
  // Init.
  for (int i = 0; i < start_lane_ids.size(); ++i) {
    const auto id = start_lane_ids[i];
    if (collected_ids.contains(id)) continue;
    const auto* lane_info = psmm.FindLaneInfoOrNull(id);
    if (lane_info == nullptr || (lane_info->IsVirtual() && !allow_virtual)) {
      continue;
    }
    const auto pos_sl = lane_info->SmoothXYToSL(pos);
    const double start_fraction =
        std::clamp(pos_sl.s / lane_info->length(), 0.0, 1.0);
    const double length =
        std::clamp(lane_info->length() - pos_sl.s, 0.0, lane_info->length());
    const bool continue_search = length < max_forward_length;
    if (continue_search) {
      search_seq.push_back(LaneNode{
          .id = id,
          .accum_length = length,
      });
    }
    filtered_start_lane_ids.push_back(id);
    lanes_topo.push_back(DrivingMapTopo::Lane{
        .id = id,
        .start_fraction = start_fraction,
        .end_fraction = 1.0,
        .outgoing_lane_ids =
            continue_search
                ? FilterVirtualLane(psmm, lane_info->outgoing_lanes(),
                                    allow_virtual)
                : std::vector<mapping::ElementId>(),
        .incoming_lane_ids =
            FilterVirtualLane(psmm, lane_info->incoming_lanes(), allow_virtual),
    });
    collected_ids.insert(id);
  }

  int seq_idx = 0;
  while (seq_idx < search_seq.size()) {
    const auto search_node = search_seq[seq_idx];
    const auto& lane_id = search_node.id;
    const auto* lane_info = psmm.FindLaneInfoOrNull(lane_id);
    for (const auto& out_lane_id : lane_info->outgoing_lanes()) {
      const auto* out_lane_info = psmm.FindLaneInfoOrNull(out_lane_id);
      if (out_lane_info == nullptr ||
          (out_lane_info->IsVirtual() && !allow_virtual)) {
        continue;
      }
      const auto accum_length = search_node.accum_length + lane_info->length();
      const bool continue_search = accum_length < max_forward_length;
      if (continue_search) {
        search_seq.push_back(LaneNode{
            .id = out_lane_id,
            .accum_length = accum_length,
        });
      }
      if (!collected_ids.contains(out_lane_id)) {
        lanes_topo.push_back(DrivingMapTopo::Lane{
            .id = out_lane_id,
            .start_fraction = 0.0,
            .end_fraction = 1.0,
            .outgoing_lane_ids =
                continue_search
                    ? FilterVirtualLane(psmm, out_lane_info->outgoing_lanes(),
                                        allow_virtual)
                    : std::vector<mapping::ElementId>(),
            .incoming_lane_ids = FilterVirtualLane(
                psmm, out_lane_info->incoming_lanes(), allow_virtual),
        });
        collected_ids.insert(out_lane_id);
      }
    }
    seq_idx++;
  }
  return DrivingMapTopo(std::move(lanes_topo),
                        std::move(filtered_start_lane_ids));
}

}  // namespace qcraft::planner

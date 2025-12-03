#include "onboard/planner/router/route_sections_util.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/container/flat_hash_set.h"
#include "absl/hash/hash.h"
#include "absl/status/status.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_format.h"
#include "absl/types/span.h"

#include "onboard/container/strong_int.h"
#include "onboard/global/logging.h"
#include "onboard/maps/lane_path_data.h"
#include "onboard/maps/lane_point.h"
#include "onboard/maps/map_or_die_macros.h"
#include "onboard/maps/maps_common.h"
#include "onboard/maps/proto/semantic_map.pb.h"
#include "onboard/maps/semantic_map_util.h"
#include "onboard/math/frenet_common.h"
#include "onboard/math/frenet_frame.h"
#include "onboard/planner/router/route_util.h"
#include "onboard/planner/util/lane_path_util.h"
#include "onboard/planner/util/lane_point_util.h"
#include "onboard/planner/util/planner_semantic_map_util.h"
#include "onboard/utils/source_location.h"
#include "onboard/utils/status_macros.h"

namespace qcraft::planner {

absl::StatusOr<RouteSections> AlignRouteSections(
    const RouteSections& global_sections, const RouteSections& local_sections) {
  if (local_sections.empty() || global_sections.empty()) {
    return absl::InvalidArgumentError("empty section is not allowed.");
  }

  const auto local_first_index_or =
      global_sections.FindSectionSegment(RouteSections::RouteSectionSegment{
          .id = local_sections.front().id,
          .start_fraction = local_sections.start_fraction(),
          .end_fraction = local_sections.start_fraction()});

  if (local_first_index_or.ok()) {
    return RouteSections(
        local_sections.start_fraction(), global_sections.end_fraction(),
        std::vector<mapping::SectionId>(global_sections.section_ids().begin() +
                                            local_first_index_or.value(),
                                        global_sections.section_ids().end()),
        global_sections.destination());
  }

  const auto global_first_index_or =
      local_sections.FindSectionSegment(RouteSections::RouteSectionSegment{
          .id = global_sections.front().id,
          .start_fraction = global_sections.start_fraction(),
          .end_fraction = global_sections.start_fraction()});

  if (!global_first_index_or.ok()) {
    return absl::NotFoundError(absl::StrCat(
        "No overlap between sections:", "global:",
        global_sections.DebugString(), "local:", local_sections.DebugString()));
  }

  // Backward extend global sections along local sections.
  std::vector<mapping::SectionId> new_section_ids(
      local_sections.section_ids().begin(),
      local_sections.section_ids().begin() + global_first_index_or.value());

  for (int i = 0; i < global_sections.size(); ++i) {
    new_section_ids.push_back(global_sections.section_ids()[i]);
  }

  return RouteSections(
      local_sections.start_fraction(), global_sections.end_fraction(),
      std::move(new_section_ids), global_sections.destination());
}

absl::StatusOr<RouteSections> SpliceRouteSections(
    const RouteSections& origin_sections,
    const RouteSections& target_sections) {
  if (target_sections.empty()) {
    return absl::InvalidArgumentError("target sections are empty.");
  }
  if (origin_sections.empty()) {
    return target_sections;
  }
  const auto origin_back_index_or =
      target_sections.FindSectionSegment(RouteSections::RouteSectionSegment{
          .id = origin_sections.back().id,
          .start_fraction = origin_sections.end_fraction(),
          .end_fraction = origin_sections.end_fraction()});
  if (origin_back_index_or.ok()) {
    std::vector<mapping::SectionId> new_section_ids(
        origin_sections.section_ids().begin(),
        origin_sections.section_ids().end());
    for (int i = *origin_back_index_or + 1; i < target_sections.size(); ++i) {
      new_section_ids.push_back(target_sections.section_ids()[i]);
    }
    return RouteSections(
        origin_sections.start_fraction(), target_sections.end_fraction(),
        std::move(new_section_ids), target_sections.destination());
  }

  const auto target_back_index_or =
      origin_sections.FindSectionSegment(RouteSections::RouteSectionSegment{
          .id = target_sections.back().id,
          .start_fraction = target_sections.end_fraction(),
          .end_fraction = target_sections.end_fraction()});
  if (!target_back_index_or.ok()) {
    return absl::NotFoundError(
        absl::StrCat("No overlap between sections:", "origin:",
                     origin_sections.DebugString(),
                     "target:", target_sections.DebugString()));
  }
  return RouteSections(
      origin_sections.start_fraction(), target_sections.end_fraction(),
      std::vector<mapping::SectionId>(
          origin_sections.section_ids().begin(),
          origin_sections.section_ids().begin() + *target_back_index_or + 1),
      target_sections.destination());
}

absl::StatusOr<RouteSections> AppendRouteSectionsToTail(
    const RouteSections& origin_sections,
    const RouteSections& global_sections) {
  if (global_sections.empty()) {
    return absl::InvalidArgumentError("Global sections are empty!");
  }
  if (origin_sections.empty()) {
    return global_sections;
  }
  bool find = false;
  const int origin_size = origin_sections.size();
  const int global_size = global_sections.size();
  int origin_start = 0, global_start = 0;
  for (int i = 0; i < origin_size; ++i) {
    for (int j = 0; j < global_size; ++j) {
      if (origin_sections.section_id(i) == global_sections.section_id(j)) {
        origin_start = i;
        global_start = j;
        find = true;
        break;
      }
    }
    if (find) break;
  }
  if (!find) {
    return absl::InvalidArgumentError(
        "Origin sections and global sections are different, should reset.");
  }

  int origin_end = origin_start, global_end = global_start;
  for (; origin_end < origin_size && global_end < global_size;
       ++origin_end, ++global_end) {
    if (origin_sections.section_id(origin_end) !=
        global_sections.section_id(global_end)) {
      break;
    }
  }
  if (origin_end != origin_size) {
    return absl::InvalidArgumentError(
        "Origin sections and global section are mismatching, should reset.");
  }
  std::vector<mapping::SectionId> new_sec_ids = {
      origin_sections.section_ids().begin(),
      origin_sections.section_ids().end()};
  new_sec_ids.insert(new_sec_ids.end(),
                     global_sections.section_ids().begin() + global_end,
                     global_sections.section_ids().end());

  return RouteSections(origin_sections.start_fraction(),
                       global_sections.end_fraction(), std::move(new_sec_ids),
                       global_sections.destination());
}

RouteSections RouteSectionsFromCompositeLanePath(
    const mapping::v2::SemanticMapManager& smm, const CompositeLanePath& clp) {
  std::vector<mapping::SectionId> all_sec_ids;
  for (const auto& lane_path : clp.lane_paths()) {
    for (const auto lane_id : lane_path.lane_ids()) {
      SMM_LANE_PROTO_OR_CONTINUE(lane_proto, smm, lane_id);
      const auto sec_id = lane_proto->section_id();
      if (!all_sec_ids.empty() && sec_id == all_sec_ids.back().value())
        continue;

      all_sec_ids.push_back(mapping::SectionId(sec_id));
    }
  }
  return RouteSections(clp.front().fraction(), clp.back().fraction(),
                       std::move(all_sec_ids), clp.back());
}

RouteSections RouteSectionsFromCompositeLanePath(
    const PlannerSemanticMapManager& psmm, const CompositeLanePath& clp) {
  std::vector<mapping::SectionId> all_sec_ids;
  for (const auto& lane_path : clp.lane_paths()) {
    for (const auto lane_id : lane_path.lane_ids()) {
      SMM_LANE_PROTO_OR_CONTINUE(lane_proto, psmm, lane_id);
      const auto sec_id = lane_proto->section_id();
      if (!all_sec_ids.empty() &&
          mapping::SectionId(sec_id) == all_sec_ids.back())
        continue;

      all_sec_ids.push_back(mapping::SectionId(sec_id));
    }
  }
  return RouteSections(clp.front().fraction(), clp.back().fraction(),
                       std::move(all_sec_ids), clp.back());
}

absl::StatusOr<PointOnRouteSections>
FindSmoothPointOnRouteSectionsByLateralOffset(
    const PlannerSemanticMapManager& psmm,
    const RouteSectionsInfo& sections_info, const Vec2d& query_point,
    double lat_dist_thres) {
  // To deal with numerical error in frenet projection.
  constexpr double kEpsilon = 0.1;  // m.

  const auto& sections = sections_info.section_segments();
  double accum_s = 0.0;
  for (int sec_idx = 0; sec_idx < sections.size(); ++sec_idx) {
    const auto& sec = sections[sec_idx];
    if (sec.start_fraction == sec.end_fraction) continue;

    for (const auto lane_id : sec.lane_ids) {
      SMM_ASSIGN_LANE_OR_CONTINUE_ISSUE(lane_info, psmm, lane_id);
      const auto points = mapping::ResampleLanePoints(
          lane_info, sec.start_fraction, sec.end_fraction,
          /*cumulative_lengths=*/nullptr);

      ASSIGN_OR_RETURN(
          const auto ff,
          BuildBruteForceFrenetFrame(points, /*down_sample_raw_points=*/true));
      const FrenetCoordinate sl = ff.XYToSL(query_point);
      if (ff.start_s() - kEpsilon < sl.s && sl.s < ff.end_s() &&
          std::abs(sl.l) < lat_dist_thres) {
        const double frac = std::max(sl.s / lane_info.length(), 0.0);
        return PointOnRouteSections{
            .accum_s = accum_s + sec.average_length * frac,
            .section_idx = sec_idx,
            .fraction = std::clamp(frac + sec.start_fraction, 0.0, 1.0),
            .lane_id = lane_id};
      }
    }
    accum_s += sec.length();
  }
  return absl::NotFoundError(absl::StrCat(
      "FindSmoothPointOnRouteSectionsByLateralOffset:Point(", query_point.x(),
      ",", query_point.y(), ")is not on route sections:",
      sections_info.route_sections()->DebugString()));
}

absl::StatusOr<mapping::LanePath>
FindClosestLanePathOnRouteSectionsToSmoothPoint(
    const PlannerSemanticMapManager& psmm, const RouteSections& sections,
    const Vec2d& query_point, double* proj_s) {
  ASSIGN_OR_RETURN(const auto lane_paths,
                   CollectAllLanePathOnRouteSectionsFromStart(psmm, sections));

  double ego_s, min_lat_error = std::numeric_limits<double>::infinity();
  const mapping::LanePath* nearest_lane_path = nullptr;
  constexpr double kLonEpsilon = 0.1;  // m.
  for (const auto& lane_path : lane_paths) {
    const auto points = SampleLanePathPoints(psmm, lane_path);
    ASSIGN_OR_CONTINUE(
        const auto ff,
        BuildBruteForceFrenetFrame(points, /*down_sample_raw_points=*/true));

    const FrenetCoordinate sl = ff.XYToSL(query_point);
    if (ff.start_s() - kLonEpsilon < sl.s && sl.s < ff.end_s()) {
      if (std::abs(sl.l) < min_lat_error) {
        min_lat_error = std::abs(sl.l);
        ego_s = sl.s;
        nearest_lane_path = &lane_path;
      }
    }
  }
  if (nearest_lane_path == nullptr) {
    return absl::NotFoundError(
        absl::StrCat("FindClosestLanePathOnRouteSectionsToSmoothPoint:Point (",
                     query_point.x(), ",", query_point.y(),
                     ") is not on route sections:", sections.DebugString()));
  }

  if (proj_s != nullptr) *proj_s = ego_s;
  return *nearest_lane_path;
}

absl::StatusOr<RouteSections> ClampRouteSectionsBeforeArcLength(
    const PlannerSemanticMapManager& psmm,
    const RouteSections& raw_route_sections, double len) {
  if (len < 0.0) {
    return absl::NotFoundError("Arc length is negative");
  }
  if (raw_route_sections.empty()) {
    return absl::NotFoundError("Invalid route section, it is empty");
  }

  std::vector<mapping::SectionId> section_ids;
  double accum_s = 0.0;
  int i = 0;
  for (; i < raw_route_sections.size(); ++i) {
    const auto& section_seg = raw_route_sections.route_section_segment(i);
    SMM_ASSIGN_SECTION_OR_BREAK(section_info, psmm, section_seg.id);

    const double this_len =
        section_info.average_length *
        (section_seg.end_fraction - section_seg.start_fraction);

    section_ids.push_back(section_seg.id);
    if (accum_s + this_len > len) {
      return RouteSections(
          raw_route_sections.start_fraction(),
          std::clamp(section_seg.end_fraction - (accum_s + this_len - len) /
                                                    section_info.average_length,
                     0.0, 1.0),
          std::move(section_ids), raw_route_sections.destination());
    }

    accum_s += this_len;
  }
  const double end_fraction = (i == raw_route_sections.size())
                                  ? raw_route_sections.end_fraction()
                                  : 1.0;
  if (i == 0) {
    return absl::NotFoundError(absl::StrCat(
        "Input route sections does not match psmm, route sections: ",
        raw_route_sections.DebugString()));
  }
  return RouteSections(raw_route_sections.start_fraction(), end_fraction,
                       std::move(section_ids),
                       raw_route_sections.destination());
}

absl::StatusOr<RouteSections> ClampRouteSectionsAfterArcLength(
    const PlannerSemanticMapManager& psmm,
    const RouteSections& raw_route_sections, double len) {
  double accum_s = 0.0;
  int i = 0;
  double start_fraction = raw_route_sections.start_fraction();
  bool map_not_load = false;
  for (; i < raw_route_sections.size(); ++i) {
    const auto& section_seg = raw_route_sections.route_section_segment(i);
    const auto* section_info = psmm.FindSectionInfoOrNull(section_seg.id);
    if (section_info == nullptr) {
      map_not_load = true;
      break;
    }
    const double this_len =
        section_info->average_length *
        (section_seg.end_fraction - section_seg.start_fraction);

    if (accum_s + this_len > len) {
      start_fraction = std::clamp(
          section_seg.end_fraction -
              (accum_s + this_len - len) / section_info->average_length,
          0.0, 1.0);
      break;
    }
    accum_s += this_len;
  }

  if (map_not_load) {
    return absl::NotFoundError(
        "Map is not loaded yet after arc length in route sections.");
  }

  if (i == raw_route_sections.size()) {
    return absl::NotFoundError("Arc length is larger than sections length");
  }

  return RouteSections(start_fraction, raw_route_sections.end_fraction(),
                       std::vector<mapping::SectionId>(
                           raw_route_sections.section_ids().begin() + i,
                           raw_route_sections.section_ids().end()),
                       raw_route_sections.destination());
}

absl::StatusOr<std::vector<mapping::LanePath>>
CollectAllLanePathOnRouteSectionsFromStart(
    const PlannerSemanticMapManager& psmm,
    const RouteSections& route_sections) {
  std::vector<mapping::LanePath> final_results;
  std::vector<mapping::LanePath> lane_path_vec;

  SMM_ASSIGN_SECTION_OR_RETURN(
      first_section_info, psmm, route_sections.front().id,
      absl::NotFoundError(absl::StrCat("Can not find first section id, ",
                                       route_sections.front().id)));
  lane_path_vec.reserve(first_section_info.lane_ids.size());
  for (const auto lane_id : first_section_info.lane_ids) {
    ASSIGN_OR_RETURN(auto first_lp,
                     BuildLanePathFromData(
                         mapping::LanePathData(
                             route_sections.front().start_fraction,
                             route_sections.front().end_fraction, {lane_id}),
                         psmm),
                     absl::NotFoundError(absl::StrFormat(
                         "Load first section %d but not load lane id %d.",
                         route_sections.front().id, lane_id)));
    lane_path_vec.push_back(std::move(first_lp));
  }

  for (int i = 1; i < route_sections.size(); ++i) {
    const auto& route_section_seg = route_sections.route_section_segment(i);
    SMM_ASSIGN_SECTION_OR_BREAK(section_info, psmm, route_section_seg.id);

    std::vector<mapping::LanePath> new_lane_path_vec;

    for (const auto& lane_path : lane_path_vec) {
      bool has_outgoing = false;
      SMM_ASSIGN_LANE_OR_CONTINUE(last_lane_info, psmm,
                                  lane_path.back().lane_id());
      for (const auto lane_id : section_info.lane_ids) {
        if (IsOutgoingLane(psmm, last_lane_info, lane_id)) {
          has_outgoing = true;
          std::vector<mapping::ElementId> lane_ids = lane_path.lane_ids();
          lane_ids.push_back(lane_id);
          new_lane_path_vec.emplace_back(
              psmm.semantic_map_manager(), std::move(lane_ids),
              lane_path.front().fraction(), route_section_seg.end_fraction);
        }
      }

      if (!has_outgoing) {
        final_results.push_back(lane_path);
      }
    }

    lane_path_vec = std::move(new_lane_path_vec);
  }

  for (auto& lane_path : lane_path_vec) {
    final_results.emplace_back(std::move(lane_path));
  }

  return final_results;
}

absl::StatusOr<std::vector<mapping::LanePath>>
CollectAllLanePathOnRouteSections(const PlannerSemanticMapManager& psmm,
                                  const RouteSections& route_sections) {
  std::vector<mapping::LanePath> final_results;
  std::vector<mapping::LanePath> lane_path_vec;

  SMM_ASSIGN_SECTION_OR_ERROR_ISSUE(first_section_info, psmm,
                                    route_sections.front().id);
  lane_path_vec.reserve(first_section_info.lane_ids.size());
  for (const auto lane_id : first_section_info.lane_ids) {
    lane_path_vec.emplace_back(
        mapping::LanePath(psmm.semantic_map_manager(), {lane_id},
                          route_sections.front().start_fraction,
                          route_sections.front().end_fraction));
  }

  for (int i = 1; i < route_sections.size(); ++i) {
    const auto& route_section_seg = route_sections.route_section_segment(i);
    SMM_ASSIGN_SECTION_OR_BREAK(section_info, psmm, route_section_seg.id);

    std::vector<mapping::LanePath> new_lane_path_vec;
    absl::flat_hash_set<mapping::ElementId> has_incoming_lane_ids;
    for (const auto& lane_path : lane_path_vec) {
      bool has_outgoing = false;
      SMM_ASSIGN_LANE_OR_CONTINUE_ISSUE(last_lane_info, psmm,
                                        lane_path.back().lane_id());
      for (const auto lane_id : section_info.lane_ids) {
        if (IsOutgoingLane(psmm, last_lane_info, lane_id)) {
          has_outgoing = true;
          has_incoming_lane_ids.insert(lane_id);
          std::vector<mapping::ElementId> lane_ids = lane_path.lane_ids();
          lane_ids.push_back(lane_id);
          new_lane_path_vec.emplace_back(
              psmm.semantic_map_manager(), std::move(lane_ids),
              lane_path.front().fraction(), route_section_seg.end_fraction);
        }
      }

      if (!has_outgoing) {
        final_results.push_back(lane_path);
      }
    }

    lane_path_vec = std::move(new_lane_path_vec);

    for (const auto lane_id : section_info.lane_ids) {
      if (!has_incoming_lane_ids.contains(lane_id)) {
        lane_path_vec.emplace_back(mapping::LanePath(
            psmm.semantic_map_manager(), {lane_id},
            /*start_fraction=*/0.0, route_section_seg.end_fraction));
      }
    }
  }

  for (auto& lane_path : lane_path_vec) {
    final_results.emplace_back(std::move(lane_path));
  }

  return final_results;
}

RouteSections BackwardExtendRouteSections(const PlannerSemanticMapManager& psmm,
                                          const RouteSections& raw_sections,
                                          double extend_len) {
  if (extend_len <= 0.0) {
    return raw_sections;
  }

  std::vector<mapping::SectionId> extend_section_ids;
  double start_fraction = raw_sections.start_fraction();
  mapping::SectionId section_id = raw_sections.front().id;
  while (extend_len > 0.0) {
    SMM_ASSIGN_SECTION_OR_BREAK_ISSUE(section_info, psmm, section_id);
    if (start_fraction == 0.0) {
      if (section_info.incoming_section_ids.empty()) {
        break;
      }
      const auto prev_id = section_info.incoming_section_ids.front();
      SMM_ASSIGN_SECTION_OR_BREAK_ISSUE(prev_section_info, psmm, prev_id);
      extend_section_ids.push_back(prev_id);
      if (extend_len <= prev_section_info.average_length) {
        start_fraction = 1.0 - extend_len / prev_section_info.average_length;
        break;
      }

      extend_len -= prev_section_info.average_length;
      section_id = prev_id;
      continue;
    }

    const double rest_len = section_info.average_length * start_fraction;
    if (rest_len > extend_len) {
      start_fraction = std::max(
          0.0, start_fraction - extend_len / section_info.average_length);
      break;
    } else {
      start_fraction = 0.0;
      extend_len -= rest_len;
    }
  }

  std::reverse(extend_section_ids.begin(), extend_section_ids.end());
  extend_section_ids.insert(extend_section_ids.end(),
                            raw_sections.section_ids().begin(),
                            raw_sections.section_ids().end());

  return RouteSections(start_fraction, raw_sections.end_fraction(),
                       std::move(extend_section_ids),
                       raw_sections.destination());
}

absl::StatusOr<RouteSections> BackwardExtendRouteSectionsFromPos(
    const PlannerSemanticMapManager& psmm, const RouteSections& raw_sections,
    const Vec2d& pos, double extend_len) {
  ASSIGN_OR_RETURN(const auto proj_sections,
                   ClampRouteSectionsBeforeArcLength(
                       psmm, raw_sections, kMaxDistanceForClampingSections));
  ASSIGN_OR_RETURN(
      const auto lane_paths,
      CollectAllLanePathOnRouteSectionsFromStart(psmm, proj_sections));
  double min_lat_error = std::numeric_limits<double>::infinity();
  double pos_arclength = 0.0;
  const mapping::LanePath* nearest_lane_path = nullptr;
  constexpr auto kShortestPathThreshold = 0.1;  // m.
  for (const auto& lane_path : lane_paths) {
    if (lane_path.length() < kShortestPathThreshold && lane_path.size() > 0) {
      const auto geom_dist =
          pos.DistanceTo(ComputeLanePointPos(psmm, lane_path.front()));
      if (geom_dist < min_lat_error) {
        nearest_lane_path = &lane_path;
        min_lat_error = geom_dist;
      }
      continue;
    }
    const auto points = SampleLanePathPoints(psmm, lane_path);
    ASSIGN_OR_CONTINUE(
        const auto ff,
        BuildBruteForceFrenetFrame(points, /*down_sample_raw_points=*/true));
    const FrenetCoordinate sl = ff.XYToSL(pos);
    constexpr double kEpsilon = 10.0;  // m.
    if (ff.start_s() - kEpsilon < sl.s && sl.s < ff.end_s()) {
      if (std::abs(sl.l) < min_lat_error) {
        min_lat_error = std::abs(sl.l);
        nearest_lane_path = &lane_path;
        pos_arclength = sl.s;
      }
    }
  }
  if (nearest_lane_path == nullptr) {
    return absl::NotFoundError(absl::StrCat(
        "BackwardExtendRouteSectionsFromPos:Point (", pos.x(), ",", pos.y(),
        ") is not on route sections:", proj_sections.DebugString()));
  }
  const auto extend_lp = BackwardExtendLanePath(
      psmm, nearest_lane_path->AfterArclength(pos_arclength), extend_len);

  const auto extended_sections =
      RouteSections::BuildFromLanePath(psmm, extend_lp);
  return SpliceRouteSections(extended_sections, raw_sections);
}

absl::StatusOr<mapping::LanePath> ForwardExtendLanePathOnRouteSections(
    const PlannerSemanticMapManager& psmm, const RouteSections& route_sections,
    const mapping::LanePath& raw_lane_path, double extend_len,
    const RouteNaviInfo& route_navi_info) {
  if (raw_lane_path.IsEmpty()) {
    return absl::NotFoundError("Raw lane path is empty!");
  }
  if (extend_len <= 0.0) {
    return raw_lane_path;
  }

  const auto& start_lp = raw_lane_path.back();
  // No avoid lane since we don't use driving distance here.
  const RouteSectionsInfo sections_info(psmm, &route_sections);
  const auto& sections = sections_info.section_segments();
  // Find start section if exists.
  int sec_idx = 0;
  for (; sec_idx < sections.size(); ++sec_idx) {
    if (sections[sec_idx].contains(start_lp)) {
      break;
    }
  }
  if (sec_idx == sections.size()) {
    return absl::NotFoundError("Start section not found.");
  }

  std::vector<mapping::ElementId> lane_ids(raw_lane_path.lane_ids());
  auto cur_lane_id = start_lp.lane_id();
  double end_fraction = start_lp.fraction();
  while (extend_len > 0.0) {
    SMM_ASSIGN_LANE_OR_BREAK(lane_info, psmm, cur_lane_id);
    if (end_fraction == 1.0) {
      if (++sec_idx == sections.size()) break;

      cur_lane_id = FindMostReasonableOutgoingLaneIdOrInvalid(
          psmm, lane_info, route_navi_info, sections[sec_idx]);
      if (cur_lane_id == mapping::kInvalidElementId) break;

      lane_ids.push_back(cur_lane_id);
      end_fraction = 0.0;
    } else {
      const double lane_len = lane_info.length();
      if (sec_idx + 1 == sections.size()) {
        end_fraction =
            std::min(sections.back().end_fraction,
                     std::min(extend_len / lane_len + end_fraction, 1.0));
        break;
      }
      const double len =
          lane_len * (sections[sec_idx].end_fraction - end_fraction);
      if (len >= extend_len) {
        end_fraction += extend_len / lane_len;
        break;
      }
      extend_len -= len;
      end_fraction = 1.0;
    }
  }

  return mapping::LanePath(psmm.semantic_map_manager(), std::move(lane_ids),
                           raw_lane_path.start_fraction(), end_fraction);
}

absl::StatusOr<mapping::LanePath> BackwardExtendLanePathOnRouteSections(
    const PlannerSemanticMapManager& psmm, const RouteSections& route_sections,
    const mapping::LanePath& raw_lane_path, double extend_len) {
  if (raw_lane_path.IsEmpty()) {
    return absl::NotFoundError("Raw lane path is empty!");
  }
  if (extend_len <= 0.0) {
    return raw_lane_path;
  }

  const auto& start_lp = raw_lane_path.front();
  // No avoid lane since we don't use driving distance here.
  const RouteSectionsInfo sections_info(psmm, &route_sections);
  const auto& sections = sections_info.section_segments();
  // Find start section if exists.
  int sec_idx = 0;
  for (; sec_idx < sections.size(); ++sec_idx) {
    if (sections[sec_idx].contains(start_lp)) {
      break;
    }
  }
  if (sec_idx == sections.size()) {
    return absl::NotFoundError(
        "Cannot find start lane path on route sections.");
  }

  std::vector<mapping::ElementId> lane_ids(raw_lane_path.lane_ids());
  auto cur_lane_id = start_lp.lane_id();
  double start_fraction = start_lp.fraction();
  while (extend_len > 0.0) {
    SMM_ASSIGN_LANE_OR_BREAK_ISSUE(lane_info, psmm, cur_lane_id);
    if (start_fraction == 0.0) {
      if (--sec_idx < 0) {
        ASSIGN_OR_RETURN(const auto tmp_lane_path,
                         BuildLanePathFromData(
                             mapping::LanePathData(start_fraction,
                                                   raw_lane_path.end_fraction(),
                                                   std::move(lane_ids)),
                             psmm));
        return BackwardExtendLanePath(psmm, tmp_lane_path, extend_len);
      }

      const auto& id_map = sections[sec_idx].id_idx_map;
      cur_lane_id = mapping::kInvalidElementId;
      for (const auto lane_id : lane_info.incoming_lanes()) {
        if (id_map.contains(lane_id)) {
          cur_lane_id = lane_id;
          break;
        }
      }
      if (cur_lane_id == mapping::kInvalidElementId) {
        ASSIGN_OR_RETURN(const auto tmp_lane_path,
                         BuildLanePathFromData(
                             mapping::LanePathData(start_fraction,
                                                   raw_lane_path.end_fraction(),
                                                   std::move(lane_ids)),
                             psmm));
        return BackwardExtendLanePath(psmm, tmp_lane_path, extend_len);
      }

      lane_ids.insert(lane_ids.begin(), cur_lane_id);
      start_fraction = 1.0;
    } else {
      const double len = lane_info.length();
      if (len * start_fraction >= extend_len) {
        start_fraction -= extend_len / len;
        break;
      } else {
        extend_len -= len * start_fraction;
        start_fraction = 0.0;
      }
    }
  }

  return mapping::LanePath(psmm.semantic_map_manager(), std::move(lane_ids),
                           start_fraction, raw_lane_path.end_fraction());
}

absl::StatusOr<mapping::LanePath> FindClosestTargetLanePathOnReset(
    const PlannerSemanticMapManager& psmm, const RouteSections& prev_sections,
    const Vec2d& ego_pos, const RouteNaviInfo& route_navi_info) {
  ASSIGN_OR_RETURN(
      const auto project_route_sections,
      ClampRouteSectionsBeforeArcLength(
          psmm, prev_sections,
          kMaxDistanceForClampingSections + kDrivePassageKeepBehindLength));

  double ego_proj_s;
  ASSIGN_OR_RETURN(const auto ego_lane_path,
                   FindClosestLanePathOnRouteSectionsToSmoothPoint(
                       psmm, project_route_sections, ego_pos, &ego_proj_s));
  const auto clamped_lane_path = ego_lane_path.BeforeArclength(ego_proj_s);

  const double local_horizon = prev_sections.planning_horizon(psmm);
  ASSIGN_OR_RETURN(
      const auto local_route_sections,
      ClampRouteSectionsBeforeArcLength(
          psmm, prev_sections, local_horizon + kDrivePassageKeepBehindLength));

  return ForwardExtendLanePathOnRouteSections(psmm, local_route_sections,
                                              clamped_lane_path, local_horizon,
                                              route_navi_info);
}

}  // namespace qcraft::planner

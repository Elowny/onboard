#include "onboard/planner/scheduler/straighten_reference_line.h"

#include <stddef.h>

#include <algorithm>
#include <cmath>
#include <iterator>
#include <map>
#include <optional>
#include <ostream>
#include <queue>
#include <string>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/container/flat_hash_set.h"
#include "absl/hash/hash.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_join.h"
#include "absl/types/span.h"
#include "gflags/gflags.h"
#include "glog/logging.h"

#include "onboard/container/strong_int.h"
#include "onboard/global/logging.h"
#include "onboard/global/trace.h"
#include "onboard/maps/lane_path.h"
#include "onboard/maps/lane_path_data.h"
#include "onboard/maps/lane_point.h"
#include "onboard/maps/map_or_die_macros.h"
#include "onboard/maps/maps_common.h"
#include "onboard/maps/proto/semantic_map.pb.h"
#include "onboard/maps/semantic_map_defs.h"
#include "onboard/math/frenet_frame.h"
#include "onboard/math/piecewise_linear_function.h"
#include "onboard/math/util.h"
#include "onboard/math/vec.h"
#include "onboard/planner/common/path_sl_boundary.h"
#include "onboard/planner/common/plot_util.h"
#include "onboard/planner/router/drive_passage.h"
#include "onboard/planner/router/drive_passage_builder.h"
#include "onboard/planner/scheduler/path_boundary_builder.h"
#include "onboard/planner/util/lane_path_util.h"
#include "onboard/utils/source_location.h"
#include "onboard/utils/status_builder.h"
#include "onboard/utils/status_macros.h"
#include "onboard/vis/common/color.h"

DEFINE_bool(planner_visualize_straighten_result, false,
            "Debug straighten result.");

namespace qcraft::planner {
namespace {
using LaneInfoPtr = const mapping::LaneInfo*;

inline bool IsLaneTypeExitOrEntrance(const mapping::LaneInfo& lane_info) {
  return lane_info.Type() == mapping::LaneProto::ENTRANCE ||
         lane_info.Type() == mapping::LaneProto::EXIT;
}

std::vector<std::vector<LaneInfoPtr>> FindRampLaneInfosFromStart(
    const PlannerSemanticMapManager& psmm,
    const mapping::LaneInfo& start_lane_info,
    absl::flat_hash_set<mapping::ElementId>* visited) {
  std::vector<std::vector<LaneInfoPtr>> results;

  std::queue<std::pair<LaneInfoPtr, std::vector<LaneInfoPtr>>> que;
  que.push({&start_lane_info, {&start_lane_info}});
  visited->insert(start_lane_info.id);
  while (!que.empty()) {
    auto [cur_lane_info, cur_lane_infos] = que.front();
    que.pop();
    bool has_ramp_outgoing = false;
    const auto& outgoing_lane_ids = cur_lane_info->proto->outgoing_lanes();
    bool load_outgoing_lanes = outgoing_lane_ids.empty();
    for (const auto out_lane_id : outgoing_lane_ids) {
      SMM_ASSIGN_LANE_OR_CONTINUE(out_lane_info, psmm,
                                  mapping::ElementId(out_lane_id));
      load_outgoing_lanes = true;
      if (IsLaneTypeExitOrEntrance(out_lane_info)) {
        has_ramp_outgoing = true;
        std::vector<LaneInfoPtr> next_lane_infos = cur_lane_infos;
        next_lane_infos.push_back(&out_lane_info);
        visited->insert(out_lane_info.id);
        que.push({&out_lane_info, std::move(next_lane_infos)});
      }
    }
    // Make sure the full ramp lane path is loaded.
    if (load_outgoing_lanes && !has_ramp_outgoing) {
      results.push_back(std::move(cur_lane_infos));
    }
  }
  return results;
}

bool ShouldStraightenRampLaneIds(const PlannerSemanticMapManager& psmm,
                                 const std::vector<LaneInfoPtr>& lane_infos) {
  VLOG(1) << "Candidate lane ids to be Straightened: "
          << absl::StrJoin(lane_infos, ",",
                           [](std::string* out, const mapping::LaneInfo* i) {
                             out->append(std::to_string(i->id));
                           });
  if (lane_infos.empty()) return false;

  std::vector<Vec2d> points;
  int points_size = 0;
  for (const auto* lane_info_ptr : lane_infos) {
    points_size += lane_info_ptr->points_smooth.size();
  }
  points.reserve(points_size);

  const auto& incoming_lane_ids = lane_infos.front()->proto->incoming_lanes();
  if (!incoming_lane_ids.empty()) {
    const auto* prev_lane_info =
        psmm.FindLaneInfoOrNull(mapping::ElementId(incoming_lane_ids[0]));
    if (prev_lane_info != nullptr) {
      const auto rbegin_iter = prev_lane_info->points_smooth.rbegin();
      points.push_back(*(std::next(rbegin_iter)));
      points.push_back(*rbegin_iter);
    }
  }

  for (const auto* lane_info_ptr : lane_infos) {
    const auto& cur_points = lane_info_ptr->points_smooth;
    if (!points.empty()) {
      points.pop_back();
    }
    points.insert(points.end(), cur_points.begin(), cur_points.end());
  }
  if (FLAGS_planner_visualize_straighten_result) {
    const std::string topic = absl::StrCat(
        "Candidates/",
        absl::StrJoin(lane_infos, ",",
                      [](std::string* out, const mapping::LaneInfo* i) {
                        out->append(std::to_string(i->id));
                      }));
    std::vector<Vec3d> points_3d(points.size());
    std::transform(points.begin(), points.end(), points_3d.begin(),
                   [](const Vec2d& point) -> Vec3d {
                     return {point, 0.1};
                   });
    DrawPointsLineStripToCanvas(points_3d, topic, vis::Color::kGreen);
  }
  // Reference direction, the absolute value of the angle change is reasonable
  // within this range.
  constexpr double kSineDiffThreshold = 0.1736;  // 10 degree.
  double prev_cross_prod = 0.0;
  for (size_t i = 1; i + 1 < points.size(); ++i) {
    const Vec2d& cur_point = points[i];
    const Vec2d& prev_point = points[i - 1];
    const Vec2d& next_point = points[i + 1];

    const double cross_prod =
        (cur_point - prev_point)
            .normalized()
            .CrossProd((next_point - prev_point).normalized());
    if (std::fabs(cross_prod) < kSineDiffThreshold) {
      prev_cross_prod = cross_prod;
      continue;
    }

    if (prev_cross_prod * cross_prod < 0.0) {
      return true;
    }
  }
  return false;
}

std::vector<std::vector<mapping::ElementId>> CollectAllZigzagLaneIds(
    const PlannerSemanticMapManager& psmm,
    const RouteSections& route_sections) {
  std::vector<std::vector<mapping::ElementId>> final_results;

  std::vector<std::vector<LaneInfoPtr>> ramp_infos;
  absl::flat_hash_set<mapping::ElementId> visited;
  for (const auto& section_id : route_sections.section_ids()) {
    SMM_ASSIGN_SECTION_OR_BREAK(section_info, psmm, section_id);
    for (const auto& lane_id : section_info.lane_ids) {
      if (visited.contains(lane_id)) {
        continue;
      }
      SMM_ASSIGN_LANE_OR_CONTINUE(lane_info, psmm, lane_id);
      if (!IsLaneTypeExitOrEntrance(lane_info)) {
        continue;
      }
      VLOG(1) << "Begin to search ramp lane infos from: " << lane_id;
      auto cur_lane_infos =
          FindRampLaneInfosFromStart(psmm, lane_info, &visited);
      ramp_infos.insert(ramp_infos.end(),
                        std::make_move_iterator(cur_lane_infos.begin()),
                        std::make_move_iterator(cur_lane_infos.end()));
    }
  }

  for (const auto& ramp_lanes : ramp_infos) {
    if (ShouldStraightenRampLaneIds(psmm, ramp_lanes)) {
      std::vector<mapping::ElementId> lane_ids;
      lane_ids.reserve(ramp_lanes.size());
      std::for_each(ramp_lanes.begin(), ramp_lanes.end(),
                    [&lane_ids](const auto* lane_info) {
                      lane_ids.push_back(lane_info->id);
                    });
      final_results.push_back(std::move(lane_ids));
    }
  }
  return final_results;
}

absl::StatusOr<SmoothedReferenceCenterResult>
StraightenLanePathBoundedByPathBoundary(const PlannerSemanticMapManager& psmm,
                                        const DrivePassage& drive_passage,
                                        const PathSlBoundary& path_boundary,
                                        const mapping::LanePath& lane_path,
                                        double half_av_width) {
  absl::flat_hash_map<mapping::ElementId,
                      PiecewiseLinearFunction<double, double>>
      straightened_lane_map;

  const Vec2d line_unit = (drive_passage.stations().front().xy() -
                           drive_passage.stations().back().xy())
                              .normalized();
  int i = 0;
  for (int j = 0; j < lane_path.size(); ++j) {
    const auto& cur_lane_seg = lane_path.lane_segment(j);
    const double cur_length = cur_lane_seg.length();
    const auto cur_lane_id = cur_lane_seg.lane_id;

    std::vector<double> lane_fraction, offset;
    lane_fraction.reserve(CeilToInt(cur_length));
    offset.reserve(CeilToInt(cur_length));
    while (i < drive_passage.size()) {
      const auto& cur_station = drive_passage.station(StationIndex(i));
      if (cur_station.lane_id() != cur_lane_id &&
          !straightened_lane_map.contains(cur_lane_id)) {
        straightened_lane_map[cur_lane_id] = PiecewiseLinearFunction(
            std::move(lane_fraction), std::move(offset));
        break;
      }
      lane_fraction.push_back(cur_station.GetLanePoint().fraction());
      auto [l_min, l_max] =
          path_boundary.QueryTargetBoundaryL(cur_station.accumulated_s());
      l_min += half_av_width;
      l_max -= half_av_width;
      const double lateral_dist =
          Vec2d(cur_station.xy() - drive_passage.stations().front().xy())
              .CrossProd(line_unit);
      offset.push_back(std::clamp(lateral_dist, l_min, l_max));
      ++i;
    }
  }

  if (FLAGS_planner_visualize_straighten_result) {
    const std::string topic = absl::StrJoin(lane_path.lane_ids(), ", ");
    std::vector<Vec3d> points;
    points.reserve(2);
    points.push_back({ArclengthToPos(psmm, lane_path, 0.0), 0.1});
    points.push_back(
        {ArclengthToPos(psmm, lane_path, lane_path.end_s(lane_path.size() - 1)),
         0.1});
    DrawPointsLineStripToCanvas(points, topic, vis::Color::kYellow);
  }
  return SmoothedReferenceCenterResult{.lane_id_to_smoothed_lateral_offset =
                                           std::move(straightened_lane_map)};
}

absl::StatusOr<SmoothedReferenceCenterResult> StraightenLanePathByLaneIds(
    const PlannerSemanticMapManager& psmm,
    const std::vector<mapping::ElementId>& lane_ids, double half_av_width) {
  ASSIGN_OR_RETURN(const auto lane_path,
                   BuildLanePathFromData(
                       mapping::LanePathData(/*start_fraction=*/0.0,
                                             /*end_fraction=*/1.0, lane_ids),
                       psmm));

  constexpr double kLanePathSampleStep = 1.0;  // m.
  ASSIGN_OR_RETURN(const auto drive_passage,
                   BuildDrivePassageFromLanePath(
                       psmm, lane_path, kLanePathSampleStep,
                       /*avoid_loop=*/true, /*backward_extend_len=*/0.0,
                       /*required_planning_horizon=*/0.0,
                       /*required_backward_len=*/0.0,
                       /*override_speed_limit_mps=*/std::nullopt,
                       FrenetFrameType::kBruteFroce),
                   _ << "StraightenLanePathByLaneIds failed.");

  ASSIGN_OR_RETURN(const auto path_boundary,
                   BuildPathBoundaryFromDrivePassage(psmm, drive_passage),
                   _ << "StraightenLanePathByLaneIds failed.");

  return StraightenLanePathBoundedByPathBoundary(
      psmm, drive_passage, path_boundary, lane_path, half_av_width);
}

}  // namespace

absl::StatusOr<SmoothedReferenceLineResultMap>
BuildStraightenedRampResultMapFromRouteSections(
    const PlannerSemanticMapManager& psmm, const RouteSections& route_sections,
    double half_av_width, SmoothedReferenceLineResultMap results) {
  SCOPED_QTRACE("BuildStraightenedRampResultMapFromRouteSections");
  const auto lane_ids_vec = CollectAllZigzagLaneIds(psmm, route_sections);
  absl::flat_hash_set<mapping::ElementId> new_lane_id_set;
  for (const auto& lane_ids : lane_ids_vec) {
    new_lane_id_set.insert(lane_ids.begin(), lane_ids.end());
    if (!results.Contains(lane_ids)) {
      auto straightened_result_or =
          StraightenLanePathByLaneIds(psmm, lane_ids, half_av_width);
      if (straightened_result_or.ok()) {
        results.AddResult(lane_ids, std::move(straightened_result_or).value());
      }
    }
  }

  std::vector<std::vector<mapping::ElementId>> lane_ids_to_delete;
  for (const auto& pair_it : results.smoothed_result_map()) {
    bool should_delete = true;
    for (const auto id : pair_it.first) {
      if (new_lane_id_set.find(id) != new_lane_id_set.end()) {
        should_delete = false;
        break;
      }
    }

    if (should_delete) {
      lane_ids_to_delete.push_back(pair_it.first);
    }
  }
  for (const auto& lane_ids : lane_ids_to_delete) {
    results.DeleteResult(lane_ids);
  }

  return results;
}
}  // namespace qcraft::planner

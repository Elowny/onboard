#include "onboard/planner/router/map_match.h"

#include <algorithm>
#include <ostream>
#include <string>
#include <unordered_set>
#include <utility>

#include "absl/status/status.h"
#include "absl/strings/str_cat.h"
#include "glog/logging.h"

#include "onboard/global/trace.h"
#include "onboard/maps/compatibility_layer.h"
#include "onboard/planner/router/proto/route_params.pb.h"
#include "onboard/planner/router/route_util.h"

namespace qcraft::planner::route::map_match {

std::vector<mapping::PointToLane> ExcludeOngoingLanes(
    const mapping::v2::SemanticMapManager& /*semantic_map_manager*/,
    const std::vector<mapping::PointToLane>& point_to_lanes) {
  for (const auto& point_to_lane : point_to_lanes) {
    VLOG(3) << point_to_lane.DebugString();
  }
  // Exclude by graph connection info. Not use the method which extending lane
  // back.
  std::unordered_set<int64_t> invalid_lanes;
  for (const auto& ptlane_i : point_to_lanes) {
    const auto& out_lane_ids = ptlane_i.lane_proto->outgoing_lanes();
    for (const auto out_lane_id : out_lane_ids) {
      for (const auto& ptlane_j : point_to_lanes) {
        if (ptlane_j.lane_proto->id() != ptlane_i.lane_proto->id() &&
            out_lane_id == ptlane_j.lane_proto->id()) {
          if (ptlane_i.dist < ptlane_j.dist) {
            VLOG(3) << ptlane_j.lane_proto->id() << " is dominated by "
                    << ptlane_i.lane_proto->id();
            invalid_lanes.insert(ptlane_j.lane_proto->id());
          } else {
            VLOG(3) << ptlane_i.lane_proto->id() << " is dominated by "
                    << ptlane_j.lane_proto->id();
            invalid_lanes.insert(ptlane_i.lane_proto->id());
          }
        }
      }
    }
  }

  if (invalid_lanes.empty()) {
    return point_to_lanes;
  }
  std::vector<mapping::PointToLane> candidate_lanes;
  for (const auto& point_to_lane : point_to_lanes) {
    if (invalid_lanes.count(point_to_lane.lane_proto->id()) > 0) {
      continue;
    }
    candidate_lanes.push_back(point_to_lane);
  }
  return candidate_lanes;
}

std::vector<mapping::PointToLane> SelectOnlyOneLaneInSection(
    const std::vector<mapping::PointToLane>& point_to_lanes) {
  std::vector<mapping::PointToLane> candidate_lanes;
  std::unordered_set<int64_t> section_ids;
  for (const auto& point_to_lane : point_to_lanes) {
    const auto lane_proto = point_to_lane.lane_proto;
    const auto pair_it = section_ids.insert(lane_proto->section_id());
    if (pair_it.second) {
      VLOG(3) << "candidate lane: " << lane_proto->id()
              << ", fraction:" << point_to_lane.fraction;
      candidate_lanes.push_back(point_to_lane);
    }
  }
  return candidate_lanes;
}

absl::StatusOr<mapping::PointToLane> GetNearestLaneOnDriving(
    const mapping::v2::SemanticMapManager& semantic_map_manager,
    const route::MapIndex* map_index, mapping::LevelId level,
    const Vec2d& global, double heading,
    const RouteParamProto& route_param_proto) {
  auto points =
      GetNearLanesFromPose(semantic_map_manager, map_index, level, global,
                           heading, route_param_proto.on_driving_param());
  if (points.empty()) {
    return absl::NotFoundError(absl::StrCat("Cannot find lane when driving, ",
                                            global.DebugStringFullPrecision(),
                                            ", heading:", heading));
  }
  return points.front();
}

void SortMatchResults(const RouteMapMatchParam& match_param,
                      std::vector<mapping::PointToLane>* point_to_lanes) {
  const auto match_cost_fn =
      [&match_param](const mapping::PointToLane& point_to_lane) {
        double cost = point_to_lane.dist * match_param.sorter().dist_weight();
        if (mapping::compat::IsVirtual(*point_to_lane.lane_proto)) {
          cost += match_param.sorter().virtual_lane_cost();
        }
        if (mapping::compat::IsPassengerVehicleAvoidLaneType(
                point_to_lane.lane_proto->type())) {
          cost += match_param.sorter().non_motor_lane_cost();
        }
        return cost;
      };
  std::sort(point_to_lanes->begin(), point_to_lanes->end(),
            [&match_cost_fn](const auto& a, const auto& b) {
              return match_cost_fn(a) < match_cost_fn(b);
            });
}

std::vector<mapping::PointToLane> GetNearLanesFromPose(
    const mapping::v2::SemanticMapManager& semantic_map_manager,
    const route::MapIndex* map_index, mapping::LevelId level,
    const Vec2d& global, double heading,
    const RouteMapMatchParam& match_param) {
  FUNC_QTRACE();
  std::vector<mapping::PointToLane> point_to_lanes =
      PointToNearLanesWithHeadingAtLevel(
          *map_index, level, global, match_param.filter().radius_error(),
          heading, match_param.filter().heading_error());
  if (point_to_lanes.empty()) {
    QLOG(INFO) << "Can not find near lanes from global pose: "
               << global.DebugStringFullPrecision()
               << "  with heading: " << heading;
    return point_to_lanes;
  }
  SortMatchResults(match_param, &point_to_lanes);
  return PostprocessingSortedPointToLanes(semantic_map_manager, point_to_lanes);
}

std::vector<mapping::PointToLane> PostprocessingSortedPointToLanes(
    const mapping::v2::SemanticMapManager& smm,
    const std::vector<mapping::PointToLane>& point_to_lanes) {
  FUNC_QTRACE();
  auto post_ptls = point_to_lanes;
  const auto& base_ptl = post_ptls.front();
  for (const auto& left_neighbor :
       base_ptl.lane_proto->lane_neighbors_on_left()) {
    const auto iter =
        std::find_if(post_ptls.begin() + 1, post_ptls.end(),
                     [&left_neighbor](const auto& ptl) {
                       return ptl.lane_proto->id() == left_neighbor.other_id();
                     });
    if (iter != post_ptls.end() &&
        CrossSolidBoundary(GetLaneNeighborBoundary(left_neighbor),
                           /*lc_left=*/true)) {
      post_ptls.erase(iter);
    }
  }

  for (const auto& right_neighbor :
       base_ptl.lane_proto->lane_neighbors_on_right()) {
    const auto iter =
        std::find_if(post_ptls.begin() + 1, post_ptls.end(),
                     [&right_neighbor](const auto& ptl) {
                       return ptl.lane_proto->id() == right_neighbor.other_id();
                     });
    if (iter != post_ptls.end() &&
        CrossSolidBoundary(GetLaneNeighborBoundary(right_neighbor),
                           /*lc_left=*/false)) {
      post_ptls.erase(iter);
    }
  }
  return ExcludeOngoingLanes(smm, post_ptls);
}

}  // namespace qcraft::planner::route::map_match

#ifndef ONBOARD_PLANNER_ROUTER_MAP_MATCH_H_
#define ONBOARD_PLANNER_ROUTER_MAP_MATCH_H_

#include <vector>

#include "absl/status/statusor.h"

#include "onboard/maps/spatial_search_util.h"
#include "onboard/maps/v2/semantic_map_manager.h"
#include "onboard/math/vec.h"
#include "onboard/planner/router/proto/route_params.pb.h"
#include "onboard/planner/router/util/map_index.h"

namespace qcraft::planner::route::map_match {

std::vector<mapping::PointToLane> ExcludeOngoingLanes(
    const mapping::v2::SemanticMapManager& semantic_map_manager,
    const std::vector<mapping::PointToLane>& point_to_lanes);

std::vector<mapping::PointToLane> SelectOnlyOneLaneInSection(
    const std::vector<mapping::PointToLane>& point_to_lanes);

absl::StatusOr<mapping::PointToLane> GetNearestLaneOnDriving(
    const mapping::v2::SemanticMapManager& semantic_map_manager,
    const route::MapIndex* map_index, mapping::LevelId level,
    const Vec2d& global, double heading,
    const RouteParamProto& route_param_proto);

std::vector<mapping::PointToLane> GetNearLanesFromPose(
    const mapping::v2::SemanticMapManager& semantic_map_manager,
    const route::MapIndex* map_index, mapping::LevelId level,
    const Vec2d& global, double heading, const RouteMapMatchParam& match_param);

void SortMatchResults(const RouteMapMatchParam& match_param,
                      std::vector<mapping::PointToLane>* point_to_lanes);

std::vector<mapping::PointToLane> PostprocessingSortedPointToLanes(
    const mapping::v2::SemanticMapManager& smm,
    const std::vector<mapping::PointToLane>& point_to_lanes);

}  // namespace qcraft::planner::route::map_match

#endif

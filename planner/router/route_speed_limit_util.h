#ifndef ONBOARD_PLANNER_ROUTER_ROUTE_SPEED_LIMIT_UTIL_H_
#define ONBOARD_PLANNER_ROUTER_ROUTE_SPEED_LIMIT_UTIL_H_

#include <optional>

#include "onboard/maps/proto/semantic_map.pb.h"
#include "onboard/maps/semantic_map_defs.h"
#include "onboard/maps/v2/semantic_map_manager.h"
#include "onboard/proto/route.pb.h"

namespace qcraft::planner {
// TODO(zuowei): Unify four below APIs with map.
// Unit: kph.
double GetLaneFractionSpeedLimit(const mapping::LaneProto& lane_proto,
                                 double fraction);

double GetLaneMinSpeedLimit(const mapping::LaneProto& lane_proto);

double GetLaneMaxSpeedLimit(const mapping::LaneProto& lane_proto);

double GetLaneAverageSpeedLimit(const mapping::LaneProto& lane_proto);

// Unit: kph.
double GetOverwrittenLaneFractionSpeedLimit(
    const mapping::v2::SemanticMapManager& smm, mapping::ElementId lane_id,
    double fraction);

double GetOverwrittenLaneMinSpeedLimit(
    const mapping::v2::SemanticMapManager& smm, mapping::ElementId lane_id);

double GetOverwrittenLaneMaxSpeedLimit(
    const mapping::v2::SemanticMapManager& smm, mapping::ElementId lane_id);

double GetOverwrittenLaneAverageSpeedLimit(
    const mapping::v2::SemanticMapManager& smm, mapping::ElementId lane_id);

// Unit: mps.
std::optional<double> FindFrontOverwrittenSpeedLimit(
    const mapping::v2::SemanticMapManager& smm,
    const RouteSectionSequenceProto& route_sections, double look_ahead_dist);

}  // namespace qcraft::planner

#endif  //  ONBOARD_PLANNER_ROUTER_ROUTE_SPEED_LIMIT_UTIL_H_

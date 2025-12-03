#include "onboard/planner/router/off_road_route_searcher.h"

#include <math.h>

#include <algorithm>
#include <initializer_list>
#include <limits>
#include <memory>
#include <utility>

#include "absl/status/status.h"

#include "common/proto/drive_mission.pb.h"
#include "common/proto/map_geometry.pb.h"

#include "onboard/container/strong_int.h"
#include "onboard/global/logging.h"
#include "onboard/global/trace.h"
#include "onboard/maps/map_or_die_macros.h"
#include "onboard/maps/proto/semantic_map.pb.h"
#include "onboard/maps/semantic_map_defs.h"
#include "onboard/maps/smm_proto_util.h"
#include "onboard/maps/spatial_search_util.h"
#include "onboard/maps/v2/semantic_map_object.h"
#include "onboard/math/geometry/intersection_util.h"
#include "onboard/math/geometry/polyline2d.h"
#include "onboard/math/geometry/proto/affine_transformation.pb.h"
#include "onboard/math/vec.h"
#include "onboard/planner/router/road_conditions_process.h"
#include "onboard/planner/router/route_core_searcher.h"
#include "onboard/planner/router/route_manager_util.h"
#include "onboard/planner/router/route_sections.h"
#include "onboard/planner/router/route_util.h"
#include "onboard/utils/source_location.h"
#include "onboard/utils/status_macros.h"

namespace qcraft::planner {

std::vector<mapping::LanePoint> FindCrossingLanePointsFromPose(
    const mapping::v2::SemanticMapManager& smm, const route::MapIndex& index,
    const CoordinateConverter& cc, const PoseProto& pose, double radius) {
  const Vec2d pos(pose.pos_smooth().x(), pose.pos_smooth().y());
  auto global2d = cc.SmoothToGlobal(pos);
  auto pt_lanes =
      PointToNearLanesAtLevel(index, cc.GetLevel(), global2d, radius);
  constexpr double kDefaultSearchAngle = M_PI_4;

  Polyline2d line_0(
      {pos, pos + Vec2d::FastUnitFromAngle(pose.yaw() - kDefaultSearchAngle) *
                      radius});
  Polyline2d line_1(
      {pos, pos + Vec2d::FastUnitFromAngle(pose.yaw() + kDefaultSearchAngle) *
                      radius});

  std::vector<mapping::LanePoint> res;
  for (const auto& pt_lane : pt_lanes) {
    // TODO(xiang): Remove this check after length() provided by proto. 20220430
    SMM_ASSIGN_LANE_OR_CONTINUE(lane_info, smm, pt_lane.lane_proto->id());
    std::vector<Vec2d> points_smooth =
        GlobalToSmoothPoints(cc, pt_lane.lane_proto->polyline().points());

    Polyline2d lane_polyline(points_smooth);

    double larger_arc_len = std::numeric_limits<double>::lowest();

    for (const auto& polyline : {line_0, line_1}) {
      Vec2d inter_point;
      double arc_len1, arc_len2;
      if (FindFirstIntersectionBetweenCurves(
              polyline, lane_polyline, &inter_point, &arc_len1, &arc_len2)) {
        larger_arc_len = std::max(larger_arc_len, arc_len2);
      }

      // TODO(weijun): Add boundary check.
    }

    if (larger_arc_len != std::numeric_limits<double>::lowest()) {
      res.emplace_back(mapping::ElementId(lane_info.Proto().id()),
                       larger_arc_len / lane_info.length());
    }
  }

  return res;
}

absl::StatusOr<OffRoadRouteSearcherOutput> SearchForRouteFromOffRoad(
    const route::RoutingRequestContext& context, const CoordinateConverter& cc,
    const PoseProto& pose, const MultipleStopsRequest& route_request,
    double search_radius, route::RouteCore* route_core) {
  FUNC_QTRACE();
  const auto& smm = *context.semantic_map_manager;
  const auto& index = *context.map_index;

  const auto starting_lane_points =
      FindCrossingLanePointsFromPose(smm, index, cc, pose, search_radius);

  if (starting_lane_points.empty()) {
    return absl::NotFoundError(
        "No near lane points are found within search radius.");
  }

  OffRoadRouteSearcherOutput output;

  double shortest_route_len = std::numeric_limits<double>::infinity();
  for (const auto& start_lane_point : starting_lane_points) {
    // TODO(weijun): Refactor FindNextDestinationIndexViaLanePoints to avoid
    // repeated computation.
    ASSIGN_OR_CONTINUE(
        const auto next_index,
        FindNextDestinationIndexViaLanePoints(
            smm, route_request,
            [&start_lane_point, &smm](const RouteSections& sections) {
              SMM_LANE_PROTO_OR_RETURN(start_lane_proto, smm,
                                       start_lane_point.lane_id(), false);
              constexpr double kLengthErrorThresh = 1.0;  // m.
              for (int i = 0; i < sections.size(); ++i) {
                const auto sec_seg = sections.route_section_segment(i);
                SMM_SECTION_PROTO_OR_CONTINUE(sec_proto, smm, sec_seg.id);
                const double start_frac = std::max(
                    0.0, sec_seg.start_fraction -
                             kLengthErrorThresh / sec_proto->average_length());
                const double end_frac = std::min(
                    1.0, sec_seg.end_fraction +
                             kLengthErrorThresh / sec_proto->average_length());
                if (start_lane_proto->section_id() == sec_seg.id.value() &&
                    start_lane_point.fraction() >= start_frac &&
                    start_lane_point.fraction() <= end_frac) {
                  return true;
                }
              }
              return false;
            },
            route_core));

    ASSIGN_OR_CONTINUE(
        const auto routing_request,
        route_request.GenerateRoutingRequestProtoToNextStop(smm, next_index));

    route::RouteCorePrecondition route_pre{
        .is_bus = true,
        .use_time = true,
        .route_restrict_district = route::RouteRestrictDistrict()};
    mapping::PointToLane origin_ptl{
        .lane_proto = FindLaneProto(smm, start_lane_point.lane_id()),
        .dist = 0.0,
        .fraction = start_lane_point.fraction()};
    ASSIGN_OR_CONTINUE(
        auto route_core_result,
        route::SearchForRouteCorePathToRoutingRequest(
            context, routing_request, {origin_ptl}, route_pre, route_core));

    const double route_len =
        CalcRouteSectionsLength(smm, route_core_result.first);

    if (route_len < shortest_route_len) {
      shortest_route_len = route_len;
      output.link_lane_point = start_lane_point;
      output.route_sections = std::move(route_core_result.first);
      output.on_road_route_lane_path = std::move(route_core_result.second);
      output.next_stop_index = next_index;
    }
  }

  if (shortest_route_len == std::numeric_limits<double>::infinity()) {
    return absl::NotFoundError("No route found.");
  }

  return output;
}

}  // namespace qcraft::planner

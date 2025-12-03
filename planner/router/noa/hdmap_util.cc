#include "onboard/planner/router/noa/hdmap_util.h"

// IWYU pragma: no_include <google/protobuf/repeated_ptr_field.h>

#include <algorithm>
#include <cmath>
#include <functional>
#include <memory>
#include <ostream>
#include <vector>

#include "glog/logging.h"

#include "common/proto/map_geometry.pb.h"

#include "onboard/lite/logging.h"
#include "onboard/maps/ehr/proto/extension.pb.h"
#include "onboard/maps/maps_helper.h"
#include "onboard/maps/proto/semantic_map.pb.h"
#include "onboard/math/coordinate_converter.h"
#include "onboard/math/geometry/segment2d.h"
#include "onboard/math/util.h"
#include "onboard/math/vec.h"
#include "onboard/planner/router/geometry/gfc.h"

namespace qcraft::planner::route::noa {

namespace {
// TODO(xiang): Move the function to utility, both used in Noa&L4.
std::vector<PointToLaneT<mapping::v2::Lane>> FilterQueryResults(
    const std::vector<const mapping::v2::Lane*>& query_results,
    const Vec2d& global, const double dist,
    const std::function<bool(const mapping::v2::Lane*)>& predicate_fn,
    const std::function<bool(const Vec2d& p1, const Vec2d& p2)>& heading_fn) {
  std::vector<PointToLaneT<mapping::v2::Lane>> point_to_lanes;
  point_to_lanes.reserve(query_results.size());

  for (const auto& lane : query_results) {
    if (!predicate_fn(lane)) {
      continue;
    }

    const auto& poly_points = lane->proto().polyline().points();
    if (poly_points.size() <= 1) {
      continue;
    }
    std::vector<Vec2d> points(poly_points.size());
    std::transform(poly_points.begin(), poly_points.end(), points.begin(),
                   [](const auto& point_proto) -> Vec2d {
                     return {point_proto.longitude(), point_proto.latitude()};
                   });
    const auto project_result =
        ProjectToLineString(points.begin(), points.end(), global, heading_fn);
    VLOG(2) << "project_result lane_id: " << lane->proto().id() << ", match: "
            << project_result.match_point.DebugStringFullPrecision()
            << ", segment_index: " << project_result.segment_index
            << ", dist_sqr: " << project_result.dist_sqr
            << ", valid : " << project_result.valid;
    if (!project_result.valid) {
      continue;
    }
    const auto pt_dist = HaversineDistance(project_result.match_point, global);
    if (pt_dist > dist) {
      continue;
    }
    if (project_result.segment_index < 0 ||
        project_result.segment_index >= poly_points.size() - 1) {
      QLOG(ERROR) << "Failed to project to lanes, invalid segment index :"
                  << project_result.segment_index;
    }

    // TODO(zuowei): Pass cc as a param and refactor L4 && L2.
    CoordinateConverter cc;
    cc.SetSmoothCoordinateOrigin(points.front());
    const auto smooth_points =
        mapping::GeoPolylineToSmoothPoints(lane->proto().polyline(), cc);
    const double smooth_frac = CalcFraction(
        smooth_points.cbegin(), smooth_points.cend(),
        project_result.segment_index,
        cc.GlobalToSmooth(project_result.match_point), /*cos_latitude=*/1.0);
    point_to_lanes.push_back({
        .lane_info = lane,
        .match_point = project_result.match_point,
        .dist = pt_dist,
        .fraction = smooth_frac,
        .segment_index = project_result.segment_index,
    });
  }
  return point_to_lanes;
}

}  // namespace

bool HasRouteGuideAttrBySection(const mapping::SectionProto& section) {
  return section.has_third_party_section_extend() &&
         section.third_party_section_extend().is_part_of_calculated_route();
}

std::vector<PointToLaneT<mapping::v2::Lane>> PointToNearLanes(
    const mapping::v2::SemanticMapSpatialIndex& smm_index, const Vec2d& global,
    const double max_distance) {
  const auto& origin_lanes =
      smm_index.FindLanesInRadius(global.x(), global.y(), max_distance);
  std::vector<const mapping::v2::Lane*> lanes;
  lanes.reserve(origin_lanes.size());
  for (const auto& lane : origin_lanes) {
    lanes.push_back(lane.get());
  }
  auto match_lanes = FilterQueryResults(
      lanes, global, max_distance,
      [](const mapping::v2::Lane*) { return true; },
      [](const Vec2d& /*p1*/, const Vec2d& /*p2*/) { return true; });
  std::sort(match_lanes.begin(), match_lanes.end(),
            [](const auto& lhs, const auto& rhs) {
              return lhs.dist < rhs.dist ||
                     (lhs.dist == rhs.dist && lhs.fraction < rhs.fraction);
            });
  return match_lanes;
}
std::vector<PointToLaneT<mapping::v2::Lane>> PointToNearLanesWithHeading(
    const mapping::v2::SemanticMapSpatialIndex& smm_index, const Vec2d& global,
    const double /*z*/, const double max_distance, const double heading,
    const double max_heading_diff) {
  const auto& origin_lanes =
      smm_index.FindLanesInRadius(global.x(), global.y(), max_distance);

  std::vector<const mapping::v2::Lane*> lanes;
  lanes.reserve(origin_lanes.size());
  for (const auto& lane : origin_lanes) {
    lanes.push_back(lane.get());
  }
  auto match_lanes = FilterQueryResults(
      lanes, global, max_distance,
      [](const mapping::v2::Lane*) { return true; },
      [heading, max_heading_diff](const Vec2d& p1, const Vec2d& p2) {
        const Segment2d seg(p1, p2);
        return std::fabs(NormalizeAngle(seg.heading() - heading)) <
               max_heading_diff;
      });
  std::sort(match_lanes.begin(), match_lanes.end(),
            [](const auto& lhs, const auto& rhs) {
              return lhs.dist < rhs.dist ||
                     (lhs.dist == rhs.dist && lhs.fraction < rhs.fraction);
            });
  return match_lanes;
}

}  // namespace qcraft::planner::route::noa

#include "onboard/planner/router/util/map_index.h"

// IWYU pragma: no_include <ostream>
// IWYU pragma: no_include <boost/geometry/index/parameters.hpp>
// IWYU pragma: no_include <boost/iterator/iterator_facade.hpp>
// IWYU pragma: no_include <boost/geometry/index/detail/predicates.hpp>
// IWYU pragma: no_include <boost/geometry/geometries/box.hpp>
// IWYU pragma: no_include "onboard/maps/proto/semantic_map.pb.h"
// IWYU pragma: no_include <boost/geometry/index/predicates.hpp>
// IWYU pragma: no_include <string>
// IWYU pragma: no_include "common/proto/map_geometry.pb.h"
// IWYU pragma: no_include "onboard/container/strong_int.h"
// IWYU pragma: no_include <boost/geometry/geometries/point_xy.hpp>
// IWYU pragma: no_include <google/protobuf/repeated_ptr_field.h>
// IWYU pragma: no_include <boost/geometry/core/cs.hpp>

#include <algorithm>
#include <cmath>
#include <functional>
#include <iterator>
#include <memory>

#include "absl/strings/str_format.h"
#include "glog/logging.h"

#include "onboard/global/trace.h"
#include "onboard/lite/logging.h"
#include "onboard/maps/maps_common.h"
#include "onboard/maps/v2/semantic_map_definition.h"
#include "onboard/maps/v2/semantic_map_object.h"
#include "onboard/planner/router/geometry/gfc.h"
namespace qcraft::planner::route {

// Precondition: size(points) >=2 && index <points.size();

void MapIndex::InitIndex(
    const mapping::v2::SemanticMapManager& semantic_map_manager) {
  SCOPED_QTRACE("MapIndex:InitIndex");
  tree_.clear();
  for (const auto& lane : semantic_map_manager.semantic_map().lanes) {
    MBR<double> mbr;
    const auto& lane_proto = lane->Proto();
    const auto first = *lane_proto.polyline().points().begin();
    mbr.x1 = mbr.x2 = first.longitude();
    mbr.y1 = mbr.y2 = first.latitude();

    for (const auto& xyz : lane_proto.polyline().points()) {
      if (xyz.longitude() < mbr.x1) {
        mbr.x1 = xyz.longitude();
      } else if (xyz.longitude() > mbr.x2) {
        mbr.x2 = xyz.longitude();
      }
      if (xyz.latitude() < mbr.y1) {
        mbr.y1 = xyz.latitude();
      } else if (xyz.latitude() > mbr.y2) {
        mbr.y2 = xyz.latitude();
      }
    }
    DBox box = {{mbr.x1, mbr.y1}, {mbr.x2, mbr.y2}};
    tree_.insert(std::make_pair(box, &lane_proto));
  }
}

void MapIndex::InitIndex(
    const mapping::SemanticMapManager& semantic_map_manager) {
  SCOPED_QTRACE("MapIndex:InitIndex");
  tree_.clear();
  for (const auto& lane : semantic_map_manager.lane_info()) {
    MBR<double> mbr;
    const auto& lane_proto = lane.Proto();
    const auto first = *lane_proto.polyline().points().begin();
    mbr.x1 = mbr.x2 = first.longitude();
    mbr.y1 = mbr.y2 = first.latitude();

    for (const auto& xyz : lane_proto.polyline().points()) {
      if (xyz.longitude() < mbr.x1) {
        mbr.x1 = xyz.longitude();
      } else if (xyz.longitude() > mbr.x2) {
        mbr.x2 = xyz.longitude();
      }
      if (xyz.latitude() < mbr.y1) {
        mbr.y1 = xyz.latitude();
      } else if (xyz.latitude() > mbr.y2) {
        mbr.y2 = xyz.latitude();
      }
    }
    DBox box = {{mbr.x1, mbr.y1}, {mbr.x2, mbr.y2}};
    tree_.insert(std::make_pair(box, &lane_proto));
  }
}

std::vector<mapping::PointToLane> MapIndex::PointToNearLanes(
    Vec2d global, double dist,
    const std::function<bool(const mapping::LaneProto*)>& predicate_fn,
    const std::function<bool(const Vec2d& p1, const Vec2d& p2)>& heading_fn)
    const {
  SCOPED_QTRACE("MapIndex::PointToNearLanes");
  constexpr double kMinCosLat = 0.1;
  // Approximate box at low latitude region.
  const auto ratio = std::cos(global.y());
  if (ratio < kMinCosLat) {
    QLOG(ERROR) << "Invalid global coordinates: " << global.DebugString();
  }
  const DBox box = {{global.x() - dist * kRadiansPerMeterLon,
                     global.y() - dist * kRadiansPerMeterLon / ratio},
                    {global.x() + dist * kRadiansPerMeterLon,
                     global.y() + dist * kRadiansPerMeterLon / ratio}};
  const auto query_results = QueryLanes(box);
  VLOG(3) << "Query box: "
          << absl::StrFormat("%.8f,%.8f,%.8f,%.8f", box.min_corner().x(),
                             box.min_corner().y(), box.max_corner().x(),
                             box.max_corner().y())
          << ", results: " << query_results.size();
  std::vector<mapping::PointToLane> point_to_lanes;
  point_to_lanes.reserve(query_results.size());
  for (const auto& result : query_results) {
    const auto* lane_proto_ptr = std::get<const mapping::LaneProto*>(result);
    if (!predicate_fn(lane_proto_ptr)) {
      continue;
    }

    const auto& poly_points = lane_proto_ptr->polyline().points();
    if (poly_points.size() <= 1) {
      continue;
    }
    // TODO(xiang): Can be use point proto insteadof sequence of Vec2d, avoid
    // copies and transforms.
    std::vector<Vec2d> points(poly_points.size());
    std::transform(poly_points.begin(), poly_points.end(), points.begin(),
                   [](const auto& point_proto) -> Vec2d {
                     return {point_proto.longitude(), point_proto.latitude()};
                   });
    const auto project_result =
        ProjectToLineString(points.begin(), points.end(), global, heading_fn);
    VLOG(2) << "project_result lane_id: " << lane_proto_ptr->id() << ", match: "
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
    const double fraction = CalcFraction(points.cbegin(), points.cend(),
                                         project_result.segment_index,
                                         project_result.match_point, ratio);
    point_to_lanes.push_back({
        .lane_proto = lane_proto_ptr,
        .segment = {points[project_result.segment_index],
                    points[project_result.segment_index + 1]},
        // TODO(xiang): Segment should not be too close.
        .match_point = project_result.match_point,
        .dist = pt_dist,
        .fraction = fraction,
        .segment_index = project_result.segment_index,
    });
  }
  return point_to_lanes;
}

std::vector<MapIndex::LaneProtoValue> MapIndex::QueryLanes(
    const DBox& box) const {
  constexpr auto kDefaultCount = 10;
  std::vector<LaneProtoValue> query_results;
  query_results.reserve(kDefaultCount);
  tree_.query(boost::geometry::index::intersects(box),
              std::back_inserter(query_results));
  return query_results;
}

}  // namespace qcraft::planner::route

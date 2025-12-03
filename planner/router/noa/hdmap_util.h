#ifndef ONBOARD_PLANNER_ROUTER_NOA_HDMAP_UTIL_H_
#define ONBOARD_PLANNER_ROUTER_NOA_HDMAP_UTIL_H_

#include <string>
#include <vector>

#include "absl/strings/str_format.h"

#include "onboard/maps/proto/semantic_map.pb.h"
#include "onboard/maps/v2/semantic_map_object.h"
#include "onboard/maps/v2/semantic_map_spatial_index.h"
#include "onboard/math/vec.h"

namespace qcraft::planner::route::noa {
template <typename LineType>
struct PointToLaneT {
  const LineType* lane_info = nullptr;
  Vec2d match_point = {0.0, 0.0};
  double dist = 0.0;
  double fraction = 0.0;
  int segment_index = -1;

  std::string DebugString() const {
    return lane_info == nullptr ? "{null}"
                                : absl::StrFormat(
                                      "{lane id: %d, dist: %f, fraction: %f, "
                                      "segment: %d, match_point: %s}",
                                      lane_info->proto().id(), dist, fraction,
                                      segment_index, match_point.DebugString());
  }
};

bool HasRouteGuideAttrBySection(const mapping::SectionProto& section);

std::vector<PointToLaneT<mapping::v2::Lane>> PointToNearLanesWithHeading(
    const mapping::v2::SemanticMapSpatialIndex& smm_index, const Vec2d& global,
    double z, double max_distance, double heading, double max_heading_diff);
std::vector<PointToLaneT<mapping::v2::Lane>> PointToNearLanes(
    const mapping::v2::SemanticMapSpatialIndex& smm_index, const Vec2d& global,
    double max_distance);
}  // namespace qcraft::planner::route::noa

#endif  // ONBOARD_PLANNER_ROUTER_NOA_HDMAP_UTIL_H_

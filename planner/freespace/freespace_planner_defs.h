#ifndef ONBOARD_PLANNER_FREESPACE_PLANNER_DEFS_H_
#define ONBOARD_PLANNER_FREESPACE_PLANNER_DEFS_H_

#include <string>
#include <vector>

#include "onboard/math/geometry/aabox2d.h"
#include "onboard/math/geometry/polygon2d.h"
#include "onboard/math/geometry/segment2d.h"
#include "onboard/planner/freespace/proto/freespace_planner.pb.h"

namespace qcraft {
namespace planner {

struct FreespaceBoundary {
  std::string id;
  FreespaceMapProto::BoundaryType type;
  std::vector<Vec2d> points;
  bool near_parking_spot;
  double height;
};

struct FreespaceObject {
  Polygon2d contour;
  double height;
};

struct SpecialBoundary {
  std::string id;
  FreespaceMapProto::SpecialBoundaryType type;
  std::vector<Vec2d> points;
};

struct FreespaceMap {
  // Freespace region is defined as an AAbox whose center is final goal.
  AABox2d region;
  // Map boundaries.
  std::vector<FreespaceBoundary> boundaries;
  // Special boundaries.
  std::vector<SpecialBoundary> special_boundaries;
};

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_FREESPACE_PLANNER_DEFS_H_

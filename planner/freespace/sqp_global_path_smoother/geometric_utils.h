#ifndef ONBOARD_PLANNER_FREESPACE_GEOMETRIC_UTILS_H
#define ONBOARD_PLANNER_FREESPACE_GEOMETRIC_UTILS_H

#include <vector>

#include "onboard/math/vec.h"
#include "onboard/planner/freespace/sqp_global_path_smoother/convex_region_defs.h"

namespace qcraft {
namespace planner {
// Find extreme points of ConvexRegion.
std::vector<Vec2d> CalVertices(const ConvexRegion& poly);
}  // namespace planner
}  // namespace qcraft
#endif  // ONBOARD_PLANNER_FREESPACE_GEOMETRIC_UTILS_H

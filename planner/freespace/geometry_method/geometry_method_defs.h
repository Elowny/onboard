#ifndef ONBOARD_PLANNER_FREESPACE_GEOMETRY_METHOD_DEFS_H
#define ONBOARD_PLANNER_FREESPACE_GEOMETRY_METHOD_DEFS_H

#include <vector>

#include "onboard/math/vec.h"

namespace qcraft {
namespace planner {

enum class GeometryPathType {
  STRAIGHT = 1,  // NOLINT(readability-identifier-naming,-warnings-as-errors)
  LEFT = 2,      // NOLINT(readability-identifier-naming,-warnings-as-errors)
  RIGHT = 3,     // NOLINT(readability-identifier-naming,-warnings-as-errors)
};

struct GeometryMethodPoint {
  Vec2d pos;
  double theta;
  Vec2d tangent;
};

struct LineCirclePath {
  GeometryMethodPoint start;
  std::vector<GeometryPathType> types;
  std::vector<double> lengths;
  std::vector<double> kappas;
  std::vector<GeometryMethodPoint> ends;
};

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_FREESPACE_GEOMETRY_METHOD_DEFS_H

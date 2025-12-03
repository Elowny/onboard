#ifndef ONBOARD_PLANNER_FREESPACE_GEOMETRY_METHOD_GEOMETRY_CONNECTION_H
#define ONBOARD_PLANNER_FREESPACE_GEOMETRY_METHOD_GEOMETRY_CONNECTION_H

#include "onboard/planner/freespace/geometry_method/geometry_method_defs.h"

namespace qcraft {
namespace planner {

// Circle-circle connection of parallel parking.
bool CircleCircleConection(const GeometryMethodPoint& start,
                           const GeometryMethodPoint& goal, double max_kappa,
                           LineCirclePath* result);

// Circle-line or line-circle connection of two poses.
bool CircleLineConection(const GeometryMethodPoint& start,
                         const GeometryMethodPoint& goal, double max_kappa,
                         LineCirclePath* result);

// Line-circle-line connection of two poses, circle kappa is max_kappa.
bool LineCircleLineConection(const GeometryMethodPoint& start,
                             const GeometryMethodPoint& goal, double max_kappa,
                             LineCirclePath* result);

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_FREESPACE_GEOMETRY_METHOD_GEOMETRY_CONNECTION_H

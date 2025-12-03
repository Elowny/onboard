#ifndef ONBOARD_PLANNER_FREESPACE_FREESPACE_PLANNER_PLOT_UTIL_H_
#define ONBOARD_PLANNER_FREESPACE_FREESPACE_PLANNER_PLOT_UTIL_H_

#include <string_view>

#include "absl/types/span.h"

#include "onboard/math/geometry/box2d.h"
#include "onboard/planner/freespace/directional_path.h"

namespace qcraft {
namespace planner {
void DrawDirectionalPath(std::string_view name,
                         absl::Span<const DirectionalPath> paths);

void DrawPathSweptVolume(std::string_view name,
                         absl::Span<const Box2d> path_swept_volume);

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_FREESPACE_FREESPACE_PLANNER_PLOT_UTIL_H_

#ifndef ONBOARD_PLANNER_INITIALIZER_GEOMETRY_GEOMETRY_GRAPH_DEBUG_H_
#define ONBOARD_PLANNER_INITIALIZER_GEOMETRY_GEOMETRY_GRAPH_DEBUG_H_

#include <string>

#include "absl/strings/str_cat.h"

namespace qcraft::planner {

enum class ResampleReason {
  RESAMPLED = 1,
  NR_ZERO_REACHABLE = 2,
  NR_ALL_REACHABLE = 3,
  NR_INVALID_RANGE = 4,
  NR_LATERAL_RESOLUTION = 5,
  NOT_INITIALIZED = 6
};

struct EdgeDebugInfo {
  int start_layer_idx;
  int start_node_on_layer_idx;
  int end_layer_idx;
  int end_node_on_layer_idx;
  int start_station_idx;
  int end_station_idx;
};

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_INITIALIZER_GEOMETRY_GEOMETRY_GRAPH_DEBUG_H_

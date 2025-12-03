#ifndef ONBOARD_PLANNER_INITIALIZER_TEST_UTIL_H_
#define ONBOARD_PLANNER_INITIALIZER_TEST_UTIL_H_

#include "absl/status/status.h"

#include "onboard/planner/initializer/geometry/geometry_graph.h"

namespace qcraft {
namespace planner {

absl::Status CheckGeometryGraphConnectivity(const GeometryGraph& graph);

};  // namespace planner

}  // namespace qcraft

#endif  // ONBOARD_PLANNER_INITIALIZER_TEST_UTIL_H_

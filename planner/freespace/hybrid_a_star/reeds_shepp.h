#ifndef ONBOARD_PLANNER_FREESPACE_HYBRID_A_STAR_REEDS_SHEPP_H
#define ONBOARD_PLANNER_FREESPACE_HYBRID_A_STAR_REEDS_SHEPP_H

#include <vector>

#include "absl/status/statusor.h"
#include "onboard/planner/freespace/hybrid_a_star/node_3d.h"

namespace qcraft {
namespace planner {

struct ReedSheppPath {
  std::vector<double> segs_lengths;
  std::vector<char> segs_types;
  double total_length = 0.0;
};

struct ReedSheppPoint {
  double x;
  double y;
  double theta;
  bool forward;
};

// Pick the shortest path from all possible combination of movement primitives
// by Reed Shepp, in general, this function shouldn't return a non-ok status.
absl::StatusOr<ReedSheppPath> GetShortestReedsShepp(const Node3d& start_node,
                                                    const Node3d& end_node,
                                                    double max_kappa);

// Return sampled path or an empty vector if there is no valid RS curve.
std::vector<ReedSheppPoint> GetSampledShortestReedsShepp(
    const Node3d& start_node, const Node3d& end_node, double max_kappa,
    double resolution);
}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_FREESPACE_HYBRID_A_STAR_REEDS_SHEPP_H

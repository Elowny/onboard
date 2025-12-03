#ifndef ONBOARD_PLANNER_COMMON_PATH_APPROX_OVERLAP_H_
#define ONBOARD_PLANNER_COMMON_PATH_APPROX_OVERLAP_H_

#include <optional>
#include <vector>

#include "onboard/math/geometry/halfplane.h"
#include "onboard/math/geometry/polygon2d.h"
#include "onboard/planner/common/path_approx.h"

namespace qcraft {
namespace planner {

// ********************* Agent Overlap **********************
// A single agent state's overlap.
struct AgentOverlap {
  double first_ra_s = 0.0;
  double last_ra_s = 0.0;
  // The heading of nearest path_segment.
  double ra_heading = 0.0;
  // 0.0 if there is a true overlap. It is negative if the agent state is at the
  // right side of the path, and it is positive if the agent state is at the
  // left side of the overlap.
  double lat_dist = 0.0;
};

// Compute the overlaps between one agent state and a path approximation in
// range [first_index, last_index]. `max_lat_dist` is the maximum lateral
// distance to compute overlap.
std::vector<AgentOverlap> ComputeAgentOverlaps(const PathApprox& path_approx,
                                               double step_length,
                                               int first_index, int last_index,
                                               const Polygon2d& polygon,
                                               double max_lat_dist,
                                               double search_radius);

std::vector<AgentOverlap> ComputeAgentOverlapsWithBuffer(
    const PathApprox& path_approx, double step_length, int first_index,
    int last_index, const Polygon2d& polygon, double max_lat_dist,
    double lat_buffer, double lon_buffer, double search_radius);

std::vector<AgentOverlap> ComputeAgentOverlapsWithBufferAndHeading(
    const PathApprox& path_approx, double step_length, int first_index,
    int last_index, const Polygon2d& polygon, double max_lat_dist,
    double lat_buffer, double lon_buffer, double search_radius, double theta,
    double max_heading_diff);

bool HasPathApproxOverlapWithPolygon(const PathApprox& path_approx,
                                     double step_length, int first_index,
                                     int last_index, const Polygon2d& polygon,
                                     double search_radius);

// ********************* Half plane overlap **********************
std::optional<double> ComputeHalfPlaneOverlaps(const PathApprox& path_approx,
                                               double step_length,
                                               int first_index, int last_index,
                                               const HalfPlane& half_plane,
                                               double search_radius);

std::optional<double> ComputeHalfPlaneOverlapsWithLateralGap(
    const PathApprox& path_approx, double step_length, int first_index,
    int last_index, const HalfPlane& half_plane, double lateral_gap,
    double search_radius);

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_COMMON_PATH_APPROX_OVERLAP_H_

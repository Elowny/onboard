#ifndef ONBOARD_PLANNER_FREESPACE_HYBRID_A_STAR_FAST_REEDS_SHEPP_H
#define ONBOARD_PLANNER_FREESPACE_HYBRID_A_STAR_FAST_REEDS_SHEPP_H

#include "onboard/planner/freespace/hybrid_a_star/node_3d.h"

namespace qcraft {
namespace planner {

struct FastReedSheppPath {
  int gear_change_num = 0;
  bool init_gear;
  double total_length = 0.0;
};

// Compute all possible RS curves off-line and store in table.
// The size of the table is 40*20*20, which represents end_x = [-10m, 10m],
// end_y = [0m, 10m] with 0.5m resolution after normalized by max_kappa, and
// end_theta = [-M_PI, M_PI] with M_PI/20 resolution. The area of the table is a
// square whose center is the start point, and the edge length is
// 20.0/max_kappa, end_pose in this area returns answer directly from the
// table, end_pose out of this area returns answer from an on-line computing.
// This function can't ensure any level of accuracy!!!
FastReedSheppPath GetShortestReedsSheppFromTable(const Node3d& start,
                                                 const Node3d& end,
                                                 double max_kappa);

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_FREESPACE_HYBRID_A_STAR_FAST_REEDS_SHEPP_H

#ifndef ONBOARD_PLANNER_SPEED_OVERLAP_INFO_H_
#define ONBOARD_PLANNER_SPEED_OVERLAP_INFO_H_

namespace qcraft::planner {

struct OverlapInfo {
  double time = 0.0;

  // On object spacetime trajectory.
  int obj_idx = 0;

  // Index of the first collision with the given object state on
  // ego obj discretized path.
  int av_start_idx = 0;

  // Index of the last collision with the given object state on ego obj
  // discretized path.
  int av_end_idx = 0;
};

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_SPEED_OVERLAP_INFO_H_

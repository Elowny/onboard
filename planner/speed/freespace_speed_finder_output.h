#ifndef ONBOARD_PLANNER_SPEED_FREESPACE_SPEED_FINDER_OUTPUT_H_
#define ONBOARD_PLANNER_SPEED_FREESPACE_SPEED_FINDER_OUTPUT_H_

#include <string>
#include <vector>

#include "onboard/planner/decision/constraint_manager.h"
#include "onboard/planner/object/partial_spacetime_object_trajectory.h"
#include "onboard/planner/object/spacetime_object_trajectory.h"
#include "onboard/planner/speed/proto/speed_finder.pb.h"
#include "onboard/proto/charts.pb.h"
#include "onboard/proto/planner.pb.h"

namespace qcraft::planner {

// TODO(ping): Delete this file after refactoring speed finder.
struct FreespaceSpeedFinderOutput {
  std::vector<ApolloTrajectoryPointProto> trajectory_points;
  std::vector<PartialSpacetimeObjectTrajectory> considered_st_objects;
  double stop_s;
  double stationary_object_stop_s;
  std::string nearest_stationary_object_id;
  SpeedFinderDebugProto speed_finder_proto;
  ConstraintManager constraint_mgr;
  vis::vantage::ChartDataProto st_graph_chart;
  vis::vantage::ChartDataProto vt_graph_chart;
  vis::vantage::ChartDataProto traj_chart;
  vis::vantage::ChartDataProto path_chart;
};

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_SPEED_FREESPACE_SPEED_FINDER_OUTPUT_H_

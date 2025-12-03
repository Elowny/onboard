#ifndef ONBOARD_PLANNER_SPEED_SPEED_FINDER_OUTPUT_H_
#define ONBOARD_PLANNER_SPEED_SPEED_FINDER_OUTPUT_H_

#include <optional>
#include <string>
#include <vector>

#include "onboard/planner/decision/constraint_manager.h"
#include "onboard/planner/object/partial_spacetime_object_trajectory.h"
#include "onboard/planner/object/spacetime_object_trajectory.h"
#include "onboard/planner/speed/proto/speed_finder.pb.h"
#include "onboard/proto/charts.pb.h"
#include "onboard/proto/planner.pb.h"

namespace qcraft::planner {

struct SpeedFinderOutput {
  std::vector<ApolloTrajectoryPointProto> trajectory_points;
  std::vector<PartialSpacetimeObjectTrajectory> considered_st_objects;
  SpeedFinderDebugProto speed_finder_proto;
  ConstraintManager constraint_mgr;
  std::optional<TrajectoryEndInfoProto> trajectory_end_info;
  vis::vantage::ChartDataProto preliminary_speed_chart;
  vis::vantage::ChartDataProto sampling_dp_chart;
  vis::vantage::ChartDataProto interactive_speed_chart;
  vis::vantage::ChartDataProto st_graph_chart;
  vis::vantage::ChartDataProto vt_graph_chart;
  vis::vantage::ChartDataProto prediction_vt_chart;
  vis::vantage::ChartDataProto traj_chart;
  vis::vantage::ChartDataProto path_chart;
};

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_SPEED_SPEED_FINDER_OUTPUT_H_

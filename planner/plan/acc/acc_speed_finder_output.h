#ifndef ONBOARD_PLANNER_SPEED_ACC_SPEED_FINDER_OUTPUT_H_
#define ONBOARD_PLANNER_SPEED_ACC_SPEED_FINDER_OUTPUT_H_

#include <optional>
#include <string>
#include <vector>

#include "common/proto/qacc.pb.h"

#include "onboard/planner/object/spacetime_object_trajectory.h"
#include "onboard/planner/speed/proto/speed_finder.pb.h"
#include "onboard/proto/charts.pb.h"
#include "onboard/proto/planner.pb.h"

namespace qcraft::planner {

struct AccSpeedFinderOutput {
  std::vector<ApolloTrajectoryPointProto> trajectory_points;
  SpeedFinderDebugProto speed_finder_proto;
  vis::vantage::ChartDataProto preliminary_speed_chart;
  vis::vantage::ChartDataProto st_graph_chart;
  vis::vantage::ChartDataProto vt_graph_chart;
  vis::vantage::ChartDataProto traj_chart;
  vis::vantage::ChartDataProto path_chart;
  vis::vantage::ChartDataProto sampling_dp_chart;
  vis::vantage::ChartDataProto interactive_speed_chart;
  vis::vantage::ChartDataProto pred_vt_chart;
  AccPreliminarySpeedDebugProto preliminary_speed_debug;
};

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_SPEED_ACC_SPEED_FINDER_OUTPUT_H_

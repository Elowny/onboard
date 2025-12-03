#ifndef ONBOARD_PLANNER_PLAN_ASYNC_PLANNER_OUTPUT_H_
#define ONBOARD_PLANNER_PLAN_ASYNC_PLANNER_OUTPUT_H_

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "onboard/planner/common/planner_status.h"
#include "onboard/planner/decision/proto/constraint.pb.h"
#include "onboard/planner/object/partial_spacetime_object_trajectory.h"
#include "onboard/planner/object/proto/planner_object.pb.h"
#include "onboard/planner/plan/async_planner_state.h"
#include "onboard/planner/plan/path_bounded_est_planner_output.h"
#include "onboard/planner/proto/planner_state.pb.h"
#include "onboard/planner/proto/trajectory_validation.pb.h"
#include "onboard/planner/speed/proto/speed_finder.pb.h"
#include "onboard/proto/trajectory_point.pb.h"

namespace qcraft::planner {

struct AsyncPlannerOutput {
  PlannerStatus est_status;

  bool scheduled_async_low_freq;
  bool low_freq_result_retrived;
  std::shared_ptr<AsyncMultiTaskEstOutput> latest_low_freq_output = nullptr;
  std::shared_ptr<PathBoundedEstPlannerOutput> retrived_low_freq_output =
      nullptr;

  // est output
  DeciderStateProto decider_state;
  std::optional<double> distance_to_traffic_light_stop_line = std::nullopt;
  std::vector<ApolloTrajectoryPointProto> traj_points;
  std::vector<PartialSpacetimeObjectTrajectory> considered_st_objects;
  std::optional<TrajectoryEndInfoProto> trajectory_end_info;
  std::optional<std::string> alerted_front_vehicle = std::nullopt;
  bool collision_warning_request = false;

  // est debug
  FilteredTrajectories filtered_prediction_trajectories;
  SpeedFinderDebugProto speed_finder_debug;
  ConstraintProto decision_constraints;
  TrajectoryValidationResultProto traj_validation_result;

  ObjectsPredictionProto speed_considered_objects_prediction;

  // chart
  vis::vantage::ChartDataBundleProto chart_data;

  // Alcc only.
  mapping::LanePath origin_lane_path;
  mapping::LanePath target_lane_path;
  QALCState alc_state;
  std::optional<Vec2dProto> lane_change_target_point = std::nullopt;
};

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_PLAN_ASYNC_PLANNER_OUTPUT_H_

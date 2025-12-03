#ifndef ONBOARD_PLANNER_PLAN_ACC_ACC_TASK_INTERNAL_H_
#define ONBOARD_PLANNER_PLAN_ACC_ACC_TASK_INTERNAL_H_

// IWYU pragma: no_include <google/protobuf/repeated_ptr_field.h>

#include <algorithm>
#include <cmath>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "absl/time/time.h"
#include "absl/types/span.h"

#include "common/proto/qacc.pb.h"

#include "onboard/math/piecewise_linear_function.h"
#include "onboard/planner/common/driving_map_topo.h"
#include "onboard/planner/common/planner_status.h"
#include "onboard/planner/plan/acc/acc_corridor.h"
#include "onboard/planner/plan/acc/acc_planner_output.h"
#include "onboard/planner/plan/acc/acc_speed_finder_output.h"
#include "onboard/planner/plan/acc/acc_target.h"
#include "onboard/planner/plan/acc/acc_task_input.h"
#include "onboard/planner/plan/acc/acc_task_output.h"
#include "onboard/planner/planner_semantic_map_manager.h"  // for PlannerSemanticMapManager
#include "onboard/planner/proto/planner_params.pb.h"
#include "onboard/planner/proto/trajectory_validation.pb.h"
#include "onboard/planner/speed/proto/speed_finder_params.pb.h"
#include "onboard/proto/hmi_content.pb.h"
#include "onboard/proto/perception/fusion/object.pb.h"
#include "onboard/proto/positioning.pb.h"  // for PoseProto
#include "onboard/proto/q_run_event.pb.h"
#include "onboard/proto/trajectory_point.pb.h"
#include "onboard/proto/vehicle.pb.h"

namespace qcraft::planner {

namespace internal {

inline MotionConstraintParamsProto FillRequiredMotionConstraintParams(
    const MotionConstraintParamsProto& motion_constraint_params,
    const AccReqParamsProto& acc_req_params, double plan_start_v) {
  MotionConstraintParamsProto new_motion_params = motion_constraint_params;
  const auto max_acc_vel_plf =
      PiecewiseLinearFunctionFromProto(acc_req_params.max_acc_vel_plf());
  const auto max_jerk_vel_plf =
      PiecewiseLinearFunctionFromProto(acc_req_params.max_jerk_vel_plf());
  const auto min_decel_vel_plf =
      PiecewiseLinearFunctionFromProto(acc_req_params.min_decel_vel_plf());
  const double max_jerk = std::fabs(max_jerk_vel_plf(plan_start_v));
  new_motion_params.set_max_deceleration(min_decel_vel_plf(plan_start_v));
  new_motion_params.set_max_acceleration(max_acc_vel_plf(plan_start_v));
  new_motion_params.set_max_decel_jerk(-max_jerk);  // Negative.
  new_motion_params.set_max_accel_jerk(max_jerk);   // Positive.
  return new_motion_params;
}

SpeedFinderParamsProto ModifySpeedFinderParams(
    const SpeedFinderParamsProto& speed_finder_params,
    QRunEvent::LccFollowingDistanceLevel following_distance_level,
    bool crowded_scene, double plan_start_v,
    std::optional<ObjectType> target_type);

inline std::vector<AccTargetDecision> GetPrevTargetsFromProto(
    const QACCTaskProto* acc_task_proto) {
  std::vector<AccTargetDecision> prev_target_decisions;
  if (acc_task_proto != nullptr && acc_task_proto->has_prev_acc_target()) {
    for (const auto& target_proto :
         acc_task_proto->prev_acc_target().leaders()) {
      AccTargetDecision target;
      target.FromProto(target_proto);
      prev_target_decisions.push_back(std::move(target));
    }
  }
  return prev_target_decisions;
}

std::vector<ApolloTrajectoryPointProto> CreateAccPastPointsList(
    absl::Span<const ApolloTrajectoryPointProto> origin_past_pts,
    const ApolloTrajectoryPointProto& plan_start_point);

void ReportAccTaskToHmi(const AccCorridor& corridor,
                        const AccTargetPerCorridor& targets,
                        HmiContentProto* hmi_content);

void ReportRunEventWhenTargetDriveoffInStandwait(
    bool is_acc_standwait, const AccTargetPerCorridor& acc_target);

struct AccSpeedFinderForCorridorInput {
  const AccCorridor* corridor = nullptr;
  const AccTargetPerCorridor* targets = nullptr;
  const AccTaskParamsProto* acc_params = nullptr;
  const ApolloTrajectoryPointProto* plan_start_point = nullptr;
  const VehicleGeometryParamsProto* vehicle_geometry_params = nullptr;
  const VehicleDriveParamsProto* vehicle_drive_params = nullptr;
  bool is_acc_standwait = false;
  const MotionConstraintParamsProto* motion_constraint_params = nullptr;
  const SpeedFinderParamsProto* speed_finder_params = nullptr;
  const std::vector<ApolloTrajectoryPointProto>* time_aligned_prev_traj =
      nullptr;
  const std::set<std::string>* crowded_side_objs = nullptr;
  absl::Span<const ApolloTrajectoryPointProto> acc_past_points;
  std::optional<double> acc_cruising_speed_limit = std::nullopt;
  absl::Time plan_time;
  QRunEvent::LccFollowingDistanceLevel following_distance_level =
      QRunEvent::LCC_FOLLOWING_DISTANCE_LEVEL_MEDIUM;
};

PlannerStatus PlanAccSpeedForCorridorAndValidate(
    const AccSpeedFinderForCorridorInput& input, AccSpeedFinderOutput* output,
    TrajectoryValidationResultProto* traj_validation_result);

inline bool EarlyExit(AccCorridorSource source,
                      const PlannerStatus& planner_status,
                      bool planner_force_no_map,
                      bool planner_acc_plan_for_all_sources) {
  if (planner_acc_plan_for_all_sources) {
    return false;
  }
  if (planner_status.ok() && !planner_force_no_map) {
    return true;
  }

  if (source == AccCorridorSource::ESTIMATED && planner_force_no_map) {
    // Force to output estimated solution.
    return true;
  }

  return false;
}

// NOLINTNEXTLINE
PlannerStatus FillAccPlannerOutput(
    std::map<AccCorridorSource, PlannerStatus> source_status,
    std::map<AccCorridorSource, TrajectoryValidationResultProto>
        source_val_results,
    std::map<AccCorridorSource, std::unique_ptr<AccTargetPerCorridor>>
        source_targets,
    std::map<AccCorridorSource, std::unique_ptr<AccCorridor>> source_corridors,
    std::map<AccCorridorSource, AccSpeedFinderOutput> source_solutions,
    AccCorridorSource active_source,
    std::vector<ApolloTrajectoryPointProto> acc_past_points,
    AccPlannerOutput* acc_planner_output);

std::unique_ptr<DrivingMapTopo> BuildDrivingMapTopo(
    const PoseProto& pose, const PlannerSemanticMapManager* psmm,
    double plan_start_v);
}  // namespace internal

PlannerStatus RunAccPlanner(const AccTaskInput& input,
                            AccPlannerOutput* acc_planner_output);

void FillAccRelatedOutput(AccPlannerOutput acc_planner_output,
                          bool is_acc_standwait,
                          bool prev_collision_warning_request,
                          double plan_start_v, double follow_time_headway,
                          absl::Time plan_time,
                          const std::optional<double>& acc_cruising_speed_limit,
                          AccTaskOutput* output);

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_PLAN_ACC_ACC_TASK_INTERNAL_H_

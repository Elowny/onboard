#include "onboard/ml_planner/ml_task_dispatcher.h"

#include <memory>
#include <utility>
#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/time/time.h"
#include "absl/types/span.h"

#include "onboard/async/thread_pool.h"
#include "onboard/global/trace.h"
#include "onboard/lite/check.h"
#include "onboard/ml_planner/ml_cruise_task.h"
#include "onboard/ml_planner/ml_cruise_task_input.h"
#include "onboard/ml_planner/ml_planner_flags.h"
#include "onboard/ml_planner/proto/ml_planner_status.pb.h"
#include "onboard/params/vehicle_param_api.h"
#include "onboard/planner/object/low_likelihood_filter.h"
#include "onboard/planner/object/motion_state_filter.h"
#include "onboard/planner/object/object_vector.h"
#include "onboard/planner/object/planner_object.h"
#include "onboard/planner/object/planner_object_manager.h"
#include "onboard/planner/object/planner_object_manager_builder.h"
#include "onboard/planner/object/predicted_motion_filter.h"
#include "onboard/planner/object/reflected_object_in_proximity_filter.h"
#include "onboard/planner/object/spacetime_trajectory_manager.h"
#include "onboard/planner/object/trajectory_filter.h"
#include "onboard/planner/object/unknown_roadway_position_object_filter.h"
#include "onboard/planner/plan/plan_task_helper.h"
#include "onboard/proto/positioning.pb.h"
#include "onboard/proto/vehicle.pb.h"
#include "onboard/utils/time_util.h"

namespace qcraft::mlplanner {

namespace {

absl::StatusOr<planner::PlannerObjectManager> PreprocessPlannerObjectManager(
    const PoseProto& pose, const VehicleGeometryParamsProto& veh_geo_params,
    planner::ObjectVector<planner::PlannerObject> planner_objects,
    double planner_prediction_probability_threshold,
    double planner_filter_reflected_object_distance, ThreadPool* thread_pool) {
  SCOPED_QTRACE("MlPlannerModule::PreprocessPlannerObjectManager");
  // Enabled low likelihood object filter.
  const planner::LowLikelihoodFilter low_likelihood_filter(
      planner_prediction_probability_threshold,
      /*only_use_most_likely_traj=*/false);
  const planner::MotionStateFilter motion_state_filter(pose, veh_geo_params);
  const planner::PredictedMotionFilter predicted_motion_filter(planner_objects);
  std::vector<const planner::TrajectoryFilter*> filters = {
      &low_likelihood_filter, &predicted_motion_filter};
  // HACK(jingjiao): Always enabe motion state filter.
  // Enabled motion state filter on ON_ROAD_CRUISE_PLAN.
  filters.push_back(&motion_state_filter);
  // TODO(lidong): Delete the code after March 01, 2022
  const planner::ReflectedObjectInProximityFilter object_in_proximity_filter(
      pose, veh_geo_params, planner_filter_reflected_object_distance);
  if (FLAGS_ml_planner_filter_reflected_object_distance > 0.0) {
    filters.push_back(&object_in_proximity_filter);
  }
  const planner::UnknownRoadwayPositionObjectFilter
      unknown_roadway_position_object_filter;
  if (FLAGS_ml_planner_filter_unknown_roadway_position_object) {
    filters.push_back(&unknown_roadway_position_object_filter);
  }
  planner::PlannerObjectManagerBuilder obj_mgr_builder;
  obj_mgr_builder.set_planner_objects(std::move(planner_objects))
      .set_filters(filters);
  return obj_mgr_builder.Build(/*filtered_trajs=*/nullptr, thread_pool);
}
}  // namespace
MLPlannerStatus RunMLTaskDispatcher(const MLPlannerInput& input,
                                    const ObjectsProto* objects_proto,
                                    MlPlannerOutputProto* output,
                                    MlPlannerDebugProto* ml_planner_debug) {
  SCOPED_QTRACE("MlPlannerModule::RunMLTaskDispatcher");
  const auto& veh_geo_params = input.vehicle_params.vehicle_geometry_params();
  auto* thread_pool = input.thread_pool.get();
  // TODO(jingjiao): Temporal hack.
  absl::Time plan_time = FromUnixDoubleSeconds(input.pose->timestamp());

  // Preprocess planner objects.
  auto preprocess_planner_objects_output = PreprocessPlannerObjects(
      *input.planner_semantic_map_manager,
      *QCHECK_NOTNULL(input.traffic_light_states),
      input.prediction_conflict_resolver_params, input.prediction.get(),
      objects_proto, plan_time, /*planner_consider_objects=*/true, thread_pool);

  auto object_manager_or = PreprocessPlannerObjectManager(
      *input.pose, veh_geo_params,
      std::move(preprocess_planner_objects_output.planner_objects),
      FLAGS_ml_planner_prediction_probability_threshold,
      FLAGS_ml_planner_filter_reflected_object_distance, thread_pool);
  if (!object_manager_or.ok()) {
    return MLPlannerStatus(MLPlannerStatusProto::OBJECT_MANAGER_FAILED,
                           object_manager_or.status().message());
  }
  auto object_manager = std::make_shared<const planner::PlannerObjectManager>(
      std::move(*object_manager_or));
  auto st_traj_mgr = std::make_shared<planner::SpacetimeTrajectoryManager>(
      absl::Span<const planner::TrajectoryFilter*>{},
      object_manager->planner_objects(), thread_pool);

  return RunMLCruiseTask(
      MLCruiseTaskInput{
          .ml_planner_input = input,
          .st_traj_mgr = st_traj_mgr,
          .object_manager = object_manager,
      },
      output, ml_planner_debug);
}

}  // namespace qcraft::mlplanner

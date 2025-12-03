#include "onboard/planner/plan/acc/acc_task_internal.h"

#include <algorithm>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <ostream>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_format.h"
#include "absl/strings/str_join.h"
#include "absl/strings/string_view.h"
#include "absl/types/span.h"
#include "glog/logging.h"

#include "onboard/eval/qevent.h"
#include "onboard/eval/qevent_base.h"
#include "onboard/global/logging.h"
#include "onboard/global/trace.h"
#include "onboard/hmi/events/run_event.h"
#include "onboard/lite/check.h"
#include "onboard/maps/lane_path.h"
#include "onboard/math/fast_math.h"
#include "onboard/math/frenet_common.h"
#include "onboard/math/frenet_frame.h"
#include "onboard/math/piecewise_linear_function.h"
#include "onboard/math/util.h"
#include "onboard/planner/common/plan_start_point_info.h"
#include "onboard/planner/common/proto/planner_status.pb.h"
#include "onboard/planner/hmi_util.h"
#include "onboard/planner/object/plot_util.h"
#include "onboard/planner/object/spacetime_object_trajectory.h"
#include "onboard/planner/object/spacetime_trajectory_manager.h"
#include "onboard/planner/plan/acc/acc_corridor.h"
#include "onboard/planner/plan/acc/acc_corridor_combined.h"
#include "onboard/planner/plan/acc/acc_corridor_util.h"
#include "onboard/planner/plan/acc/acc_corridor_with_map.h"
#include "onboard/planner/plan/acc/acc_corridor_without_map.h"
#include "onboard/planner/plan/acc/acc_defs.h"
#include "onboard/planner/plan/acc/acc_flags.h"
#include "onboard/planner/plan/acc/acc_preliminary_speed.h"
#include "onboard/planner/plan/acc/acc_speed_finder.h"
#include "onboard/planner/plan/acc/acc_speed_finder_input.h"
#include "onboard/planner/plan/acc/acc_speed_finder_output.h"
#include "onboard/planner/plan/acc/acc_target.h"
#include "onboard/planner/plan/acc/acc_target_selector_with_map.h"
#include "onboard/planner/plan/acc/acc_target_selector_without_map.h"
#include "onboard/planner/planner_defs.h"
#include "onboard/planner/planner_flags.h"
#include "onboard/planner/planner_main_loop_internal.h"
#include "onboard/planner/planner_util.h"
#include "onboard/planner/proto/trajectory_validation.pb.h"
#include "onboard/planner/scheduler/proto/lane_change.pb.h"
#include "onboard/planner/second_order_trajectory_point.h"
#include "onboard/planner/trajectory_validation/trajectory_validation.h"
#include "onboard/proto/charts.pb.h"
#include "onboard/proto/perception/fusion/object.pb.h"
#include "onboard/proto/planner.pb.h"
#include "onboard/proto/trajectory.pb.h"
#include "onboard/proto/turn_signal.pb.h"
#include "onboard/utils/map_util.h"
#include "onboard/utils/time_util.h"
#include "onboard/vis/common/color.h"

namespace qcraft::planner {

namespace {

constexpr double kCorridorPreviewTime = kAccTrajectoryTimeHorizon;  // s.

const std::vector<AccCorridorSource> kSourcesByPriority = {
    AccCorridorSource::MAP, AccCorridorSource::COMBINED,
    AccCorridorSource::ESTIMATED};

}  // namespace
namespace internal {
inline PiecewiseLinearFunction<double, double>
ConvertToFollowHeadwayTimeVelocityKphPlf(
    QRunEvent::LccFollowingDistanceLevel level) {
  switch (level) {
    case QRunEvent::LCC_FOLLOWING_DISTANCE_LEVEL_NEAR:
      return PiecewiseLinearFunction<double, double>({0.0, 120.0}, {0.8, 0.9});
    case QRunEvent::LCC_FOLLOWING_DISTANCE_LEVEL_SLIGHTLY_NEAR:
      return PiecewiseLinearFunction<double, double>({0.0, 120.0}, {0.9, 1.05});
    case QRunEvent::LCC_FOLLOWING_DISTANCE_LEVEL_NONE:
    case QRunEvent::LCC_FOLLOWING_DISTANCE_LEVEL_MEDIUM:
      return PiecewiseLinearFunction<double, double>({0.0, 120.0},
                                                     {1.025, 1.3});
    case QRunEvent::LCC_FOLLOWING_DISTANCE_LEVEL_SLIGHTLY_FAR:
      return PiecewiseLinearFunction<double, double>({0.0, 120.0}, {1.2, 1.6});
    case QRunEvent::LCC_FOLLOWING_DISTANCE_LEVEL_FAR:
      return PiecewiseLinearFunction<double, double>({0.0, 120.0}, {1.4, 2.0});
  }
}

inline bool IsObjectTypeToIncreaseFollowDistance(ObjectType type) {
  // Large vehicle, cycs, unknown objects.
  switch (type) {
    case ObjectType::OT_LARGE_VEHICLE:
    case ObjectType::OT_MOTORCYCLIST:
    case ObjectType::OT_CYCLIST:
    case ObjectType::OT_TRICYCLIST:
    case ObjectType::OT_UNKNOWN_MOVABLE:
    case ObjectType::OT_UNKNOWN_STATIC:
      return true;
    case ObjectType::OT_WARNING_TRIANGLE:
    case ObjectType::OT_PEDESTRIAN:
    case ObjectType::OT_CONE:
    case ObjectType::OT_BARRIER:
    case ObjectType::OT_BARRIER_ANTI_COLLISION_BUCKET:
    case ObjectType::OT_BARRIER_ANTI_COLLISION_POST:
    case ObjectType::OT_FOD:
    case ObjectType::OT_VEGETATION:
    case ObjectType::OT_VEHICLE:
      return false;
  }
}

SpeedFinderParamsProto ModifySpeedFinderParams(
    const SpeedFinderParamsProto& speed_finder_params,
    QRunEvent::LccFollowingDistanceLevel follow_distance_level,
    bool /*crowded_scene*/, double plan_start_v,
    std::optional<ObjectType> target_type) {
  constexpr double kSpecialVehicleExtraFollowTimeHeadway = 0.3;
  constexpr double kSpecialVehicleExtraFollowStandstillDistance = 0.5;  // m.
  const auto current_level_plf =
      ConvertToFollowHeadwayTimeVelocityKphPlf(follow_distance_level);
  SpeedFinderParamsProto new_speed_finder_params = speed_finder_params;

  double follow_time_headway = current_level_plf(Mps2Kph(plan_start_v));
  double follow_standstill_distance =
      speed_finder_params.follow_standstill_distance();
  double large_vehicle_follow_time_headway =
      follow_time_headway + kSpecialVehicleExtraFollowTimeHeadway;

  if (target_type.has_value() &&
      IsObjectTypeToIncreaseFollowDistance(*target_type)) {
    follow_time_headway += kSpecialVehicleExtraFollowTimeHeadway;
    follow_standstill_distance += kSpecialVehicleExtraFollowStandstillDistance;
    if (*target_type == ObjectType::OT_LARGE_VEHICLE) {
      large_vehicle_follow_time_headway = follow_time_headway;
    }
  }

  new_speed_finder_params.set_follow_time_headway(follow_time_headway);
  new_speed_finder_params.set_large_vehicle_follow_time_headway(
      large_vehicle_follow_time_headway);
  new_speed_finder_params.set_follow_standstill_distance(
      follow_standstill_distance);

  // TODO(changqing): check crowded_scene object effects once acc speed finder
  // and general speed finder is merged.
  return new_speed_finder_params;
}

std::vector<ApolloTrajectoryPointProto> CreateAccPastPointsList(
    absl::Span<const ApolloTrajectoryPointProto> origin_past_pts,
    const ApolloTrajectoryPointProto& plan_start_point) {
  std::vector<ApolloTrajectoryPointProto> acc_past_pts;
  acc_past_pts.reserve(origin_past_pts.size());
  const auto heading = plan_start_point.path_point().theta();
  const auto* ref_pt = &plan_start_point;
  for (int i = origin_past_pts.size() - 1; i >= 0; --i) {
    auto past_pt = origin_past_pts[i];
    double heading_cos_sin[2];
    fast_math::CosAndSin<7>(heading, heading_cos_sin);
    const double past_x =
        ref_pt->path_point().x() +
        (past_pt.path_point().s() - ref_pt->path_point().s()) *
            heading_cos_sin[0];
    const double past_y =
        ref_pt->path_point().y() +
        (past_pt.path_point().s() - ref_pt->path_point().s()) *
            heading_cos_sin[1];
    past_pt.mutable_path_point()->set_x(past_x);
    past_pt.mutable_path_point()->set_y(past_y);
    past_pt.mutable_path_point()->set_theta(heading);
    acc_past_pts.push_back(std::move(past_pt));
    ref_pt = &acc_past_pts.back();
  }
  std::reverse(acc_past_pts.begin(), acc_past_pts.end());
  return acc_past_pts;
}

const SpacetimeObjectTrajectory* FindClosestAccTargetOrNull(
    const AccCorridor& corridor, const AccTargetPerCorridor& targets) {
  const SpacetimeObjectTrajectory* closest_st_traj = nullptr;
  double min_s = std::numeric_limits<double>::infinity();
  const auto& st_traj_mgr = *targets.st_traj_mgr();
  const auto& path_ff = corridor.frenet_frame;
  for (const auto& target : targets.leaders) {
    const auto st_trajs =
        st_traj_mgr.FindTrajectoriesByObjectId(target.object_id);
    for (const auto* st_traj : st_trajs) {
      const auto& pose = st_traj->pose();
      const auto pose_sl = path_ff.XYToSL(pose.pos());
      if (pose_sl.s < min_s) {
        closest_st_traj = st_traj;
        min_s = pose_sl.s;
      }
    }
  }
  return closest_st_traj;
}

bool AccGetCollisionWarningRequest(bool prev_collision_warning_request,
                                   const AccCorridor& corridor,
                                   const SpacetimeObjectTrajectory& st_traj,
                                   double plan_start_v,
                                   double follow_time_headway) {
  const auto& object_box = st_traj.bounding_box();
  const auto& path_ff = corridor.frenet_frame;
  const auto obj_fbox_or = path_ff.QueryFrenetBoxAt(object_box);
  const auto pose_sl = path_ff.XYToSL(st_traj.pose().pos());
  double min_s = pose_sl.s;
  if (obj_fbox_or.ok()) {
    min_s = obj_fbox_or->s_min;
  }
  AlertedFrontVehicleInfo obj_info{
      .obj_id = std::string(st_traj.object_id()),
      .obj_v = st_traj.pose().v(),
      .min_s = min_s,
  };
  return GetCollisionWarningRequest(prev_collision_warning_request, obj_info,
                                    plan_start_v, follow_time_headway);
}

void ReportAccTaskToHmi(const AccCorridor& corridor,
                        const AccTargetPerCorridor& targets,
                        bool prev_collision_warning_request,
                        double plan_start_v, double follow_time_headway,
                        HmiContentProto* hmi_content) {
  // 0.0 Find closest target.
  const auto* hmi_acc_target_traj =
      FindClosestAccTargetOrNull(corridor, targets);
  if (hmi_acc_target_traj == nullptr) {
    return;
  }

  // 1.0 Add to alerted hightlight object.
  auto& acc_following_target = *hmi_content->add_highlight_objects();
  acc_following_target.set_object_id(
      std::string(hmi_acc_target_traj->object_id()));
  acc_following_target.set_type(
      HmiContentProto::HighlightObjectInfo::HLO_ACC_FOLLOWING_TARGET);

  // 2.0 If it's vehicle or large vehicle, decide whether to send collision
  // warning.
  if (hmi_acc_target_traj->object_type() == ObjectType::OT_VEHICLE ||
      hmi_acc_target_traj->object_type() == ObjectType::OT_LARGE_VEHICLE) {
    const auto collision_warning_request = AccGetCollisionWarningRequest(
        prev_collision_warning_request, corridor, *hmi_acc_target_traj,
        plan_start_v, follow_time_headway);
    hmi_content->set_collision_warning_request(collision_warning_request);
  }
}

void ReportRunEventWhenTargetDriveoffInStandwait(
    bool is_acc_standwait, const AccTargetPerCorridor& acc_target) {
  if (!is_acc_standwait) {
    return;
  }
  if (!acc_target.has_leaders()) {
    return;
  }
  if (!acc_target.st_traj_mgr()->moving_object_trajs().empty()) {
    QRUNEVENT_WITH_BOOL_NOTICE(QRunEvent::KEY_QEVENT_FRONT_TARGET_CAR_DRIVE_OFF,
                               true);
  }
}

PlannerStatus PlanAccSpeedForCorridorAndValidate(
    const AccSpeedFinderForCorridorInput& input, AccSpeedFinderOutput* output,
    TrajectoryValidationResultProto* traj_validation_result) {
  FUNC_QTRACE();
  QCHECK_NOTNULL(input.plan_start_point);
  QCHECK_NOTNULL(input.corridor);
  QCHECK_NOTNULL(input.targets);
  QCHECK_NOTNULL(input.acc_params);
  QCHECK_NOTNULL(input.vehicle_geometry_params);
  QCHECK_NOTNULL(input.vehicle_drive_params);
  QCHECK_NOTNULL(input.crowded_side_objs);
  QCHECK_NOTNULL(input.speed_finder_params);
  QCHECK_NOTNULL(input.motion_constraint_params);

  const auto& acc_cruising_speed_limit = input.acc_cruising_speed_limit;
  const auto& motion_constraint_params = *input.motion_constraint_params;
  const auto& corridor = *input.corridor;
  const auto& plan_start_point = *input.plan_start_point;
  const auto source = input.targets->source;

  // 0. Modify speed finder params (follow_time_headway and
  // follow_standstill_distance) according to user's follow_distance_level,
  // current speed and optional AccTarget ObjectType.
  const auto* closest_target_traj =
      FindClosestAccTargetOrNull(corridor, *input.targets);
  std::optional<ObjectType> target_type_opt = std::nullopt;
  if (closest_target_traj != nullptr) {
    target_type_opt = closest_target_traj->object_type();
  }
  const auto speed_finder_params = internal::ModifySpeedFinderParams(
      *input.speed_finder_params, input.following_distance_level,
      /*crowded_scene=*/!input.crowded_side_objs->empty(), plan_start_point.v(),
      target_type_opt);

  // 1. Calculate preliminary speed.
  AccPreliminarySpeedInput preliminary_speed_input{
      .plan_start_point = input.plan_start_point,
      .corridor = input.corridor,
      .targets = input.targets,
      .optional_ext_speed_limit = input.acc_cruising_speed_limit,
      .av_front_to_rac = input.vehicle_geometry_params->front_edge_to_center(),
      .ttc = speed_finder_params.follow_time_headway(),
      .standstill_dist = speed_finder_params.follow_standstill_distance(),
      .is_acc_standwait = input.is_acc_standwait,
  };
  auto preliminary_speed_out = RunAccPreliminarySpeed(preliminary_speed_input);

  // 2. Speed planning.
  AccSpeedFinderInput acc_speed_finder_input{
      .base_name = "acc_speed",
      .traj_mgr = input.targets->st_traj_mgr(),
      .path = &(corridor.path),
      .speed_limit = &preliminary_speed_out.speed_limit,
      .time_aligned_prev_traj = input.time_aligned_prev_traj,
      .plan_start_v = plan_start_point.v(),
      .plan_start_a = plan_start_point.a(),
      .user_speed_limit = acc_cruising_speed_limit.value_or(
          Mph2Mps(motion_constraint_params.default_speed_limit())),
      .plan_time = input.plan_time,
      .crowded_side_objs = input.crowded_side_objs,
      .motion_constraint_params = input.motion_constraint_params,
      .speed_finder_params = input.speed_finder_params,
      .vehicle_geom = input.vehicle_geometry_params,
  };
  auto acc_speed_output_or = FindAccSpeed(acc_speed_finder_input);
  if (!acc_speed_output_or.ok()) {
    return PlannerStatus(
        PlannerStatusProto::ACC_SPEED_FAILED,
        absl::StrFormat("Acc find speed for corridor %s fails: %s.",
                        AccCorridorSource_Name(source),
                        acc_speed_output_or.status().message()));
  }
  // 3. Validate trajectory.
  if (!ValidateAccTrajectory(
          *input.vehicle_geometry_params, *input.vehicle_drive_params,
          motion_constraint_params, acc_speed_output_or->trajectory_points,
          input.acc_past_points, traj_validation_result)) {
    return PlannerStatus(
        PlannerStatusProto::ACC_TRAJECTORY_VALIDATION_FAILED,
        absl::StrFormat("Acc corridor %s trajectory validation fails: %s.",
                        AccCorridorSource_Name(source),
                        traj_validation_result->DebugString()));
  }
  *output = std::move(acc_speed_output_or).value();
  output->preliminary_speed_debug =
      std::move(preliminary_speed_out.preliminary_speed_debug);
  return PlannerStatus(PlannerStatusProto::OK,
                       absl::StrFormat("Acc plan ok for corridor %s.",
                                       AccCorridorSource_Name(source)));
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
    AccPlannerOutput* acc_planner_output) {
  // Given chosen active source (based on flag, whether to early exit etc.),
  // collect final result and return acc planner status.
  acc_planner_output->active_corridor_source = active_source;
  acc_planner_output->acc_past_points = std::move(acc_past_points);
  acc_planner_output->source_status = std::move(source_status);
  acc_planner_output->corridors = std::move(source_corridors);
  acc_planner_output->targets = std::move(source_targets);

  auto* traj_val_result = FindOrNull(source_val_results, active_source);
  auto* speed_solution = FindOrNull(source_solutions, active_source);
  const auto* active_source_status =
      FindOrNull(acc_planner_output->source_status, active_source);
  if (traj_val_result == nullptr || speed_solution == nullptr ||
      active_source_status == nullptr) {
    return PlannerStatus(
        PlannerStatusProto::ACC_ALL_PLAN_FAIL,
        "When fill acc planner output: Acc fails to plan for all corridors.");
  }
  acc_planner_output->traj_validation_result = std::move(*traj_val_result);
  acc_planner_output->acc_speed_output = std::move(*speed_solution);
  return *active_source_status;
}

}  // namespace internal

// NOLINTNEXTLINE
PlannerStatus RunAccPlanner(const AccTaskInput& input,
                            AccPlannerOutput* acc_planner_output) {
  SCOPED_QTRACE("RunAccPlanner");
  const auto& plan_start_point = input.plan_start_point_info->start_point;
  const auto* psmm = input.planner_semantic_map_manager;

  // 0. Modify speed finder params and motion constraints for ACC. Calculate
  // common infomation for different kinds of corridors.
  const auto& vehicle_geometry_params = *input.vehicle_geometry_params;
  const auto& vehicle_drive_params = *input.vehicle_drive_params;
  std::set<std::string> crowded_side_objs;
  if (psmm != nullptr && FLAGS_planner_acc_avoid_crowded_scene_cutin) {
    constexpr double kCrowdedSceneLookForwardDistance = 15.0;
    constexpr double kCrowdedSceneLookBackwardDistance = 5.0;
    crowded_side_objs = CrowdedSceneObjects(
        *psmm, *input.st_traj_mgr, *input.pose, vehicle_geometry_params,
        /*look_forward_length=*/kCrowdedSceneLookForwardDistance,
        /*look_backward_length=*/kCrowdedSceneLookBackwardDistance);
    VLOG(1) << "Acc task internal: crowded side objects? "
            << absl::StrJoin(crowded_side_objs, ",");
  }

  const auto motion_constraint_params =
      internal::FillRequiredMotionConstraintParams(
          input.acc_params->motion_constraint_params(),
          input.acc_params->acc_req_params(), plan_start_point.v());
  const auto past_points = CreatePastPointsList(
      input.plan_time, *input.prev_trajectory,
      input.plan_start_point_info->reset_reason != ResetReasonProto::SPEED_ONLY,
      kMaxAccPastPointNum);
  auto acc_past_points =
      internal::CreateAccPastPointsList(past_points, plan_start_point);

  std::map<AccCorridorSource, PlannerStatus> source_status;
  std::map<AccCorridorSource, TrajectoryValidationResultProto>
      source_val_results;
  std::map<AccCorridorSource, std::unique_ptr<AccTargetPerCorridor>>
      source_targets;
  std::map<AccCorridorSource, std::unique_ptr<AccCorridor>> source_corridors;
  std::map<AccCorridorSource, AccSpeedFinderOutput> source_solutions;
  std::map<AccCorridorSource, internal::AccSpeedFinderForCorridorInput>
      source_speed_val_inputs;
  AccCorridorSource curr_source;

  const auto prev_primary_targets =
      internal::GetPrevTargetsFromProto(input.acc_task_proto);
  const auto& acc_cruising_speed_limit = input.lcc_cruising_speed_limit;

  // 0. Build shared driving map topo and collect all possible lane paths as
  // with_map / combined corridors' input.
  std::unique_ptr<DrivingMapTopo> driving_map = nullptr;
  if (psmm != nullptr) {
    auto driving_map_or =
        BuildDrivingMapTopo(*input.pose, *psmm, plan_start_point.v());
    if (driving_map_or.ok()) {
      driving_map =
          std::make_unique<DrivingMapTopo>(std::move(*driving_map_or));
    }
  }

  // 1. Find ACC map corridor and targets.
  {
    SCOPED_QTRACE("AccTaskInternal::PlanForMapCorridor");
    curr_source = AccCorridorSource::MAP;
    if (psmm != nullptr && driving_map != nullptr) {
      auto corridor_map_or = BuildAccCorridorFromClosestLanePath(
          *input.pose, plan_start_point, *psmm, *driving_map,
          input.steering_percentage, vehicle_geometry_params,
          vehicle_drive_params, kAccCorridorSampleInterval,
          kCorridorPreviewTime);
      if (corridor_map_or.ok()) {
        source_corridors[curr_source] =
            std::make_unique<AccCorridor>(std::move(corridor_map_or).value());
      } else {
        source_status[curr_source] =
            PlannerStatus(PlannerStatusProto::ACC_CORRIDOR_FAILED,
                          absl::StrFormat("ACC corridor MAP build failed: %s.",
                                          corridor_map_or.status().message()));
        VLOG(2) << corridor_map_or.status().message();
        QEVENT_EVERY_N_SECONDS(
            "xiangjun", "acc_map_corridor_failed", 5.0, [&](QEvent* qevent) {
              qevent->AddField(
                  "plan_time",
                  ToUnixDoubleSeconds(input.plan_start_point_info->plan_time));
            });
      }
    } else {
      source_status[curr_source] = PlannerStatus(
          PlannerStatusProto::ACC_CORRIDOR_FAILED,
          "Acc corridor with map build fail: empty psmm or empty driving map.");
    }

    if (source_corridors[curr_source].get() != nullptr) {
      const AccTargetInput targets_map_input({
          .corridor = source_corridors[curr_source].get(),
          .st_traj_mgr = input.st_traj_mgr,
          .prev_primary_targets = &prev_primary_targets,
          .av_pose = input.pose,
          .vehicle_geom = &vehicle_geometry_params,
          .plan_start_point = &plan_start_point,
      });
      AccTargetPerCorridor targets_map =
          SelectAccTargetWithMap(targets_map_input);
      source_targets[curr_source] =
          std::make_unique<AccTargetPerCorridor>(std::move(targets_map));
      if (FLAGS_planner_draw_acc_target_st_trajectories) {
        const auto& trajs_map =
            source_targets[curr_source]->st_traj_mgr()->trajectories();
        for (const auto& traj : trajs_map) {
          DrawSpacetimeObjectTrajectory(
              traj, absl::StrFormat("map target st trajs: %s", traj.traj_id()),
              vis::Color::kDarkViolet);
        }
      }
      source_speed_val_inputs[curr_source] =
          internal::AccSpeedFinderForCorridorInput{
              .corridor = source_corridors[curr_source].get(),
              .targets = source_targets[curr_source].get(),
              .acc_params = input.acc_params,
              .plan_start_point = &plan_start_point,
              .vehicle_geometry_params = input.vehicle_geometry_params,
              .vehicle_drive_params = input.vehicle_drive_params,
              .is_acc_standwait = input.is_acc_standwait,
              .motion_constraint_params = &motion_constraint_params,
              .speed_finder_params = &input.acc_params->speed_finder_params(),
              .time_aligned_prev_traj = input.time_aligned_prev_traj,
              .crowded_side_objs = &crowded_side_objs,
              .acc_past_points = absl::MakeSpan(acc_past_points),
              .acc_cruising_speed_limit = acc_cruising_speed_limit,
              .plan_time = input.plan_start_point_info->plan_time,
              .following_distance_level = input.following_distance_level,
          };
      source_status[curr_source] = internal::PlanAccSpeedForCorridorAndValidate(
          source_speed_val_inputs[curr_source], &source_solutions[curr_source],
          &source_val_results[curr_source]);
      if (source_status[curr_source].ok() &&
          !FLAGS_planner_acc_plan_for_all_sources &&
          !FLAGS_planner_force_no_map) {
        return internal::FillAccPlannerOutput(
            std::move(source_status), std::move(source_val_results),
            std::move(source_targets), std::move(source_corridors),
            std::move(source_solutions), curr_source,
            std::move(acc_past_points), acc_planner_output);
      }
    }
  }

  // 2. Find Acc combined corridor and targets.
  {
    SCOPED_QTRACE("AccTaskInternal::PlanForCombinedCorridor");
    curr_source = AccCorridorSource::COMBINED;
    // Combined corridor rely on estimated corridor input.
    auto corridor_estimate_or = BuildAccCorridorFromPlanStartPoint(
        *input.pose, plan_start_point, input.steering_percentage,
        vehicle_geometry_params, vehicle_drive_params,
        kAccCorridorSampleInterval, kCorridorPreviewTime, input.average_kappa);
    if (corridor_estimate_or.ok()) {
      // Fill estimated result!
      source_corridors[AccCorridorSource::ESTIMATED] =
          std::make_unique<AccCorridor>(
              std::move(corridor_estimate_or).value());
    } else {
      source_status[AccCorridorSource::ESTIMATED] = PlannerStatus(
          PlannerStatusProto::ACC_CORRIDOR_FAILED,
          absl::StrFormat("Acc corridor %s build fail: %s.",
                          AccCorridorSource_Name(curr_source),
                          corridor_estimate_or.status().message()));
    }
    const auto* corridor_estimated =
        source_corridors[AccCorridorSource::ESTIMATED].get();
    if (psmm != nullptr && corridor_estimated != nullptr &&
        driving_map.get() != nullptr) {
      auto corridor_combined_or = BuildPossiblePreviewLaneKeepAccCorridor(
          *psmm, plan_start_point, corridor_estimated->path,
          corridor_estimated->frenet_frame, *driving_map.get(),
          corridor_estimated->credible_length, kAccCorridorSampleInterval,
          kCorridorPreviewTime);
      if (corridor_combined_or.ok()) {
        source_corridors[curr_source] = std::make_unique<AccCorridor>(
            std::move(corridor_combined_or).value());
      } else {
        source_status[curr_source] = PlannerStatus(
            PlannerStatusProto::ACC_CORRIDOR_FAILED,
            absl::StrFormat("Acc corridor %s build fail: %s.",
                            AccCorridorSource_Name(curr_source),
                            corridor_combined_or.status().message()));
        VLOG(2) << corridor_combined_or.status().message();
      }
    } else {
      source_status[curr_source] =
          PlannerStatus(PlannerStatusProto::ACC_CORRIDOR_FAILED,
                        "Acc combined corridor build fail: empty psmm or empty "
                        "estimated corridor or empty driving map!");
    }
    // Solve target based on corridor and plan speed.
    if (source_corridors[curr_source].get() != nullptr) {
      const AccTargetInput targets_combined_input({
          .corridor = source_corridors[curr_source].get(),
          .st_traj_mgr = input.st_traj_mgr,
          .prev_primary_targets = &prev_primary_targets,
          .av_pose = input.pose,
          .vehicle_geom = &vehicle_geometry_params,
          .plan_start_point = &plan_start_point,
      });
      AccTargetPerCorridor targets_combined =
          SelectAccTargetWithoutMap(targets_combined_input);
      targets_combined.source = curr_source;
      source_targets[curr_source] =
          std::make_unique<AccTargetPerCorridor>(std::move(targets_combined));
      if (FLAGS_planner_draw_acc_target_st_trajectories) {
        const auto& trajs_map =
            source_targets[curr_source]->st_traj_mgr()->trajectories();
        for (const auto& traj : trajs_map) {
          DrawSpacetimeObjectTrajectory(
              traj,
              absl::StrFormat("combined target st trajs: %s", traj.traj_id()),
              vis::Color::kSeaGreen);
        }
      }
      source_speed_val_inputs[curr_source] =
          internal::AccSpeedFinderForCorridorInput{
              .corridor = source_corridors[curr_source].get(),
              .targets = source_targets[curr_source].get(),
              .acc_params = input.acc_params,
              .plan_start_point = &plan_start_point,
              .vehicle_geometry_params = input.vehicle_geometry_params,
              .vehicle_drive_params = input.vehicle_drive_params,
              .is_acc_standwait = input.is_acc_standwait,
              .motion_constraint_params = &motion_constraint_params,
              .speed_finder_params = &input.acc_params->speed_finder_params(),
              .time_aligned_prev_traj = input.time_aligned_prev_traj,
              .crowded_side_objs = &crowded_side_objs,
              .acc_past_points = absl::MakeSpan(acc_past_points),
              .acc_cruising_speed_limit = acc_cruising_speed_limit,
              .plan_time = input.plan_start_point_info->plan_time,
              .following_distance_level = input.following_distance_level,
          };
      source_status[curr_source] = internal::PlanAccSpeedForCorridorAndValidate(
          source_speed_val_inputs[curr_source], &source_solutions[curr_source],
          &source_val_results[curr_source]);
      if (source_status[curr_source].ok() &&
          !FLAGS_planner_acc_plan_for_all_sources &&
          !FLAGS_planner_force_no_map) {
        return internal::FillAccPlannerOutput(
            std::move(source_status), std::move(source_val_results),
            std::move(source_targets), std::move(source_corridors),
            std::move(source_solutions), curr_source,
            std::move(acc_past_points), acc_planner_output);
      }
    }
  }

  // 3 Find ACC no map targets and plan speed. (corridor should be solved at
  // plan for combined part).
  {
    SCOPED_QTRACE("AccTaskInternal::PlanForEstimatedCorridor");
    curr_source = AccCorridorSource::ESTIMATED;
    if (source_corridors[curr_source].get() != nullptr) {
      // Estimated corridor should have been solved at COMBINED solution stage.
      const AccTargetInput targets_estimate_input({
          .corridor = source_corridors[curr_source].get(),
          .st_traj_mgr = input.st_traj_mgr,
          .prev_primary_targets = &prev_primary_targets,
          .av_pose = input.pose,
          .vehicle_geom = &vehicle_geometry_params,
          .plan_start_point = &plan_start_point,
      });
      AccTargetPerCorridor targets_estimate =
          SelectAccTargetWithoutMap(targets_estimate_input);
      source_targets[curr_source] =
          std::make_unique<AccTargetPerCorridor>(std::move(targets_estimate));
      if (FLAGS_planner_draw_acc_target_st_trajectories) {
        const auto& trajs_estimate =
            source_targets[curr_source]->st_traj_mgr()->trajectories();
        for (const auto& traj : trajs_estimate) {
          DrawSpacetimeObjectTrajectory(
              traj,
              absl::StrFormat("estimate target st trajs: %s", traj.traj_id()),
              vis::Color::kOliveDrab);
        }
      }
      source_speed_val_inputs[curr_source] =
          internal::AccSpeedFinderForCorridorInput{
              .corridor = source_corridors[curr_source].get(),
              .targets = source_targets[curr_source].get(),
              .acc_params = input.acc_params,
              .plan_start_point = &plan_start_point,
              .vehicle_geometry_params = input.vehicle_geometry_params,
              .vehicle_drive_params = input.vehicle_drive_params,
              .is_acc_standwait = input.is_acc_standwait,
              .motion_constraint_params = &motion_constraint_params,
              .speed_finder_params = &input.acc_params->speed_finder_params(),
              .time_aligned_prev_traj = input.time_aligned_prev_traj,
              .crowded_side_objs = &crowded_side_objs,
              .acc_past_points = absl::MakeSpan(acc_past_points),
              .acc_cruising_speed_limit = acc_cruising_speed_limit,
              .plan_time = input.plan_start_point_info->plan_time,
              .following_distance_level = input.following_distance_level,
          };
      source_status[curr_source] = internal::PlanAccSpeedForCorridorAndValidate(
          source_speed_val_inputs[curr_source], &source_solutions[curr_source],
          &source_val_results[curr_source]);
      if (FLAGS_planner_force_no_map ||
          (source_status[curr_source].ok() &&
           !FLAGS_planner_acc_plan_for_all_sources)) {
        // Not planning for all sources and current solution is ok. Or
        // force_no_map is on, use estimated solution whether it is successful
        // or not.
        return internal::FillAccPlannerOutput(
            std::move(source_status), std::move(source_val_results),
            std::move(source_targets), std::move(source_corridors),
            std::move(source_solutions), curr_source,
            std::move(acc_past_points), acc_planner_output);
      }
    }
  }

  // 4. Plan for all sources. Iterate through status and source by priority,
  // choose the best available one.
  for (const auto& source : kSourcesByPriority) {
    const auto* status = FindOrNull(source_status, source);
    if (status == nullptr) continue;  // Which is weird.
    if (status->ok()) {
      return internal::FillAccPlannerOutput(
          std::move(source_status), std::move(source_val_results),
          std::move(source_targets), std::move(source_corridors),
          std::move(source_solutions), source, std::move(acc_past_points),
          acc_planner_output);
    }
  }
  return PlannerStatus(PlannerStatusProto::ACC_ALL_PLAN_FAIL,
                       "Acc fails to plan for all corridors.");
}

// NOLINTNEXTLINE
void FillAccRelatedOutput(AccPlannerOutput acc_planner_output,
                          bool is_acc_standwait,
                          bool prev_collision_warning_request,
                          double plan_start_v, double follow_time_headway,
                          absl::Time plan_time,
                          const std::optional<double>& acc_cruising_speed_limit,
                          AccTaskOutput* output) {
  SCOPED_QTRACE("FillAccRelatedOutput");

  auto& acc_speed_output = acc_planner_output.acc_speed_output;

  TrajectoryProto trajectory_info;
  FillTrajectoryProto(plan_time, acc_speed_output.trajectory_points,
                      acc_planner_output.acc_past_points, mapping::LanePath(),
                      LaneChangeStateProto(), TURN_SIGNAL_NONE, DoorDecision(),
                      /*is_aeb_triggered=*/false, DrivingStateProto(),
                      acc_planner_output.traj_validation_result,
                      &trajectory_info);
  output->trajectory_info = std::move(trajectory_info);

  output->chart_data.add_charts()->Swap(
      &acc_speed_output.preliminary_speed_chart);
  output->chart_data.add_charts()->Swap(&acc_speed_output.st_graph_chart);
  output->chart_data.add_charts()->Swap(&acc_speed_output.vt_graph_chart);
  output->chart_data.add_charts()->Swap(&acc_speed_output.traj_chart);
  output->chart_data.add_charts()->Swap(&acc_speed_output.path_chart);
  output->chart_data.add_charts()->Swap(&acc_speed_output.sampling_dp_chart);
  output->chart_data.add_charts()->Swap(
      &acc_speed_output.interactive_speed_chart);
  output->chart_data.add_charts()->Swap(&acc_speed_output.pred_vt_chart);

  // Send trajectory and selected targets to assist state QACCTaskProto.
  // Update to acc task proto (planner state proto).
  const auto& active_source = acc_planner_output.active_corridor_source;
  const AccCorridor* corridor_for_use_ptr = nullptr;
  const AccTargetPerCorridor* targets_for_use = nullptr;
  corridor_for_use_ptr = acc_planner_output.corridors[active_source].get();
  targets_for_use = acc_planner_output.targets[active_source].get();
  targets_for_use->ToProto(output->acc_task_proto.mutable_prev_acc_target());
  internal::ReportAccTaskToHmi(*corridor_for_use_ptr, *targets_for_use,
                               prev_collision_warning_request, plan_start_v,
                               follow_time_headway, &(output->hmi_content));
  internal::ReportRunEventWhenTargetDriveoffInStandwait(is_acc_standwait,
                                                        *targets_for_use);

  // Update to debug proto.
  acc_planner_output.ToDebugProto(output->debug_info.mutable_acc_debug());

  // Set current state
  output->acc_state = QACCState::ACC_ENABLED;
  if (acc_cruising_speed_limit.has_value()) {
    output->cruising_speed_limit = acc_cruising_speed_limit.value();
    VLOG(2) << "Output speed limit " << acc_cruising_speed_limit.value();
  }
}
}  // namespace qcraft::planner

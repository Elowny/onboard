#include "onboard/planner/plan/st_path_planner.h"

// IWYU pragma: no_include <ext/alloc_traits.h>
//              for __alloc_traits<>::value_type

#include <algorithm>
#include <map>
#include <ostream>
#include <string>
#include <utility>
#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_format.h"
#include "absl/strings/string_view.h"
#include "absl/types/span.h"
#include "glog/logging.h"

#include "onboard/async/thread_pool.h"
#include "onboard/container/strong_int.h"
#include "onboard/global/trace.h"
#include "onboard/lite/check.h"
#include "onboard/lite/logging.h"
#include "onboard/maps/lane_point.h"
#include "onboard/math/frenet_frame.h"
#include "onboard/planner/common/lane_change_safety.h"
#include "onboard/planner/common/planner_status.h"
#include "onboard/planner/common/proto/lane_change_safety.pb.h"
#include "onboard/planner/common/proto/planner_status.pb.h"
#include "onboard/planner/decision/decider_output.h"
#include "onboard/planner/initializer/initializer_input.h"
#include "onboard/planner/initializer/initializer_output.h"
#include "onboard/planner/initializer/search_motion.h"
#include "onboard/planner/min_length_path_extension.h"
#include "onboard/planner/object/planner_object.h"
#include "onboard/planner/object/proto/planner_object.pb.h"
#include "onboard/planner/object/spacetime_object_trajectory.h"
#include "onboard/planner/object/spacetime_trajectory_manager.h"
#include "onboard/planner/optimization/ddp/trajectory_optimizer.h"
#include "onboard/planner/optimization/ddp/trajectory_optimizer_input.h"
#include "onboard/planner/optimization/ddp/trajectory_optimizer_output.h"
#include "onboard/planner/optimization/ddp/trajectory_optimizer_state.h"
#include "onboard/planner/optimization/ddp/trajectory_optimizer_util.h"
#include "onboard/planner/optimization/proto/optimizer.pb.h"
#include "onboard/planner/planner_defs.h"
#include "onboard/planner/planner_flags.h"
#include "onboard/planner/router/drive_passage.h"
#include "onboard/planner/scheduler/proto/lane_change.pb.h"
#include "onboard/planner/scheduler/scheduler_output.h"
#include "onboard/planner/trajectory_point.h"
#include "onboard/planner/trajectory_util.h"
#include "onboard/planner/util/lane_path_util.h"
#include "onboard/planner/util/planner_semantic_map_util.h"
#include "onboard/planner/util/planner_status_macros.h"
#include "onboard/planner/util/vehicle_geometry_util.h"
#include "onboard/utils/file_util.h"
#include "onboard/utils/status_macros.h"

namespace qcraft::planner {

namespace {

// TODO(jingqiao): Use input instead of directly reading file.
void UpdateTrajOptParams(
    const std::string& traj_opt_params_address,
    TrajectoryOptimizerParamsProto* trajectory_optimizer_params) {
  VLOG(3) << "Trajectory optimizer params before update: ";
  VLOG(3) << trajectory_optimizer_params->DebugString();
  if (!file_util::TextFileToProto(traj_opt_params_address,
                                  trajectory_optimizer_params)) {
    QCHECK(false) << "Read trajectory optimizer params as text file failed!!!!";
  }
  QLOG(INFO) << "New trajectory optimizer params are used.";
}

std::vector<ApolloTrajectoryPointProto> ResampleOptimizerTrajectory(
    const std::vector<ApolloTrajectoryPointProto>& opt_traj) {
  std::vector<ApolloTrajectoryPointProto> resampled(kTrajectorySteps);
  for (int i = 0; i < kTrajectorySteps; ++i) {
    resampled[i] = QueryApolloTrajectoryPointByT(
        opt_traj.begin(), opt_traj.end(), i * kTrajectoryTimeStep);
  }
  return resampled;
}

std::vector<ApolloTrajectoryPointProto> StitchPreviousTrajectoryAndStTrajectory(
    int stitch_index,
    const std::vector<ApolloTrajectoryPointProto>& time_aligned_prev_traj,
    const std::vector<ApolloTrajectoryPointProto>& st_trajectory) {
  if (time_aligned_prev_traj.empty() || stitch_index <= 0) {
    return st_trajectory;
  }
  std::vector<ApolloTrajectoryPointProto> res_traj;
  res_traj.reserve(stitch_index + st_trajectory.size());
  for (int idx = 0; idx < stitch_index; ++idx) {
    res_traj.push_back(time_aligned_prev_traj[idx]);
  }
  const double s_offset = time_aligned_prev_traj[stitch_index].path_point().s();
  for (const auto& point : st_trajectory) {
    res_traj.push_back(point);
    res_traj.back().mutable_path_point()->set_s(point.path_point().s() +
                                                s_offset);
  }
  return res_traj;
}

absl::Status AddSpaceTimePlannerTrajectoryById(
    const SpacetimeTrajectoryManager& traj_mgr, absl::string_view traj_id,
    SpacetimePlannerObjectTrajectoryReason::Type reason,
    SpacetimePlannerObjectTrajectories* res) {
  if (!res->trajectory_ids.contains(traj_id)) {
    const auto* traj = traj_mgr.FindTrajectoryById(traj_id);
    if (traj == nullptr) {
      return absl::NotFoundError(
          absl::StrCat("Could not find trajectory ", traj_id));
    }
    ASSIGN_OR_RETURN(
        auto truncated_traj,
        traj->CreateTruncatedCopy(res->st_start_offset,
                                  kSpacetimePlannerTrajectoryHorizon));
    res->trajectories.push_back(std::move(truncated_traj));
    res->trajectory_infos.push_back(
        {.traj_index = traj->traj_index(),
         .object_id = traj->planner_object().is_sim_agent()
                          ? traj->planner_object().base_id()
                          : traj->planner_object().id(),
         .reason = reason});
    res->trajectory_ids.insert(std::string(traj_id));
  }
  return absl::OkStatus();
}

std::vector<TrajectoryPoint> ConvertCaptainTrajectoryToOptimizerInput(
    int trajectory_steps, double trajectory_time_step, double shift_time,
    absl::Span<const ApolloTrajectoryPointProto> captain_traj) {
  QCHECK_GT(trajectory_steps, 0);
  QCHECK_GT(trajectory_time_step, 0.0);
  if (captain_traj.empty()) return {};
  std::vector<ApolloTrajectoryPointProto> sampled_traj;
  for (int i = 0; i < trajectory_steps; ++i) {
    const double t = shift_time + trajectory_time_step * i;
    if (t > captain_traj.back().relative_time()) break;
    ApolloTrajectoryPointProto interp_point = QueryApolloTrajectoryPointByT(
        captain_traj.begin(), captain_traj.end(), t);
    // Time alignment.
    interp_point.set_relative_time(trajectory_time_step * i);
    sampled_traj.push_back(interp_point);
  }
  return ToTrajectoryPoint(sampled_traj);
}

void ModifyTrajOptParamsStyle(
    const TrajectoryOptimizerParamsProto&
        trajectory_optimizer_lc_radical_params,
    const TrajectoryOptimizerParamsProto& trajectory_optimizer_lc_normal_params,
    const TrajectoryOptimizerParamsProto&
        trajectory_optimizer_lc_conservative_params,
    LaneChangeStage lc_stage, LaneChangeStyle lc_style,
    TrajectoryOptimizerParamsProto* trajectory_optimizer_params) {
  if (lc_stage == LaneChangeStage::LCS_EXECUTING) {
    switch (lc_style) {
      case LC_STYLE_NORMAL:
        *trajectory_optimizer_params = trajectory_optimizer_lc_normal_params;
        break;
      case LC_STYLE_RADICAL:
        *trajectory_optimizer_params = trajectory_optimizer_lc_radical_params;
        break;
      case LC_STYLE_CONSERVATIVE:
        *trajectory_optimizer_params =
            trajectory_optimizer_lc_conservative_params;
        break;
    }
  }
}

}  // namespace

// NOLINTNEXTLINE(readability-function-size)
PlannerStatus RunStPathPlanner(StPathPlannerInput input,
                               StPathPlannerOutput* out,
                               ThreadPool* thread_pool) {
  SCOPED_QTRACE("SpacetimePathPlanner");
  QCHECK_NOTNULL(input.st_path_start_point_info);
  QCHECK_NOTNULL(input.vehicle_params);
  QCHECK_NOTNULL(input.planner_semantic_map_manager);
  QCHECK_NOTNULL(input.stalled_objects);
  QCHECK_NOTNULL(input.traj_mgr);
  QCHECK_NOTNULL(input.prev_target_lane_path_from_start);
  QCHECK_NOTNULL(input.time_aligned_prev_traj);
  QCHECK_NOTNULL(input.prev_initializer_state);

  // For rebuilding constraint manager on lc pause.
  QCHECK_NOTNULL(input.start_point_info);
  QCHECK_NOTNULL(input.obj_mgr);
  QCHECK_NOTNULL(input.tl_info_map);
  QCHECK_NOTNULL(input.prev_decider_state);

  // Params.
  QCHECK_NOTNULL(input.decision_constraint_config);
  QCHECK_NOTNULL(input.initializer_params);
  QCHECK_NOTNULL(input.trajectory_optimizer_params);
  QCHECK_NOTNULL(input.motion_constraint_params);
  QCHECK_NOTNULL(input.planner_functions_params);
  QCHECK_NOTNULL(input.vehicle_models_params);
  QCHECK_NOTNULL(input.trajectory_optimizer_lc_radical_params);
  QCHECK_NOTNULL(input.trajectory_optimizer_lc_normal_params);
  QCHECK_NOTNULL(input.trajectory_optimizer_lc_conservative_params);

  const auto& psmm = *input.planner_semantic_map_manager;
  const auto& vehicle_params = *input.vehicle_params;
  const auto& vehicle_geom_params = vehicle_params.vehicle_geometry_params();
  const auto& vehicle_drive_params = vehicle_params.vehicle_drive_params();

  out->st_planner_object_traj = std::move(input.init_st_planner_object_traj);

  const std::vector<ApolloTrajectoryPointProto> empty_ref_traj;
  const ml::captain_net::CaptainNetOutput empty_captain_net_output;
  // Run initializer.
  InitializerInput initializer_input{
      .planner_semantic_map_manager = &psmm,
      .path_start_point_info = input.st_path_start_point_info,
      .path_look_ahead_duration = input.path_look_ahead_duration,
      .lane_change_state = &input.scheduler_output.lane_change_state,
      .lane_change_style = input.lane_change_style,
      .stalled_objects = input.stalled_objects,
      .drive_passage = &input.scheduler_output.drive_passage,
      .st_traj_mgr = input.traj_mgr,
      .sl_boundary = &input.scheduler_output.sl_boundary,
      .prev_initializer_state = input.prev_initializer_state,
      .decision_constraint_config = input.decision_constraint_config,
      .initializer_params = input.initializer_params,
      .motion_constraint_params = input.motion_constraint_params,
      .vehicle_params = input.vehicle_params,
      .st_planner_object_traj = &out->st_planner_object_traj,
      .plan_id = input.plan_id,
      .log_av_trajectory = input.log_av_trajectory,
      .scene_reasoning = input.scene_reasoning,
      .borrow_lane = input.scheduler_output.borrow_lane,
      .av_frenet_box = &input.scheduler_output.av_frenet_box_on_drive_passage,
      .is_run_model_l4 = input.is_run_model_l4,

      .start_point_info = input.start_point_info,
      .smooth_result_map = input.smooth_result_map,
      .obj_mgr = input.obj_mgr,
      .tl_info_map = input.tl_info_map,
      .prev_target_lane_path_from_start =
          input.prev_target_lane_path_from_start,
      .prev_decider_state = input.prev_decider_state,
      .parking_brake_release_time = input.parking_brake_release_time,
      .enable_pull_over = input.enable_pull_over,
      .enable_traffic_light_stopping = input.enable_traffic_light_stopping,
      .brake_to_stop = input.brake_to_stop,
      .enable_force_stop = input.enable_force_stop,
      .captain_net_output =
          (FLAGS_planner_use_ml_trajectory_as_initializer_ref_traj ||
           FLAGS_planner_initializer_only_activate_nodes_near_capnet_traj)
              ? input.captain_net_output
              : &empty_captain_net_output,
  };

  absl::flat_hash_set<std::string> unsafe_object_ids;
  auto initializer_output_or = RunInitializer(
      initializer_input, &unsafe_object_ids, &input.scheduler_output,
      &input.decider_output, &out->initializer_debug_proto,
      &out->lane_change_safety_debug_proto, &out->chart_data, thread_pool);
  out->scheduler_output = std::move(input.scheduler_output);
  out->constraint_manager = std::move(input.decider_output.constraint_manager);
  out->decider_state = std::move(input.decider_output.decider_state);
  out->unsafe_object_ids = std::move(unsafe_object_ids);
  if (!initializer_output_or.ok()) {
    return PlannerStatus(PlannerStatusProto::INITIALIZER_FAILED,
                         initializer_output_or.status().message());
  }
  auto initializer_output = std::move(initializer_output_or).value();
  out->leading_trajs = std::move(initializer_output.leading_trajs);

  if (out->scheduler_output.lane_change_state.stage() ==
      LaneChangeStage::LCS_PAUSE) {
    const auto start_lane_id =
        out->scheduler_output.drive_passage.lane_path().front().lane_id();
    if (input.prev_target_lane_path_from_start->IsEmpty() ||
        input.prev_target_lane_path_from_start->front().lane_id() !=
            start_lane_id) {
      QLOG(WARNING) << absl::StrFormat(
          "Scheduler branch deleted: Target lane %d not safe", start_lane_id);
      return PlannerStatus(
          PlannerStatusProto::LC_SAFETY_CHECK_FAILED,
          absl::StrCat("Unsafe to initiate lane change to ",
                       out->scheduler_output.drive_passage.lane_path()
                           .front()
                           .lane_id()));
    }
  }

  out->follower_set = std::move(initializer_output.follower_set);
  out->follower_max_decel = initializer_output.follower_max_decel;
  // Update spacetime planner trajectory from leading obj.
  for (const auto& [traj_id, _] : out->leading_trajs) {
    const auto status = AddSpaceTimePlannerTrajectoryById(
        *input.traj_mgr, traj_id,
        SpacetimePlannerObjectTrajectoryReason::LEADING,
        &out->st_planner_object_traj);
    VLOG_IF(3, !status.ok()) << status.ToString();
  }
  out->initializer_state = std::move(initializer_output.initializer_state);

  auto mutable_trajectory_optimizer_params = *input.trajectory_optimizer_params;

  // Modify style settings for trajectory optimizer.
  if (FLAGS_planner_enable_lc_style_params) {
    ModifyTrajOptParamsStyle(*input.trajectory_optimizer_lc_radical_params,
                             *input.trajectory_optimizer_lc_normal_params,
                             *input.trajectory_optimizer_lc_conservative_params,
                             out->scheduler_output.lane_change_state.stage(),
                             input.lane_change_style,
                             &mutable_trajectory_optimizer_params);
  }

  // Setting autotuned trajectory optimizer params.
  // BANDAID(jingqiao): Refactor to solve code divergence in the future.
  if (FLAGS_auto_tuning_mode || FLAGS_compare_different_weight) {
    UpdateTrajOptParams(FLAGS_traj_opt_params_file_address,
                        &mutable_trajectory_optimizer_params);
    // For readability.
  } else if (FLAGS_update_learned_alphas) {
    if (FLAGS_update_learned_alphas_except_lane_change) {
      if (out->scheduler_output.lane_change_state.stage() ==
          LaneChangeStage::LCS_NONE) {
        UpdateTrajOptParams(FLAGS_traj_opt_params_file_address,
                            &mutable_trajectory_optimizer_params);
      }
    } else {
      UpdateTrajOptParams(FLAGS_traj_opt_params_file_address,
                          &mutable_trajectory_optimizer_params);
    }
  }
  VLOG(3) << "Actual cost weights used in trajectory optimizer: ";
  VLOG(3)
      << mutable_trajectory_optimizer_params.cost_weight_params().DebugString();

  // Time alignment.
  const std::vector<ApolloTrajectoryPointProto> previous_trajectory =
      ShiftTrajectoryByTime(
          input.st_path_start_point_info->relative_index_from_plan_start_point *
              kTrajectoryTimeStep,
          *input.time_aligned_prev_traj,
          input.motion_constraint_params->max_accel_jerk(),
          input.motion_constraint_params->max_decel_jerk());
  const std::vector<TrajectoryPoint> captain_trajectory =
      ConvertCaptainTrajectoryToOptimizerInput(
          mutable_trajectory_optimizer_params.trajectory_steps(),
          mutable_trajectory_optimizer_params.trajectory_time_step(),
          input.st_path_start_point_info->relative_index_from_plan_start_point *
              kTrajectoryTimeStep,
          /*captain_traj=*/
          FLAGS_planner_use_ml_trajectory_as_optimizer_ref_traj
              ? input.captain_net_output->traj_points
              : empty_ref_traj);

  // Optional prev optimizer state.
  std::optional<TrajectoryOptimizerState> trajectory_optimizer_state;
  if (input.trajectory_optimizer_state_proto != nullptr) {
    trajectory_optimizer_state.emplace(*input.trajectory_optimizer_state_proto);
  }

  // Run optimizer.
  TrajectoryOptimizerInput opt_input{
      .trajectory = initializer_output.traj_points,
      .previous_trajectory = previous_trajectory,
      .trajectory_optimizer_state = std::move(trajectory_optimizer_state),
      .st_traj_mgr = input.traj_mgr,
      .st_planner_object_traj = &out->st_planner_object_traj,
      .drive_passage = &out->scheduler_output.drive_passage,
      .path_sl_boundary = &out->scheduler_output.sl_boundary,
      .constraint_mgr = &out->constraint_manager,
      .leading_trajs = &out->leading_trajs,
      .planner_semantic_map_mgr = &psmm,
      .plan_start_point = input.st_path_start_point_info->start_point,
      .plan_start_time = input.st_path_start_point_info->plan_time,
      .plan_id = input.plan_id,
      .captain_trajectory = &captain_trajectory,
      .lc_stage = out->scheduler_output.lane_change_state.stage(),
      .trajectory_optimizer_params = &mutable_trajectory_optimizer_params,
      .motion_constraint_params = input.motion_constraint_params,
      .planner_functions_params = input.planner_functions_params,
      .vehicle_models_params = input.vehicle_models_params,
      .veh_geo_params = &vehicle_geom_params,
      .veh_drive_params = &vehicle_drive_params};

  RETURN_PLANNER_STATUS_OR_ASSIGN(
      auto opt_output,
      OptimizeTrajectory(opt_input, &out->optimizer_debug_proto,
                         &out->chart_data, thread_pool),
      PlannerStatusProto::OPTIMIZER_FAILED);

  // Optimizer state.
  out->trajectory_optimizer_state_proto =
      opt_output.trajectory_optimizer_state.ToProto();

  // Optimizer Auto Tuning
  out->candidate_auto_tuning_traj_proto =
      std::move(opt_output.candidate_auto_tuning_traj_proto);
  out->expert_auto_tuning_traj_proto =
      std::move(opt_output.expert_auto_tuning_traj_proto);
  out->nudge_object_info = std::move(opt_output.nudge_object_info);

  if (FLAGS_compare_different_weight) {
    TrajectoryOptimizerDebugProto original_optimizer_debug;
    opt_input.trajectory_optimizer_params = input.trajectory_optimizer_params;
    RETURN_PLANNER_STATUS_OR_ASSIGN(
        auto original_opt_output,
        OptimizeTrajectory(opt_input, &original_optimizer_debug,
                           &out->chart_data, thread_pool),
        PlannerStatusProto::PLANNER_ABNORMAL_EXIT);
    const std::string base_name =
        absl::StrFormat("traj_opt_%d", opt_input.plan_id);
    optimizer::AddCompareTrajCanvas(base_name, original_opt_output.trajectory,
                                    "original_weight", opt_output.trajectory,
                                    "auto_tuned_weight");
    if (FLAGS_compare_based_on_original_weight) {
      opt_output = std::move(original_opt_output);
      out->optimizer_debug_proto = std::move(original_optimizer_debug);
    }
  }
  QCHECK(!opt_output.trajectory_proto.empty());

  if (FLAGS_planner_est_scheduler_seperate_lc_pause &&
      (out->scheduler_output.lane_change_state.stage() ==
           LaneChangeStage::LCS_EXECUTING ||
       out->scheduler_output.lane_change_state.stage() ==
           LaneChangeStage::LCS_RETURN)) {
    const auto target_lane_path_ext = BackwardExtendLanePath(
        psmm,
        out->scheduler_output.drive_passage.extend_lane_path().BeforeArclength(
            kLaneChangeCheckForwardLength),
        kLaneChangeCheckBackwardLength);
    RETURN_PLANNER_STATUS_OR_ASSIGN(
        const auto target_frenet_frame,
        BuildKdTreeFrenetFrame(SampleLanePathPoints(psmm, target_lane_path_ext),
                               /*down_sample_raw_points=*/true),
        PlannerStatusProto::LC_SAFETY_CHECK_FAILED);

    constexpr double kLaneSpeedLimitPreviewTime = 6.0;  // s.
    const auto lane_point =
        out->scheduler_output.drive_passage.lane_path().ArclengthToLanePoint(
            input.st_path_start_point_info->start_point.v() *
            kLaneSpeedLimitPreviewTime);
    const double speed_limit = psmm.QueryLaneSpeedLimitByFraction(
        lane_point.lane_id(), lane_point.fraction());

    const auto resampled_traj =
        ResampleOptimizerTrajectory(opt_output.trajectory_proto);

    absl::flat_hash_set<std::string> follower_set;
    double follower_max_decel = 0.0;
    std::string unsafe_obj_id;
    LaneChangeSafetyDebugProto lane_change_safety_debug_proto;
    auto lc_safety_status = CheckLaneChangeSafety(
        resampled_traj, target_frenet_frame, speed_limit, *input.traj_mgr,
        vehicle_geom_params, input.lane_change_style,
        input.path_look_ahead_duration, &follower_set, &follower_max_decel,
        &unsafe_obj_id, &lane_change_safety_debug_proto);
    out->lane_change_safety_debug_proto =
        std::move(lane_change_safety_debug_proto);
    if (!lc_safety_status.ok()) {
      return PlannerStatus(
          PlannerStatusProto::LC_SAFETY_CHECK_FAILED,
          absl::StrCat(
              "Lane change to ",
              out->scheduler_output.drive_passage.lane_path().front().lane_id(),
              " not safe: ", lc_safety_status.message()));
    } else {
      out->follower_set = std::move(follower_set);
      out->follower_max_decel = follower_max_decel;
      out->unsafe_object_ids = {unsafe_obj_id};
    }
  }

  // Path Extension
  std::vector<ApolloTrajectoryPointProto> st_trajectory =
      StitchPreviousTrajectoryAndStTrajectory(
          input.st_path_start_point_info->relative_index_from_plan_start_point,
          *input.time_aligned_prev_traj, opt_output.trajectory_proto);

  constexpr double kRequiredMinPathLength = 20.0;
  // This parameter is consistent with that in trajectory curvature check.
  constexpr double kCurvatureRelaxFactor = 1.05;
  const double max_curvature = ComputeRelaxedCenterMaxCurvature(
      vehicle_geom_params, vehicle_drive_params);

  RETURN_PLANNER_STATUS_OR_ASSIGN(auto raw_path_points,
                                  ExtendPathAndDeleteUnreasonablePart(
                                      st_trajectory, kRequiredMinPathLength,
                                      kCurvatureRelaxFactor * max_curvature),
                                  PlannerStatusProto::PATH_EXTENSION_FAILED);

  out->path = DiscretizedPath::CreateResampledPath(raw_path_points,
                                                   kPathSampleInterval);
  out->st_path_points = std::move(raw_path_points);

  return OkPlannerStatus();
}

}  // namespace qcraft::planner

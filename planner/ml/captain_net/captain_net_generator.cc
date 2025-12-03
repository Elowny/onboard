#include "onboard/planner/ml/captain_net/captain_net_generator.h"

// IWYU pragma: no_include <ext/alloc_traits.h>
//              for __alloc_traits<>::value_type
#include <algorithm>  // for max, stable_sort
#include <iterator>

#include "absl/status/statusor.h"                // for StatusOr
#include "google/protobuf/repeated_ptr_field.h"  // for RepeatedPtrField

#include "onboard/eval/qevent.h"       // for QEVENT
#include "onboard/eval/qevent_base.h"  // for QEvent
#include "onboard/global/logging.h"    // for BufferedLoggerWrapper, QLOG...
#include "onboard/global/trace.h"      // for SCOPED_QTRACE, ScopedTrace
#include "onboard/maps/lane_path.h"
#include "onboard/planner/common/proto/planner_status.pb.h"  // for PlannerStatusProto
#include "onboard/planner/ml/captain_net/captain_net.h"  // for CaptainNetOutput, LanesFeature
#include "onboard/planner/ml/captain_net/inference/captain_net_inference.h"  // for CaptainNetInference
#include "onboard/planner/ml/captain_net/inference/captain_net_inferencer.h"  // for CaptainNetInferencer
#include "onboard/planner/ml/captain_net/inference/captain_net_j5_inferencer.h"  // for CaptainNetInferencer
#include "onboard/planner/ml/captain_net/inference/input_assemble.h"
#include "onboard/planner/ml/captain_net/post_process/model_output_process.h"
#include "onboard/planner/ml/captain_net/post_process/post_process.h"  // for RunCaptainNetPostProcess
#include "onboard/planner/ml/captain_net/post_process/trajectory_alignment.h"  // for AlignTrajectory
#include "onboard/planner/ml/captain_net/proto/captain_net_debug.pb.h"  // for SingleModeTrajProto, Capnet...
#include "onboard/planner/ml/captain_net/validation/trajectory_validation.h"  // for CheckTrajectoryValidation
#include "onboard/planner/ml/condition_feature_extractor/condition_feature.h"  // for LineSegmentsFeature
#include "onboard/planner/ml/condition_feature_extractor/condition_feature_extractor.h"  // for ExtractLanePathFeature
#include "onboard/planner/planner_flags.h"  // for FLAGS_planner_use_ml_trajec...
#include "onboard/prediction/feature_extractor/act_net_feature.h"  // for ActNetPolylineFeature, ActN...
#include "onboard/proto/trajectory_point.pb.h"  // for ApolloTrajectoryPointProto
#include "onboard/utils/status_macros.h"        // for ASSIGN_OR_RETURN
#include "onboard/utils/time_util.h"            // for ToUnixDoubleSeconds

namespace qcraft::planner::ml {

namespace {

void SendValidationQevent(const captain_net::CaptainNetOutput& output) {
  if (output.validation.is_kinematic_infeasible) {
    QEVENT("planner_ml",
           "captain_net_trajectory_failure_kinematic_infeasibility",
           [&](QEvent* /*qevent*/) {});
  }

  if (output.validation.is_curb_collsion) {
    QEVENT("planner_ml", "captain_net_trajectory_failure_curb_collision",
           [&](QEvent* /*qevent*/) {});
  }

  if (output.validation.is_object_collision) {
    QEVENT("planner_ml", "captain_net_trajectory_failure_object_collision",
           [&](QEvent* /*qevent*/) {});
  }

  if (output.validation.is_path_boundary_collision) {
    QEVENT("planner_ml",
           "captain_net_trajectory_failure_path_boundary_violation",
           [&](QEvent* /*qevent*/) {});
  }

  if (output.validation.is_reverse_driving) {
    QEVENT("planner_ml", "captain_net_trajectory_failure_reverse_driving",
           [&](QEvent* /*qevent*/) {});
  }

  if (output.validation.is_speed_limit_violation) {
    QEVENT("planner_ml", "captain_net_trajectory_failure_speed_limit_violation",
           [&](QEvent* /*qevent*/) {});
  }

  if (FLAGS_planner_use_ml_trajectory_end_to_end
          ? !(output.validation.IsValidAsE2E())
          : !(output.validation.IsValidAsRef())) {
    QEVENT("planner_ml", "captain_net_trajectory_invalid",
           [&](QEvent* /*qevent*/) {});
  }
}

}  // namespace

PlannerStatus GenerateCaptainNetTrajectory(
    const PlannerSemanticMapManager& psmm,
    const std::vector<SchedulerOutput>& multi_tasks,
    const ml::ContextFeature& context_feature,
    const PlanStartPointInfo& start_point_info,
    const VehicleGeometryParamsProto& vehicle_geometry_params,
    const VehicleDriveParamsProto& vehicle_drive_params,
    const std::vector<SpacetimeTrajectoryManager>& st_traj_mgrs,
    const ModelPool* planner_model_pool,
    std::vector<captain_net::CaptainNetOutput>* captain_net_results_ptr,
    std::vector<PlannerStatus>* status_list_ptr,
    std::vector<EstPlannerDebug>* est_debugs, ThreadPool* thread_pool) {
  SCOPED_QTRACE("GenerateCaptainNetTrajectory");
  // Check model inference existence
  if (planner_model_pool == nullptr) {
    return PlannerStatus(PlannerStatusProto::MODEL_INFERENCE_FAILED,
                         "Planner model pool is not instantiated.");
  }

  MultiLanePathFeature target_lanes_feature;
  {
    SCOPED_QTRACE("CaptainNet::ConditionFeatureExtraction");
    std::vector<const mapping::LanePath*> target_lane_paths;
    std::transform(multi_tasks.begin(), multi_tasks.end(),
                   std::back_inserter(target_lane_paths), [](const auto& task) {
                     return &task.drive_passage.extend_lane_path();
                   });
    ASSIGN_OR_RETURN(target_lanes_feature,
                     ml::ExtractMultiLanePathFeature(
                         context_feature, psmm, target_lane_paths,
                         captain_net::kMaxTargetLanesNum),
                     PlannerStatus(PlannerStatusProto::MODEL_INFERENCE_FAILED,
                                   "Fail to extract condition feature."));
  }

  // Assemble input features.
  captain_net::CaptainNetFeature input_features;
  {
    SCOPED_QTRACE("CaptainNet::InputFeatureAssemble");
    input_features =
        AssembleInputFeature(context_feature, target_lanes_feature);
  }

  // Predict trajectories conditioned on lane-path.
  // Prob_out and traj_out in dimenstion of [lanes, modes, points].
  std::vector<std::vector<std::vector<float>>> prob_out;
  std::vector<std::vector<std::vector<float>>> traj_out;
  {
    SCOPED_QTRACE("CaptainNet::ModelInference");
    if (FLAGS_planner_enable_captain_net_j5) {
      if (!planner_model_pool->GetCaptainNetJ5Inferencer().PlanningTrajectory(
              input_features, &prob_out, &traj_out)) {
        return PlannerStatus(PlannerStatusProto::MODEL_INFERENCE_FAILED,
                             "CaptainNet J5 Inference Failure.");
      }
    } else if (FLAGS_planner_enable_captain_net_onnx_trt) {
      if (!planner_model_pool->GetCaptainNetInferencer().PlanningTrajectory(
              input_features, &prob_out, &traj_out)) {
        return PlannerStatus(PlannerStatusProto::MODEL_INFERENCE_FAILED,
                             "CaptainNet Inference Failure.");
      }
    } else {
      if (!planner_model_pool->GetCaptainNetInference().PlanningTrajectory(
              input_features, &prob_out, &traj_out)) {
        return PlannerStatus(PlannerStatusProto::MODEL_INFERENCE_FAILED,
                             "CaptainNet Inference Failure.");
      }
    }
  }

  std::vector<std::vector<captain_net::CaptainNetOutput>> multimode_results;
  {
    SCOPED_QTRACE("CaptainNet::TransferToCaptainNetOutput");
    const auto& speed_vec = context_feature.act_net_feature.agent_feature.speed;
    // HACK(Jinqiao): Only use speed_x at cur_time since speed is
    // rotated to east on cur_time thus speed_y is zero.
    const auto current_speed = speed_vec[speed_vec.size() - 2];

    multimode_results = CaptainNetOutputProcess(
        multi_tasks.size(), traj_out, prob_out, current_speed,
        FLAGS_planner_captain_net_use_dkm);
  }

  // Align trajectory with plan_start_point.
  {
    SCOPED_QTRACE("CaptainNet::AlignTrajectory");
    for (int i = 0; i < multi_tasks.size(); ++i) {
      for (int j = 0; j < captain_net::kModes; ++j) {
        AlignTrajectory(start_point_info.start_point,
                        context_feature.current_ts,
                        ToUnixDoubleSeconds(start_point_info.plan_time),
                        &(multimode_results[i][j]));
      }
    }
  }

  // Run post process if movability failure
  {
    SCOPED_QTRACE("CaptainNet::PostProcess");
    for (int i = 0; i < multi_tasks.size(); ++i) {
      for (int j = 0; j < captain_net::kModes; ++j) {
        multimode_results[i][j].original_traj_points =
            multimode_results[i][j].traj_points;
        RunCaptainNetPostProcess(vehicle_geometry_params, vehicle_drive_params,
                                 &(multimode_results[i][j]));
      }
    }
  }

  // Check trajectory validation and send metric qevent.
  auto& status_list = *status_list_ptr;
  {
    SCOPED_QTRACE("CaptainNet::ValidationCheck");
    for (int i = 0; i < multi_tasks.size(); ++i) {
      auto fake_considered_st_objects =
          ml::FakeConsideredStObjects(st_traj_mgrs[i]);
      for (int j = 0; j < captain_net::kModes; ++j) {
        ml::CheckTrajectoryValidation(psmm, fake_considered_st_objects,
                                      start_point_info.full_stop,
                                      multi_tasks[i], vehicle_geometry_params,
                                      &(multimode_results[i][j]), thread_pool);
      }
    }
  }

  // Pick most probable and safe trajectory from multimode results.
  {
    for (int i = 0; i < multi_tasks.size(); ++i) {
      // Sort the different trajectories by mode prob in descending and valid
      // one in the front.
      std::stable_sort(
          multimode_results[i].begin(), multimode_results[i].end(),
          [](const auto& a, const auto& b) {
            // return a.mode_prob > b.mode_prob;
            bool a_valid = FLAGS_planner_use_ml_trajectory_end_to_end
                               ? a.validation.IsValidAsE2E()
                               : a.validation.IsValidAsRef();
            bool b_valid = FLAGS_planner_use_ml_trajectory_end_to_end
                               ? b.validation.IsValidAsE2E()
                               : b.validation.IsValidAsRef();
            if (a_valid != b_valid) {
              return a_valid > b_valid;
            } else {
              return a.mode_prob > b.mode_prob;
            }
          });

      // Pick the most probable and valid one if possible and set results.
      captain_net_results_ptr->at(i) = multimode_results[i][0];

      // Set valudation qevent for the picked mode.
      SendValidationQevent(captain_net_results_ptr->at(i));

      // Set status proro;
      bool valid =
          FLAGS_planner_use_ml_trajectory_end_to_end
              ? captain_net_results_ptr->at(i).validation.IsValidAsE2E()
              : captain_net_results_ptr->at(i).validation.IsValidAsRef();
      if (!valid) {
        status_list[i] =
            PlannerStatus(PlannerStatusProto::TRAJECTORY_VALIDATION_FAILED,
                          "CheckCaptainTrajectoryValidation failed.");
      } else {
        status_list[i] = OkPlannerStatus();
      }

      // Set debug proto;
      for (int j = 0; j < captain_net::kModes; ++j) {
        auto* traj_debug =
            (*est_debugs)[i].capnet_traj_debug.add_multimode_trajs();
        *traj_debug->mutable_capnet_traj_before_postprocess() = {
            multimode_results[i][j].original_traj_points.begin(),
            multimode_results[i][j].original_traj_points.end()};
        *traj_debug->mutable_capnet_traj_after_postprocess() = {
            multimode_results[i][j].traj_points.begin(),
            multimode_results[i][j].traj_points.end()};
        traj_debug->set_traj_post_processed(
            multimode_results[i][j].is_post_processed);
        traj_debug->set_is_kinematic_infeasible(
            multimode_results[i][j].validation.is_kinematic_infeasible);
        traj_debug->set_is_curb_collsion(
            multimode_results[i][j].validation.is_curb_collsion);
        traj_debug->set_is_object_collision(
            multimode_results[i][j].validation.is_object_collision);
        traj_debug->set_is_path_boundary_collision(
            multimode_results[i][j].validation.is_path_boundary_collision);
        traj_debug->set_is_reverse_driving(
            multimode_results[i][j].validation.is_reverse_driving);
        traj_debug->set_is_speed_limit_violation(
            multimode_results[i][j].validation.is_speed_limit_violation);
        traj_debug->set_traj_valid(
            FLAGS_planner_use_ml_trajectory_end_to_end
                ? multimode_results[i][j].validation.IsValidAsE2E()
                : multimode_results[i][j].validation.IsValidAsRef());
        traj_debug->set_mode_prob(multimode_results[i][j].mode_prob);
      }
    }
  }

  return OkPlannerStatus();
}
}  // namespace qcraft::planner::ml

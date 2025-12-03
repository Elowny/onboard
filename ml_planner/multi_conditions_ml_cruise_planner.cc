#include "onboard/ml_planner/multi_conditions_ml_cruise_planner.h"

#include <algorithm>
#include <memory>
#include <utility>
#include <vector>

#include "absl/status/statusor.h"
#include "google/protobuf/repeated_ptr_field.h"

#include "onboard/global/trace.h"
#include "onboard/maps/lane_path.h"
#include "onboard/maps/proto/lane_path.pb.h"
#include "onboard/math/geometry/proto/affine_transformation.pb.h"
#include "onboard/ml_planner/ml_planner_flags.h"
#include "onboard/ml_planner/ml_planner_input.h"
#include "onboard/ml_planner/proto/captain_net_output.pb.h"
#include "onboard/ml_planner/proto/ml_planner_status.pb.h"
#include "onboard/planner/ml/captain_net/captain_net.h"
#include "onboard/planner/ml/captain_net/inference/captain_net_inference.h"
#include "onboard/planner/ml/captain_net/inference/captain_net_inferencer.h"
#include "onboard/planner/ml/captain_net/inference/captain_net_j5_inferencer.h"
#include "onboard/planner/ml/captain_net/inference/input_assemble.h"
#include "onboard/planner/ml/captain_net/post_process/model_output_process.h"
#include "onboard/planner/ml/captain_net/post_process/trajectory_alignment.h"
#include "onboard/planner/ml/condition_feature_extractor/condition_feature.h"
#include "onboard/planner/ml/condition_feature_extractor/condition_feature_extractor.h"
#include "onboard/planner/ml/context_feature_extractor/context_feature.h"
#include "onboard/planner/ml/model_pool.h"
#include "onboard/prediction/feature_extractor/act_net_feature.h"
#include "onboard/proto/positioning.pb.h"
#include "onboard/proto/trajectory_point.pb.h"
#include "onboard/utils/status_macros.h"

namespace qcraft::mlplanner {
namespace {

CaptainNetTrajectoryProto CaptainNetTrajToProto(
    const planner::ml::captain_net::CaptainNetOutput& captain_net_output) {
  CaptainNetTrajectoryProto captain_net_traj;
  *captain_net_traj.mutable_original_trajectory() = {
      captain_net_output.traj_points.begin(),
      captain_net_output.traj_points.end()};
  *captain_net_traj.mutable_sdxs() = {captain_net_output.sdxs.begin(),
                                      captain_net_output.sdxs.end()};
  *captain_net_traj.mutable_sdys() = {captain_net_output.sdys.begin(),
                                      captain_net_output.sdys.end()};
  *captain_net_traj.mutable_covariances() = {
      captain_net_output.covariances.begin(),
      captain_net_output.covariances.end()};
  captain_net_traj.set_mode_prob(captain_net_output.mode_prob);
  return captain_net_traj;
}

CaptainNetOutputProto CaptainNetOutputToProto(
    const std::vector<std::vector<planner::ml::captain_net::CaptainNetOutput>>&
        multimode_results) {
  CaptainNetOutputProto captain_net_output;
  for (const auto& multimode_result : multimode_results) {
    MultimodeCaptainNetTrajectoryProto multimode_trajectory;
    for (const auto& captain_net_traj : multimode_result) {
      multimode_trajectory.mutable_multimode_trajectory()->Add(
          CaptainNetTrajToProto(captain_net_traj));
    }
    captain_net_output.mutable_multi_condition_trajectory()->Add(
        std::move(multimode_trajectory));
  }
  return captain_net_output;
}

void CaptainNetOutputProtoToDebug(
    const CaptainNetOutputProto& captain_net_output,
    CaptainNetDebugProto* captain_net_debug) {
  for (const auto& multimode_trajectory :
       captain_net_output.multi_condition_trajectory()) {
    MultimodeCaptainNetTrajectoryDebugProto multimode_trajectory_debug;
    for (const auto& captain_net_traj :
         multimode_trajectory.multimode_trajectory()) {
      CaptainNetTrajectoryDebugProto single_traj_debug;
      *single_traj_debug.mutable_trajectory() = captain_net_traj;
      multimode_trajectory_debug.mutable_multimode_trajectory_debug()->Add(
          std::move(single_traj_debug));
    }
    captain_net_debug->mutable_multi_condition_trajectory_debug()->Add(
        std::move(multimode_trajectory_debug));
  }
}

}  // namespace
MLPlannerStatus RunMultiConditionsMLCruisePlanner(
    const MultiConditionsMLCruisePlannerInput& input,
    MlPlannerOutputProto* output, MlPlannerDebugProto* ml_planner_debug) {
  SCOPED_QTRACE("MlPlannerModule::MultiConditionsMLCruisePlanner");

  const auto& context_feature = *input.context_feature;
  auto* model_pool = input.ml_planner_input.model_pool.get();

  // Check model inference existence
  if (model_pool == nullptr) {
    return MLPlannerStatus(MLPlannerStatusProto::MODEL_INFERENCE_FAILED,
                           "ML planner model pool is not instantiated.");
  }

  planner::ml::MultiLanePathFeature target_lanes_feature;
  {
    SCOPED_QTRACE("CaptainNet::ConditionFeatureExtraction");
    std::vector<const mapping::LanePath*> target_lane_paths;
    for (const mapping::LanePath& lane_path : *input.lane_paths) {
      target_lane_paths.push_back(&lane_path);
    }
    ASSIGN_OR_RETURN(
        target_lanes_feature,
        planner::ml::ExtractMultiLanePathFeature(
            context_feature,
            *input.ml_planner_input.planner_semantic_map_manager,
            target_lane_paths, planner::ml::captain_net::kMaxTargetLanesNum),
        MLPlannerStatus(MLPlannerStatusProto::MODEL_INFERENCE_FAILED,
                        "Fail to extract condition feature."));
  }

  // Assemble input features.
  planner::ml::captain_net::CaptainNetFeature input_features;
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
    if (FLAGS_ml_planner_enable_captain_net_j5) {
      if (!model_pool->GetCaptainNetJ5Inferencer().PlanningTrajectory(
              input_features, &prob_out, &traj_out)) {
        return MLPlannerStatus(MLPlannerStatusProto::MODEL_INFERENCE_FAILED,
                               "CaptainNet J5 Inference Failure.");
      }
    } else if (FLAGS_ml_planner_enable_captain_net_onnx_trt) {
      if (!model_pool->GetCaptainNetInferencer().PlanningTrajectory(
              input_features, &prob_out, &traj_out)) {
        return MLPlannerStatus(MLPlannerStatusProto::MODEL_INFERENCE_FAILED,
                               "CaptainNet Inference Failure.");
      }
    } else {
      if (!model_pool->GetCaptainNetInference().PlanningTrajectory(
              input_features, &prob_out, &traj_out)) {
        return MLPlannerStatus(MLPlannerStatusProto::MODEL_INFERENCE_FAILED,
                               "CaptainNet Inference Failure.");
      }
    }
  }

  std::vector<std::vector<planner::ml::captain_net::CaptainNetOutput>>
      multimode_results;
  {
    SCOPED_QTRACE("CaptainNet::TransferToCaptainNetOutput");
    const auto& speed_vec = context_feature.act_net_feature.agent_feature.speed;
    // HACK(Jinqiao): Only use speed_x at cur_time since speed is
    // rotated to east on cur_time thus speed_y is zero.
    const auto current_speed = speed_vec[speed_vec.size() - 2];

    int task_size = std::min(static_cast<int>(input.lane_paths->size()),
                             planner::ml::captain_net::kMaxTargetLanesNum);

    multimode_results = planner::ml::CaptainNetOutputProcess(
        task_size, traj_out, prob_out, current_speed,
        FLAGS_ml_planner_captain_net_use_dkm);
  }

  auto captain_net_output = CaptainNetOutputToProto(multimode_results);
  CaptainNetOutputProtoToDebug(captain_net_output,
                               ml_planner_debug->mutable_captain_net_debug());

  for (int i = 0; i < ml_planner_debug->captain_net_debug()
                          .multi_condition_trajectory_debug()
                          .size();
       ++i) {
    mapping::LanePathProto lane_path_proto;
    ((*input.lane_paths)[i]).ToProto(&lane_path_proto);
    *(ml_planner_debug->mutable_captain_net_debug()
          ->mutable_multi_condition_trajectory_debug(i)
          ->mutable_lane_path()) = std::move(lane_path_proto);
  }

  // Align trajectory with pose. Only for vis purpose.
  {
    SCOPED_QTRACE("CaptainNet::AlignTrajectory");

    auto* pose_proto = input.ml_planner_input.pose.get();

    ApolloTrajectoryPointProto fake_plan_start_point;
    fake_plan_start_point.mutable_path_point()->set_x(
        pose_proto->pos_smooth().x());
    fake_plan_start_point.mutable_path_point()->set_y(
        pose_proto->pos_smooth().y());
    fake_plan_start_point.mutable_path_point()->set_theta(pose_proto->yaw());

    for (int i = 0; i < multimode_results.size(); ++i) {
      for (int j = 0; j < multimode_results[0].size(); ++j) {
        planner::ml::AlignTrajectory(fake_plan_start_point,
                                     context_feature.current_ts,
                                     input.ml_planner_input.pose->timestamp(),
                                     &(multimode_results[i][j]));
        *(ml_planner_debug->mutable_captain_net_debug()
              ->mutable_multi_condition_trajectory_debug(i)
              ->mutable_multimode_trajectory_debug(j)
              ->mutable_ml_planner_module_vis_trajectory()) = {
            multimode_results[i][j].traj_points.begin(),
            multimode_results[i][j].traj_points.end()};
      }
    }
  }

  *(output->mutable_captain_net_output()) = std::move(captain_net_output);
  return OkMLPlannerStatus();
}

}  // namespace qcraft::mlplanner

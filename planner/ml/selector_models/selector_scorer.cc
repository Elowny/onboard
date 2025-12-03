#include "onboard/planner/ml/selector_models/selector_scorer.h"

#include <algorithm>  // for max
#include <cstdint>    // for int64_t

#include "absl/container/flat_hash_map.h"  // for BitMask, operator!=, flat_hash_map, operator==
#include "absl/status/status.h"  // for FailedPreconditionError, Status

#include "onboard/container/strong_int.h"  // for operator<<
#include "onboard/global/trace.h"          // for SCOPED_QTRACE, ScopedTrace
#include "onboard/maps/lane_path.h"        // for LanePath
#include "onboard/maps/lane_point.h"       // for LanePoint
#include "onboard/planner/ml/common/feature_extraction_utils.h"  // for ExtractRefPathFeature, ExtractTrajectoryFeature
#include "onboard/planner/ml/model_pool.h"  // for ModelPool
#include "onboard/planner/ml/selector_models/selector_scoring_net.h"  // for ActorsFeature, TargetLanesFeature, SelectorScoringN...
#include "onboard/planner/ml/selector_models/selector_scoring_net_inference.h"  // for SelectorScoringNetInference
#include "onboard/planner/router/drive_passage.h"        // for DrivePassage
#include "onboard/planner/scheduler/scheduler_output.h"  // for SchedulerOutput
#include "onboard/planner/selector/candidate_stats.h"  // for RouteLookAhseadStats
#include "onboard/planner/selector/proto/selector_debug.pb.h"  // for SelectorMLData, SelectorDebugProto
#include "onboard/prediction/proto/act_net.pb.h"
#include "onboard/prediction/proto/prophnet.pb.h"
#include "onboard/proto/prediction.pb.h"  // for RepeatedField, ActNetDumpedFeatureProto_MapDumpedFe...
#include "onboard/utils/map_util.h"  // for FindOrDie, FindWithDefault
#include "onboard/utils/status_macros.h"  // for StatusAdaptorForMacros, RETURN_IF_ERROR

namespace qcraft::planner {

absl::StatusOr<bool> CheckSelectorScoringPrecondition(
    const SelectorInput& input, const std::vector<PlannerStatus>& est_status,
    const std::vector<EstPlannerOutput>& planner_outputs) {
  // Check input precondition
  const auto& prediction_debug = *input.prediction_debug;
  if (!prediction_debug.has_features() ||
      prediction_debug.features().act_net_data_size() == 0) {
    return absl::FailedPreconditionError(
        "No valid feature in CheckSelectorScoringPrecondition.");
  }
  // Check model inference existence
  if (input.planner_model_pool == nullptr) {
    return absl::FailedPreconditionError(
        "Planner model pool is not instantiated.");
  }
  if (est_status.size() != planner_outputs.size()) {
    return absl::FailedPreconditionError(
        "est_status.size() != planner_outputs.size() in "
        "CheckSelectorScoringPrecondition.");
  }
  if (est_status.size() == 0) {
    return absl::FailedPreconditionError(
        "No valid est_status & planner_outputs in "
        "CheckSelectorScoringPrecondition.");
  }
  return true;
}

absl::StatusOr<std::map<int, float>> ScoringSelectorTrajectory(
    const SelectorInput& input, const SelectorCommonFeature& common_feature,
    const std::vector<PlannerStatus>& est_status,
    const std::vector<SpacetimeTrajectoryManager>& st_traj_mgr_list,
    const std::vector<EstPlannerOutput>& planner_outputs) {
  SelectorDebugProto selector_debug;
  auto* selector_ml_data = selector_debug.mutable_ml_data();
  // Check input precondition
  const auto& prediction_debug = *input.prediction_debug;
  if (!prediction_debug.has_features() ||
      prediction_debug.features().act_net_data_size() == 0) {
    return absl::FailedPreconditionError(
        "No valid feature in CheckSelectorScoringPrecondition.");
  }

  // Check model inference existence
  if (input.planner_model_pool == nullptr) {
    return absl::FailedPreconditionError(
        "Planner model pool is not instantiated");
  }

  std::vector<int> idx_vec;
  selector_scoring_net::SelectorScoringNetFeature input_features;
  {
    SCOPED_QTRACE("InputPreprocessing");
    for (int idx = 0, size = planner_outputs.size(); idx < size; ++idx) {
      if (!est_status[idx].ok()) continue;
      if (planner_outputs[idx].scheduler_output.is_expert) continue;
      if (planner_outputs[idx].scheduler_output.is_fallback) continue;
      idx_vec.push_back(idx);
    }

    if (idx_vec.size() == 0) {
      return std::map<int, float>();
    }
    if (idx_vec.size() == 1) {
      std::map<int, float> scores_map;
      scores_map[idx_vec[0]] = 1.0;
      return scores_map;
    }

    // Generate candidate trajectory feature
    for (int idx : idx_vec) {
      RETURN_IF_ERROR(ExtractTrajectoryFeature(
          prediction_debug, planner_outputs[idx].traj_points,
          selector_ml_data->add_candidate_trajs_model_features()));
      RETURN_IF_ERROR(ExtractRefPathFeature(
          prediction_debug, planner_outputs[idx].scheduler_output.sl_boundary,
          selector_ml_data
              ->add_candidate_smooth_ref_center_line_model_features()));
    }

    // Assemble inputs
    const auto& act_net_data = prediction_debug.features().act_net_data(0);
    const auto& objects_feature = act_net_data.objects_feature();
    const auto& lanes_feature = act_net_data.lane_centers_feature();
    const auto& bounds_feature = act_net_data.lane_boundaries_feature();
    const auto& cw_feature = act_net_data.crosswalks_feature();
    const auto& objects_mask = act_net_data.objects_mask();
    const auto& lanes_mask = act_net_data.lcs_mask();
    const auto& bounds_mask = act_net_data.lbs_mask();
    const auto& cw_mask = act_net_data.cws_mask();

    input_features = selector_scoring_net::SelectorScoringNetFeature{
        .actors_feature =
            selector_scoring_net::ActorsFeature{
                .trajs = std::vector<float>(objects_feature.pos_diff().begin(),
                                            objects_feature.pos_diff().end()),
                .speeds = std::vector<float>(objects_feature.speed().begin(),
                                             objects_feature.speed().end()),
                .headings = std::vector<float>(objects_feature.yaw().begin(),
                                               objects_feature.yaw().end()),
                .types = std::vector<float>(objects_feature.type().begin(),
                                            objects_feature.type().end()),
                .cur_poses = std::vector<float>(objects_feature.pos().begin(),
                                                objects_feature.pos().end()),
                .mask =
                    std::vector<int>(objects_mask.begin(), objects_mask.end()),
            },
        .lanes_feature =
            selector_scoring_net::LanesFeature{
                .lane_centers = std::vector<float>(lanes_feature.seg().begin(),
                                                   lanes_feature.seg().end()),
                .lane_lights = std::vector<float>(lanes_feature.light().begin(),
                                                  lanes_feature.light().end()),
                .lane_types = std::vector<int64_t>(lanes_feature.type().begin(),
                                                   lanes_feature.type().end()),
                .mask = std::vector<int>(lanes_mask.begin(), lanes_mask.end()),
            },
        .bounds_feature =
            selector_scoring_net::LaneBoundaryFeature{
                .boundaries = std::vector<float>(bounds_feature.seg().begin(),
                                                 bounds_feature.seg().end()),
                .boundary_types =
                    std::vector<int64_t>(bounds_feature.light().begin(),
                                         bounds_feature.light().end()),
                .mask = std::vector<int>(bounds_mask.begin(), bounds_mask
                                                                  .end()),
            },
        .cws_feature =
            selector_scoring_net::CrossWalkFeature{
                .encirclingline = std::vector<float>(cw_feature.seg().begin(),
                                                     cw_feature.seg().end()),
                .mask = std::vector<int>(cw_mask.begin(), cw_mask.end()),
            },
        .c_trajs_feature = std::vector<selector_scoring_net::ActorsFeature>(),
        .tl_feature = std::vector<selector_scoring_net::TargetLanesFeature>(),
    };

    // TODO(Jinyun): Pass rather than reconstruct route stat here.
    RouteLookAheadStats route_stats(
        common_feature, *input.psmm, *input.sections_info,
        *input.plan_start_point, *input.stalled_objects, est_status,
        st_traj_mgr_list, planner_outputs);
    for (int i = 0, size = idx_vec.size(); i < size; ++i) {
      const auto& c_traj_feature =
          selector_ml_data->candidate_trajs_model_features(i);
      input_features.c_trajs_feature.push_back({
          .trajs = std::vector<float>(c_traj_feature.trajs().begin(),
                                      c_traj_feature.trajs().end()),
          .speeds = std::vector<float>(c_traj_feature.speeds().begin(),
                                       c_traj_feature.speeds().end()),
          .headings = std::vector<float>(c_traj_feature.headings().begin(),
                                         c_traj_feature.headings().end()),
          .types = std::vector<float>(c_traj_feature.types().begin(),
                                      c_traj_feature.types().end()),
          .cur_poses = std::vector<float>(c_traj_feature.cur_poses().begin(),
                                          c_traj_feature.cur_poses().end()),
      });
      const auto& tl_feature =
          selector_ml_data->candidate_smooth_ref_center_line_model_features(i);
      const auto start_id = planner_outputs[idx_vec[i]]
                                .scheduler_output.drive_passage.lane_path()
                                .front()
                                .lane_id();
      input_features.tl_feature.push_back({
          .lane_centers = std::vector<float>(tl_feature.lane_centers().begin(),
                                             tl_feature.lane_centers().end()),
          .stats = {static_cast<float>(
                        FindOrDie(route_stats.lc_num_to_targets_map, start_id)),
                    static_cast<float>(
                        FindOrDie(route_stats.driving_dist_map, start_id)),
                    static_cast<float>(FindOrDieNoPrint(
                        route_stats.len_along_route_map,
                        planner_outputs[idx_vec[i]].scheduler_output.Hash())),
                    static_cast<float>(FindOrDie(
                        route_stats.is_right_most_lane_map, start_id)),
                    static_cast<float>(
                        FindWithDefault(route_stats.len_before_intersection_map,
                                        start_id, 0.0))},
      });
    }
  }

  std::map<int, float> scores_map;
  {
    SCOPED_QTRACE("ModelInference");
    auto scores_or = input.planner_model_pool->GetSelectorScoringNetInference()
                         .EvaluateScores(input_features);

    if (!scores_or.ok()) {
      return scores_or.status();
    }

    for (int i = 0, size = scores_or->size(); i < size; ++i) {
      scores_map[idx_vec[i]] = scores_or->at(i);
    }
  }
  return scores_map;
}

}  // namespace qcraft::planner

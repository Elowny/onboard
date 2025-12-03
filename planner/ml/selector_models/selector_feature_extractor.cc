#include "onboard/planner/ml/selector_models/selector_feature_extractor.h"

// IWYU pragma: no_include <ext/alloc_traits.h>
//              for __alloc_traits<>::value_type

#include <iterator>  // for begin, end
#include <memory>    // for allocator_traits<>::v...
#include <string>    // for string
#include <utility>   // for move

#include "absl/container/flat_hash_map.h"        // for operator!=, BitMask
#include "absl/status/statusor.h"                // for StatusOr
#include "google/protobuf/repeated_ptr_field.h"  // for RepeatedPtrField

#include "onboard/container/strong_int.h"  // for operator<<, StrongInt
#include "onboard/maps/lane_path.h"        // for LanePath
#include "onboard/maps/lane_point.h"       // for LanePoint
#include "onboard/planner/ml/condition_feature_extractor/condition_feature.h"  // for LineSegmentsFeature
#include "onboard/planner/ml/condition_feature_extractor/condition_feature_extractor.h"  // for ExtractLanePathFeature
#include "onboard/planner/ml/condition_feature_extractor/condition_feature_proto_converter.h"  // for LineSegmentsFeatureTo...
#include "onboard/planner/ml/condition_feature_extractor/proto/condition_feature.pb.h"  // for LineSegmentsFeatureProto
#include "onboard/planner/router/drive_passage.h"        // for DrivePassage
#include "onboard/planner/scheduler/scheduler_output.h"  // for SchedulerOutput
#include "onboard/planner/selector/candidate_stats.h"  // for RouteLookAheadStats
#include "onboard/planner/selector/cost_feature_base.h"  // for CostFeatureBase
#include "onboard/proto/trajectory_point.pb.h"  // for ApolloTrajectoryPoint...
#include "onboard/utils/map_util.h"             // for FindOrDie, FindWithDe...
#include "onboard/utils/status_macros.h"        // for ASSIGN_OR_RETURN
#include "onboard/utils/time_util.h"            // for ToUnixDoubleSeconds

namespace qcraft::planner {
absl::Status DumpSelectorEvaluations(
    const SelectorInput& input, const SelectorCommonFeature& common_feature,
    const CostFeatures& cost_features,
    const std::vector<PlannerStatus>& est_status,
    const std::vector<SpacetimeTrajectoryManager>& st_traj_mgr_list,
    const std::vector<EstPlannerOutput>& planner_outputs,
    SelectorDebugProto* selector_debug) {
  RouteLookAheadStats route_stats(common_feature, *input.psmm,
                                  *input.sections_info, *input.plan_start_point,
                                  *input.stalled_objects, est_status,
                                  st_traj_mgr_list, planner_outputs);
  for (int idx = 0, size = planner_outputs.size(); idx < size; ++idx) {
    const auto& planner_output = planner_outputs[idx];
    bool is_expert = planner_output.scheduler_output.is_expert;
    if (!est_status[idx].ok() && is_expert) {
      return absl::FailedPreconditionError("Expert data not available!");
    }
    if (!est_status[idx].ok()) continue;
    auto* selector_ml_data = selector_debug->mutable_ml_data();
    auto* traj_feature = is_expert
                             ? selector_ml_data->mutable_expert_traj_features()
                             : selector_ml_data->add_candidate_trajs_features();
    const auto start_id =
        planner_output.scheduler_output.drive_passage.lane_path()
            .front()
            .lane_id();
    traj_feature->set_start_lane_id(start_id.value());

    for (const auto& feature : cost_features) {
      std::vector<std::string> extra_info;
      TrajFeatureOutput traj_feature_output;
      ASSIGN_OR_RETURN(const auto feature_vec,
                       feature->ComputeCost(planner_outputs[idx], &extra_info,
                                            &traj_feature_output));
      auto& feat_debug = *traj_feature->add_features();
      feat_debug.set_name(feature->name());
      feat_debug.mutable_sub_names()->Add(feature->sub_names().begin(),
                                          feature->sub_names().end());
      feat_debug.mutable_values()->Add(std::begin(feature_vec),
                                       std::end(feature_vec));
      feat_debug.set_is_common(feature->is_common());
      for (auto& info : extra_info) {
        *feat_debug.add_extra_info() = std::move(info);
      }
    }
    auto* traj = is_expert ? selector_ml_data->mutable_expert_traj()
                           : selector_ml_data->add_candidate_trajs();
    *traj->mutable_trajectory_points() = {
        planner_outputs[idx].traj_points.begin(),
        planner_outputs[idx].traj_points.end()};

    {
      auto start_ts = ToUnixDoubleSeconds(input.plan_time);
      auto* traj_model_input =
          is_expert ? selector_ml_data->mutable_expert_traj_model_input()
                    : selector_ml_data->add_candidate_trajs_model_inputs();
      ASSIGN_OR_RETURN(auto traj_feature,
                       ml::ExtractTrajectoryFeature(
                           *input.context_feature,
                           planner_outputs[idx].traj_points, start_ts));
      *traj_model_input = ml::TrajectoryFeatureToProto(traj_feature);

      auto* lane_model_input =
          is_expert ? selector_ml_data->mutable_expert_lane_model_input()
                    : selector_ml_data->add_candidate_lanes_model_inputs();
      ASSIGN_OR_RETURN(
          auto lane_model_feature,
          ml::ExtractLanePathFeature(*input.context_feature, *input.psmm,
                                     planner_output.scheduler_output
                                         .drive_passage.extend_lane_path()));
      *lane_model_input = ml::LineSegmentsFeatureToProto(lane_model_feature);

      auto* path_boundary_input =
          is_expert
              ? selector_ml_data->mutable_expert_path_boundary_model_input()
              : selector_ml_data->add_candidate_path_boundaries_model_inputs();
      ASSIGN_OR_RETURN(auto path_boundary_feature,
                       ExtractPathBoundaryFeature(
                           *input.context_feature,
                           planner_output.scheduler_output.sl_boundary));
      *path_boundary_input =
          ml::PathBoundaryFeatureToProto(path_boundary_feature);

      auto* lane_stats = selector_ml_data->add_candidate_lane_stats();
      lane_stats->set_lc_num_to_targets(
          FindOrDie(route_stats.lc_num_to_targets_map, start_id));
      lane_stats->set_driving_dist(
          FindOrDie(route_stats.driving_dist_map, start_id));
      lane_stats->set_length_along_route(
          FindOrDieNoPrint(route_stats.len_along_route_map,
                           planner_output.scheduler_output.Hash()));
      lane_stats->set_is_right_most_lane(
          FindOrDie(route_stats.is_right_most_lane_map, start_id));
      lane_stats->set_len_before_intersection(FindWithDefault(
          route_stats.len_before_intersection_map, start_id, 0.0));
    }
  }

  return absl::OkStatus();
}

}  // namespace qcraft::planner

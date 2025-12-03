#include "onboard/planner/ml/initializer_models/initializer_feature_extractor.h"

#include <algorithm>
#include <memory>
#include <ostream>
#include <string>
#include <utility>

#include "absl/algorithm/container.h"
#include "absl/strings/str_format.h"
#include "absl/types/span.h"

#include "onboard/lite/logging.h"
#include "onboard/math/util.h"
#include "onboard/planner/initializer/cost_provider.h"
#include "onboard/planner/initializer/motion_form.h"
#include "onboard/planner/initializer/motion_search_util.h"
#include "onboard/planner/ml/initializer_models/complete_motion_form.h"
#include "onboard/planner/planner_defs.h"
#include "onboard/planner/trajectory_util.h"
#include "onboard/proto/trajectory.pb.h"
#include "onboard/proto/trajectory_point.pb.h"
#include "onboard/utils/time_util.h"

namespace qcraft::planner {
void SampledDpMotionEvaluation(
    int traj_steps,
    const MotionEdgeVector<MotionSearchOutput::SearchCost>& search_costs,
    const std::vector<MotionEdgeIndex>& terminated_edge_idxes,
    MotionSearchOutput* const output) {
  for (int i = 0, n = terminated_edge_idxes.size(); i < n; ++i) {
    const auto cur_idx = terminated_edge_idxes[i];

    std::vector<double> feature_costs = search_costs[cur_idx].feature_cost;
    auto cost_provider_weights = output->cost_provider->weights();
    const int cost_provider_weights_size = cost_provider_weights.size();
    for (int i = 0; i < cost_provider_weights_size; ++i) {
      feature_costs[i] = feature_costs[i] / cost_provider_weights[i];
    }

    double weighted_total_cost = search_costs[cur_idx].cost_to_come;

    std::vector<double> weights;
    weights.reserve(cost_provider_weights_size);
    for (int i = 0; i < cost_provider_weights_size; ++i) {
      weights.push_back(cost_provider_weights[i]);
    }

    auto candidate_traj =
        ConstructTrajFromLastEdge(traj_steps, *output->motion_graph, cur_idx);

    auto candidate_evaluation = MotionSearchOutput::TrajectoryEvaluationDumping{
        .weighted_total_cost = weighted_total_cost,
        .dumped_weights = std::move(weights),
        .feature_costs = std::move(feature_costs),
        .traj = std::move(candidate_traj)};

    output->candidates_evaluation.push_back(std::move(candidate_evaluation));
  }
}

absl::Status ExpertDpMotionEvaluation(int traj_steps, absl::Time plan_time,
                                      const GeometryFormBuilder& form_builder,
                                      const TrajectoryProto& log_av_trajectory,
                                      MotionSearchOutput* const output) {
  // Load expert future trajectory.
  if (log_av_trajectory.trajectory_point().empty()) {
    QLOG(ERROR) << "log_av_trajectory is empty.";
    return absl::FailedPreconditionError("log_av_trajectory is empty.");
  }
  // Interpolate the trajectory from plan_time with kDeltaT
  const double planner_start_timestamp_sec = ToUnixDoubleSeconds(plan_time);
  const double log_av_trajectory_start_timestamp_sec =
      log_av_trajectory.trajectory_start_timestamp() +
      log_av_trajectory.trajectory_point(0).relative_time();
  const double log_av_trajectory_end_timestamp_sec =
      log_av_trajectory.trajectory_start_timestamp() +
      log_av_trajectory
          .trajectory_point(log_av_trajectory.trajectory_point_size() - 1)
          .relative_time();
  constexpr double kDeltaT = 0.2;  // s
  const int horizon = CeilToInt(kTrajectoryTimeHorizon / kDeltaT) + 1;
  if (log_av_trajectory.trajectory_start_timestamp() >
          planner_start_timestamp_sec ||
      log_av_trajectory.trajectory_start_timestamp() +
              log_av_trajectory
                  .trajectory_point(log_av_trajectory.trajectory_point_size() -
                                    1)
                  .relative_time() <
          planner_start_timestamp_sec + (horizon - 1) * kDeltaT) {
    QLOG(ERROR) << absl::StrFormat(
        "log_av_trajectory time range is limited: "
        "log_av_trajectory start time [%f], "
        "planner_start_timestamp_sec [%f], "
        "log_av_trajectory end time [%f], "
        "planner_start_timestamp_sec end time [%f], ",
        log_av_trajectory_start_timestamp_sec, planner_start_timestamp_sec,
        log_av_trajectory_end_timestamp_sec,
        planner_start_timestamp_sec + (horizon - 1) * kDeltaT);
    return absl::FailedPreconditionError(
        "Fail to interpolate log_av_trajectory.");
  }

  std::vector<ApolloTrajectoryPointProto> expert_trajs;
  for (int i = 0; i < horizon; ++i) {
    const double query_t = planner_start_timestamp_sec + i * kDeltaT -
                           log_av_trajectory_start_timestamp_sec;
    auto point = QueryApolloTrajectoryPointByT(
        log_av_trajectory.trajectory_point().begin(),
        log_av_trajectory.trajectory_point().end(), query_t);
    point.set_relative_time(i * kDeltaT);
    expert_trajs.push_back(std::move(point));
  }

  // Load expert future trajectory into one MotionForm.
  const std::unique_ptr<MotionForm> expert_complete_motion =
      std::make_unique<CompleteMotion>(&form_builder, expert_trajs);
  // Evaluate the MotionForm and save the results to initializerOutput.
  const int feature_size = output->cost_provider->weights().size();
  std::vector<double> weighted_feature_costs;
  weighted_feature_costs.resize(feature_size);

  output->cost_provider->ComputeDpCost(0.0, expert_complete_motion.get(),
                                       absl::MakeSpan(weighted_feature_costs));

  double weighted_total_cost = absl::c_accumulate(weighted_feature_costs, 0.0);

  std::vector<double> feature_costs = std::move(weighted_feature_costs);
  auto cost_provider_weights = output->cost_provider->weights();
  const int cost_provider_weights_size = cost_provider_weights.size();
  for (int i = 0; i < cost_provider_weights_size; ++i) {
    feature_costs[i] = feature_costs[i] / cost_provider_weights[i];
  }

  std::vector<double> weights;
  weights.reserve(cost_provider_weights_size);
  for (int i = 0; i < cost_provider_weights_size; ++i) {
    weights.push_back(cost_provider_weights[i]);
  }

  auto expert_traj =
      ResampleTrajectoryPoints(traj_steps, {expert_complete_motion.get()});

  // Note(Jinyun): Use the computed cost to filtering out wrongly-behaved
  // expert trajectory. Susceptible to changes in cost calculation and name.
  MotionSearchOutput::IsFilteredReasons is_filtered_reasons;
  constexpr double kEpsilon = 1e-6;
  constexpr double kBoundTolerance = 5.0;
  for (int i = 0; i < cost_provider_weights_size; ++i) {
    if (output->cost_provider->cost_names()[i] ==
            "dp_lane_boundary.outer_bound" &&
        feature_costs[i] > kBoundTolerance) {
      is_filtered_reasons.is_out_of_bound = true;
    }
    if (output->cost_provider->cost_names()[i] ==
            "dp_stop_constraint.stop_constraint" &&
        feature_costs[i] > kEpsilon) {
      is_filtered_reasons.is_violating_stop_constraint = true;
    }
    if (output->cost_provider->cost_names()[i] ==
            "dp_dynamic_collision.collision_cost" &&
        feature_costs[i] > kEpsilon) {
      is_filtered_reasons.is_dynamic_collision = true;
    }
    if (output->cost_provider->cost_names()[i] ==
            "dp_leading_object.leading_object" &&
        feature_costs[i] > kEpsilon) {
      is_filtered_reasons.is_violating_leading_objects = true;
    }
  }

  output->expert_evaluation = MotionSearchOutput::TrajectoryEvaluationDumping{
      .weighted_total_cost = weighted_total_cost,
      .dumped_weights = std::move(weights),
      .feature_costs = std::move(feature_costs),
      .traj = std::move(expert_traj),
      .is_filtered_reasons = is_filtered_reasons,
  };

  return absl::OkStatus();
}

void ParseFeaturesDumpingProto(
    const MotionSearchOutput& search_output,
    ExpertEvaluationProto* expert_proto,
    SampledDpMotionEvaluationProto* candidates_proto) {
  // Sanity Check
  if (search_output.expert_evaluation.traj.empty() ||
      search_output.candidates_evaluation.empty()) {
    QLOG(WARNING) << "No expert evaluation or candidates evaluation for "
                     "ParseFeaturesDumpingProto.";
    return;
  }
  // Set expert_proto.
  expert_proto->Clear();

  for (const auto& name : search_output.cost_provider->cost_names()) {
    *expert_proto->add_cost_names() = name;
  }

  expert_proto->mutable_costs()->Reserve(expert_proto->cost_names_size());
  for (const auto& c : search_output.expert_evaluation.feature_costs) {
    expert_proto->add_costs(c);
  }

  expert_proto->set_total_cost(
      search_output.expert_evaluation.weighted_total_cost);

  expert_proto->mutable_trajectory()->mutable_trajectory_points()->Reserve(
      search_output.expert_evaluation.traj.size());
  for (const auto& point : search_output.expert_evaluation.traj) {
    auto* new_trajectory_point =
        expert_proto->mutable_trajectory()->add_trajectory_points();
    *new_trajectory_point = point;
  }

  *expert_proto->mutable_weights() = {
      search_output.cost_provider->weights().begin(),
      search_output.cost_provider->weights().end()};

  expert_proto->mutable_is_filtered_reasons()->set_is_out_of_bound(
      search_output.expert_evaluation.is_filtered_reasons.is_out_of_bound);
  expert_proto->mutable_is_filtered_reasons()->set_is_violating_stop_constraint(
      search_output.expert_evaluation.is_filtered_reasons
          .is_violating_stop_constraint);
  expert_proto->mutable_is_filtered_reasons()->set_is_dynamic_collision(
      search_output.expert_evaluation.is_filtered_reasons.is_dynamic_collision);
  expert_proto->mutable_is_filtered_reasons()->set_is_violating_leading_objects(
      search_output.expert_evaluation.is_filtered_reasons
          .is_violating_leading_objects);

  // Set candidates_proto.
  candidates_proto->Clear();

  auto candidates_cost_name = candidates_proto->mutable_cost_names();
  candidates_cost_name->CopyFrom(expert_proto->cost_names());

  // Get trajectories.
  candidates_proto->mutable_traj_costs()->Reserve(
      search_output.candidates_evaluation.size());
  candidates_proto->mutable_trajectory()->Reserve(
      search_output.candidates_evaluation.size());
  for (const auto& traj_eval : search_output.candidates_evaluation) {
    auto* traj_cost = candidates_proto->add_traj_costs();
    for (const auto& c : traj_eval.feature_costs) {
      traj_cost->add_costs(c);
    }
    traj_cost->set_total_cost(traj_eval.weighted_total_cost);

    auto* traj_proto = candidates_proto->add_trajectory();
    traj_proto->mutable_trajectory_points()->Reserve(traj_eval.traj.size());
    for (const auto& point : traj_eval.traj) {
      auto* new_trajectory_point = traj_proto->add_trajectory_points();
      *new_trajectory_point = point;
    }
  }

  candidates_proto->set_min_cost(search_output.min_cost);
  *candidates_proto->mutable_weights() = {
      search_output.cost_provider->weights().begin(),
      search_output.cost_provider->weights().end()};
}

}  // namespace qcraft::planner

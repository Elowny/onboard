#include "onboard/planner/ml/initializer_models/initializer_post_evaluator.h"

#include <algorithm>
#include <limits>
#include <memory>
#include <ostream>

#include "absl/algorithm/container.h"
#include "absl/time/clock.h"
#include "absl/time/time.h"
#include "glog/logging.h"

#include "onboard/async/parallel_for.h"
#include "onboard/eval/qevent.h"
#include "onboard/eval/qevent_base.h"
#include "onboard/global/logging.h"
#include "onboard/planner/initializer/cost_provider.h"
#include "onboard/planner/initializer/dp_motion_searcher_util.h"
#include "onboard/planner/initializer/motion_form.h"
#include "onboard/planner/initializer/motion_search_util.h"
#include "onboard/planner/ml/initializer_models/complete_motion_form.h"
#include "onboard/proto/trajectory_point.pb.h"

namespace qcraft::planner {

namespace {
BestEdgeInfo FindBestEdgeFromTopK(
    int traj_steps, const GeometryFormBuilder& form_builder,
    const MotionEdgeVector<MotionSearchOutput::SearchCost>& search_costs,
    absl::Span<const MotionEdgeIndex> terminated_idxes,
    const MotionGraph* motion_graph, const CostProviderBase* cost_provider,
    ThreadPool* thread_pool) {
  const int top_traj_num =
      std::min(static_cast<int>(terminated_idxes.size()), 10);
  auto top_k_trajs =
      TopKTrajectories(terminated_idxes, search_costs, top_traj_num);

  std::vector<double> costs(top_k_trajs.size(), 0.0);
  ParallelFor(0, top_k_trajs.size(), thread_pool, [&](int i) {
    top_k_trajs[i].traj_points = ConstructTrajFromLastEdge(
        traj_steps, *motion_graph, top_k_trajs[i].idx);
    // Load searched candidate trajectory into one MotionForm.
    const std::unique_ptr<MotionForm> candidate_complete_motion =
        std::make_unique<CompleteMotion>(&form_builder,
                                         top_k_trajs[i].traj_points);
    // Evaluate the MotionForm and save the results to initializerOutput.
    const int feature_size = cost_provider->weights().size();
    std::vector<double> weighted_feature_costs;
    weighted_feature_costs.resize(feature_size);

    cost_provider->ComputeDpCost(0.0, candidate_complete_motion.get(),
                                 absl::MakeSpan(weighted_feature_costs));

    costs[i] = absl::c_accumulate(weighted_feature_costs, 0.0);
  });

  MotionEdgeIndex best_final_edge = MotionEdgeVector<MotionEdge>::kInvalidIndex;
  int min_idx = -1;
  double min_cost = std::numeric_limits<double>::max();
  for (int i = 0, size = costs.size(); i < size; ++i) {
    if (costs[i] < min_cost) {
      min_cost = costs[i];
      best_final_edge = top_k_trajs[i].idx;
      min_idx = i;
    }
  }

  if (min_idx != 0) {
    QEVENT_EVERY_N_SECONDS(
        "Jinyun", "initializer_post_evaluation_chose_different_trajectory", 5.0,
        [&](QEvent* qevent) {
          qevent->AddField("post_evaluation_choice_number", min_idx);
        });
  } else {
    QEVENT_EVERY_N_SECONDS(
        "Jinyun", "initializer_post_evaluation_chose_same_trajectory", 5.0,
        [&](QEvent* qevent) {
          qevent->AddField("post_evaluation_choice_number", min_idx);
        });
  }

  return {.idx = best_final_edge,
          .total_cost = min_cost,
          .is_created_stationary_motion = false};
}
}  // namespace

void PostEvaluateTrajs(const PostEvaluatorInput& input,
                       SingleTrajInfo* const traj_output,
                       ThreadPool* thread_pool) {
  VLOG(1) << "--------- Start of One Post Evaluation-----------";
  const auto start_time = absl::Now();

  if (input.pre_best->is_created_stationary_motion) {
    QEVENT_EVERY_N_SECONDS(
        "Jinyun", "initializer_generating_a_stationary_motion", 5.0,
        [&](QEvent* qevent) {
          qevent->AddField("initializer_generating_a_stationary_motion", true);
        });
  } else {
    std::unique_ptr<CostProviderBase> post_cost_provider =
        std::make_unique<DpCostProvider>(
            *input.drive_passage, *input.initializer_params,
            *input.motion_constraint_params, *input.stop_s, *input.st_traj_mgr,
            *input.leading_groups, input.leading_group_idx, *input.vehicle_geom,
            input.collision_checker, input.path_sl,
            traj_output->ref_speed_table.get(), input.captain_net_output,
            input.is_lane_change, input.max_accumulated_s,
            /*is_post_evaluation=*/true);
    const auto best_post_eval_edge_info = FindBestEdgeFromTopK(
        input.initializer_params->traj_steps(), *input.form_builder,
        *input.search_costs, input.terminated_idxes,
        traj_output->motion_graph.get(), post_cost_provider.get(), thread_pool);
    traj_output->last_edge_index = best_post_eval_edge_info.idx;
    traj_output->total_cost = best_post_eval_edge_info.total_cost;
    VLOG(1) << "After Post Evaluation, best_final_edge: "
            << traj_output->last_edge_index.value()
            << " Total cost: " << traj_output->total_cost;
  }

  const auto duration = absl::ToDoubleMilliseconds(absl::Now() - start_time);
  VLOG(1) << "Time(ms) spent for One Post Evaluation: " << duration;
  VLOG(1) << "--------- End of One Post Evaluation-------------";
  QEVENT_EVERY_N_SECONDS(
      "jinyun", "initializer_post_evaluation_time_spent", 5.0,
      [&](QEvent* qevent) { qevent->AddField("time(ms)", duration); });
}

}  // namespace qcraft::planner

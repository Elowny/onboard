#ifndef ONBOARD_PLANNER_ML_INITIALIZER_MODELS_INITIALIZER_FEATURE_EXTRACTOR_H_
#define ONBOARD_PLANNER_ML_INITIALIZER_MODELS_INITIALIZER_FEATURE_EXTRACTOR_H_

#include <vector>

#include "absl/status/status.h"

#include "onboard/planner/initializer/geometry/geometry_form_builder.h"
#include "onboard/planner/initializer/initializer_output.h"
#include "onboard/planner/initializer/motion_graph.h"
#include "onboard/planner/initializer/proto/initializer.pb.h"
#include "onboard/proto/trajectory.pb.h"

namespace qcraft::planner {

void SampledDpMotionEvaluation(
    int traj_steps,
    const MotionEdgeVector<MotionSearchOutput::SearchCost>& search_costs,
    const std::vector<MotionEdgeIndex>& terminated_edge_idxes,
    MotionSearchOutput* const output);

absl::Status ExpertDpMotionEvaluation(int traj_steps, absl::Time plan_time,
                                      const GeometryFormBuilder& form_builder,
                                      const TrajectoryProto& log_av_trajectory,
                                      MotionSearchOutput* const output);

/**
 * @brief: Dumping expert trajectory raw feature cost and all searched DP
 * trajectories's raw feature costs.
 * **/
void ParseFeaturesDumpingProto(
    const MotionSearchOutput& search_output,
    ExpertEvaluationProto* expert_proto,
    SampledDpMotionEvaluationProto* candidates_proto);

}  // namespace qcraft::planner

// NOLINTNEXTLINE
// NOLINTNEXTLINE
#endif  // ONBOARD_PLANNER_ML_INITIALIZER_MODELS_INITIALIZER_FEATURE_EXTRACTOR_H_

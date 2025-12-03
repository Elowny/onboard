#ifndef ONBOARD_PLANNER_ML_INITIALIZER_MODELS_INITIALIZER_POST_EVALUATOR_H_
#define ONBOARD_PLANNER_ML_INITIALIZER_MODELS_INITIALIZER_POST_EVALUATOR_H_

#include <string>
#include <vector>

#include "absl/types/span.h"

#include "onboard/async/thread_pool.h"
#include "onboard/planner/common/path_sl_boundary.h"
#include "onboard/planner/initializer/collision_checker.h"
#include "onboard/planner/initializer/dp_motion_searcher_defs.h"
#include "onboard/planner/initializer/geometry/geometry_form_builder.h"
#include "onboard/planner/initializer/geometry/geometry_graph.h"
#include "onboard/planner/initializer/initializer_output.h"
#include "onboard/planner/initializer/motion_graph.h"
#include "onboard/planner/initializer/motion_state.h"
#include "onboard/planner/initializer/multi_traj_selector.h"
#include "onboard/planner/initializer/ref_speed_table.h"
#include "onboard/planner/ml/captain_net/captain_net.h"
#include "onboard/planner/object/spacetime_trajectory_manager.h"
#include "onboard/planner/proto/planner_params.pb.h"
#include "onboard/planner/router/drive_passage.h"
#include "onboard/proto/vehicle.pb.h"

namespace qcraft::planner {

struct PostEvaluatorInput {
  const GeometryFormBuilder* form_builder = nullptr;
  const BestEdgeInfo* pre_best;
  const DrivePassage* drive_passage;
  const std::vector<double>* stop_s;
  const SpacetimeTrajectoryManager* st_traj_mgr;
  const std::vector<std::vector<std::string>>* leading_groups;
  const int leading_group_idx;
  const VehicleGeometryParamsProto* vehicle_geom;
  const CollisionChecker* collision_checker;
  const PathSlBoundary* path_sl;
  const InitializerConfig* initializer_params;
  const MotionConstraintParamsProto* motion_constraint_params;
  const RefSpeedTable* ref_speed_table;
  const ml::captain_net::CaptainNetOutput* captain_net_output;
  bool is_lane_change;
  double max_accumulated_s;
  const bool is_post_evaluation = true;
  const MotionEdgeVector<MotionSearchOutput::SearchCost>* search_costs;
  absl::Span<const MotionEdgeIndex> terminated_idxes;
  const MotionState* sdc_motion;
  MotionNodeIndex sdc_node_idx;
  GeometryNodeIndex sdc_geom_node;
  const MotionGraph* motion_graph;
};

void PostEvaluateTrajs(const PostEvaluatorInput& input,
                       SingleTrajInfo* traj_output, ThreadPool* thread_pool);

}  // namespace qcraft::planner
#endif  // ONBOARD_PLANNER_ML_INITIALIZER_MODELS_INITIALIZER_POST_EVALUATOR_H_

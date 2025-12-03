#include "onboard/prediction/post_process/conflict_resolver.h"

// IWYU pragma: no_include <ext/alloc_traits.h>

#include <memory>
#include <ostream>
#include <set>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "absl/cleanup/cleanup.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_format.h"
#include "absl/strings/str_join.h"
#include "glog/logging.h"

#include "onboard/async/async_util.h"
#include "onboard/async/parallel_for.h"
#include "onboard/async/thread_pool.h"
#include "onboard/global/trace.h"
#include "onboard/lite/logging.h"
#include "onboard/maps/semantic_map_defs.h"
#include "onboard/prediction/post_process/conflict_resolver_input.h"
#include "onboard/prediction/post_process/conflict_resolver_params.h"
#include "onboard/prediction/post_process/object_conflict_manager.h"
#include "onboard/prediction/post_process/object_conflict_resolver.h"
#include "onboard/prediction/predicted_trajectory.h"
#include "onboard/utils/source_location.h"

namespace qcraft {
namespace prediction {
namespace {
struct ModifiedTrajectoryResult {
  std::string traj_id;
  bool valid = false;
  PredictedTrajectory trajectory;
};

}  // namespace

void ResolveConflict(
    const planner::PlannerSemanticMapManager& semantic_map_manager,
    const std::set<mapping::ElementId>& red_tls,
    const ConflictResolverParams& resolver_params,
    const std::map<ObjectIDType, ObjectPredictionPostProcess>&
        mutable_object_predictions,
    ConflictResolverDebugProto* debug_proto, ThreadPool* thread_pool) {
  debug_proto->Clear();
  const auto resolution_level =
      resolver_params.config_params().resolution_level();
  if (resolution_level == CRL_NONE) {
    return;
  }
  SCOPED_QTRACE("ResolveConflict");
  // Generate as mush common infomation as possible before resolving conflicts
  // for each trajectory.
  // 1. Create Object conflict manager.
  auto ptr_obj_conflict_mgr = std::make_unique<ObjectConflictManager>(
      semantic_map_manager, resolver_params.GetGeneralConfig(),
      resolution_level, mutable_object_predictions, thread_pool);
  auto& obj_con_mgr = *ptr_obj_conflict_mgr;
  const auto ocm_init_status = obj_con_mgr.Init();
  if (!ocm_init_status.ok()) {
    QLOG(WARNING) << "Prediction conflict manager init object conflict "
                     "manager fails, "
                     "return original prediction results. Error message: "
                  << ocm_init_status.message();
    return;
  }

  const auto order_or = obj_con_mgr.GetResolvingOrder();
  if (!order_or.ok()) {
    return;
  }

  // 2. Create object conflict resolver input according to resolving orders.
  int num_resolved = 0;
  for (const auto& trajs_cur_stage : *order_or) {
    VLOG(1) << "------------------------Stage-------------------------";
    VLOG(1) << "Resolve: " << absl::StrJoin(trajs_cur_stage, ",");
    // Trajectories in the same stage can be solved in parallel.
    std::vector<ObjectConflictResolverInput> object_resolver_inputs;
    object_resolver_inputs.reserve(trajs_cur_stage.size());
    for (int i = 0, n = trajs_cur_stage.size(); i < n; ++i) {
      const auto& traj_id = trajs_cur_stage[i];
      object_resolver_inputs.push_back(ObjectConflictResolverInput({
          .traj_id = traj_id,
          .semantic_map_mgr = &semantic_map_manager,
          .obj_con_mgr = ptr_obj_conflict_mgr.get(),  // unmutable.
          .red_tls = &red_tls,
          .params = &resolver_params,
      }));
    }

    std::vector<ModifiedTrajectoryResult> modified_trajectories_container;
    modified_trajectories_container.resize(object_resolver_inputs.size());
    const absl::Cleanup cleanup_modified_trajectories_container =
        [&modified_trajectories_container] {
          DestroyContainerAsyncMarkSource(
              std::move(modified_trajectories_container),
              (QCRAFT_LOC).ToString());
        };
    // TODO(changqing): collect debug info outside Parallelfor.
    ParallelFor(0, object_resolver_inputs.size(), thread_pool, [&](int i) {
      auto modified_predicted_trajectory_or = ResolveObjectConflict(
          object_resolver_inputs[i], thread_pool, debug_proto);
      if (modified_predicted_trajectory_or.ok()) {
        modified_trajectories_container[i] = ModifiedTrajectoryResult({
            .traj_id = object_resolver_inputs[i].traj_id,
            .valid = true,
            .trajectory = std::move(*modified_predicted_trajectory_or),
        });
      }
    });

    for (auto& modified_trajectory_result : modified_trajectories_container) {
      SCOPED_QTRACE("BatchUpdateModifiedTrajectory");
      if (modified_trajectory_result.valid == false) continue;
      // Update predicted trajectory result so resolution in next stage will
      // get the latest results.
      VLOG(1) << "Result for: " << modified_trajectory_result.traj_id << ". "
              << modified_trajectory_result.trajectory.annotation();
      num_resolved++;
      // Add debug info to proto to avoid data race.
      auto* modified_traj = debug_proto->add_modified_trajs();
      modified_traj->set_traj_id(modified_trajectory_result.traj_id);
      modified_traj->set_annotation(
          modified_trajectory_result.trajectory.annotation());
      ptr_obj_conflict_mgr->AddModifiedTrajectory(
          modified_trajectory_result.traj_id,
          std::move(modified_trajectory_result.trajectory));
    }
    SCOPED_QTRACE_ARG1("ConflictResolver::OneStage", "num_resolved",
                       num_resolved);
  }

  // Get all modified predictions from object conflict manager.
  const auto modify_predictions_status =
      ptr_obj_conflict_mgr->ModifiedPredictions(mutable_object_predictions);
  if (modify_predictions_status.ok()) {
    return;  // Results are returned here.
  }
  QLOG(WARNING) << absl::StrFormat(
      "Conflict resolution in prediction fail: %s. Return original "
      "prediction result.",
      modify_predictions_status.message());
  return;
}
}  // namespace prediction
}  // namespace qcraft

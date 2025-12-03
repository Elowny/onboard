#include "onboard/prediction/post_process/object_conflict_resolver.h"

// IWYU pragma: no_include <ext/alloc_traits.h>
//              for __alloc_traits<>::value_type
#include <algorithm>
#include <memory>
#include <ostream>
#include <set>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_set.h"
#include "absl/status/status.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_format.h"
#include "absl/strings/str_join.h"
#include "absl/types/span.h"
#include "glog/logging.h"

#include "onboard/container/strong_int.h"
#include "onboard/global/trace.h"
#include "onboard/lite/check.h"
#include "onboard/planner/speed/speed_vector.h"
#include "onboard/prediction/post_process/conflict_resolver_params.h"
#include "onboard/prediction/post_process/object_conflict_manager.h"
#include "onboard/prediction/post_process/object_st_map.h"
#include "onboard/prediction/post_process/object_svt_graph.h"
#include "onboard/prediction/post_process/relation_analyzer.h"
#include "onboard/prediction/post_process/svt_cost_provider.h"
#include "onboard/prediction/prediction_flags.h"
#include "onboard/prediction/util/lane_path_finder.h"
#include "onboard/prediction/util/trajectory_developer.h"
#include "onboard/proto/perception.pb.h"
#include "onboard/utils/status_macros.h"

namespace qcraft::prediction {

// Resolve conflict for one PredictedTrajectory for one specified object.
absl::StatusOr<PredictedTrajectory> ResolveObjectConflict(
    const ObjectConflictResolverInput& input, ThreadPool* thread_pool,
    ConflictResolverDebugProto* debug_proto) {
  SCOPED_QTRACE("ResolveObjectConflict");
  QCHECK_NOTNULL(input.obj_con_mgr);
  QCHECK_NOTNULL(input.semantic_map_mgr);
  QCHECK_NOTNULL(input.params);
  QCHECK_NOTNULL(input.red_tls);

  const auto& obj_con_mgr = *input.obj_con_mgr;
  const auto& params = *input.params;
  const auto& red_tls = *input.red_tls;
  const auto& semantic_map_mgr = *input.semantic_map_mgr;
  const auto& traj_id = input.traj_id;

  // Get object specific infos from object conflict manager.
  ASSIGN_OR_RETURN(const PredictedTrajectory* ptr_predicted_trajectory,
                   obj_con_mgr.GetPredictedTrajectoryByTrajId(traj_id));
  ASSIGN_OR_RETURN(const TrajectoryInfo* ptr_traj_info,
                   obj_con_mgr.GetTrajectoryInfoByTrajId(traj_id));
  const auto& traj_info = *ptr_traj_info;
  const auto& predicted_trajectory = *ptr_predicted_trajectory;
  const auto& path = *traj_info.path;
  const auto& ref_speed = *traj_info.ref_speed;
  auto mutable_predicted_trajectory = predicted_trajectory;  // Make a copy?

  std::vector<std::string> influencers;
  if (traj_info.influencers != nullptr) {
    const auto& influencer_idxes = *traj_info.influencers;
    influencers.reserve(influencer_idxes.size());
    const auto traj_nodes = obj_con_mgr.traj_nodes();
    for (const auto& idx : influencer_idxes) {
      influencers.push_back(traj_nodes[idx.value()].traj_id);
    }
  }

  if (influencers.empty() && red_tls.empty()) {
    return absl::NotFoundError("No influencers, exit before building st map.");
  }

  VLOG(1) << absl::StrFormat(">>>>>>> Resolving for >>> %s: influencers: %s.",
                             traj_id, absl::StrJoin(influencers, ","));

  // Step 1. Build st map.
  ASSIGN_OR_RETURN(const ObjectProto* ptr_object_proto,
                   obj_con_mgr.GetObjectProtoByTrajId(traj_id));
  const auto& object_proto = *ptr_object_proto;
  std::unique_ptr<ObjectStMap> st_map =
      std::make_unique<ObjectStMap>(obj_con_mgr, object_proto, path, ref_speed,
                                    predicted_trajectory, &params, thread_pool);

  // Step 2.1 Project influencers onto st map.
  const auto stationary_map_status = st_map->MapStationaryObjects(influencers);
  if (!stationary_map_status.ok()) {
    return absl::InternalError(
        absl::StrFormat("Map stationary objects for path of %s error: %s",
                        traj_id, stationary_map_status.message()));
  }

  // Step 2.2 Map possible red traffic light stopline.
  const auto lane_path_or = FindConsecutiveLanesForDiscretizedPath(
      semantic_map_mgr, path, predicted_trajectory.is_reversed());
  if (lane_path_or.ok()) {
    st_map->MapTLRedLanes(red_tls, *lane_path_or);
  }
  if (!st_map->HasConflict()) {
    return absl::NotFoundError("Do not have conflict.");
  }
  // Step 2. Build cost provider.
  const auto& cost_config = params.GetCostConfig();
  const auto& object_config = params.GetConfigByObjectType(object_proto.type());
  const auto stoplines = st_map->QueryStoplines();
  const auto stationary_objects = st_map->QueryStationaryObjects();
  SvtCostProviderInput cost_provider_input({
      .stationary_objects = &stationary_objects,
      .stoplines = &stoplines,
      .ref_speed = &ref_speed,
      .cost_config = &cost_config,
      .stationary_follow_distance = object_config.stationary_follow_distance(),
  });
  std::unique_ptr<SvtCostProvider> cost_provider =
      std::make_unique<SvtCostProvider>(cost_provider_input);

  // Step 3. Solve.
  const auto& general_config = params.GetGeneralConfig();
  ObjectSvtGraphInput svt_graph_input({
      .traj_id = traj_id,
      .object_proto = &object_proto,
      .stoplines = &stoplines,
      .stationary_objects = &stationary_objects,
      .ref_speed = &ref_speed,
      .cost_provider = cost_provider.get(),
      .general_config = &general_config,
      .object_config = &object_config,
  });
  std::unique_ptr<ObjectSvtGraph> svt_graph =
      std::make_unique<ObjectSvtGraph>(svt_graph_input, thread_pool);
  const auto new_speed_vec_or =
      svt_graph->Search(FLAGS_prediction_conflict_resolver_visual_on);

  // Step 4. Collect result and add annotations.
  const auto& origin_annotation = predicted_trajectory.annotation();
  if (new_speed_vec_or.ok()) {
    VLOG(3) << absl::StrFormat("Modifying %s trajectory: ", traj_id);
    auto modified_points = CombinePathAndSpeedForPredictedTrajectoryPoints(
        path, *new_speed_vec_or);
    if (!predicted_trajectory.points().empty() && !modified_points.empty()) {
      std::string annotation = absl::StrFormat(
          "%s. CR: %f m -> %f m, %f s -> %f s. %f m/s -> %f m/s. %s mapped.",
          origin_annotation, predicted_trajectory.points().back().s(),
          modified_points.back().s(), predicted_trajectory.points().back().t(),
          modified_points.back().t(), predicted_trajectory.points().back().v(),
          modified_points.back().v(), st_map->Annotation());
      *mutable_predicted_trajectory.mutable_points() =
          std::move(modified_points);
      mutable_predicted_trajectory.set_annotation(std::move(annotation));
    } else {
      mutable_predicted_trajectory.set_annotation(
          absl::StrCat(origin_annotation, "CR: Modified points empty!"));
    }
    if (FLAGS_prediction_conflict_resolver_visual_on) {
      svt_graph->ToProto(debug_proto);
    }
  } else {
    mutable_predicted_trajectory.set_annotation(absl::StrCat(
        origin_annotation, "CR: Speed vec fail: ", st_map->Annotation()));
  }
  VLOG(1) << "--------- Result: " << mutable_predicted_trajectory.annotation();
  return mutable_predicted_trajectory;
}
}  // namespace qcraft::prediction

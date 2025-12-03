#include "onboard/planner/freespace/freespace_stop_finder.h"

#include <algorithm>
#include <cmath>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "absl/cleanup/cleanup.h"
#include "absl/status/status.h"
#include "absl/strings/str_cat.h"
#include "absl/time/clock.h"
#include "absl/time/time.h"

#include "onboard/async/async_util.h"
#include "onboard/eval/qevent.h"
#include "onboard/eval/qevent_base.h"
#include "onboard/global/timer.h"
#include "onboard/global/trace.h"
#include "onboard/lite/check.h"
#include "onboard/lite/logging.h"
#include "onboard/math/piecewise_linear_function.h"
#include "onboard/planner/common/discretized_path.h"
#include "onboard/planner/decision/constraint_manager.h"
#include "onboard/planner/freespace/freespace_util.h"
#include "onboard/planner/planner_defs.h"
#include "onboard/planner/proto/planner_params.pb.h"
#include "onboard/planner/speed/decider/st_boundary_pre_decider.h"
#include "onboard/planner/speed/plot_util.h"
#include "onboard/planner/speed/proto/speed_finder.pb.h"
#include "onboard/planner/speed/proto/speed_finder_params.pb.h"
#include "onboard/planner/speed/speed_decision_util.h"
#include "onboard/planner/speed/speed_finder_util.h"
#include "onboard/planner/speed/st_boundary.h"
#include "onboard/planner/speed/st_boundary_with_decision.h"
#include "onboard/planner/speed/st_graph.h"
#include "onboard/planner/speed/standstill_distance_decider.h"
#include "onboard/proto/trajectory_point.pb.h"
#include "onboard/utils/source_location.h"
#include "onboard/utils/time_util.h"

namespace qcraft {
namespace planner {

absl::StatusOr<FreespaceStopFinderOutput> FindFreespaceStop(
    const DiscretizedPath& path, bool forward, double plan_start_v,
    absl::Time plan_time, const PlannerSemanticMapManager* psmm,
    const SpacetimeTrajectoryManager& st_traj_mgr,
    const PlannerClusterObjectManager& /*cluster_obj_mgr*/,
    const ConstraintManager& constraint_mgr,
    const absl::flat_hash_set<std::string>& stalled_objects,
    const absl::flat_hash_set<PlannerClusterObject::Id>&
    /*stalled_cluster_objects*/,
    const VehicleGeometryParamsProto& vehicle_geometry_params,
    const VehicleOctagonModelParamsProto& vehicle_model_params,
    const MotionConstraintParamsProto& motion_constraint_params,
    const SpeedFinderParamsProto& speed_finder_params,
    ThreadPool* thread_pool) {
  SCOPED_QTRACE("FreespacePlanner/FindFreespaceStop");

  ScopedMultiTimer timer("stop_finder");
  const absl::Cleanup timeout_trigger = [start_time = absl::Now()]() {
    constexpr double kSpeedFinderTimeLimitMs = 30.0;
    const auto speed_total_time_ms =
        absl::ToDoubleMilliseconds(absl::Now() - start_time);
    if (speed_total_time_ms > kSpeedFinderTimeLimitMs) {
      QEVENT_EVERY_N_SECONDS("pingshi", "freespace_stop_finder_timeout", 10.0,
                             [&](QEvent* qevent) {
                               qevent->AddField("time_ms", speed_total_time_ms);
                             });
    }
  };

  FreespaceStopFinderOutput output;
  QCHECK(!path.empty());
  if (path.size() < 2) {
    return absl::FailedPreconditionError(absl::StrCat(
        "Input path has only 1 path point: ", path.front().DebugString()));
  }

  plan_start_v = std::abs(plan_start_v);

  // Make st_boundary
  const auto av_shapes = BuildFreespaceAvShapes(vehicle_geometry_params, path,
                                                forward, vehicle_model_params);
  const auto path_kd_tree = BuildPathKdTree(path);

  auto st_graph = std::make_unique<StGraph>(
      &path, kTrajectorySteps, plan_start_v,
      motion_constraint_params.max_deceleration(), &vehicle_geometry_params,
      &speed_finder_params.st_graph_params(), &av_shapes, path_kd_tree.get(),
      /*path_approx=*/nullptr, /*path_approx_for_mirrors=*/nullptr);

  auto st_boundaries =
      st_graph->GetStBoundaries(st_traj_mgr, /*leading_objs=*/{},
                                constraint_mgr, psmm, /*drive_passage=*/nullptr,
                                /*path_sl_boundary=*/nullptr, thread_pool);
  timer.Mark("map_st_boundaries");

  // Make st_boundary_with_decision.
  auto st_boundaries_with_decision =
      InitializeStBoundaryWithDecision(std::move(st_boundaries));

  // Make standstill distance decision.
  const StandstillDistanceDeciderInput standstill_distance_decider_input{
      .speed_finder_params = &speed_finder_params,
      .stalled_object_ids = &stalled_objects,
      .planner_semantic_map_manager = nullptr,
      .lane_path = nullptr,
      .st_traj_mgr = &st_traj_mgr,
      .constraint_mgr = &constraint_mgr,
      .extra_follow_standstill_for_large_vehicle =
          PiecewiseLinearFunctionFromProto(
              speed_finder_params
                  .extra_follow_standstill_distance_for_large_vehicle_plf())(
              plan_start_v)};
  for (auto& st_boundary_with_decision : st_boundaries_with_decision) {
    DecideStandstillDistanceForStBoundary(standstill_distance_decider_input,
                                          &st_boundary_with_decision);
  }

  // Only keep all zero-distance stationary st-boudnaries & the nearest
  // non-zero-distance stationary st-boundary.
  KeepNearestStationarySpacetimeTrajectoryStBoundary(
      &st_boundaries_with_decision);

  // Make pre-decision.
  MakeFreespacePreDecisionForStBoundaries(&st_boundaries_with_decision);
  timer.Mark("pre_decider");

  // Make follow decisions for all st-boundaries without prior decision.
  for (auto& st_boundary_with_decision : st_boundaries_with_decision) {
    if (st_boundary_with_decision.decision_type() == StBoundaryProto::UNKNOWN) {
      st_boundary_with_decision.set_decision_type(StBoundaryProto::FOLLOW);
      st_boundary_with_decision.set_decision_reason(StBoundaryProto::FREESPACE);
    }
  }

  // Set st_boundary debug info.
  SetStBoundaryDebugInfo(st_boundaries_with_decision,
                         &output.stop_finder_debug);

  // Compute the minimum stop_s.
  const auto min_s_info = ComputeMinStopSInfo(st_boundaries_with_decision);

  // Output.
  speed::ExportStBoundaryToChart(
      /*base_name=*/"freespace", kTrajectorySteps, st_boundaries_with_decision,
      output.stop_finder_debug, path.length(), &output.st_graph_chart);
  output.stop_s = min_s_info.min_stop_s.has_value()
                      ? std::min(*min_s_info.min_stop_s, path.length())
                      : path.length();
  output.stationary_object_stop_s =
      min_s_info.min_stationary_object_stop_s.has_value()
          ? *min_s_info.min_stationary_object_stop_s
          : std::numeric_limits<double>::infinity();
  output.nearest_stationary_object_id =
      min_s_info.nearest_stationary_object_id.has_value()
          ? *min_s_info.nearest_stationary_object_id
          : "";
  output.stop_finder_debug.set_trajectory_start_timestamp(
      ToUnixDoubleSeconds(plan_time));

  DestroyContainerAsyncMarkSource(std::move(st_graph), (QCRAFT_LOC).ToString());

  return output;
}

}  // namespace planner
}  // namespace qcraft

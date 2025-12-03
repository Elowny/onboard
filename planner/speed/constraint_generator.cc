#include "onboard/planner/speed/constraint_generator.h"

// IWYU pragma: no_include <memory>
//              for allocator_traits<>::value_type

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <string>

#include "absl/container/flat_hash_map.h"

#include "onboard/eval/qevent.h"
#include "onboard/eval/qevent_base.h"
#include "onboard/global/logging.h"
#include "onboard/global/trace.h"
#include "onboard/lite/check.h"
#include "onboard/math/geometry/halfplane.h"
#include "onboard/math/util.h"
#include "onboard/math/vec.h"
#include "onboard/planner/object/planner_object.h"
#include "onboard/planner/second_order_trajectory_point.h"
#include "onboard/planner/speed/decider/close_object_slowdown_decider.h"
#include "onboard/planner/speed/path_semantic_analyzer.h"
#include "onboard/planner/speed/proto/speed_finder.pb.h"
#include "onboard/planner/speed/st_boundary.h"
#include "onboard/planner/util/path_util.h"
#include "onboard/proto/trajectory_point.pb.h"

namespace qcraft {
namespace planner {

SpeedConstraintGeneratorOutput GenerateStationaryCloseObjectConstraints(
    absl::Span<const StBoundaryWithDecision> st_boundaries_with_decision,
    const StGraph& st_graph, const SpacetimeTrajectoryManager& traj_mgr,
    const DiscretizedPath& path, const DrivePassage& drive_passage,
    const PathSlBoundary& path_sl_boundary, double av_speed) {
  FUNC_QTRACE();
  constexpr double kCloseObjectSlowdownMaxDistance = 1.5;  // m
  SpeedConstraintGeneratorOutput output;
  const auto close_spacetime_objects = st_graph.GetCloseSpaceTimeObjects(
      st_boundaries_with_decision, traj_mgr.stationary_object_trajs(),
      kCloseObjectSlowdownMaxDistance);
  if (close_spacetime_objects.empty()) return output;
  output.path_speed_regions = MakeCloseObjectSlowdownDecision(
      close_spacetime_objects, drive_passage, path, av_speed, path_sl_boundary);
  return output;
}

SpeedConstraintGeneratorOutput GenerateDenseTrafficFlowConstraint(
    absl::Span<const StBoundaryWithDecision> st_boundaries_with_decision,
    const SpacetimeTrajectoryManager& traj_mgr,
    const std::vector<PathPointSemantic>& path_semantics,
    const DiscretizedPath& path, double plan_start_v,
    const VehicleGeometryParamsProto& vehicle_geometry_params) {
  SpeedConstraintGeneratorOutput output;
  constexpr double kAvSpeedThres = 2.0;  // m/s.
  if (plan_start_v > kAvSpeedThres) return output;

  if (path_semantics.empty()) return output;
  const auto current_lane_semantic = path_semantics.front().lane_semantic;
  if (current_lane_semantic == LaneSemantic::NONE ||
      current_lane_semantic == LaneSemantic::ROAD ||
      current_lane_semantic == LaneSemantic::INTERSECTION_STRAIGHT) {
    return output;
  }

  constexpr double kMaxOverlapMinS = 10.0;          // m.
  constexpr double kThetaDiffThres = M_PI_2 / 3.0;  // 30 deg.
  // first: object_id, second: min_t
  absl::flat_hash_map<std::string, double> objects_in_range;
  for (const StBoundaryWithDecision& stb_wd : st_boundaries_with_decision) {
    const StBoundary* raw_stb = stb_wd.st_boundary();
    if (raw_stb->source_type() != StBoundarySourceTypeProto::ST_OBJECT) {
      continue;
    }
    if (raw_stb->min_s() < kMaxOverlapMinS) {
      const auto object_id = raw_stb->object_id();
      QCHECK(object_id.has_value());
      const auto* obj =
          QCHECK_NOTNULL(traj_mgr.FindObjectByObjectId(*object_id));
      const double theta_diff =
          std::fabs(NormalizeAngle(path.front().theta() - obj->pose().theta()));
      if (theta_diff > kThetaDiffThres) {
        if (!objects_in_range.contains(*object_id) ||
            raw_stb->min_t() < objects_in_range[*object_id]) {
          objects_in_range[*object_id] = raw_stb->min_t();
        }
      }
    }
  }

  double objects_min_t = std::numeric_limits<double>::max();
  for (const auto& [_, min_t] : objects_in_range) {
    objects_min_t = std::min(objects_min_t, min_t);
  }

  constexpr int kDenseTrafficThres = 3;
  constexpr double kMinTThres = 5.0;  // s.
  if (objects_in_range.size() > kDenseTrafficThres &&
      objects_min_t < kMinTThres) {
    constexpr double kStopTime = 1.0;  // s.
    const double dist_to_front_edge = plan_start_v * kStopTime;
    const PathPoint center_pt = path.Evaluate(
        dist_to_front_edge + vehicle_geometry_params.front_edge_to_center());
    const Vec2d center = ToVec2d(center_pt);
    const Vec2d unit = Vec2d::FastUnitFromAngle(center_pt.theta());
    constexpr double kHalfWidth = 4.5;  // m.
    const HalfPlane fence(center - kHalfWidth * unit.Perp(),
                          center + kHalfWidth * unit.Perp());
    auto& path_stop_line = output.path_stop_lines.emplace_back();
    constexpr char kId[] = "dense_traffic_flow";
    fence.ToProto(path_stop_line.mutable_half_plane());
    path_stop_line.set_id(kId);
    path_stop_line.set_standoff(0.0);
    path_stop_line.set_s(dist_to_front_edge);
    path_stop_line.mutable_source()->mutable_dense_traffic_flow()->set_id(kId);

    QEVENT_EVERY_N_SECONDS("pingshi", "dense_traffic_flow_stopline_generated",
                           2.0, [&](QEvent* qevent) {
                             qevent->AddField("object_num",
                                              objects_in_range.size());
                           });
  }
  return output;
}

}  // namespace planner
}  // namespace qcraft

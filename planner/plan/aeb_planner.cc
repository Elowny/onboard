#include "onboard/planner/plan/aeb_planner.h"

#include <algorithm>
#include <memory>
#include <utility>

// IWYU pragma: no_include <google/protobuf/repeated_ptr_field.h>

#include "onboard/global/trace.h"
#include "onboard/maps/proto/online_semantic_map.pb.h"
#include "onboard/maps/proto/semantic_map.pb.h"
#include "onboard/math/geometry/box2d.h"
#include "onboard/math/geometry/segment2d.h"
#include "onboard/math/segment_matcher/segment_matcher_kdtree.h"
#include "onboard/math/util.h"
#include "onboard/math/vec.h"
#include "onboard/params/vehicle_param_api.h"
#include "onboard/planner/emergency_stop.h"
#include "onboard/planner/planner_flags.h"
#include "onboard/planner/proto/planner_params.pb.h"
#include "onboard/planner/util/vehicle_geometry_util.h"
#include "onboard/proto/vehicle.pb.h"

namespace qcraft::planner::aeb {

namespace {

bool CheckOnlineSemanticMapCurbCollision(
    const mapping::OnlineSemanticMapProto& map,
    const std::vector<ApolloTrajectoryPointProto>& prev_traj_points,
    const VehicleGeometryParamsProto& vehicle_geometry_params) {
  if (prev_traj_points.empty()) {
    return false;
  }

  std::vector<Segment2d> segments;
  for (const auto& boundary : map.boundaries()) {
    if (boundary.points_size() < 2) {
      continue;
    }
    std::vector<std::pair<int, int>> curb_ranges;
    for (int i = 0, size = boundary.points_size() - 1; i < size; ++i) {
      const auto& cur = boundary.points(i);
      const auto& next = boundary.points(i + 1);
      if (cur.type() == mapping::LaneBoundaryProto::CURB &&
          next.type() == mapping::LaneBoundaryProto::CURB) {
        segments.emplace_back(
            Vec2d(cur.smooth_point().x(), cur.smooth_point().y()),
            Vec2d(next.smooth_point().x(), next.smooth_point().y()));
      }
    }
  }
  SegmentMatcherKdtree curbs(segments);

  const int horizon = std::min(FLAGS_planner_check_aeb_curb_traj_horizon,
                               static_cast<int>(prev_traj_points.size()));
  const double veh_width = vehicle_geometry_params.width();
  const double veh_length = vehicle_geometry_params.length();
  const double radius = Hypot(veh_width, veh_length) * 0.5;
  for (int i = 0; i < horizon; ++i) {
    const auto& pt = prev_traj_points[i];
    const auto av_point = ComputeAvGeometryCenter(
        Vec2d(pt.path_point().x(), pt.path_point().y()),
        pt.path_point().theta(), vehicle_geometry_params);
    const auto curb_segs =
        curbs.GetSegmentInRadius(av_point.x(), av_point.y(), radius);
    if (!curb_segs.empty()) {
      const auto av_box = Box2d(Vec2d(pt.path_point().x(), pt.path_point().y()),
                                pt.path_point().theta(), veh_length, veh_width);
      for (const auto* curb_seg : curb_segs) {
        if (av_box.HasOverlap(*curb_seg)) {
          return true;
        }
      }
    }
  }
  return false;
}

}  // namespace

std::optional<aeb::EmergencyStopInfo> GenerateEmergencyStopInfo(
    const PlannerInput& input,
    const std::vector<ApolloTrajectoryPointProto>& prev_traj_points) {
  FUNC_QTRACE();
  if (FLAGS_planner_check_aeb) {
    // TODO(hang, bo): Implement emergency stop check and set
    // aeb_output.emergency_stop_info here.
    if (input.online_semantic_map != nullptr &&
        input.planner_state_proto != nullptr &&
        CheckOnlineSemanticMapCurbCollision(
            *input.online_semantic_map, prev_traj_points,
            input.vehicle_params.vehicle_geometry_params())) {
      return {};
    }
  }
  return std::nullopt;
}

std::vector<ApolloTrajectoryPointProto> PlanEmergencyStopTrajectory(
    const ApolloTrajectoryPointProto& plan_start_point,
    double path_s_inc_from_prev, bool reset,
    const std::vector<ApolloTrajectoryPointProto>& prev_traj_points,
    const PlannerParamsProto& planner_params) {
  FUNC_QTRACE();
  return aeb::PlanEmergencyStopTrajectory(
      plan_start_point, path_s_inc_from_prev, reset, prev_traj_points,
      planner_params.emergency_stop_params(),
      planner_params.motion_constraint_params());
}

}  // namespace qcraft::planner::aeb

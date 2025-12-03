#include "onboard/planner/standby_state.h"

// IWYU pragma: no_include <google/protobuf/repeated_ptr_field.h>

#include "onboard/lite/check.h"
#include "onboard/math/geometry/halfplane.h"
#include "onboard/math/geometry/proto/affine_transformation.pb.h"
#include "onboard/math/vec.h"
#include "onboard/planner/decision/proto/constraint.pb.h"
#include "onboard/planner/planner_defs.h"
#include "onboard/planner/proto/est_planner_debug.pb.h"
#include "onboard/proto/trajectory_point.pb.h"

namespace qcraft {
namespace planner {
namespace {

// Note: This stop line just used for visualization.
void AddStopLineConstraint(const VehicleGeometryParamsProto& vehicle_geom,
                           const PoseProto& pose, std::string_view stop_line_id,
                           std::string_view stopline_reason,
                           ConstraintProto::StopLineProto* stop_line) {
  QCHECK_NOTNULL(stop_line);
  const auto tangent = Vec2d::FastUnitFromAngle(pose.yaw());
  const Vec2d cur_point(pose.pos_smooth().x(), pose.pos_smooth().y());
  const Vec2d front_edge_center_point =
      cur_point + tangent * vehicle_geom.front_edge_to_center();
  constexpr double kHalfplaneHalfWidth = 5.0;  // m.
  const Vec2d start =
      front_edge_center_point - tangent.Perp() * kHalfplaneHalfWidth;
  const Vec2d end =
      front_edge_center_point + tangent.Perp() * kHalfplaneHalfWidth;
  HalfPlane::ToProto(start, end, stop_line->mutable_half_plane());
  stop_line->set_id(stop_line_id.data());
  stop_line->mutable_source()->mutable_standby()->set_reason(
      stopline_reason.data());
}

}  // namespace

PlannerStatus EnterStandbyState(const PoseProto& pose,
                                const VehicleGeometryParamsProto& vehicle_geom,
                                std::string_view standby_reason,
                                TrajectoryProto* trajectory,
                                PlannerDebugProto* planner_debug) {
  QCHECK_NOTNULL(trajectory);
  QCHECK_NOTNULL(planner_debug);

  PathPoint start_path_point;
  start_path_point.set_x(pose.pos_smooth().x());
  start_path_point.set_y(pose.pos_smooth().y());
  start_path_point.set_z(pose.pos_smooth().z());
  start_path_point.set_theta(pose.yaw());

  // Rewrite forward trajectory.
  trajectory->mutable_trajectory_point()->Clear();
  trajectory->mutable_trajectory_point()->Reserve(kTrajectorySteps);
  for (int i = 0; i < kTrajectorySteps; ++i) {
    ApolloTrajectoryPointProto* traj_point = trajectory->add_trajectory_point();
    *traj_point->mutable_path_point() = start_path_point;
    traj_point->set_relative_time(i * kTrajectoryTimeStep);
  }

  trajectory->set_trajectory_start_timestamp(pose.timestamp());
  trajectory->mutable_driving_state()->set_type(DrivingStateProto::STANDBY);

  // For visualization only.
  AddStopLineConstraint(vehicle_geom, pose, /*stop_line_id=*/"standby",
                        standby_reason,
                        planner_debug->add_est_planner_debugs()
                            ->mutable_constraint()
                            ->add_stop_line());
  return OkPlannerStatus();
}

}  // namespace planner
}  // namespace qcraft

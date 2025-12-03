#include "onboard/planner/freespace/path_manager_util.h"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <memory>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "glog/logging.h"
#include "google/protobuf/repeated_ptr_field.h"

#include "gtest/gtest.h"

#include "onboard/global/buffered_logger.h"
#include "onboard/lite/check.h"
#include "onboard/maps/map_selector.h"
#include "onboard/maps/semantic_map_manager.h"
#include "onboard/math/geometry/aabox2d.h"
#include "onboard/math/geometry/proto/affine_transformation.pb.h"
#include "onboard/math/geometry/util.h"
#include "onboard/math/util.h"
#include "onboard/math/vec.h"
#include "onboard/params/param_manager.h"
#include "onboard/params/v2/proto/vehicle/common.pb.h"
#include "onboard/params/v2/proto/vehicle/installation.pb.h"
#include "onboard/params/vehicle_param_api.h"
#include "onboard/planner/common/discretized_path.h"
#include "onboard/planner/freespace/freespace_planner_defs.h"
#include "onboard/planner/freespace/hybrid_a_star/hybrid_a_star.h"
#include "onboard/planner/object/planner_object.h"
#include "onboard/planner/object/spacetime_trajectory_manager.h"
#include "onboard/planner/planner_params_builder.h"
#include "onboard/planner/proto/planner_params.pb.h"
#include "onboard/planner/test_util/object_prediction_builder.h"
#include "onboard/planner/test_util/perception_object_builder.h"
#include "onboard/planner/test_util/planner_object_builder.h"
#include "onboard/planner/test_util/predicted_trajectory_builder.h"
#include "onboard/planner/test_util/util.h"
#include "onboard/proto/perception.pb.h"
#include "onboard/proto/perception/fusion/object.pb.h"
#include "onboard/vis/canvas/canvas.h"
#include "onboard/vis/common/color.h"

#include "offboard/vis/vantage/vantage_server/vantage_canvas_client.h"

namespace qcraft {
namespace planner {
namespace {
void DrawObject(std::string_view name,
                const SpacetimeTrajectoryManager& st_traj_mgr) {
  vis::Canvas& canvas =
      vis::vantage::GetCanvasClient()->GetCanvas(std::string(name));
  canvas.SetGroundZero(1);
  for (const auto traj : st_traj_mgr.stationary_object_trajs()) {
    canvas.DrawPolygon(traj->planner_object().contour(), /*z*/ 0.0,
                       vis::Color::kLightGreen);
  }
  vis::vantage::GetCanvasClient()->FlushAll();
}

struct State {
  Vec2d pos;
  double theta;  // Heading angle.
  double delta;  // Steering angle.
};

struct AvInputState {
  Vec2d av_pos;
  Vec2d av_v;
  double av_theta;  // Heading angle.
  double av_delta;  // Steering angle.
  double phi;       // Steering angle rate.
  double accel;
};

// TODO(yumeng): Try to reuse ThirdOrderBicycleDdp::EvaluateF to generate
// third-order trajectory, whose curvature is continuous.
std::vector<State> GenerateKinematicPath(
    const qcraft::VehicleGeometryParamsProto& vehicle_gem,
    const VehicleDriveParamsProto& vehicle_drive,
    const AvInputState& input_state, double delta_t, int generate_states_num,
    bool /*is_forward*/) {
  // This value calculated by robot-bus Q8001 (max_steer_angle / steer_ratio).
  const double max_delta =
      vehicle_drive.max_steer_angle() / vehicle_drive.steer_ratio();

  std::vector<State> states;
  states.reserve(generate_states_num);
  // Add init state.
  states.push_back({.pos = input_state.av_pos,
                    .theta = input_state.av_theta,
                    .delta = input_state.av_delta});

  double v = input_state.av_v.norm();
  for (int i = 0; i < generate_states_num; ++i) {
    // Kinematic bicycle model.
    const Vec2d xy_dot = v * Vec2d::FastUnitFromAngle(states[i].theta);
    const double theta_dot = v * std::tan(states[i].delta) /
                             vehicle_gem.wheel_base();  // rotation rate (𝜔)
    const double delta_dot = input_state.phi;
    v = v + input_state.accel * delta_t;

    // Derive next state.
    State next_state;
    next_state.pos = states[i].pos + xy_dot * delta_t;
    next_state.theta = states[i].theta + theta_dot * delta_t;
    next_state.delta = states[i].delta + delta_dot * delta_t;
    next_state.delta = std::clamp(next_state.delta, -max_delta, max_delta);
    states.push_back(std::move(next_state));
  }
  return states;
}

google::protobuf::RepeatedPtrField<DirectionalPathProto>
GenerateDirectionalPath(const VehicleGeometryParamsProto& vehicle_geom,
                        const VehicleDriveParamsProto& vehicle_drive) {
  constexpr double kDt = 0.1;  // s
  constexpr int kStatesNum = 50;
  Vec2d av_pos_smooth(40.0, 10.0);
  AvInputState forward_input_state{.av_pos = av_pos_smooth,
                                   .av_v = Vec2d(0.0, 0.0),
                                   .av_theta = 0.0,
                                   .av_delta = -0.45,
                                   .phi = 0.0,
                                   .accel = 1.0};
  // Forward
  const std::vector<State> forward_states =
      GenerateKinematicPath(vehicle_geom, vehicle_drive, forward_input_state,
                            kDt, kStatesNum, /*is_forward*/ true);

  std::vector<PathPoint> path_points_forward;
  path_points_forward.reserve(forward_states.size());
  for (int i = 0; i < forward_states.size(); ++i) {
    PathPoint point;
    point.set_x(forward_states[i].pos.x());
    point.set_y(forward_states[i].pos.y());
    point.set_theta(forward_states[i].theta);
    path_points_forward.push_back(std::move(point));
  }
  DiscretizedPath path_forward(std::move(path_points_forward));

  AvInputState backward_input_state{.av_pos = forward_states.back().pos,
                                    .av_v = Vec2d(0.0, 0.0),
                                    .av_theta = forward_states.back().theta,
                                    .av_delta = 0.45,
                                    .phi = 0.0,
                                    .accel = -1.0};
  // Backward
  const std::vector<State> backwark_states = GenerateKinematicPath(
      vehicle_geom, vehicle_drive, backward_input_state, kDt, kStatesNum,
      /*is_forward*/ false);
  std::vector<PathPoint> path_points_backward;
  path_points_backward.reserve(backwark_states.size());
  for (int i = 0; i < backwark_states.size(); ++i) {
    PathPoint point;
    point.set_x(backwark_states[i].pos.x());
    point.set_y(backwark_states[i].pos.y());
    point.set_theta(NormalizeAngle(backwark_states[i].theta + M_PI));
    path_points_backward.push_back(std::move(point));
  }
  DiscretizedPath path_backward(std::move(path_points_backward));

  PathManagerStateProto state;
  const DirectionalPath path1 = {.path = std::move(path_forward),
                                 .forward = true};
  path1.ToProto(state.add_paths());
  const DirectionalPath path2 = {.path = std::move(path_backward),
                                 .forward = false};
  path2.ToProto(state.add_paths());
  return state.paths();
}

TEST(FreespacePlannerUtilTest, PathSafetyCheck) {
  SetMap("dojo");
  SemanticMapManager semantic_map_manager;
  semantic_map_manager.LoadWholeMap().Build();

  RunParamsProtoV2 run_params;
  auto param_manager = CreateParamManagerFromCarId("Q1001");
  CHECK(param_manager != nullptr);
  param_manager->GetRunParams(&run_params);
  const VehicleGeometryParamsProto vehicle_geom =
      run_params.vehicle_params().vehicle_geometry_params();
  const VehicleDriveParamsProto vehicle_drive_params =
      run_params.vehicle_params().vehicle_drive_params();
  const auto planner_params_status = BuildPlannerParams(
      vehicle_geom, VEHICLE_LINCOLN_MKZ, VehicleInstallationProto::VP_DBQ_V3);
  QCHECK(planner_params_status.ok());
  const auto& planner_params = *planner_params_status;

  std::vector<PlannerObject> planner_objects;
  {
    PerceptionObjectBuilder perception_builder;
    const auto perception_obj = perception_builder.set_id("Agent1")
                                    .set_type(OT_VEHICLE)
                                    .set_timestamp(1.0)
                                    .set_velocity(0.0)
                                    .set_yaw(0)
                                    .set_length_width(1.3, 1.3)
                                    .set_pos(Vec2d(46.886, 9.248))
                                    .set_box_center(Vec2d(46.886, 9.248))
                                    .Build();

    PlannerObjectBuilder builder;
    builder.set_type(OT_VEHICLE)
        .set_object(perception_obj)
        .set_stationary(true)
        .get_object_prediction_builder()
        ->add_predicted_trajectory()
        ->set_stationary_traj(Vec2dFromProto(perception_obj.pos()),
                              perception_obj.yaw())
        .set_probability(0.5);

    PlannerObject object = builder.Build();
    planner_objects.push_back(std::move(object));
  }
  const SpacetimeTrajectoryManager st_traj_mgr(planner_objects);
  DrawObject("freespace_planner/st_traj", st_traj_mgr);

  constexpr int kCurrentIndex = 0;
  const auto paths =
      GenerateDirectionalPath(vehicle_geom, vehicle_drive_params);
  const FreespaceMap freespace_map;
  PoseProto ego_pose;
  const auto iter = paths.rbegin()->path().rbegin();
  ego_pose.mutable_pos_smooth()->set_x(iter->x());
  ego_pose.mutable_pos_smooth()->set_y(iter->y());
  ego_pose.set_yaw(iter->theta());
  const auto res = PathSafetyCheck(
      vehicle_geom,
      planner_params.freespace_params_for_parking().path_finder_params(),
      planner_params.vehicle_models_params()
          .freespace_vehicle_octagon_model_params(),
      ego_pose, st_traj_mgr.stationary_object_trajs(), freespace_map, paths,
      kCurrentIndex);

  EXPECT_TRUE(!res.ok());
}

TEST(FreespacePlannerUtilTest, UpdatePathManagerState) {
  const PlannerParamsProto planner_params = DefaultPlannerParams();
  const VehicleGeometryParamsProto vehicle_geo_params =
      DefaultVehicleGeometry();
  const VehicleDriveParamsProto vehicle_drive_params =
      DefaultVehicleDriveParams();

  std::vector<Vec2d> boundary1 = {Vec2d(20.0, 3.5),  Vec2d(13.0, 3.5),
                                  Vec2d(13.0, 10.0), Vec2d(10.0, 10.0),
                                  Vec2d(10.0, 3.5),  Vec2d(-2.0, 3.5)};
  std::vector<FreespaceBoundary> boundaries;
  boundaries.reserve(boundary1.size() - 1);
  for (int i = 0; i + 1 < boundary1.size(); ++i) {
    boundaries.push_back({.id = "b" + std::to_string(i),
                          .type = FreespaceMapProto::PARKING_SPOT,
                          .points = {boundary1[i], boundary1[i + 1]}});
  }
  FreespaceMap freespace_map = {.region = AABox2d(11.0, 9.0, Vec2d(9.0, 3.0)),
                                .boundaries = boundaries};
  std::vector<PlannerObject> objects;
  std::vector<const SpacetimeObjectTrajectory*> stalled_object_trajs;

  PathPoint start;
  start.set_x(0.0);
  start.set_y(0.0);
  start.set_theta(0.0);

  PathPoint end;
  end.set_x(11.5);
  end.set_y(8.0);
  end.set_theta(-M_PI * 0.5);

  PathFinderDebugProto path_finder_debug_info;
  const auto coarse_path_status = FindPath(
      planner_params.freespace_params_for_parking().path_finder_params(),
      vehicle_geo_params, vehicle_drive_params,
      planner_params.vehicle_models_params()
          .freespace_vehicle_octagon_model_params(),
      FreespaceTaskProto::PERPENDICULAR_PARKING, freespace_map,
      stalled_object_trajs, start, end, &path_finder_debug_info);
  QCHECK(coarse_path_status.ok());

  PathManagerStateProto state;
  state.set_curr_path_idx(0);
  state.set_drive_state(PathManagerStateProto::DRIVING);
  for (const auto& path : *coarse_path_status) {
    path.ToProto(state.add_paths());
  }

  const FreespaceTaskProto::TaskType task_type =
      FreespaceTaskProto::PERPENDICULAR_PARKING;
  const double steer_percentage_to_front_wheel_angle_rate =
      0.01 * vehicle_drive_params.max_steer_angle() /
      vehicle_drive_params.steer_ratio();
  PoseProto av_pose;
  Chassis chassis;
  bool switched_to_new_path = false;
  // DRIVING to SWITCHING_TO_NEXT.
  av_pose.mutable_pos_smooth()->set_x(
      coarse_path_status->front().path.back().x());
  av_pose.mutable_pos_smooth()->set_y(
      coarse_path_status->front().path.back().y());
  UpdatePathManagerState(vehicle_geo_params, vehicle_drive_params, task_type,
                         av_pose, chassis, &state, &switched_to_new_path);
  EXPECT_TRUE(state.drive_state() == PathManagerStateProto::SWITCHING_TO_NEXT);
  // SWITCHING_TO_NEXT to DRIVING.
  const double target_kappa =
      coarse_path_status->at(1).forward
          ? coarse_path_status->at(1).path.front().kappa()
          : -coarse_path_status->at(1).path.front().kappa();
  chassis.set_steering_percentage(
      std::atan(target_kappa * vehicle_geo_params.wheel_base()) /
      steer_percentage_to_front_wheel_angle_rate);
  chassis.set_gear_location(coarse_path_status->at(1).forward
                                ? Chassis::GEAR_DRIVE
                                : Chassis::GEAR_REVERSE);
  UpdatePathManagerState(vehicle_geo_params, vehicle_drive_params, task_type,
                         av_pose, chassis, &state, &switched_to_new_path);
  EXPECT_TRUE(state.drive_state() == PathManagerStateProto::DRIVING);
  // DRIVING to CENTER_STEER.
  av_pose.mutable_pos_smooth()->set_x(
      coarse_path_status->back().path.back().x());
  av_pose.mutable_pos_smooth()->set_y(
      coarse_path_status->back().path.back().y());
  state.set_curr_path_idx(coarse_path_status->size() - 1);
  chassis.set_steering_percentage(100.0);
  UpdatePathManagerState(vehicle_geo_params, vehicle_drive_params, task_type,
                         av_pose, chassis, &state, &switched_to_new_path);
  EXPECT_TRUE(state.drive_state() == PathManagerStateProto::CENTER_STEER);
  // CENTER_STEER to REACH_FINAL_GOAL.
  chassis.set_steering_percentage(0.0);
  chassis.set_gear_location(Chassis::GEAR_PARKING);
  UpdatePathManagerState(vehicle_geo_params, vehicle_drive_params, task_type,
                         av_pose, chassis, &state, &switched_to_new_path);
  EXPECT_TRUE(state.drive_state() == PathManagerStateProto::REACH_FINAL_GOAL);
  // DRIVING to REACH_FINAL_GOAL.
  state.set_drive_state(PathManagerStateProto::DRIVING);
  UpdatePathManagerState(vehicle_geo_params, vehicle_drive_params, task_type,
                         av_pose, chassis, &state, &switched_to_new_path);
  EXPECT_TRUE(state.drive_state() == PathManagerStateProto::REACH_FINAL_GOAL);
}
}  // namespace
}  // namespace planner
}  // namespace qcraft

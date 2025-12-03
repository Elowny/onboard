#include "onboard/planner/freespace/freespace_planner.h"

#include "absl/time/clock.h"
#include "gflags/gflags.h"

#include "gtest/gtest.h"

#include "onboard/base/macros.h"
#include "onboard/container/strong_int.h"
#include "onboard/maps/semantic_map_defs.h"
#include "onboard/math/geometry/polygon2d.h"
#include "onboard/math/geometry/proto/affine_transformation.pb.h"
#include "onboard/math/vec.h"
#include "onboard/planner/freespace/freespace_constraint_builder.h"
#include "onboard/planner/test_util/load_psmm_util.h"
#include "onboard/planner/test_util/util.h"

#include "offboard/vis/vantage/vantage_server/vantage_canvas_client.h"

DECLARE_bool(send_hybrid_a_star_output_path_to_canvas);
DECLARE_bool(send_geometry_parking_path_to_canvas);
DECLARE_bool(sqp_global_smoother_send_global_smooth_path_to_canvas);

namespace qcraft::planner {
namespace {

const PlannerParamsProto kPlannerParams = DefaultPlannerParams();
const FreespaceParamsProto kFreespaceParams =
    kPlannerParams.freespace_params_for_parking();
const VehicleGeometryParamsProto kVehGeoParams = DefaultVehicleGeometry();
const VehicleDriveParamsProto kVehDriveParams =
    planner::DefaultVehicleDriveParams();

inline PathPoint GetGoalFromParkingSpot(
    const mapping::ParkingSpotInfo& parking_spot_info,
    const VehicleGeometryParamsProto& veh_geo_params) {
  PathPoint path_point_goal;
  const Vec2d goal_tangent = parking_spot_info.unit_direction();
  const double offset =
      veh_geo_params.front_edge_to_center() - veh_geo_params.length() * 0.5;
  Vec2d goal_pos =
      parking_spot_info.polygon().centroid() - goal_tangent * offset;
  path_point_goal.set_x(goal_pos.x());
  path_point_goal.set_y(goal_pos.y());
  path_point_goal.set_theta(goal_tangent.Angle());

  return path_point_goal;
}

[[maybe_unused]] inline void EnableCanvaVisualization() {
  FLAGS_send_hybrid_a_star_output_path_to_canvas = true;
  FLAGS_send_geometry_parking_path_to_canvas = true;
  FLAGS_sqp_global_smoother_send_global_smooth_path_to_canvas = true;
}

inline void DisableCanvaVisualization() {
  FLAGS_send_hybrid_a_star_output_path_to_canvas = false;
  FLAGS_send_geometry_parking_path_to_canvas = false;
  FLAGS_sqp_global_smoother_send_global_smooth_path_to_canvas = false;
}

// From scenario dojo.mkz.vertical_parking_spot_1.
TEST(FreespacePlannerTest, TestPerpendicularParking) {
  // Uncomment the following line to visualize result on canvas.
  // EnableCanvaVisualization();

  // Input.
  AutonomyStateProto autonomy_state;
  autonomy_state.set_autonomy_state(AutonomyStateProto::AUTO_DRIVE);
  PoseProto pose;
  pose.mutable_pos_smooth()->set_x(571.4276751302342);
  pose.mutable_pos_smooth()->set_y(21.2580210154242);
  pose.set_yaw(3.131691529657195);
  Chassis chassis;
  const PlannerObjectManager empty_object_mgr;
  const auto& psmm = CreateDojoTestPSMM();
  const auto& cc = psmm.coordinate_converter();
  absl::flat_hash_set<std::string> stalled_object_ids;
  ApolloTrajectoryPointProto plan_start_point;
  plan_start_point.mutable_path_point()->set_x(pose.pos_smooth().x());
  plan_start_point.mutable_path_point()->set_y(pose.pos_smooth().y());
  plan_start_point.mutable_path_point()->set_theta(pose.yaw());
  absl::Time plan_time = absl::Now();
  const mapping::ParkingSpotInfo* parking_spot_info =
      psmm.FindParkingSpotByIdOrNull(mapping::ElementId(4002));
  EXPECT_TRUE(parking_spot_info != nullptr);
  FreespaceTaskProto::TaskType task_type =
      FreespaceTaskProto::PERPENDICULAR_PARKING;
  const auto path_point_goal =
      GetGoalFromParkingSpot(*parking_spot_info, kVehGeoParams);
  const auto freespace_map = ConstructFreespaceMap(
      task_type, kFreespaceParams.path_finder_params().region_half_width(),
      kVehGeoParams, &psmm, pose, parking_spot_info, path_point_goal);
  EXPECT_OK(freespace_map);

  // State. Should set global goal ref here.
  FreespacePlannerStateProto freespace_planner_state;
  freespace_planner_state.set_task_type(task_type);
  freespace_planner_state.mutable_global_goal_ref()->set_source_type(
      GlobalGoalReferenceProto::NONE);
  auto* mutable_ref =
      freespace_planner_state.mutable_global_goal_ref()->mutable_none_ref();
  mutable_ref->mutable_smooth_goal()->mutable_pos()->set_x(path_point_goal.x());
  mutable_ref->mutable_smooth_goal()->mutable_pos()->set_y(path_point_goal.y());
  mutable_ref->mutable_smooth_goal()->set_theta(path_point_goal.theta());

  // Output.
  FreespacePlannerDebugProto freespace_debug;
  vis::vantage::ChartsDataProto charts_data;

  FreespacePlannerInput freespace_input{
      .new_task = true,
      .force_stop = false,
      .autonomy_state = &autonomy_state,
      .ego_pose = &pose,
      .coordinate_converter = &cc,
      .chassis = &chassis,
      .obj_mgr = &empty_object_mgr,
      .psmm = &psmm,
      .stalled_object_ids = &stalled_object_ids,
      .plan_start_point = &plan_start_point,
      .start_point_reset = false,
      .reset_reason = ResetReasonProto::NONE,
      .plan_time = plan_time,
      .freespace_map = &*freespace_map,
      .parking_spot_info = parking_spot_info,
      .freespace_params = &kFreespaceParams,
      .vehicle_models_params = &kPlannerParams.vehicle_models_params(),
      .veh_geo_params = &kVehGeoParams,
      .veh_drive_params = &kVehDriveParams};
  const auto freespace_planner_output = RunFreespacePlanner(
      freespace_input, &freespace_planner_state, &freespace_debug, &charts_data,
      /*thread_pool=*/nullptr);

  EXPECT_OK(freespace_planner_output);

  vis::vantage::GetCanvasClient()->FlushAll();
  DisableCanvaVisualization();
}

// From scenario dojo.mkz.parallel_parking_3.
TEST(FreespacePlannerTest, TestParallelParking) {
  // Uncomment the following line to visualize result on canvas.
  // EnableCanvaVisualization();

  // Input.
  AutonomyStateProto autonomy_state;
  autonomy_state.set_autonomy_state(AutonomyStateProto::AUTO_DRIVE);
  PoseProto pose;
  pose.mutable_pos_smooth()->set_x(595.2);
  pose.mutable_pos_smooth()->set_y(-9.0);
  pose.set_yaw(1.57);
  Chassis chassis;
  const PlannerObjectManager empty_object_mgr;
  const auto& psmm = CreateDojoTestPSMM();
  const auto& cc = psmm.coordinate_converter();
  absl::flat_hash_set<std::string> stalled_object_ids;
  ApolloTrajectoryPointProto plan_start_point;
  plan_start_point.mutable_path_point()->set_x(pose.pos_smooth().x());
  plan_start_point.mutable_path_point()->set_y(pose.pos_smooth().y());
  plan_start_point.mutable_path_point()->set_theta(pose.yaw());
  absl::Time plan_time = absl::Now();
  const mapping::ParkingSpotInfo* parking_spot_info =
      psmm.FindParkingSpotByIdOrNull(mapping::ElementId(13317));
  EXPECT_TRUE(parking_spot_info != nullptr);
  FreespaceTaskProto::TaskType task_type = FreespaceTaskProto::PARALLEL_PARKING;
  const auto path_point_goal =
      GetGoalFromParkingSpot(*parking_spot_info, kVehGeoParams);
  const auto freespace_map = ConstructFreespaceMap(
      task_type, kFreespaceParams.path_finder_params().region_half_width(),
      kVehGeoParams, &psmm, pose, parking_spot_info, path_point_goal);
  EXPECT_OK(freespace_map);

  // State. Should set global goal ref here.
  FreespacePlannerStateProto freespace_planner_state;
  freespace_planner_state.set_task_type(task_type);
  freespace_planner_state.mutable_global_goal_ref()->set_source_type(
      GlobalGoalReferenceProto::NONE);
  auto* mutable_ref =
      freespace_planner_state.mutable_global_goal_ref()->mutable_none_ref();
  mutable_ref->mutable_smooth_goal()->mutable_pos()->set_x(path_point_goal.x());
  mutable_ref->mutable_smooth_goal()->mutable_pos()->set_y(path_point_goal.y());
  mutable_ref->mutable_smooth_goal()->set_theta(path_point_goal.theta());

  // Output.
  FreespacePlannerDebugProto freespace_debug;
  vis::vantage::ChartsDataProto charts_data;

  FreespacePlannerInput freespace_input{
      .new_task = true,
      .force_stop = false,
      .autonomy_state = &autonomy_state,
      .ego_pose = &pose,
      .coordinate_converter = &cc,
      .chassis = &chassis,
      .obj_mgr = &empty_object_mgr,
      .psmm = &psmm,
      .stalled_object_ids = &stalled_object_ids,
      .plan_start_point = &plan_start_point,
      .start_point_reset = false,
      .reset_reason = ResetReasonProto::NONE,
      .plan_time = plan_time,
      .freespace_map = &*freespace_map,
      .parking_spot_info = parking_spot_info,
      .freespace_params = &kFreespaceParams,
      .vehicle_models_params = &kPlannerParams.vehicle_models_params(),
      .veh_geo_params = &kVehGeoParams,
      .veh_drive_params = &kVehDriveParams};
  const auto freespace_planner_output = RunFreespacePlanner(
      freespace_input, &freespace_planner_state, &freespace_debug, &charts_data,
      /*thread_pool=*/nullptr);

  EXPECT_OK(freespace_planner_output);

  vis::vantage::GetCanvasClient()->FlushAll();
  DisableCanvaVisualization();
}

}  // namespace
}  // namespace qcraft::planner

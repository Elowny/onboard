#include "onboard/planner/planner_params_builder.h"

#include <memory>
#include <vector>

#include "google/protobuf/repeated_ptr_field.h"

#include "gtest/gtest.h"

#include "onboard/base/macros.h"
#include "onboard/math/geometry/proto/aabox3d.pb.h"
#include "onboard/math/geometry/segment2d.h"
#include "onboard/math/vec.h"
#include "onboard/params/param_manager.h"
#include "onboard/params/vehicle_param_api.h"
#include "onboard/planner/common/vehicle_octagon_model.h"
#include "onboard/planner/planner_flags.h"
#include "onboard/planner/test_util/util.h"
#include "onboard/vis/canvas/canvas.h"
#include "onboard/vis/common/color.h"

#include "offboard/vis/vantage/vantage_server/vantage_canvas_client.h"

namespace qcraft::planner {

namespace {

TEST(PlannerParamBuilder, LoadTest) {
  const auto geo_param = DefaultVehicleGeometry();

  // IPC/Orin
  EXPECT_OK(BuildPlannerParams(geo_param, VEHICLE_LINCOLN_MKZ,
                               VehicleInstallationProto::VP_DBQ_V3));
  EXPECT_OK(BuildPlannerParams(geo_param, VEHICLE_JINLV_MINIBUS,
                               VehicleInstallationProto::VP_DBQ_V3));
  EXPECT_OK(BuildPlannerParams(geo_param, VEHICLE_ZHONGTONG,
                               VehicleInstallationProto::VP_DBQ_V3));
  // VP_PBQ_V2 (vision only) is not supported for IPC/Orin yet.
  EXPECT_NOT_OK(BuildPlannerParams(geo_param, VEHICLE_LINCOLN_MKZ,
                                   VehicleInstallationProto::VP_PBQ_V2));
  EXPECT_NOT_OK(BuildPlannerParams(geo_param, VEHICLE_JINLV_MINIBUS,
                                   VehicleInstallationProto::VP_PBQ_V2));
  EXPECT_NOT_OK(BuildPlannerParams(geo_param, VEHICLE_ZHONGTONG,
                                   VehicleInstallationProto::VP_PBQ_V2));

  // X9
}

TEST(PlannerParamBuilder, X9LoadTest) {
  // Set X9 platform.
  FLAGS_planner_running_platform = 2;
  const auto geo_param = DefaultVehicleGeometry();

  // Base test.
  {
    EXPECT_OK(BuildPlannerParams(geo_param, VEHICLE_LINCOLN_MKZ,
                                 VehicleInstallationProto::VP_DBQ_V3));
    EXPECT_NOT_OK(BuildPlannerParams(geo_param, VEHICLE_JINLV_MINIBUS,
                                     VehicleInstallationProto::VP_DBQ_V3));
    EXPECT_OK(BuildPlannerParams(geo_param, VEHICLE_LINCOLN_MKZ,
                                 VehicleInstallationProto::VP_PBQ_V2));
  }

  // Reset IPC platform.
  FLAGS_planner_running_platform = 0;
}

TEST(PlannerParamBuilder, S32gLoadTest) {
  // Set s32g platform.
  FLAGS_planner_running_platform = 3;
  const auto geo_param = DefaultVehicleGeometry();

  // Base test.
  {
    EXPECT_OK(BuildPlannerParams(geo_param, VEHICLE_LINCOLN_MKZ,
                                 VehicleInstallationProto::VP_DBQ_V3));
    EXPECT_NOT_OK(BuildPlannerParams(geo_param, VEHICLE_JINLV_MINIBUS,
                                     VehicleInstallationProto::VP_DBQ_V3));
    EXPECT_OK(BuildPlannerParams(geo_param, VEHICLE_LINCOLN_MKZ,
                                 VehicleInstallationProto::VP_PBQ_V2));
  }

  // Reset IPC platform.
  FLAGS_planner_running_platform = 0;
}

// How to use: bazel run onboard/planner/planner_params_builder_test,  and all
// models are shown on canvas.
TEST(PlannerParamBuilder, ShowPlannerVehicleModel) {
  // Vehicle ID and vehicle type must be consistent!
  auto param_manager = qcraft::CreateParamManagerFromCarId("Q8001");
  RunParamsProtoV2 run_params;
  param_manager->GetRunParams(&run_params);

  const auto& veh_geo_params =
      run_params.vehicle_params().vehicle_geometry_params();
  const auto planner_params_status =
      BuildPlannerParams(veh_geo_params, VEHICLE_JINLV_MINIBUS,
                         VehicleInstallationProto::VP_DBQ_V3);

  EXPECT_TRUE(planner_params_status.ok());

  const auto& vehicle_octagon_models =
      planner_params_status->vehicle_models_params()
          .freespace_vehicle_octagon_model_params();
  const auto& freespace_vehicle_circle_models =
      planner_params_status->vehicle_models_params()
          .freespace_local_smoother_vehicle_model_params();
  const auto& optimizer_vehicle_circle_models =
      planner_params_status->vehicle_models_params()
          .trajectory_optimizer_vehicle_model_params();

  // Draw vehicle
  vis::Canvas& canvas1 =
      vis::vantage::GetCanvasClient()->GetCanvas("origin_model");
  canvas1.DrawLine(Vec3d(-veh_geo_params.back_edge_to_center(),
                         0.5 * veh_geo_params.width(), 2.0),
                   Vec3d(-veh_geo_params.back_edge_to_center(),
                         -0.5 * veh_geo_params.width(), 2.0),
                   vis::Color::kRed);
  canvas1.DrawLine(Vec3d(-veh_geo_params.back_edge_to_center(),
                         -0.5 * veh_geo_params.width(), 2.0),
                   Vec3d(veh_geo_params.front_edge_to_center(),
                         -0.5 * veh_geo_params.width(), 2.0),
                   vis::Color::kRed);
  canvas1.DrawLine(Vec3d(veh_geo_params.front_edge_to_center(),
                         -0.5 * veh_geo_params.width(), 2.0),
                   Vec3d(veh_geo_params.front_edge_to_center(),
                         0.5 * veh_geo_params.width(), 2.0),
                   vis::Color::kRed);
  canvas1.DrawLine(Vec3d(veh_geo_params.front_edge_to_center(),
                         0.5 * veh_geo_params.width(), 2.0),
                   Vec3d(-veh_geo_params.back_edge_to_center(),
                         0.5 * veh_geo_params.width(), 2.0),
                   vis::Color::kRed);
  canvas1.DrawLine(
      Vec3d(veh_geo_params.wheel_base(), 0.5 * veh_geo_params.width(), 2.0),
      Vec3d(veh_geo_params.wheel_base(), -0.5 * veh_geo_params.width(), 2.0),
      vis::Color::kRed);
  canvas1.DrawLine(Vec3d(0.0, 0.5 * veh_geo_params.width(), 2.0),
                   Vec3d(0.0, -0.5 * veh_geo_params.width(), 2.0),
                   vis::Color::kRed);
  canvas1.DrawBox(Vec3d(veh_geo_params.left_mirror().x(),
                        veh_geo_params.left_mirror().y(), 2.0),
                  0.0,
                  Vec2d(veh_geo_params.left_mirror().width(),
                        veh_geo_params.left_mirror().length()),
                  vis::Color::kRed);
  canvas1.DrawBox(Vec3d(veh_geo_params.right_mirror().x(),
                        veh_geo_params.right_mirror().y(), 2.0),
                  0.0,
                  Vec2d(veh_geo_params.right_mirror().width(),
                        veh_geo_params.right_mirror().length()),
                  vis::Color::kRed);

  // Draw freespace circle
  vis::Canvas& canvas2 =
      vis::vantage::GetCanvasClient()->GetCanvas("freespace_circle_model");
  for (int i = 0; i < freespace_vehicle_circle_models.circles().size(); ++i) {
    const auto& param = freespace_vehicle_circle_models.circles()[i];
    canvas2.DrawCircle(Vec3d(Vec2d::FastUnitFromAngle(param.angle_to_axis()) *
                                 param.dist_to_rac(),
                             2.0),
                       param.radius(), vis::Color::kWhite);
  }
  for (int i = 0; i < freespace_vehicle_circle_models.mirror_circles().size();
       ++i) {
    const auto& param = freespace_vehicle_circle_models.mirror_circles()[i];
    canvas2.DrawCircle(Vec3d(Vec2d::FastUnitFromAngle(param.angle_to_axis()) *
                                 param.dist_to_rac(),
                             2.0),
                       param.radius(), vis::Color::kWhite);
  }

  // Draw optimizer circle
  vis::Canvas& canvas3 =
      vis::vantage::GetCanvasClient()->GetCanvas("optimizer_circle_model");
  for (int i = 0; i < optimizer_vehicle_circle_models.circles().size(); ++i) {
    const auto& param = optimizer_vehicle_circle_models.circles()[i];
    canvas3.DrawCircle(Vec3d(Vec2d::FastUnitFromAngle(param.angle_to_axis()) *
                                 param.dist_to_rac(),
                             2.0),
                       param.radius(), vis::Color::kWhite);
  }
  for (int i = 0; i < optimizer_vehicle_circle_models.mirror_circles().size();
       ++i) {
    const auto& param = optimizer_vehicle_circle_models.mirror_circles()[i];
    canvas3.DrawCircle(Vec3d(Vec2d::FastUnitFromAngle(param.angle_to_axis()) *
                                 param.dist_to_rac(),
                             2.0),
                       param.radius(), vis::Color::kWhite);
  }

  // Draw octagon.
  vis::Canvas& canvas4 =
      vis::vantage::GetCanvasClient()->GetCanvas("freespace_octagon_model");
  const double offset = 0.5 * (veh_geo_params.front_edge_to_center() -
                               veh_geo_params.back_edge_to_center());
  const VehicleOctagonModel octagon(
      Vec2d(offset, 0.0), 0.0, veh_geo_params.length(), veh_geo_params.width(),
      vehicle_octagon_models.front_corner_side_length(),
      vehicle_octagon_models.rear_corner_side_length());
  for (const auto& line : octagon.line_segments()) {
    canvas4.DrawLine(Vec3d(line.start(), 2.0), Vec3d(line.end(), 2.0),
                     vis::Color::kGreen);
  }
  canvas4.DrawCircle(Vec3d(vehicle_octagon_models.mirror_offset_x(),
                           vehicle_octagon_models.mirror_offset_y(), 2.0),
                     vehicle_octagon_models.mirror_radius(),
                     vis::Color::kGreen);
  canvas4.DrawCircle(Vec3d(vehicle_octagon_models.mirror_offset_x(),
                           -vehicle_octagon_models.mirror_offset_y(), 2.0),
                     vehicle_octagon_models.mirror_radius(),
                     vis::Color::kGreen);
}

}  // namespace

}  // namespace qcraft::planner

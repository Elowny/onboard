#include "onboard/planner/freespace/geometry_method/perpendicular_parking_out.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <memory>
#include <utility>

#include "gtest/gtest.h"

#include "onboard/math/vec.h"
#include "onboard/params/param_manager.h"
#include "onboard/params/vehicle_param_api.h"
#include "onboard/planner/freespace/geometry_method/geometry_method_util.h"
#include "onboard/planner/test_util/util.h"
#include "onboard/vis/canvas/canvas.h"
#include "onboard/vis/canvas/proto/canvas_buffer.pb.h"
#include "onboard/vis/common/color.h"

#include "offboard/vis/vantage/vantage_server/vantage_client_man.h"

namespace qcraft {
namespace planner {
namespace {

constexpr double kMaxKappa = 0.2;

TEST(PerpendicularParkingOutTest, StraightForward) {
  auto param_manager = qcraft::CreateParamManagerFromCarId("Q0001");
  RunParamsProtoV2 run_params;
  param_manager->GetRunParams(&run_params);
  const PlannerParamsProto planner_params = DefaultPlannerParams();

  std::vector<Segment2d> virtual_boundaries;
  virtual_boundaries.reserve(4);
  virtual_boundaries.emplace_back(Vec2d(-10.0, 10.0), Vec2d(10.0, 10.0));
  virtual_boundaries.emplace_back(Vec2d(-10.0, 4.0), Vec2d(-1.5, 4.0));
  virtual_boundaries.emplace_back(Vec2d(1.5, 4.0), Vec2d(10.0, 4.0));
  virtual_boundaries.emplace_back(Vec2d(-10.0, -2.0), Vec2d(10.0, -2.0));

  GeometryMethodPoint start = {Vec2d(0.0, 0.0), M_PI_2,
                               Vec2d::FastUnitFromAngle(M_PI_2)};
  const double park_out_distance = 3.5;

  std::vector<std::pair<std::string, Segment2d>> named_segments;
  absl::flat_hash_map<std::string, const FreespaceObject*> objects_map;
  absl::flat_hash_map<std::string, const FreespaceBoundary*> boundaries_map;
  const SegmentMatcherKdtree segments_kd_tree(named_segments);

  const auto res = FindPerpendicularStraightParkingOutPath(
      run_params.vehicle_params().vehicle_geometry_params(), kMaxKappa,
      planner_params.freespace_params_for_parking().path_finder_params(),
      planner_params.vehicle_models_params()
          .freespace_vehicle_octagon_model_params(),
      segments_kd_tree, objects_map, boundaries_map, virtual_boundaries, start,
      park_out_distance, PARKING_OUT_DIR_PERP_FWD);

  EXPECT_TRUE(res.ok());
  qcraft::vantage_client_man::CreateVantageClientMan();
  vis::Canvas& canvas = vantage_client_man::GetCanvas(
      "geo_parking_test/perpendicular_parking_out_straight_forward");
  SendLineCirclePathToCanvas(
      &canvas, run_params.vehicle_params().vehicle_geometry_params(),
      planner_params.vehicle_models_params()
          .freespace_vehicle_octagon_model_params(),
      *res, /*step=*/0.2);
  for (const auto& virtual_boundary : virtual_boundaries) {
    canvas.DrawLine(Vec3d(virtual_boundary.start()),
                    Vec3d(virtual_boundary.end()), vis::Color::kBlue, 3,
                    vis::BorderStyleProto::SOLID);
  }
  qcraft::vantage_client_man::FlushAll();
}

TEST(PerpendicularParkingOutTest, FrontLeft) {
  auto param_manager = qcraft::CreateParamManagerFromCarId("Q0001");
  RunParamsProtoV2 run_params;
  param_manager->GetRunParams(&run_params);
  const PlannerParamsProto planner_params = DefaultPlannerParams();

  std::vector<Segment2d> virtual_boundaries;
  virtual_boundaries.reserve(4);
  virtual_boundaries.emplace_back(Vec2d(-10.0, 10.0), Vec2d(10.0, 10.0));
  virtual_boundaries.emplace_back(Vec2d(-10.0, 4.0), Vec2d(-1.5, 4.0));
  virtual_boundaries.emplace_back(Vec2d(1.5, 4.0), Vec2d(10.0, 4.0));
  virtual_boundaries.emplace_back(Vec2d(-10.0, -2.0), Vec2d(10.0, -2.0));

  GeometryMethodPoint start = {Vec2d(0.0, 0.0), M_PI_2,
                               Vec2d::FastUnitFromAngle(M_PI_2)};
  const Box2d target_box = Box2d(Vec2d(-5.0, 7.0), M_PI, 1.0, 1.0);

  std::vector<std::pair<std::string, Segment2d>> named_segments;
  absl::flat_hash_map<std::string, const FreespaceObject*> objects_map;
  absl::flat_hash_map<std::string, const FreespaceBoundary*> boundaries_map;
  const SegmentMatcherKdtree segments_kd_tree(named_segments);

  const auto res = FindPerpendicularTurningParkingOutPath(
      run_params.vehicle_params().vehicle_geometry_params(), kMaxKappa,
      planner_params.freespace_params_for_parking().path_finder_params(),
      planner_params.vehicle_models_params()
          .freespace_vehicle_octagon_model_params(),
      segments_kd_tree, objects_map, boundaries_map, virtual_boundaries, start,
      target_box, PARKING_OUT_DIR_PERP_LEFT_FWD);

  EXPECT_TRUE(res.ok());
  qcraft::vantage_client_man::CreateVantageClientMan();
  vis::Canvas& canvas1 = vantage_client_man::GetCanvas(
      "geo_parking_test/perpendicular_parking_out_front_left1");
  SendLineCirclePathToCanvas(
      &canvas1, run_params.vehicle_params().vehicle_geometry_params(),
      planner_params.vehicle_models_params()
          .freespace_vehicle_octagon_model_params(),
      *res, /*step=*/0.2);
  for (const auto& virtual_boundary : virtual_boundaries) {
    canvas1.DrawLine(Vec3d(virtual_boundary.start()),
                     Vec3d(virtual_boundary.end()), vis::Color::kBlue, 3,
                     vis::BorderStyleProto::SOLID);
  }
  for (const auto& edge : target_box.GetEdgesCounterClockwise()) {
    canvas1.DrawLine(Vec3d(edge.start()), Vec3d(edge.end()), vis::Color::kGreen,
                     3, vis::BorderStyleProto::DASHED);
  }

  const Box2d target_box2 = Box2d(Vec2d(-5.0, 1.0), M_PI, 1.0, 20.0);
  const auto res2 = FindPerpendicularTurningParkingOutPath(
      run_params.vehicle_params().vehicle_geometry_params(), kMaxKappa,
      planner_params.freespace_params_for_parking().path_finder_params(),
      planner_params.vehicle_models_params()
          .freespace_vehicle_octagon_model_params(),
      segments_kd_tree, objects_map, boundaries_map, virtual_boundaries, start,
      target_box2, PARKING_OUT_DIR_PERP_LEFT_FWD);

  EXPECT_TRUE(res2.ok());
  qcraft::vantage_client_man::CreateVantageClientMan();
  vis::Canvas& canvas2 = vantage_client_man::GetCanvas(
      "geo_parking_test/perpendicular_parking_out_front_left2");
  SendLineCirclePathToCanvas(
      &canvas2, run_params.vehicle_params().vehicle_geometry_params(),
      planner_params.vehicle_models_params()
          .freespace_vehicle_octagon_model_params(),
      *res2, /*step=*/0.2);
  for (const auto& virtual_boundary : virtual_boundaries) {
    canvas2.DrawLine(Vec3d(virtual_boundary.start()),
                     Vec3d(virtual_boundary.end()), vis::Color::kBlue, 3,
                     vis::BorderStyleProto::SOLID);
  }
  for (const auto& edge : target_box2.GetEdgesCounterClockwise()) {
    canvas2.DrawLine(Vec3d(edge.start()), Vec3d(edge.end()), vis::Color::kGreen,
                     3, vis::BorderStyleProto::DASHED);
  }
  qcraft::vantage_client_man::FlushAll();

  const Box2d target_box3 = Box2d(Vec2d(-5.0, 1.0), M_PI, 1.0, 1.0);
  const auto res3 = FindPerpendicularTurningParkingOutPath(
      run_params.vehicle_params().vehicle_geometry_params(), kMaxKappa,
      planner_params.freespace_params_for_parking().path_finder_params(),
      planner_params.vehicle_models_params()
          .freespace_vehicle_octagon_model_params(),
      segments_kd_tree, objects_map, boundaries_map, virtual_boundaries, start,
      target_box3, PARKING_OUT_DIR_PERP_LEFT_FWD);

  EXPECT_FALSE(res3.ok());
}

TEST(PerpendicularParkingOutTest, FrontRight) {
  auto param_manager = qcraft::CreateParamManagerFromCarId("Q0001");
  RunParamsProtoV2 run_params;
  param_manager->GetRunParams(&run_params);
  const PlannerParamsProto planner_params = DefaultPlannerParams();

  std::vector<Segment2d> virtual_boundaries;
  virtual_boundaries.reserve(4);
  virtual_boundaries.emplace_back(Vec2d(-10.0, 10.0), Vec2d(10.0, 10.0));
  virtual_boundaries.emplace_back(Vec2d(-10.0, 4.0), Vec2d(-1.5, 4.0));
  virtual_boundaries.emplace_back(Vec2d(1.5, 4.0), Vec2d(10.0, 4.0));
  virtual_boundaries.emplace_back(Vec2d(-10.0, -2.0), Vec2d(10.0, -2.0));

  GeometryMethodPoint start = {Vec2d(0.0, 0.0), M_PI_2,
                               Vec2d::FastUnitFromAngle(M_PI_2)};
  const Box2d target_box = Box2d(Vec2d(5.0, 7.0), 0.0, 1.0, 1.0);

  std::vector<std::pair<std::string, Segment2d>> named_segments;
  absl::flat_hash_map<std::string, const FreespaceObject*> objects_map;
  absl::flat_hash_map<std::string, const FreespaceBoundary*> boundaries_map;
  const SegmentMatcherKdtree segments_kd_tree(named_segments);

  const auto res = FindPerpendicularTurningParkingOutPath(
      run_params.vehicle_params().vehicle_geometry_params(), kMaxKappa,
      planner_params.freespace_params_for_parking().path_finder_params(),
      planner_params.vehicle_models_params()
          .freespace_vehicle_octagon_model_params(),
      segments_kd_tree, objects_map, boundaries_map, virtual_boundaries, start,
      target_box, PARKING_OUT_DIR_PERP_RIGHT_FWD);

  EXPECT_TRUE(res.ok());
  qcraft::vantage_client_man::CreateVantageClientMan();
  vis::Canvas& canvas = vantage_client_man::GetCanvas(
      "geo_parking_test/perpendicular_parking_out_front_right");
  SendLineCirclePathToCanvas(
      &canvas, run_params.vehicle_params().vehicle_geometry_params(),
      planner_params.vehicle_models_params()
          .freespace_vehicle_octagon_model_params(),
      *res, /*step=*/0.2);
  for (const auto& virtual_boundary : virtual_boundaries) {
    canvas.DrawLine(Vec3d(virtual_boundary.start()),
                    Vec3d(virtual_boundary.end()), vis::Color::kBlue, 3,
                    vis::BorderStyleProto::SOLID);
  }
  for (const auto& edge : target_box.GetEdgesCounterClockwise()) {
    canvas.DrawLine(Vec3d(edge.start()), Vec3d(edge.end()), vis::Color::kGreen,
                    3, vis::BorderStyleProto::DASHED);
  }
  qcraft::vantage_client_man::FlushAll();
}

TEST(PerpendicularParkingOutTest, StraightBack) {
  auto param_manager = qcraft::CreateParamManagerFromCarId("Q0001");
  RunParamsProtoV2 run_params;
  param_manager->GetRunParams(&run_params);
  const PlannerParamsProto planner_params = DefaultPlannerParams();

  std::vector<Segment2d> virtual_boundaries;
  virtual_boundaries.reserve(4);
  virtual_boundaries.emplace_back(Vec2d(-10.0, 5.0), Vec2d(10.0, 5.0));
  virtual_boundaries.emplace_back(Vec2d(-10.0, -1.0), Vec2d(-1.5, -1.0));
  virtual_boundaries.emplace_back(Vec2d(1.5, -1.0), Vec2d(10.0, -1.0));
  virtual_boundaries.emplace_back(Vec2d(-10.0, -7.0), Vec2d(10.0, -7.0));

  GeometryMethodPoint start = {Vec2d(0.0, 0.0), M_PI_2,
                               Vec2d::FastUnitFromAngle(M_PI_2)};
  const double park_out_distance = 3.5;

  std::vector<std::pair<std::string, Segment2d>> named_segments;
  absl::flat_hash_map<std::string, const FreespaceObject*> objects_map;
  absl::flat_hash_map<std::string, const FreespaceBoundary*> boundaries_map;
  const SegmentMatcherKdtree segments_kd_tree(named_segments);

  const auto res = FindPerpendicularStraightParkingOutPath(
      run_params.vehicle_params().vehicle_geometry_params(), kMaxKappa,
      planner_params.freespace_params_for_parking().path_finder_params(),
      planner_params.vehicle_models_params()
          .freespace_vehicle_octagon_model_params(),
      segments_kd_tree, objects_map, boundaries_map, virtual_boundaries, start,
      park_out_distance, PARKING_OUT_DIR_PERP_BACK);

  EXPECT_TRUE(res.ok());
  qcraft::vantage_client_man::CreateVantageClientMan();
  vis::Canvas& canvas = vantage_client_man::GetCanvas(
      "geo_parking_test/perpendicular_parking_out_straight_back");
  SendLineCirclePathToCanvas(
      &canvas, run_params.vehicle_params().vehicle_geometry_params(),
      planner_params.vehicle_models_params()
          .freespace_vehicle_octagon_model_params(),
      *res, /*step=*/0.2);
  for (const auto& virtual_boundary : virtual_boundaries) {
    canvas.DrawLine(Vec3d(virtual_boundary.start()),
                    Vec3d(virtual_boundary.end()), vis::Color::kBlue, 3,
                    vis::BorderStyleProto::SOLID);
  }
  qcraft::vantage_client_man::FlushAll();
}

TEST(PerpendicularParkingOutTest, BackLeft) {
  auto param_manager = qcraft::CreateParamManagerFromCarId("Q0001");
  RunParamsProtoV2 run_params;
  param_manager->GetRunParams(&run_params);
  const PlannerParamsProto planner_params = DefaultPlannerParams();

  std::vector<Segment2d> virtual_boundaries;
  virtual_boundaries.reserve(4);
  virtual_boundaries.emplace_back(Vec2d(-10.0, 5.0), Vec2d(10.0, 5.0));
  virtual_boundaries.emplace_back(Vec2d(-10.0, -1.0), Vec2d(-1.5, -1.0));
  virtual_boundaries.emplace_back(Vec2d(1.5, -1.0), Vec2d(10.0, -1.0));
  virtual_boundaries.emplace_back(Vec2d(-10.0, -10.0), Vec2d(10.0, -10.0));

  GeometryMethodPoint start = {Vec2d(0.0, 0.0), M_PI_2,
                               Vec2d::FastUnitFromAngle(M_PI_2)};
  const Box2d target_box = Box2d(Vec2d(-5.0, -4.0), M_PI, 1.0, 1.0);

  std::vector<std::pair<std::string, Segment2d>> named_segments;
  absl::flat_hash_map<std::string, const FreespaceObject*> objects_map;
  absl::flat_hash_map<std::string, const FreespaceBoundary*> boundaries_map;
  const SegmentMatcherKdtree segments_kd_tree(named_segments);

  const auto res = FindPerpendicularTurningParkingOutPath(
      run_params.vehicle_params().vehicle_geometry_params(), kMaxKappa,
      planner_params.freespace_params_for_parking().path_finder_params(),
      planner_params.vehicle_models_params()
          .freespace_vehicle_octagon_model_params(),
      segments_kd_tree, objects_map, boundaries_map, virtual_boundaries, start,
      target_box, PARKING_OUT_DIR_PERP_LEFT_BACK);

  EXPECT_TRUE(res.ok());
  qcraft::vantage_client_man::CreateVantageClientMan();
  vis::Canvas& canvas = vantage_client_man::GetCanvas(
      "geo_parking_test/perpendicular_parking_out_back_left");
  SendLineCirclePathToCanvas(
      &canvas, run_params.vehicle_params().vehicle_geometry_params(),
      planner_params.vehicle_models_params()
          .freespace_vehicle_octagon_model_params(),
      *res, /*step=*/0.2);
  for (const auto& virtual_boundary : virtual_boundaries) {
    canvas.DrawLine(Vec3d(virtual_boundary.start()),
                    Vec3d(virtual_boundary.end()), vis::Color::kBlue, 3,
                    vis::BorderStyleProto::SOLID);
  }
  for (const auto& edge : target_box.GetEdgesCounterClockwise()) {
    canvas.DrawLine(Vec3d(edge.start()), Vec3d(edge.end()), vis::Color::kGreen,
                    3, vis::BorderStyleProto::DASHED);
  }
  qcraft::vantage_client_man::FlushAll();
}

TEST(PerpendicularParkingOutTest, BackRight) {
  auto param_manager = qcraft::CreateParamManagerFromCarId("Q0001");
  RunParamsProtoV2 run_params;
  param_manager->GetRunParams(&run_params);
  const PlannerParamsProto planner_params = DefaultPlannerParams();

  std::vector<Segment2d> virtual_boundaries;
  virtual_boundaries.reserve(4);
  virtual_boundaries.emplace_back(Vec2d(-10.0, 5.0), Vec2d(10.0, 5.0));
  virtual_boundaries.emplace_back(Vec2d(-10.0, -1.0), Vec2d(-1.5, -1.0));
  virtual_boundaries.emplace_back(Vec2d(1.5, -1.0), Vec2d(10.0, -1.0));
  virtual_boundaries.emplace_back(Vec2d(-10.0, -10.0), Vec2d(10.0, -10.0));

  GeometryMethodPoint start = {Vec2d(0.0, 0.0), M_PI_2,
                               Vec2d::FastUnitFromAngle(M_PI_2)};
  const Box2d target_box = Box2d(Vec2d(5.0, -4.0), 0.0, 1.0, 1.0);

  std::vector<std::pair<std::string, Segment2d>> named_segments;
  absl::flat_hash_map<std::string, const FreespaceObject*> objects_map;
  absl::flat_hash_map<std::string, const FreespaceBoundary*> boundaries_map;
  const SegmentMatcherKdtree segments_kd_tree(named_segments);

  const auto res = FindPerpendicularTurningParkingOutPath(
      run_params.vehicle_params().vehicle_geometry_params(), kMaxKappa,
      planner_params.freespace_params_for_parking().path_finder_params(),
      planner_params.vehicle_models_params()
          .freespace_vehicle_octagon_model_params(),
      segments_kd_tree, objects_map, boundaries_map, virtual_boundaries, start,
      target_box, PARKING_OUT_DIR_PERP_RIGHT_BACK);

  EXPECT_TRUE(res.ok());
  qcraft::vantage_client_man::CreateVantageClientMan();
  vis::Canvas& canvas = vantage_client_man::GetCanvas(
      "geo_parking_test/perpendicular_parking_out_back_right");
  SendLineCirclePathToCanvas(
      &canvas, run_params.vehicle_params().vehicle_geometry_params(),
      planner_params.vehicle_models_params()
          .freespace_vehicle_octagon_model_params(),
      *res, /*step=*/0.2);
  for (const auto& virtual_boundary : virtual_boundaries) {
    canvas.DrawLine(Vec3d(virtual_boundary.start()),
                    Vec3d(virtual_boundary.end()), vis::Color::kBlue, 3,
                    vis::BorderStyleProto::SOLID);
  }
  for (const auto& edge : target_box.GetEdgesCounterClockwise()) {
    canvas.DrawLine(Vec3d(edge.start()), Vec3d(edge.end()), vis::Color::kGreen,
                    3, vis::BorderStyleProto::DASHED);
  }
  qcraft::vantage_client_man::FlushAll();
}

}  // namespace
}  // namespace planner
}  // namespace qcraft

#include "onboard/planner/freespace/geometry_method/geometry_path_collision_checker.h"

#include <memory>

#include "gtest/gtest.h"

#include "onboard/params/param_manager.h"
#include "onboard/params/vehicle_param_api.h"
#include "onboard/planner/freespace/geometry_method/geometry_method_util.h"
#include "onboard/planner/test_util/util.h"

namespace qcraft {
namespace planner {
namespace {

constexpr double kMaxKappa = 0.2;

TEST(LinePathCollisionChecker, HasOverlapWithPointTest) {
  auto param_manager = qcraft::CreateParamManagerFromCarId("Q0001");
  RunParamsProtoV2 run_params;
  param_manager->GetRunParams(&run_params);
  const PlannerParamsProto planner_params = DefaultPlannerParams();

  const GeometryMethodPoint start = {Vec2d(0.0, 0.0), 0.0,
                                     Vec2d::FastUnitFromAngle(0.0)};
  const GeometryPathType type = GeometryPathType::STRAIGHT;
  const double length = 20.0;
  const auto end = ExtendPathByConstantKappa(start, kMaxKappa, length, type);

  LinePathCollisionChecker checker(
      run_params.vehicle_params().vehicle_geometry_params(),
      planner_params.vehicle_models_params()
          .freespace_vehicle_octagon_model_params(),
      start, end, type, length, kMaxKappa);
  EXPECT_EQ(checker.type(), GeometryPathType::STRAIGHT);
  EXPECT_NEAR(checker.kappa(), kMaxKappa, 1.0e-6);

  // Lateral_buffer=0.0, longitudinal_buffer=0.0, consider_mirror=false.
  ASSERT_FALSE(
      checker.HasOverlapWithBuffer(Vec2d(-0.927, 0.838), 0.0, 0.0, false));
  ASSERT_FALSE(
      checker.HasOverlapWithBuffer(Vec2d(13.379, -0.969), 0.0, 0.0, false));
  ASSERT_FALSE(
      checker.HasOverlapWithBuffer(Vec2d(23.777, -0.826), 0.0, 0.0, false));
  ASSERT_TRUE(
      checker.HasOverlapWithBuffer(Vec2d(-0.992, -0.692), 0.0, 0.0, false));
  ASSERT_TRUE(
      checker.HasOverlapWithBuffer(Vec2d(12.260, 0.154), 0.0, 0.0, false));
}

TEST(LinePathCollisionChecker, HasOverlapWithSegmentTest) {
  auto param_manager = qcraft::CreateParamManagerFromCarId("Q0001");
  RunParamsProtoV2 run_params;
  param_manager->GetRunParams(&run_params);
  const PlannerParamsProto planner_params = DefaultPlannerParams();

  const GeometryMethodPoint start = {Vec2d(0.0, 0.0), 0.0,
                                     Vec2d::FastUnitFromAngle(0.0)};
  const GeometryPathType type = GeometryPathType::STRAIGHT;
  const double length = 20.0;
  const auto end = ExtendPathByConstantKappa(start, kMaxKappa, length, type);

  LinePathCollisionChecker checker(
      run_params.vehicle_params().vehicle_geometry_params(),
      planner_params.vehicle_models_params()
          .freespace_vehicle_octagon_model_params(),
      start, end, type, length, kMaxKappa);
  EXPECT_EQ(checker.type(), GeometryPathType::STRAIGHT);
  EXPECT_NEAR(checker.kappa(), kMaxKappa, 1.0e-6);

  // Lateral_buffer=0.0, longitudinal_buffer=0.0, consider_mirror=false.
  ASSERT_TRUE(checker.HasOverlapWithBuffer(
      Segment2d(Vec2d(10.0, 0.5), Vec2d(10.0, -0.5)), 0.0, 0.0, false));
  ASSERT_FALSE(checker.HasOverlapWithBuffer(
      Segment2d(Vec2d(1.0, 1.2), Vec2d(2.0, 1.2)), 0.0, 0.0, false));
}

TEST(CirclePathCollisionChecker, HasOverlapWithPointTest) {
  auto param_manager = qcraft::CreateParamManagerFromCarId("Q0001");
  RunParamsProtoV2 run_params;
  param_manager->GetRunParams(&run_params);
  const PlannerParamsProto planner_params = DefaultPlannerParams();

  const GeometryMethodPoint start = {Vec2d(1.0, 2.0), 1.0,
                                     Vec2d::FastUnitFromAngle(1.0)};
  const GeometryPathType type = GeometryPathType::RIGHT;
  const double length = 6.0;
  const auto end = ExtendPathByConstantKappa(start, kMaxKappa, length, type);

  CirclePathCollisionChecker checker(
      run_params.vehicle_params().vehicle_geometry_params(),
      planner_params.vehicle_models_params()
          .freespace_vehicle_octagon_model_params(),
      start, end, type, length, kMaxKappa, /*use_fast_overlap=*/true);

  // Lateral_buffer=0.0, longitudinal_buffer=0.0, consider_mirror=false.
  ASSERT_FALSE(
      checker.HasOverlapWithBuffer(Vec2d(1.243, 0.745), 0.0, 0.0, false));
  ASSERT_FALSE(
      checker.HasOverlapWithBuffer(Vec2d(0.379, 0.975), 0.0, 0.0, false));
  ASSERT_FALSE(
      checker.HasOverlapWithBuffer(Vec2d(5.076, 3.347), 0.0, 0.0, false));
  ASSERT_FALSE(
      checker.HasOverlapWithBuffer(Vec2d(10.070, 3.460), 0.0, 0.0, false));
  ASSERT_TRUE(
      checker.HasOverlapWithBuffer(Vec2d(5.015, 6.203), 0.0, 0.0, false));
  ASSERT_TRUE(
      checker.HasOverlapWithBuffer(Vec2d(3.571, 3.912), 0.0, 0.0, false));
}

TEST(CirclePathCollisionChecker, HasOverlapWithSegmentTest) {
  auto param_manager = qcraft::CreateParamManagerFromCarId("Q0001");
  RunParamsProtoV2 run_params;
  param_manager->GetRunParams(&run_params);
  const PlannerParamsProto planner_params = DefaultPlannerParams();

  const GeometryMethodPoint start = {Vec2d(1.0, 2.0), 1.0,
                                     Vec2d::FastUnitFromAngle(1.0)};
  const GeometryPathType type = GeometryPathType::LEFT;
  const double length = -5.0;
  const auto end = ExtendPathByConstantKappa(start, kMaxKappa, length, type);

  CirclePathCollisionChecker checker(
      run_params.vehicle_params().vehicle_geometry_params(),
      planner_params.vehicle_models_params()
          .freespace_vehicle_octagon_model_params(),
      start, end, type, length, kMaxKappa, /*use_fast_overlap=*/true);

  // Lateral_buffer=0.0, longitudinal_buffer=0.0, consider_mirror=false.
  ASSERT_FALSE(checker.HasOverlapWithBuffer(
      Segment2d(Vec2d(1.117, 5.501), Vec2d(3.296, 6.088)), 0.0, 0.0, false));
  ASSERT_FALSE(checker.HasOverlapWithBuffer(
      Segment2d(Vec2d(-5.0, -1.5), Vec2d(-5.0, 0.5)), 0.0, 0.0, false));
  ASSERT_TRUE(checker.HasOverlapWithBuffer(
      Segment2d(Vec2d(-1.104, 1.480), Vec2d(-0.950, 1.290)), 0.0, 0.0, false));
  ASSERT_TRUE(checker.HasOverlapWithBuffer(
      Segment2d(Vec2d(-0.565, 1.020), Vec2d(-0.340, 1.078)), 0.0, 0.0, false));
}

TEST(CirclePathCollisionChecker, HasOverlapWithPolygonTest) {
  auto param_manager = qcraft::CreateParamManagerFromCarId("Q0001");
  RunParamsProtoV2 run_params;
  param_manager->GetRunParams(&run_params);
  const PlannerParamsProto planner_params = DefaultPlannerParams();

  const GeometryMethodPoint start = {Vec2d(-1.0, -2.0), -1.0,
                                     Vec2d::FastUnitFromAngle(-1.0)};
  const GeometryPathType type = GeometryPathType::RIGHT;
  const double length = 25.0;
  const auto end = ExtendPathByConstantKappa(start, kMaxKappa, length, type);

  CirclePathCollisionChecker checker(
      run_params.vehicle_params().vehicle_geometry_params(),
      planner_params.vehicle_models_params()
          .freespace_vehicle_octagon_model_params(),
      start, end, type, length, kMaxKappa, /*use_fast_overlap=*/true);

  // Lateral_buffer=0.0, longitudinal_buffer=0.0, consider_mirror=false.
  ASSERT_TRUE(checker.HasOverlapWithBuffer(
      Polygon2d({Vec2d(-6.0, -9.0), Vec2d(-5.0, -9.0), Vec2d(-5.0, -10.0)}),
      0.0, 0.0, false));
  ASSERT_TRUE(checker.HasOverlapWithBuffer(
      Polygon2d({Vec2d(-8.0, -1.5), Vec2d(-8.0, -2.5), Vec2d(-7.0, -2.5)}), 0.0,
      0.0, false));
  ASSERT_FALSE(checker.HasOverlapWithBuffer(
      Polygon2d({Vec2d(-8.0, -3.0), Vec2d(-7.0, -3.0), Vec2d(-7.5, -3.5)}), 0.0,
      0.0, false));
  ASSERT_FALSE(checker.HasOverlapWithBuffer(
      Polygon2d({Vec2d(-2.464, 0.789), Vec2d(-3.336, -1.471),
                 Vec2d(-2.129, -1.388), Vec2d(-1.271, -0.789)}),
      0.0, 0.0, false));
}

}  // namespace
}  // namespace planner
}  // namespace qcraft

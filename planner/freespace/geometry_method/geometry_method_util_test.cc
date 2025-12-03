#include "onboard/planner/freespace/geometry_method/geometry_method_util.h"

#include <algorithm>
#include <cmath>
#include <memory>
#include <utility>

#include "gtest/gtest.h"

#include "onboard/math/vec.h"
#include "onboard/params/param_manager.h"
#include "onboard/params/vehicle_param_api.h"
#include "onboard/planner/freespace/freespace_util.h"
#include "onboard/planner/freespace/proto/freespace_planner.pb.h"
#include "onboard/planner/test_util/util.h"

namespace qcraft {
namespace planner {
namespace {

constexpr double kMaxKappa = 0.2;
constexpr double kEps = 1.0e-6;

TEST(GeometryMethodUtilTest, ExtendPathByConstantKappaTest) {
  const GeometryMethodPoint start = {Vec2d(0.0, 0.0), 0.0,
                                     Vec2d::FastUnitFromAngle(0.0)};

  const auto end_1 = ExtendPathByConstantKappa(
      start, /*kappa=*/0.0,
      /*length=*/-1.0, /*type=*/GeometryPathType::STRAIGHT);
  EXPECT_NEAR(end_1.pos.x(), -1.0, kEps);
  EXPECT_NEAR(end_1.pos.y(), 0.0, kEps);
  EXPECT_NEAR(end_1.theta, 0.0, kEps);

  const auto end_2 = ExtendPathByConstantKappa(
      start, kMaxKappa,
      /*length=*/0.5 * M_PI / kMaxKappa, /*type=*/GeometryPathType::LEFT);
  EXPECT_NEAR(end_2.pos.x(), 1.0 / kMaxKappa, kEps);
  EXPECT_NEAR(end_2.pos.y(), 1.0 / kMaxKappa, kEps);
  EXPECT_NEAR(end_2.theta, 0.5 * M_PI, kEps);

  const auto end_3 = ExtendPathByConstantKappa(
      start, kMaxKappa,
      /*length=*/-0.5 * M_PI / kMaxKappa, /*type=*/GeometryPathType::RIGHT);
  EXPECT_NEAR(end_3.pos.x(), -1.0 / kMaxKappa, kEps);
  EXPECT_NEAR(end_3.pos.y(), -1.0 / kMaxKappa, kEps);
  EXPECT_NEAR(end_3.theta, 0.5 * M_PI, kEps);
}

TEST(GeometryMethodUtilTest, ConnectPathsAndComputeGearChangeTimesTest) {
  const GeometryMethodPoint start = {Vec2d(0.0, 0.0), 0.0,
                                     Vec2d::FastUnitFromAngle(0.0)};
  const auto end =
      ExtendPathByConstantKappa(start, /*kappa=*/0.0, /*length=*/1.0,
                                /*type=*/GeometryPathType::STRAIGHT);
  const LineCirclePath path1 = {.start = start,
                                .types = {GeometryPathType::STRAIGHT},
                                .lengths = {1.0},
                                .kappas = {0.0},
                                .ends = {end}};
  EXPECT_EQ(ComputeGearChangeTimes(path1), 0);
  const LineCirclePath path2 = {
      .start = end,
      .types = {GeometryPathType::RIGHT},
      .lengths = {-1.0},
      .kappas = {kMaxKappa},
      .ends = {ExtendPathByConstantKappa(end, kMaxKappa, /*length=*/1.0,
                                         /*type=*/GeometryPathType::RIGHT)}};
  LineCirclePath result;
  ConnectPaths(std::vector<LineCirclePath>{path1, path2}, &result);
  EXPECT_EQ(result.types.size(), 2);
  EXPECT_EQ(ComputeGearChangeTimes(result), 1);
}

TEST(GeometryMethodUtilTest, ReversePathTest) {
  const GeometryMethodPoint start = {Vec2d(0.0, 0.0), 0.0,
                                     Vec2d::FastUnitFromAngle(0.0)};
  LineCirclePath path = {
      .start = start,
      .types = {GeometryPathType::STRAIGHT},
      .lengths = {1.0},
      .kappas = {0.0},
      .ends = {ExtendPathByConstantKappa(start, /*kappa=*/0.0, /*length=*/1.0,
                                         /*type=*/GeometryPathType::STRAIGHT)}};
  ReversePath(&path);
  EXPECT_TRUE(path.lengths.front() < 0.0);
}

TEST(GeometryMethodUtilTest, ConvertLineCirclePathToDiscretePointsTest) {
  const GeometryMethodPoint start = {Vec2d(0.0, 0.0), 0.0,
                                     Vec2d::FastUnitFromAngle(0.0)};
  const LineCirclePath path = {
      .start = start,
      .types = {GeometryPathType::STRAIGHT},
      .lengths = {1.0},
      .kappas = {0.0},
      .ends = {ExtendPathByConstantKappa(start, /*kappa=*/0.0, /*length=*/1.0,
                                         /*type=*/GeometryPathType::STRAIGHT)}};
  const auto discrete_path =
      ConvertLineCirclePathToDiscretePoints(path, /*step=*/0.1001);
  EXPECT_EQ(discrete_path.size(), 11);
}

TEST(GeometryMethodUtilTest, PathAndPoseValidityTest) {
  auto param_manager = qcraft::CreateParamManagerFromCarId("Q0001");
  RunParamsProtoV2 run_params;
  param_manager->GetRunParams(&run_params);
  const PlannerParamsProto planner_params = DefaultPlannerParams();
  const auto& vehicle_geometry_params =
      run_params.vehicle_params().vehicle_geometry_params();
  const auto& vehicle_model_params =
      planner_params.vehicle_models_params()
          .freespace_vehicle_octagon_model_params();
  const auto& path_finder_params =
      planner_params.freespace_params_for_parking().path_finder_params();

  // Construct boundary k-D tree and map.
  const FreespaceBoundary freespace_boundary = {
      .id = "b",
      .type = FreespaceMapProto::CURB,
      .points = {Vec2d(0.0, 0.0), Vec2d(1.0, 0.0)},
      .near_parking_spot = false,
      .height = 0.0};
  std::vector<Segment2d> virtual_boundaries;
  std::vector<std::pair<std::string, Segment2d>> named_segments;
  absl::flat_hash_map<std::string, const FreespaceObject*> objects_map;
  absl::flat_hash_map<std::string, const FreespaceBoundary*> boundaries_map;
  for (int i = 0; i + 1 < freespace_boundary.points.size(); ++i) {
    std::string id = "b" + std::to_string(i);
    const Segment2d seg(freespace_boundary.points[i],
                        freespace_boundary.points[i + 1]);
    named_segments.emplace_back(id, seg);
    boundaries_map.emplace(id, &freespace_boundary);
  }
  SegmentMatcherKdtree segments_kd_tree(named_segments);

  const GeometryMethodPoint pose1 = {Vec2d(0.0, 0.0), 0.0,
                                     Vec2d::FastUnitFromAngle(0.0)};
  const LineCirclePath path1 = {
      .start = pose1,
      .types = {GeometryPathType::STRAIGHT},
      .lengths = {1.0},
      .kappas = {0.0},
      .ends = {ExtendPathByConstantKappa(pose1, /*kappa=*/0.0, /*length=*/1.0,
                                         /*type=*/GeometryPathType::STRAIGHT)}};
  EXPECT_FALSE(CheckPathValidityWithKDTreeAndVirtualBoundaries(
      vehicle_geometry_params, path_finder_params, vehicle_model_params,
      segments_kd_tree, objects_map, boundaries_map, virtual_boundaries,
      path1));
  EXPECT_FALSE(CheckPoseValidityWithKDTreeAndVirtualBoundaries(
      vehicle_geometry_params, path_finder_params, vehicle_model_params,
      segments_kd_tree, objects_map, boundaries_map, virtual_boundaries,
      pose1));

  const GeometryMethodPoint pose2 = {Vec2d(0.0, 3.0), 0.0,
                                     Vec2d::FastUnitFromAngle(0.0)};
  const LineCirclePath path2 = {
      .start = pose2,
      .types = {GeometryPathType::RIGHT},
      .lengths = {5.0},
      .kappas = {0.05},
      .ends = {ExtendPathByConstantKappa(pose2, /*kappa=*/0.05, /*length=*/5.0,
                                         /*type=*/GeometryPathType::RIGHT)}};
  EXPECT_TRUE(CheckPathValidityWithKDTreeAndVirtualBoundaries(
      vehicle_geometry_params, path_finder_params, vehicle_model_params,
      segments_kd_tree, objects_map, boundaries_map, virtual_boundaries,
      path2));
  EXPECT_TRUE(CheckPoseValidityWithKDTreeAndVirtualBoundaries(
      vehicle_geometry_params, path_finder_params, vehicle_model_params,
      segments_kd_tree, objects_map, boundaries_map, virtual_boundaries,
      pose2));
}

TEST(GeometryMethodUtilTest,
     GetFirstOverlapDistanceWithKDTreeAndVirtualBoundaries) {
  auto param_manager = qcraft::CreateParamManagerFromCarId("Q0001");
  RunParamsProtoV2 run_params;
  param_manager->GetRunParams(&run_params);
  const PlannerParamsProto planner_params = DefaultPlannerParams();
  const auto& vehicle_geometry_params =
      run_params.vehicle_params().vehicle_geometry_params();
  const auto& vehicle_model_params =
      planner_params.vehicle_models_params()
          .freespace_vehicle_octagon_model_params();
  const auto& path_finder_params =
      planner_params.freespace_params_for_parking().path_finder_params();

  // Construct boundary k-D tree and map.
  const FreespaceBoundary freespace_boundary = {
      .id = "b",
      .type = FreespaceMapProto::CURB,
      .points = {Vec2d(0.0, 0.0), Vec2d(1.0, 0.0)},
      .near_parking_spot = false,
      .height = 0.0};
  std::vector<Segment2d> virtual_boundaries;
  std::vector<std::pair<std::string, Segment2d>> named_segments;
  absl::flat_hash_map<std::string, const FreespaceObject*> objects_map;
  absl::flat_hash_map<std::string, const FreespaceBoundary*> boundaries_map;
  for (int i = 0; i + 1 < freespace_boundary.points.size(); ++i) {
    std::string id = "b" + std::to_string(i);
    const Segment2d seg(freespace_boundary.points[i],
                        freespace_boundary.points[i + 1]);
    named_segments.emplace_back(id, seg);
    boundaries_map.emplace(id, &freespace_boundary);
  }
  SegmentMatcherKdtree segments_kd_tree(named_segments);

  const GeometryMethodPoint pose = {Vec2d(0.5, -5.0), M_PI_2,
                                    Vec2d::FastUnitFromAngle(M_PI_2)};
  const auto dist1 = GetFirstOverlapDistanceWithKDTreeAndVirtualBoundaries(
      vehicle_geometry_params, path_finder_params, vehicle_model_params,
      segments_kd_tree, objects_map, boundaries_map, virtual_boundaries, pose,
      /*kappa=*/0.0, /*forward=*/true, /*expected_max_distance=*/100.0);
  const auto buffers =
      GetVehicleBufferForBoundary(path_finder_params, freespace_boundary);
  EXPECT_TRUE(dist1.has_value());
  EXPECT_NEAR(
      *dist1,
      5.0 - vehicle_geometry_params.front_edge_to_center() - buffers.second,
      kEps);

  const auto dist2 = GetFirstOverlapDistanceWithKDTreeAndVirtualBoundaries(
      vehicle_geometry_params, path_finder_params, vehicle_model_params,
      segments_kd_tree, objects_map, boundaries_map, virtual_boundaries, pose,
      /*kappa=*/0.0, /*forward=*/false, /*expected_max_distance=*/100.0);
  EXPECT_FALSE(dist2.has_value());
}

}  // namespace
}  // namespace planner
}  // namespace qcraft

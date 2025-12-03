#include "onboard/planner/freespace/geometry_method/perpendicular_parking.h"

#include <algorithm>
#include <cmath>
#include <memory>
#include <utility>

#include "gtest/gtest.h"

#include "onboard/math/vec.h"
#include "onboard/params/param_manager.h"
#include "onboard/params/vehicle_param_api.h"
#include "onboard/planner/test_util/util.h"

namespace qcraft {
namespace planner {
namespace {

TEST(PerpendicularParkingTest, GeneralTest) {
  auto param_manager = qcraft::CreateParamManagerFromCarId("Q0001");
  RunParamsProtoV2 run_params;
  param_manager->GetRunParams(&run_params);
  const PlannerParamsProto planner_params = DefaultPlannerParams();
  constexpr double kMaxKappa = 0.2;

  const std::vector<Vec2d> points = {Vec2d(-5.0, 4.5),  Vec2d(-1.7, 4.5),
                                     Vec2d(-1.7, -1.3), Vec2d(1.7, -1.3),
                                     Vec2d(1.7, 4.5),   Vec2d(5.0, 4.5)};
  std::vector<Segment2d> virtual_boundaries;
  for (int i = 0; i + 1 < points.size(); ++i) {
    virtual_boundaries.emplace_back(points[i], points[i + 1]);
  }
  virtual_boundaries.emplace_back(Vec2d(-8.0, 9.0), Vec2d(8.0, 9.0));

  GeometryMethodPoint start = {Vec2d(5.0, 6.0), M_PI,
                               Vec2d::FastUnitFromAngle(M_PI)};
  GeometryMethodPoint goal = {Vec2d(0.0, 0.0), 0.5 * M_PI,
                              Vec2d::FastUnitFromAngle(0.5 * M_PI)};

  std::vector<std::pair<std::string, Segment2d>> named_segments;
  absl::flat_hash_map<std::string, const FreespaceObject*> objects_map;
  absl::flat_hash_map<std::string, const FreespaceBoundary*> boundaries_map;
  const SegmentMatcherKdtree segments_kd_tree(named_segments);

  const auto result_status = FindPerpendicularParkingPath(
      run_params.vehicle_params().vehicle_geometry_params(), kMaxKappa,
      planner_params.freespace_params_for_parking().path_finder_params(),
      planner_params.vehicle_models_params()
          .freespace_vehicle_octagon_model_params(),
      segments_kd_tree, objects_map, boundaries_map, virtual_boundaries, start,
      goal);

  ASSERT_TRUE(result_status.ok());
}

}  // namespace
}  // namespace planner
}  // namespace qcraft

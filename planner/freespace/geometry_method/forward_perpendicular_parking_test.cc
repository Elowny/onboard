#include "onboard/planner/freespace/geometry_method/forward_perpendicular_parking.h"

#include <algorithm>
#include <cmath>
#include <memory>
#include <utility>

#include "absl/status/status.h"
#include "glog/logging.h"

#include "gtest/gtest.h"

#include "onboard/math/vec.h"
#include "onboard/params/param_manager.h"
#include "onboard/params/vehicle_param_api.h"
#include "onboard/planner/freespace/geometry_method/geometry_method_util.h"
#include "onboard/planner/test_util/util.h"
#include "onboard/vis/canvas/canvas.h"
#include "onboard/vis/common/color.h"

#include "offboard/vis/vantage/vantage_server/vantage_canvas_client.h"

namespace qcraft {
namespace planner {
namespace {

TEST(ForwardPerpendicularParking, GeneralTest) {
  auto param_manager = qcraft::CreateParamManagerFromCarId("Q0001");
  RunParamsProtoV2 run_params;
  param_manager->GetRunParams(&run_params);
  const PlannerParamsProto planner_params = DefaultPlannerParams();
  constexpr double kMaxKappa = 0.2;

  vis::Canvas& canvas = vis::vantage::GetCanvasClient()->GetCanvas(
      "geo_parking_test/forward_perpendicular_parking");

  constexpr double kSpotWidth = 3.5;
  const std::vector<Vec2d> points = {Vec2d(-7.0, 5.0),
                                     Vec2d(-0.5 * kSpotWidth, 5.0),
                                     Vec2d(-0.5 * kSpotWidth, 0.0),
                                     Vec2d(0.5 * kSpotWidth, 0.0),
                                     Vec2d(0.5 * kSpotWidth, 5.0),
                                     Vec2d(7.0, 5.0)};
  std::vector<Segment2d> virtual_boundaries;
  for (int i = 0; i + 1 < points.size(); ++i) {
    virtual_boundaries.emplace_back(points[i], points[i + 1]);
    canvas.DrawLine(Vec3d(points[i]), Vec3d(points[i + 1]), vis::Color::kRed);
  }
  GeometryMethodPoint start = {Vec2d(-6.0, 12.0), -0.25 * M_PI,
                               Vec2d::FastUnitFromAngle(-0.25 * M_PI)};
  GeometryMethodPoint goal = {Vec2d(0.0, 4.0), -0.5 * M_PI,
                              Vec2d::FastUnitFromAngle(-0.5 * M_PI)};

  std::vector<std::pair<std::string, Segment2d>> named_segments;
  absl::flat_hash_map<std::string, const FreespaceObject*> objects_map;
  absl::flat_hash_map<std::string, const FreespaceBoundary*> boundaries_map;
  const SegmentMatcherKdtree segments_kd_tree(named_segments);

  const auto result_status = FindForwardPerpendicularParkingPath(
      run_params.vehicle_params().vehicle_geometry_params(), kMaxKappa,
      planner_params.freespace_params_for_parking().path_finder_params(),
      planner_params.vehicle_models_params()
          .freespace_vehicle_octagon_model_params(),
      segments_kd_tree, objects_map, boundaries_map, virtual_boundaries, start,
      goal);
  LOG(INFO) << result_status.status();
  ASSERT_TRUE(result_status.ok());

  SendLineCirclePathToCanvas(
      &canvas, run_params.vehicle_params().vehicle_geometry_params(),
      planner_params.vehicle_models_params()
          .freespace_vehicle_octagon_model_params(),
      *result_status, /*step=*/0.2);
}

}  // namespace
}  // namespace planner
}  // namespace qcraft

#include "onboard/planner/freespace/geometry_method/geometry_parking.h"

#include <memory>

#include "gtest/gtest.h"

#include "onboard/container/strong_int.h"
#include "onboard/maps/map_selector.h"
#include "onboard/maps/semantic_map_defs.h"
#include "onboard/math/geometry/aabox2d.h"
#include "onboard/math/geometry/polygon2d.h"
#include "onboard/math/vec.h"
#include "onboard/params/param_manager.h"
#include "onboard/params/vehicle_param_api.h"
#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/planner/test_util/load_psmm_util.h"
#include "onboard/planner/test_util/util.h"

namespace qcraft {
namespace planner {
namespace {

TEST(GeometryParkingTest, Test) {
  SetMap("dojo");
  const auto& psmm = CreateDojoTestPSMM();
  const auto parking_spot_info =
      psmm.FindParkingSpotByIdOrNull(mapping::ElementId(4002));

  auto param_manager = qcraft::CreateParamManagerFromCarId("Q0001");
  RunParamsProtoV2 run_params;
  param_manager->GetRunParams(&run_params);
  const PlannerParamsProto planner_params = DefaultPlannerParams();
  const auto& veh_geo_params =
      run_params.vehicle_params().vehicle_geometry_params();

  const FreespaceMap freespace_map = {
      .region = AABox2d(), .boundaries = {}, .special_boundaries = {}};

  PathPoint start;
  start.set_x(571.4);
  start.set_y(19.7);
  start.set_theta(3.13);

  const Vec2d goal_tangent = parking_spot_info->unit_direction();
  const double offset =
      veh_geo_params.front_edge_to_center() - veh_geo_params.length() * 0.5;
  const Vec2d goal_pos =
      parking_spot_info->polygon().centroid() - goal_tangent * offset;
  PathPoint goal;
  goal.set_x(goal_pos.x());
  goal.set_y(goal_pos.y());
  goal.set_theta(goal_tangent.Angle());

  PathFinderDebugProto debug_info;
  const auto path_status = FindSinglePath(
      planner_params.freespace_params_for_parking().path_finder_params(),
      veh_geo_params, run_params.vehicle_params().vehicle_drive_params(),
      planner_params.vehicle_models_params()
          .freespace_vehicle_octagon_model_params(),
      FreespaceTaskProto::PERPENDICULAR_PARKING,
      FreespaceReplanReasonProto::NONE, freespace_map,
      /*stalled_object_trajs=*/{}, parking_spot_info, start, goal, &debug_info);

  EXPECT_TRUE(path_status.ok());
}

}  // namespace
}  // namespace planner
}  // namespace qcraft

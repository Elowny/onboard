#include "onboard/planner/freespace/freespace_util.h"

#include <memory>

#include "absl/status/statusor.h"
#include "glog/logging.h"

#include "gtest/gtest.h"

#include "onboard/container/strong_int.h"
#include "onboard/global/buffered_logger.h"
#include "onboard/lite/check.h"
#include "onboard/maps/maps_common.h"
#include "onboard/maps/semantic_map_defs.h"
#include "onboard/math/geometry/polygon2d.h"
#include "onboard/math/util.h"
#include "onboard/params/param_manager.h"
#include "onboard/params/v2/proto/vehicle/common.pb.h"
#include "onboard/params/v2/proto/vehicle/installation.pb.h"
#include "onboard/params/vehicle_param_api.h"
#include "onboard/planner/planner_params_builder.h"
#include "onboard/planner/test_util/load_psmm_util.h"

namespace qcraft {
namespace planner {

TEST(FreespaceUtilTest, MaybeAdjustGoal) {
  RunParamsProtoV2 run_params;
  auto param_manager = CreateParamManagerFromCarId("Q1001");
  CHECK(param_manager != nullptr);
  param_manager->GetRunParams(&run_params);
  const auto& vehicle_geom =
      run_params.vehicle_params().vehicle_geometry_params();
  const auto planner_params_status = BuildPlannerParams(
      vehicle_geom, VEHICLE_LINCOLN_MKZ, VehicleInstallationProto::VP_DBQ_V3);
  QCHECK(planner_params_status.ok());

  const PlannerObjectManager obj_mgr;
  const absl::flat_hash_set<std::string> stalled_objects;
  const auto& whole_psmm = CreateDojoTestPSMM();
  const auto* parking_spot_info =
      whole_psmm.FindParkingSpotByIdOrNull(mapping::ElementId(4002));
  QCHECK(parking_spot_info != nullptr);
  const Vec2d goal_tangent = parking_spot_info->unit_direction();
  const double offset =
      vehicle_geom.front_edge_to_center() - vehicle_geom.length() * 0.5;
  constexpr double kGoalOffset = 0.5;  // m.
  const Vec2d goal_pos = parking_spot_info->polygon().centroid() -
                         goal_tangent * (offset + kGoalOffset);
  PathPoint goal;
  goal.set_x(goal_pos.x());
  goal.set_y(goal_pos.y());
  goal.set_theta(goal_tangent.Angle());

  const auto new_goal =
      MaybeAdjustGoal(planner_params_status->freespace_params_for_parking()
                          .path_finder_params(),
                      vehicle_geom,
                      planner_params_status->vehicle_models_params()
                          .freespace_vehicle_octagon_model_params(),
                      &whole_psmm, &obj_mgr, stalled_objects, goal,
                      /*is_parking_task=*/true,
                      /*adjust_dir=*/'F', /*max_adjust_dist=*/1.0,
                      /*adjust_step=*/0.1, /*planner_buffer=*/0.0);

  EXPECT_GT(Hypot(new_goal.x() - goal.x(), new_goal.y() - goal.y()),
            0.1 - 1.0e-6);
}

TEST(FreespaceUtilTest, MaybeAdjustGoal2) {
  RunParamsProtoV2 run_params;
  auto param_manager = CreateParamManagerFromCarId("Q1001");
  CHECK(param_manager != nullptr);
  param_manager->GetRunParams(&run_params);
  const auto& vehicle_geom =
      run_params.vehicle_params().vehicle_geometry_params();
  const auto planner_params_status = BuildPlannerParams(
      vehicle_geom, VEHICLE_LINCOLN_MKZ, VehicleInstallationProto::VP_DBQ_V3);
  QCHECK(planner_params_status.ok());

  std::vector<planner::FreespaceObject> stationary_objects;
  const auto& whole_psmm = CreateDojoTestPSMM();
  const auto* parking_spot_info =
      whole_psmm.FindParkingSpotByIdOrNull(mapping::ElementId(4002));
  QCHECK(parking_spot_info != nullptr);
  const Vec2d goal_tangent = parking_spot_info->unit_direction();
  const double offset =
      vehicle_geom.front_edge_to_center() - vehicle_geom.length() * 0.5;
  constexpr double kGoalOffset = 0.5;  // m.
  const Vec2d goal_pos = parking_spot_info->polygon().centroid() -
                         goal_tangent * (offset + kGoalOffset);
  PathPoint goal;
  goal.set_x(goal_pos.x());
  goal.set_y(goal_pos.y());
  goal.set_theta(goal_tangent.Angle());

  const auto new_goal =
      MaybeAdjustGoal(planner_params_status->freespace_params_for_parking()
                          .path_finder_params(),
                      vehicle_geom,
                      planner_params_status->vehicle_models_params()
                          .freespace_vehicle_octagon_model_params(),
                      &whole_psmm, stationary_objects, goal,
                      /*is_parking_task=*/true,
                      /*adjust_dir=*/'F', /*max_adjust_dist=*/1.0,
                      /*adjust_step=*/0.1, /*planner_buffer=*/0.0);

  EXPECT_GT(Hypot(new_goal.x() - goal.x(), new_goal.y() - goal.y()),
            0.1 - 1.0e-6);
}

}  // namespace planner
}  // namespace qcraft

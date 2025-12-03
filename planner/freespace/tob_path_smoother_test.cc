#include "onboard/planner/freespace/tob_path_smoother.h"

#include <algorithm>
#include <cmath>
#include <memory>
#include <ostream>

#include "absl/hash/hash.h"
#include "absl/status/status.h"
#include "absl/types/span.h"
#include "gflags/gflags.h"
#include "glog/logging.h"

#include "gtest/gtest.h"

#include "onboard/global/buffered_logger.h"
#include "onboard/lite/check.h"
#include "onboard/math/geometry/aabox2d.h"
#include "onboard/math/util.h"
#include "onboard/math/vec.h"
#include "onboard/params/param_manager.h"
#include "onboard/params/v2/proto/vehicle/common.pb.h"
#include "onboard/params/v2/proto/vehicle/installation.pb.h"
#include "onboard/params/vehicle_param_api.h"
#include "onboard/planner/common/discretized_path.h"
#include "onboard/planner/freespace/hybrid_a_star/hybrid_a_star.h"
#include "onboard/planner/object/planner_object.h"
#include "onboard/planner/object/spacetime_object_trajectory.h"
#include "onboard/planner/planner_params_builder.h"
#include "onboard/planner/test_util/perception_object_builder.h"
#include "onboard/planner/test_util/planner_object_builder.h"
#include "onboard/proto/perception/fusion/object.pb.h"

DECLARE_bool(send_freespace_local_smoother_path_to_canvas);
DECLARE_bool(draw_boundary_cost_buffer_circle);
DECLARE_bool(send_msd_static_boundary_to_canvas);

namespace qcraft {
namespace planner {
namespace {

TEST(LocalSmootherTest, ReverseParking) {
  FLAGS_send_freespace_local_smoother_path_to_canvas = true;
  FLAGS_draw_boundary_cost_buffer_circle = true;
  FLAGS_send_msd_static_boundary_to_canvas = true;
  auto param_manager = qcraft::CreateParamManagerFromCarId("Q1001");
  RunParamsProtoV2 run_params;
  param_manager->GetRunParams(&run_params);

  const auto& vehicle_geo_params =
      run_params.vehicle_params().vehicle_geometry_params();
  const auto& vehicle_drive_params =
      run_params.vehicle_params().vehicle_drive_params();
  const auto planner_params_status =
      BuildPlannerParams(vehicle_geo_params, VEHICLE_LINCOLN_MKZ,
                         VehicleInstallationProto::VP_DBQ_V3);
  QCHECK(planner_params_status.ok());
  const auto& planner_params = *planner_params_status;

  std::vector<Vec2d> boundary1 = {Vec2d(20.0, 3.5),  Vec2d(13.0, 3.5),
                                  Vec2d(13.0, 10.0), Vec2d(10.0, 10.0),
                                  Vec2d(10.0, 3.5),  Vec2d(-2.0, 3.5)};
  std::vector<FreespaceBoundary> boundaries;
  boundaries.reserve(boundary1.size());
  for (int i = 0; i + 1 < boundary1.size(); ++i) {
    boundaries.push_back({.id = "b" + std::to_string(i),
                          .type = FreespaceMapProto::CURB,
                          .points = {boundary1[i], boundary1[i + 1]},
                          .near_parking_spot = true,
                          .height = i % 2 == 0 ? 10.0 : 0.0});
  }
  boundaries.push_back({.id = "BARRIER",
                        .type = FreespaceMapProto::BARRIER,
                        .points = {Vec2d(-1.0, -2.0), Vec2d(3.0, -2.0)},
                        .near_parking_spot = false,
                        .height = 5.0});
  std::vector<SpecialBoundary> special_boundaries;
  special_boundaries.push_back(
      {.id = "CROSSABLE_LANE_LINE",
       .type = FreespaceMapProto::CROSSABLE_LANE_LINE,
       .points = {Vec2d(-1.0, -2.0), Vec2d(3.0, -2.0)}});

  FreespaceMap static_map = {.region = AABox2d(11.0, 9.0, Vec2d(9.0, 3.0)),
                             .boundaries = boundaries,
                             .special_boundaries = special_boundaries};
  // Add objects.
  std::vector<PlannerObject> objects;
  PerceptionObjectBuilder perception_builder;
  auto perception_obj = perception_builder.set_id("test")
                            .set_type(OT_VEHICLE)
                            .set_length_width(1.8, 1.8)
                            .set_box_center(Vec2d(13.75, 4.52))
                            .set_pos(Vec2d(13.75, 4.52))
                            .Build();
  PlannerObjectBuilder builder;
  builder.set_type(OT_VEHICLE).set_object(perception_obj).set_stationary(true);
  objects.push_back(builder.Build());
  SpacetimeTrajectoryManager st_traj_mgr(objects);
  std::vector<const SpacetimeObjectTrajectory*> stalled_object_trajs;
  stalled_object_trajs.reserve(st_traj_mgr.trajectories().size());
  for (const auto traj : st_traj_mgr.stationary_object_trajs()) {
    stalled_object_trajs.push_back(traj);
  }
  absl::flat_hash_set<std::string> stalled_objects;
  stalled_objects.insert("test");
  const PlannerClusterObjectManager cluster_obj_mgr;
  const absl::flat_hash_set<PlannerClusterObject::Id> stalled_cluster_objects;

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
      FreespaceTaskProto::PERPENDICULAR_PARKING, static_map,
      stalled_object_trajs, start, end, &path_finder_debug_info);
  LOG(INFO) << "Hybrid A star status: "
            << coarse_path_status.status().ToString();
  ASSERT_TRUE(coarse_path_status.ok());

  FreespaceLocalSmootherDebugProto smoother_debug_info;
  vis::vantage::ChartsDataProto smoother_chart_info;
  std::vector<PathPoint> prev_path = {};
  for (const auto& path : *coarse_path_status) {
    TrajectoryPoint plan_start_point;
    plan_start_point.set_pos(
        Vec2d(path.path.front().x(), path.path.front().y()));
    plan_start_point.set_theta(
        path.forward ? path.path.front().theta()
                     : NormalizeAngle(path.path.front().theta() + M_PI));
    plan_start_point.set_kappa(path.forward ? path.path.front().kappa()
                                            : -path.path.front().kappa());
    const auto res = SmoothLocalPath(
        vehicle_geo_params, vehicle_drive_params,
        planner_params.freespace_params_for_parking()
            .motion_constraint_params(),
        planner_params.freespace_params_for_parking().local_smoother_params(),
        planner_params.vehicle_models_params()
            .freespace_local_smoother_vehicle_model_params(),
        /*owner=*/"freesspace", static_map, st_traj_mgr, cluster_obj_mgr,
        stalled_objects, stalled_cluster_objects, path, plan_start_point,
        /*reset=*/true, prev_path, &smoother_debug_info, &smoother_chart_info);
    LOG(INFO) << "Path smoother status: " << res.status().ToString();
    ASSERT_TRUE(res.ok());

    const auto res2 = SmoothLocalPath(
        vehicle_geo_params, vehicle_drive_params,
        planner_params.freespace_params_for_parking()
            .motion_constraint_params(),
        planner_params.freespace_params_for_parking().local_smoother_params(),
        planner_params.vehicle_models_params()
            .freespace_local_smoother_vehicle_model_params(),
        /*owner=*/"freesspace", static_map, st_traj_mgr, cluster_obj_mgr,
        stalled_objects, stalled_cluster_objects, path, plan_start_point,
        /*reset=*/false, {res->path.begin(), res->path.end()},
        &smoother_debug_info, &smoother_chart_info);
    ASSERT_TRUE(res2.ok());
  }
}

}  // namespace
}  // namespace planner
}  // namespace qcraft

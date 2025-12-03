#include "onboard/planner/decision/pedestrians_decider.h"

#include <initializer_list>
#include <memory>
#include <optional>

#include "absl/types/span.h"
#include "glog/logging.h"

#include "gtest/gtest.h"

#include "onboard/container/strong_int.h"
#include "onboard/global/buffered_logger.h"
#include "onboard/maps/semantic_map_defs.h"
#include "onboard/math/vec.h"
#include "onboard/params/param_manager.h"
#include "onboard/params/vehicle_param_api.h"
#include "onboard/planner/object/object_vector.h"
#include "onboard/planner/object/planner_object.h"
#include "onboard/planner/object/planner_object_manager_builder.h"
#include "onboard/planner/object/spacetime_trajectory_manager.h"
#include "onboard/planner/router/drive_passage_builder.h"
#include "onboard/planner/scheduler/path_boundary_builder.h"
#include "onboard/planner/test_util/load_psmm_util.h"
#include "onboard/planner/test_util/object_prediction_builder.h"
#include "onboard/planner/test_util/perception_object_builder.h"
#include "onboard/planner/test_util/predicted_trajectory_builder.h"
#include "onboard/prediction/object_prediction.h"
#include "onboard/proto/perception.pb.h"
#include "onboard/proto/perception/fusion/object.pb.h"
#include "onboard/proto/prediction.pb.h"
#include "onboard/utils/status_macros.h"

namespace qcraft::planner {
namespace {

TEST(BuildPedestriansConstraints, BaseTest) {
  // Get vehicle params.
  RunParamsProtoV2 run_params;
  auto param_manager = CreateParamManagerFromCarId("Q8001");
  CHECK(param_manager != nullptr);
  param_manager->GetRunParams(&run_params);
  const VehicleGeometryParamsProto& vehicle_geometry_params =
      run_params.vehicle_params().vehicle_geometry_params();

  const auto& psmm = CreateDojoTestPSMM();

  const auto current_lane_path = mapping::LanePath(
      psmm.semantic_map_manager(), /*lane_ids=*/
      {mapping::ElementId(2448), mapping::ElementId(1), mapping::ElementId(34)},
      /*start_fraction=*/0.0, /*end_fraction=*/1.0);

  ASSIGN_OR_DIE(const auto drive_passage,
                BuildDrivePassageFromLanePath(
                    psmm, current_lane_path, /*step_s=*/1.0,
                    /*avoid_loop=*/true, /*backward_extend_len=*/0.0,
                    /*required_planning_horizon=*/0.0,
                    /*required_backward_len=*/0.0,
                    /*override_speed_limit_mps=*/std::nullopt));

  ASSIGN_OR_DIE(const auto path_boundary,
                BuildPathBoundaryFromDrivePassage(psmm, drive_passage));

  ApolloTrajectoryPointProto plan_start_point;
  plan_start_point.mutable_path_point()->set_x(0.0);
  plan_start_point.mutable_path_point()->set_y(0.0);
  plan_start_point.set_v(10.0);

  // Valid object type test.
  {
    for (const auto object_type : {OT_PEDESTRIAN, OT_CYCLIST, OT_TRICYCLIST}) {
      ObjectsProto perception_objects;
      const auto object_1 = PerceptionObjectBuilder()
                                .set_id("PED_1")
                                .set_type(object_type)
                                .set_pos(Vec2d(41.350, 4.043))
                                .set_yaw(-1.55)
                                .set_velocity(1.0)
                                .Build();
      *perception_objects.add_objects() = object_1;

      ObjectPredictionBuilder pred_builder;
      pred_builder.set_object(object_1)
          .add_predicted_trajectory()
          ->set_probability(1.0)
          .set_straight_line(/*start=*/Vec2d(75.443, 2.587),
                             /*end=*/Vec2d(75.443, -6.843),
                             /*init_v=*/1.0, /*last_v=*/1.0);

      const auto object_pred_1 = pred_builder.Build();
      ObjectsPredictionProto prediction_objects;
      object_pred_1.ToProto(prediction_objects.add_objects());

      SpacetimeTrajectoryManager st_traj_mgr(
          BuildPlannerObjects(&perception_objects, &prediction_objects,
                              /*align_time=*/0.0,
                              /*thread_pool=*/nullptr));

      ASSIGN_OR_DIE(
          const auto speed_regions,
          BuildPedestriansConstraints(
              vehicle_geometry_params, psmm, plan_start_point, drive_passage,
              current_lane_path, /*s_offset*/ 0.0, path_boundary, st_traj_mgr));

      EXPECT_EQ(speed_regions.size(), 1);
    }
  }

  // Invalid object type test.
  {
    for (const auto object_type :
         {OT_UNKNOWN_STATIC, OT_VEHICLE, OT_LARGE_VEHICLE, OT_MOTORCYCLIST,
          OT_FOD, OT_UNKNOWN_MOVABLE, OT_VEGETATION, OT_BARRIER, OT_CONE,
          OT_WARNING_TRIANGLE}) {
      ObjectsProto perception_objects;
      const auto object_1 = PerceptionObjectBuilder()
                                .set_id("PED_1")
                                .set_type(object_type)
                                .set_pos(Vec2d(41.350, 4.043))
                                .set_yaw(-1.55)
                                .set_velocity(1.0)
                                .Build();
      *perception_objects.add_objects() = object_1;

      ObjectPredictionBuilder pred_builder;
      pred_builder.set_object(object_1)
          .add_predicted_trajectory()
          ->set_probability(1.0)
          .set_straight_line(/*start=*/Vec2d(75.443, 2.587),
                             /*end=*/Vec2d(75.443, -6.843),
                             /*init_v=*/1.0, /*last_v=*/1.0);

      const auto object_pred_1 = pred_builder.Build();
      ObjectsPredictionProto prediction_objects;
      object_pred_1.ToProto(prediction_objects.add_objects());

      SpacetimeTrajectoryManager st_traj_mgr(
          BuildPlannerObjects(&perception_objects, &prediction_objects,
                              /*align_time=*/0.0,
                              /*thread_pool=*/nullptr));

      ASSIGN_OR_DIE(
          const auto speed_regions,
          BuildPedestriansConstraints(
              vehicle_geometry_params, psmm, plan_start_point, drive_passage,
              current_lane_path, /*s_offset*/ 0.0, path_boundary, st_traj_mgr));

      EXPECT_TRUE(speed_regions.empty());
    }
  }
}

}  // namespace
}  // namespace qcraft::planner

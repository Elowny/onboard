#include "onboard/planner/plan/parking_task.h"

#include <memory>

#include "absl/time/clock.h"

#include "gtest/gtest.h"

#include "onboard/async/thread_pool.h"
#include "onboard/base/macros.h"
#include "onboard/container/strong_int.h"
#include "onboard/math/vec.h"
#include "onboard/params/param_manager.h"
#include "onboard/params/vehicle_param_api.h"
#include "onboard/planner/planner_flags.h"
#include "onboard/planner/test_util/load_psmm_util.h"
#include "onboard/planner/test_util/util.h"
#include "onboard/utils/time_util.h"

namespace qcraft::planner {
namespace {
TEST(ParkingTaskTest, Test) {
  auto param_manager = CreateParamManagerFromCarId("Q0001");

  RunParamsProtoV2 run_params;
  param_manager->GetRunParams(&run_params);
  const auto& vehicle_params = run_params.vehicle_params();

  PlannerParamsProto planner_params = DefaultPlannerParams();

  const auto& whole_psmm = CreateDojoTestPSMM();
  const PoseProto sdc_pose =
      CreatePose(ToUnixDoubleSeconds(absl::Now()), Vec2d(595.2, -7.0),
                 /*heading=*/1.57, Vec2d(0.0, 0.0));
  const auto plan_start_point = ConvertToTrajPointProto(sdc_pose);
  const PlanStartPointInfo plan_start_point_info{
      .reset = false,
      .start_point = plan_start_point,
      .path_s_increment_from_previous_frame = 0.0,
      .plan_time = absl::Now(),
      .full_stop = true,
  };
  const Chassis chassis;

  auto object_manager_ptr = std::make_shared<const PlannerObjectManager>();

  ParkingTaskOutput output;
  auto thread_pool =
      std::make_unique<ThreadPool>(FLAGS_planner_thread_pool_size);

  CoordinateConverter coordinate_converter;

  const AutonomyStateProto autonomy_state;
  const TrajectoryProto prev_trajectory_proto;
  FreespacePlannerStateProto state;
  const auto status = RunParkingTask(
      ParkingTaskInput{
          .reset = true,
          .autonomy_state = &autonomy_state,
          .psmm = &whole_psmm,
          .coordinate_converter = &coordinate_converter,
          .parking_spot_id = mapping::ElementId(4002),
          .pose = &sdc_pose,
          .chassis = &chassis,
          .plan_start_point_info = &plan_start_point_info,
          .plan_time = absl::Now(),
          .freespace_params = &planner_params.freespace_params_for_parking(),
          .vehicle_models_params = &planner_params.vehicle_models_params(),
          .veh_geo_params = &vehicle_params.vehicle_geometry_params(),
          .veh_drive_params = &vehicle_params.vehicle_drive_params(),
          .prev_trajectory_proto = &prev_trajectory_proto,
          .object_manager = object_manager_ptr.get(),
      },
      &state, &output, thread_pool.get());

  EXPECT_OK(status);
}
}  // namespace
}  // namespace qcraft::planner

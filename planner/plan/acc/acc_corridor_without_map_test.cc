#include "onboard/planner/plan/acc/acc_corridor_without_map.h"

#include <memory>
#include <vector>

#include "absl/status/status.h"
#include "absl/types/span.h"

#include "gtest/gtest.h"

#include "onboard/base/macros.h"
#include "onboard/math/geometry/proto/affine_transformation.pb.h"
#include "onboard/params/param_manager.h"
#include "onboard/params/vehicle_param_api.h"
#include "onboard/planner/common/discretized_path.h"
#include "onboard/planner/common/path_sl_boundary.h"
#include "onboard/planner/planner_defs.h"
#include "onboard/planner/test_util/util.h"

namespace qcraft {
namespace planner {
namespace {

TEST(BuildAccCorridorFromPlanStartPoint, GoStraight) {
  auto param_manager = CreateParamManagerFromCarId("Q1001");
  RunParamsProtoV2 run_params;
  param_manager->GetRunParams(&run_params);
  const auto& vehicle_geom_params =
      run_params.vehicle_params().vehicle_geometry_params();
  const auto& vehicle_drive_params =
      run_params.vehicle_params().vehicle_drive_params();

  PoseProto av_pose;
  av_pose.mutable_pos_smooth()->set_x(1.0);
  av_pose.mutable_pos_smooth()->set_y(0.0);
  av_pose.set_yaw(0.0);
  av_pose.set_curvature(0.0);
  av_pose.mutable_vel_body()->set_x(10.0);

  const auto plan_start_point = ConvertToTrajPointProto(av_pose);

  const auto corridor_or = BuildAccCorridorFromPlanStartPoint(
      av_pose, plan_start_point, /*steering_percentage=*/0.0,
      vehicle_geom_params, vehicle_drive_params,
      /*corridor_step_s=*/2.0, kAccTrajectorySteps,
      /*av_kappa_cache_average=*/0.0);
  EXPECT_OK(corridor_or.status());
  auto corridor = *corridor_or;
  const auto boundary = corridor.boundary;
  const auto path = corridor.path;
  const double credible_length = corridor.credible_length;
  EXPECT_NEAR(boundary.end_s(), credible_length, 1.0);
  constexpr double kEpsilon = 1e-5;
  for (int i = 0; i < boundary.size() - 1; ++i) {
    EXPECT_NEAR(boundary.reference_center_l_vector()[i],
                av_pose.pos_smooth().y(), kEpsilon);
  }

  EXPECT_GT(path.length(), credible_length);
  for (const auto& pt : path) {
    EXPECT_NEAR(pt.y(), av_pose.pos_smooth().y(), kEpsilon);
    EXPECT_NEAR(pt.theta(), av_pose.yaw(), kEpsilon);
    EXPECT_NEAR(pt.kappa(), av_pose.curvature(), kEpsilon);
  }
}

}  // namespace
}  // namespace planner
}  // namespace qcraft

#include "onboard/planner/decision/beyond_length_along_route.h"

#include <limits>
#include <memory>
#include <optional>
#include <vector>

#include "glog/logging.h"

#include "gtest/gtest.h"

#include "onboard/container/strong_int.h"
#include "onboard/global/buffered_logger.h"
#include "onboard/maps/lane_point.h"
#include "onboard/maps/map_selector.h"
#include "onboard/maps/semantic_map_defs.h"
#include "onboard/math/proto/piecewise_linear_function.pb.h"
#include "onboard/math/vec.h"
#include "onboard/params/param_manager.h"
#include "onboard/params/vehicle_param_api.h"
#include "onboard/planner/composite_lane_path.h"
#include "onboard/planner/planner_defs.h"
#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/planner/router/drive_passage_builder.h"
#include "onboard/planner/router/plot_util.h"
#include "onboard/planner/router/route_sections.h"
#include "onboard/planner/router/route_sections_util.h"
#include "onboard/planner/test_util/load_psmm_util.h"
#include "onboard/planner/test_util/route_builder.h"
#include "onboard/planner/test_util/util.h"
#include "onboard/proto/positioning.pb.h"
#include "onboard/proto/vehicle.pb.h"
#include "onboard/utils/status_macros.h"

namespace qcraft::planner {
namespace {
constexpr double kEpsilon = 1e-2;
TEST(BeyondLengthAlongRouteTest, NormalTest) {
  auto planner_params = DefaultPlannerParams();
  auto param_manager = CreateParamManagerFromCarId("Q0001");
  CHECK(param_manager != nullptr);
  RunParamsProtoV2 run_params;
  param_manager->GetRunParams(&run_params);
  const VehicleGeometryParamsProto vehicle_geometry_params =
      run_params.vehicle_params().vehicle_geometry_params();

  // Load planner semantic map.
  SetMap("dojo");

  const auto& psmm = CreateDojoTestPSMM();
  const auto* smm = psmm.semantic_map_manager();
  const auto& cc = psmm.coordinate_converter();

  // Build route sections.
  const PoseProto sdc_pose = CreatePose(
      /*timestamp=*/10.0, Vec2d(118.340, -0.488), 0.0, Vec2d(11.0, 0.0));
  const auto route_path = RoutingToLanePoint(
      *smm, cc, sdc_pose, mapping::LanePoint(mapping::ElementId(160), 1.0));
  const auto route_sections =
      RouteSectionsFromCompositeLanePath(*smm, *route_path);
  SendRouteSectionsAreaToCanvas(psmm, route_sections, "route_sections");

  // Build drive passage.
  ASSIGN_OR_DIE(const auto drive_passage,
                BuildDrivePassage(psmm, /*vision_map_ptr=*/nullptr,
                                  route_path->lane_paths().front(),
                                  route_path->lane_paths().front(),
                                  /*anchor_point=*/mapping::LanePoint(),
                                  route_sections.planning_horizon(psmm),
                                  route_sections.destination(),
                                  /*all_lanes_virtual=*/false,
                                  /*override_speed_limit_mps=*/std::nullopt));
  SendDrivePassageToCanvas(drive_passage, "drive_passage");

  // Constant speed profile test.
  {
    const double length_along_route = 2.0;
    const bool borrow_boundary = false;
    const double ego_v = 1.0;
    ASSIGN_OR_DIE(const auto speed_profile,
                  BuildBeyondLengthAlongRouteConstraint(
                      drive_passage, planner_params.motion_constraint_params(),
                      length_along_route, borrow_boundary, ego_v));
    const auto& t_set = speed_profile.vt_upper_constraint().x();
    const auto& v_set = speed_profile.vt_upper_constraint().y();

    EXPECT_EQ(t_set.size(), 10);
    EXPECT_EQ(t_set.size(), v_set.size());
    for (int i = 0; i < t_set.size(); ++i) {
      if (v_set[i] != std::numeric_limits<double>::max()) {
        EXPECT_NEAR(v_set[i], kMinLCSpeed, kEpsilon);
      }
    }
  }

  // Uniform and decelerated motion test.
  {
    const double length_along_route = 100.0;
    const bool borrow_boundary = false;
    const double ego_v = 10.0;
    ASSIGN_OR_DIE(const auto speed_profile,
                  BuildBeyondLengthAlongRouteConstraint(
                      drive_passage, planner_params.motion_constraint_params(),
                      length_along_route, borrow_boundary, ego_v));
    const auto& t_set = speed_profile.vt_upper_constraint().x();
    const auto& v_set = speed_profile.vt_upper_constraint().y();

    EXPECT_EQ(t_set.size(), 10);
    EXPECT_EQ(t_set.size(), v_set.size());
    for (int i = 0; i < t_set.size(); ++i) {
      if (i < 7) {
        EXPECT_NEAR(v_set[i], std::numeric_limits<double>::max(), kEpsilon);
      } else if (i == 7) {
        EXPECT_NEAR(v_set[i], 9.89645, kEpsilon);
      } else if (i == 8) {
        EXPECT_NEAR(v_set[i], 8.29645, kEpsilon);
      } else {
        EXPECT_NEAR(v_set[i], 6.69645, kEpsilon);
      }
    }
  }
  EXPECT_TRUE(true);
}
}  // namespace
}  // namespace qcraft::planner

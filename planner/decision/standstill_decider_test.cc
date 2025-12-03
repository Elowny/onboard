#include "onboard/planner/decision/standstill_decider.h"

#include <memory>
#include <optional>
#include <utility>

#include "glog/logging.h"

#include "gtest/gtest.h"

#include "onboard/container/strong_int.h"
#include "onboard/global/buffered_logger.h"
#include "onboard/maps/lane_path.h"
#include "onboard/maps/semantic_map_defs.h"
#include "onboard/math/geometry/halfplane.h"
#include "onboard/params/param_manager.h"
#include "onboard/params/vehicle_param_api.h"
#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/planner/router/drive_passage_builder.h"
#include "onboard/planner/test_util/load_psmm_util.h"
#include "onboard/utils/status_macros.h"

namespace qcraft::planner {
namespace {

ConstraintProto::StopLineProto GenerateStopLineProto(
    const DrivePassage& passage, SourceProto::TypeCase type,
    double stop_line_s) {
  ConstraintProto::StopLineProto stop_line;
  ASSIGN_OR_DIE(const auto curbs, passage.QueryCurbPointAtS(stop_line_s));

  stop_line.set_s(stop_line_s);
  stop_line.set_standoff(0.0);
  stop_line.set_time(0.0);
  HalfPlane halfplane(curbs.first, curbs.second);
  halfplane.ToProto(stop_line.mutable_half_plane());

  switch (type) {
    case SourceProto::TypeCase::kTrafficLight:
      stop_line.mutable_source()->mutable_traffic_light();
      break;
    case SourceProto::TypeCase::kCrosswalk:
      stop_line.mutable_source()->mutable_crosswalk();
      break;
    case SourceProto::TypeCase::kPullOver:
      stop_line.mutable_source()->mutable_pull_over();
      break;
    case SourceProto::TypeCase::kBrakeToStop:
      stop_line.mutable_source()->mutable_brake_to_stop();
      break;
    case SourceProto::TypeCase::kCloseObject:
      stop_line.mutable_source()->mutable_close_object();
      break;
    case SourceProto::TypeCase::kSpeedBump:
      stop_line.mutable_source()->mutable_speed_bump();
      break;
    case SourceProto::TypeCase::kIntersection:
      stop_line.mutable_source()->mutable_intersection();
      break;
    case SourceProto::TypeCase::kLcEndOfCurrentLane:
      stop_line.mutable_source()->mutable_lc_end_of_current_lane();
      break;
    case SourceProto::TypeCase::kPedestrianObject:
      stop_line.mutable_source()->mutable_pedestrian_object();
      break;
    case SourceProto::TypeCase::kToll:
      stop_line.mutable_source()->mutable_toll();
      break;
    case SourceProto::TypeCase::kNoBlock:
      stop_line.mutable_source()->mutable_no_block();
      break;
    case SourceProto::TypeCase::kEndOfPathBoundary:
      stop_line.mutable_source()->mutable_end_of_path_boundary();
      break;
    case SourceProto::TypeCase::kEndOfCurrentLanePath:
      stop_line.mutable_source()->mutable_end_of_current_lane_path();
      break;
    case SourceProto::TypeCase::kRouteDestination:
      stop_line.mutable_source()->mutable_route_destination();
      break;
    case SourceProto::TypeCase::kParkingBrakeRelease:
      stop_line.mutable_source()->mutable_parking_brake_release();
      break;
    case SourceProto::TypeCase::kBlockingStaticObject:
      stop_line.mutable_source()->mutable_blocking_static_object();
      break;
    case SourceProto::TypeCase::kStandby:
      stop_line.mutable_source()->mutable_standby();
      break;
    case SourceProto::TypeCase::kStopSign:
      stop_line.mutable_source()->mutable_stop_sign();
      break;
    case SourceProto::TypeCase::kStandstill:
      stop_line.mutable_source()->mutable_standstill();
      break;
    case SourceProto::TypeCase::kEndOfLocalPath:
      stop_line.mutable_source()->mutable_end_of_local_path();
      break;
    case SourceProto::TypeCase::kBeyondLengthAlongRoute:
      stop_line.mutable_source()->mutable_beyond_length_along_route();
      break;
    case SourceProto::TypeCase::kSolidLineWithinBoundary:
      stop_line.mutable_source()->mutable_solid_line_within_boundary();
      break;
    case SourceProto::TypeCase::kOccludedObject:
      stop_line.mutable_source()->mutable_occluded_object();
      break;
    case SourceProto::TypeCase::kDenseTrafficFlow:
      stop_line.mutable_source()->mutable_dense_traffic_flow();
      break;
    case SourceProto::TypeCase::kStopPolyline:
      stop_line.mutable_source()->mutable_stop_polyline();
      break;
    case SourceProto::TypeCase::TYPE_NOT_SET:
      break;
  }

  return stop_line;
}

TEST(StandStillDeciderTest, InvalidAvStateTest) {
  // Load planner semantic map.
  const auto& psmm = planner::CreateDojoTestPSMM();

  // Get vehicle params.
  RunParamsProtoV2 run_params;
  auto param_manager = CreateParamManagerFromCarId("Q8001");
  CHECK(param_manager != nullptr);
  param_manager->GetRunParams(&run_params);
  const VehicleGeometryParamsProto vehicle_geometry_params =
      run_params.vehicle_params().vehicle_geometry_params();

  // Build drive passage.
  const auto current_lane_path = mapping::LanePath(
      psmm.semantic_map_manager(), /*lane_ids=*/
      {mapping::ElementId(2448), mapping::ElementId(1), mapping::ElementId(34)},
      /*start_fraction=*/0.0, /*end_fraction=*/1.0);
  const auto drive_passage = *BuildDrivePassageFromLanePath(
      psmm, current_lane_path, /*step_s=*/1.0,
      /*avoid_loop=*/true, /*backward_extend_len=*/0.0,
      /*required_planning_horizon=*/0.0,
      /*required_backward_len=*/0.0,
      /*override_speed_limit_mps=*/std::nullopt);

  // There is no stop line ahead.
  {
    ApolloTrajectoryPointProto plan_start_point;
    plan_start_point.mutable_path_point()->set_x(0.0);
    plan_start_point.mutable_path_point()->set_y(0.0);
    plan_start_point.set_v(0.0);

    ASSIGN_OR_DIE(
        const auto standstill_stop_lines,
        BuildStandstillConstraints(vehicle_geometry_params, plan_start_point,
                                   drive_passage, /*stop_lines*/ {}));
    EXPECT_TRUE(standstill_stop_lines.empty());
  }

  // The av speed is greater than 0.2 m/s^2.
  {
    ApolloTrajectoryPointProto plan_start_point;
    plan_start_point.mutable_path_point()->set_x(0.0);
    plan_start_point.mutable_path_point()->set_y(0.0);
    plan_start_point.set_v(1.0);

    ASSIGN_OR_DIE(const auto standstill_stop_lines,
                  BuildStandstillConstraints(
                      vehicle_geometry_params, plan_start_point, drive_passage,
                      /*stop_lines*/
                      {GenerateStopLineProto(
                          drive_passage, SourceProto::TypeCase::kTrafficLight,
                          /*stop_line_s*/ 69.7)}));
    EXPECT_TRUE(standstill_stop_lines.empty());
  }

  // The dist between av front edge and stop line is greater than 5.0m.
  {
    ApolloTrajectoryPointProto plan_start_point;
    plan_start_point.mutable_path_point()->set_x(56.0);
    plan_start_point.mutable_path_point()->set_y(0.0);
    plan_start_point.set_v(0.0);
    ASSIGN_OR_DIE(const auto standstill_stop_lines,
                  BuildStandstillConstraints(
                      vehicle_geometry_params, plan_start_point, drive_passage,
                      /*stop_lines*/
                      {GenerateStopLineProto(
                          drive_passage, SourceProto::TypeCase::kTrafficLight,
                          /*stop_line_s*/ 69.7)}));
    EXPECT_TRUE(standstill_stop_lines.empty());
  }
}

TEST(StandStillDeciderTest, StopLineTypeTest) {
  // Load planner semantic map.
  const auto& psmm = planner::CreateDojoTestPSMM();

  // Get vehicle params.
  RunParamsProtoV2 run_params;
  auto param_manager = CreateParamManagerFromCarId("Q8001");
  CHECK(param_manager != nullptr);
  param_manager->GetRunParams(&run_params);
  const VehicleGeometryParamsProto vehicle_geometry_params =
      run_params.vehicle_params().vehicle_geometry_params();

  // Build drive passage.
  const auto current_lane_path = mapping::LanePath(
      psmm.semantic_map_manager(), /*lane_ids=*/
      {mapping::ElementId(2448), mapping::ElementId(1), mapping::ElementId(34)},
      /*start_fraction=*/0.0, /*end_fraction=*/1.0);
  const auto drive_passage = *BuildDrivePassageFromLanePath(
      psmm, current_lane_path, /*step_s=*/1.0,
      /*avoid_loop=*/true, /*backward_extend_len=*/0.0,
      /*required_planning_horizon=*/0.0,
      /*required_backward_len=*/0.0,
      /*override_speed_limit_mps=*/std::nullopt);

  ApolloTrajectoryPointProto plan_start_point;
  plan_start_point.mutable_path_point()->set_x(61.0);
  plan_start_point.mutable_path_point()->set_y(0.0);
  plan_start_point.set_v(0.0);

  // Valid stop line type test.
  {
    const std::vector<SourceProto::TypeCase> valid_stop_line_types = {
        SourceProto::TypeCase::kTrafficLight, SourceProto::TypeCase::kCrosswalk,
        SourceProto::TypeCase::kPullOver, SourceProto::TypeCase::kBrakeToStop};

    for (const auto type : valid_stop_line_types) {
      ASSIGN_OR_DIE(
          const auto standstill_stop_lines,
          BuildStandstillConstraints(
              vehicle_geometry_params, plan_start_point, drive_passage,
              /*stop_lines*/
              {GenerateStopLineProto(drive_passage, type,
                                     /*stop_line_s*/ 69.7)}));
      EXPECT_EQ(standstill_stop_lines.size(), 1);
    }
  }

  // Invalid stop line type test.
  {
    const std::vector<SourceProto::TypeCase> invalid_stop_line_types = {
        SourceProto::TypeCase::kCloseObject,
        SourceProto::TypeCase::kSpeedBump,
        SourceProto::TypeCase::kIntersection,
        SourceProto::TypeCase::kLcEndOfCurrentLane,
        SourceProto::TypeCase::kPedestrianObject,
        SourceProto::TypeCase::kToll,
        SourceProto::TypeCase::kNoBlock,
        SourceProto::TypeCase::kEndOfPathBoundary,
        SourceProto::TypeCase::kEndOfCurrentLanePath,
        SourceProto::TypeCase::kRouteDestination,
        SourceProto::TypeCase::kParkingBrakeRelease,
        SourceProto::TypeCase::kBlockingStaticObject,
        SourceProto::TypeCase::kStandby,
        SourceProto::TypeCase::kStandstill,
        SourceProto::TypeCase::kEndOfLocalPath,
        SourceProto::TypeCase::kBeyondLengthAlongRoute,
        SourceProto::TypeCase::kSolidLineWithinBoundary,
        SourceProto::TypeCase::kOccludedObject,
        SourceProto::TypeCase::kDenseTrafficFlow,
        SourceProto::TypeCase::kStopPolyline,
        SourceProto::TypeCase::TYPE_NOT_SET};

    for (const auto type : invalid_stop_line_types) {
      ASSIGN_OR_DIE(
          const auto standstill_stop_lines,
          BuildStandstillConstraints(
              vehicle_geometry_params, plan_start_point, drive_passage,
              /*stop_lines*/
              {GenerateStopLineProto(drive_passage, type,
                                     /*stop_line_s*/ 69.7)}));
      EXPECT_TRUE(standstill_stop_lines.empty());
    }
  }
}

}  // namespace
}  // namespace qcraft::planner

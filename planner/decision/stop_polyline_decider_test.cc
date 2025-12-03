#include "onboard/planner/decision/stop_polyline_decider.h"

#include <memory>
#include <optional>
#include <ostream>
#include <utility>

#include "glog/logging.h"
#include "google/protobuf/repeated_ptr_field.h"

#include "gtest/gtest.h"

#include "onboard/container/strong_int.h"
#include "onboard/global/buffered_logger.h"
#include "onboard/maps/lane_path.h"
#include "onboard/maps/proto/online_semantic_map.pb.h"
#include "onboard/maps/semantic_map_defs.h"
#include "onboard/math/geometry/box2d.h"
#include "onboard/math/geometry/proto/affine_transformation.pb.h"
#include "onboard/math/geometry/util.h"
#include "onboard/math/piecewise_linear_function.h"
#include "onboard/math/util.h"
#include "onboard/math/vec.h"
#include "onboard/params/param_manager.h"
#include "onboard/params/vehicle_param_api.h"
#include "onboard/planner/router/drive_passage_builder.h"
#include "onboard/planner/test_util/load_psmm_util.h"
#include "onboard/planner/util/online_map_converter.h"
#include "onboard/planner/util/planner_semantic_map_manager_builder.h"
#include "onboard/planner/util/vehicle_geometry_util.h"
#include "onboard/proto/trajectory_point.pb.h"
#include "onboard/proto/vehicle.pb.h"
#include "onboard/utils/status_macros.h"

namespace qcraft::planner {
namespace {

mapping::OnlinePolylineProto MockOnlinePolylineProto(const Vec2d& begin,
                                                     const Vec2d& end, int id) {
  mapping::OnlinePolylineProto proto;
  proto.set_id(id);
  proto.set_type(mapping::OnlinePolylineProto::STOP_LINE);

  const double length = begin.DistanceTo(end);
  const PiecewiseLinearFunction<Vec2d> smooth_points_plf({0.0, length},
                                                         {begin, end});

  constexpr double kSampleStep = 0.5;
  const int size = CeilToInt(length / kSampleStep) + 1;
  proto.mutable_smooth_points()->Reserve(size);
  for (double s = 0.0; s < length + kSampleStep; s += kSampleStep) {
    const auto resample_point = smooth_points_plf(s);
    auto* new_smooth_point = proto.add_smooth_points();
    new_smooth_point->set_x(resample_point.x());
    new_smooth_point->set_y(resample_point.y());
  }
  return proto;
}

TEST(StopPolyLineDeciderTest, BaseTest) {
  // Get vehicle params.
  RunParamsProtoV2 run_params;
  auto param_manager = CreateParamManagerFromCarId("Q8001");
  CHECK(param_manager != nullptr);
  param_manager->GetRunParams(&run_params);
  const VehicleGeometryParamsProto vehicle_geometry_params =
      run_params.vehicle_params().vehicle_geometry_params();

  // Plan start point.
  ApolloTrajectoryPointProto plan_start_point;
  plan_start_point.mutable_path_point()->set_x(0.0);
  plan_start_point.mutable_path_point()->set_y(0.0);
  const auto ego_pos = Vec2dFromApolloTrajectoryPointProto(plan_start_point);

  // Build online semantic map according whole map.
  const auto& whole_psmm = CreateDojoTestPSMM();
  ASSIGN_OR_DIE(auto online_smm_proto,
                RunOnlineSemanticMapConverter(whole_psmm,
                                              OnlineSemanticMapConverterOption{
                                                  .smooth_x = ego_pos.x(),
                                                  .smooth_y = ego_pos.y(),
                                                  .smooth_yaw = 0.0,
                                                  .look_ahead_distance = 120.0,
                                                  .look_back_distance = 10.0,
                                              }));
  // Mock stop line.
  *online_smm_proto.add_polylines() = MockOnlinePolylineProto(
      /*begin*/ Vec2d(70.0, 3.5), /*end*/ Vec2d(70.0, -3.5), /*id*/ 1);

  // Build psmm according online semantic map.
  ASSIGN_OR_DIE(const auto psmm_ptr, BuildOnlineMapPsmm(online_smm_proto));
  const auto& psmm = *psmm_ptr;

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

  // Calculate av frenet box.
  const Box2d av_box = ComputeAvBox(
      ego_pos, plan_start_point.path_point().theta(), vehicle_geometry_params);
  ASSIGN_OR_DIE(const auto av_frenet_box,
                drive_passage.QueryFrenetBoxAt(av_box));

  const StopPolylineDeciderStateProto empty_decider_state_proto;

  // Generate stop line test.
  {
    ASSIGN_OR_DIE(auto output, BuildStopPolylineConstraints(
                                   psmm, drive_passage, av_frenet_box,
                                   empty_decider_state_proto,
                                   /*is_pass_nearest_stop_line*/ false));
    const auto stop_lines = std::move(output.stop_lines);
    EXPECT_EQ(stop_lines.size(), 1);
    const auto& stop_line = stop_lines.front();
    EXPECT_NEAR(stop_line.s(), 70.1181, 1e-2);
  }

  // Pass nearest stop line test 1.
  {
    ASSIGN_OR_DIE(auto output, BuildStopPolylineConstraints(
                                   psmm, drive_passage, av_frenet_box,
                                   empty_decider_state_proto,
                                   /*is_pass_nearest_stop_line*/ true));
    const auto stop_lines = std::move(output.stop_lines);
    EXPECT_TRUE(stop_lines.empty());
    const auto stop_polyline_state = std::move(output.stop_polyline_state);
    const auto& passable_stop_point = stop_polyline_state.passable_stop_point();
    EXPECT_NEAR(passable_stop_point.x(), 70.0, 1e-2);
    EXPECT_NEAR(passable_stop_point.y(), 0.25, 1e-2);
  }

  // Pass nearest stop line test 2.
  {
    StopPolylineDeciderStateProto decider_state_proto;
    auto* mutable_passable_stop_point =
        decider_state_proto.mutable_passable_stop_point();
    mutable_passable_stop_point->set_x(70.0);
    mutable_passable_stop_point->set_y(0.25);

    ASSIGN_OR_DIE(auto output,
                  BuildStopPolylineConstraints(
                      psmm, drive_passage, av_frenet_box, decider_state_proto,
                      /*is_pass_nearest_stop_line*/ true));
    const auto stop_lines = std::move(output.stop_lines);
    EXPECT_TRUE(stop_lines.empty());
    const auto stop_polyline_state = std::move(output.stop_polyline_state);
    const auto& passable_stop_point = stop_polyline_state.passable_stop_point();
    EXPECT_NEAR(passable_stop_point.x(), 70.0, 1e-2);
    EXPECT_NEAR(passable_stop_point.y(), 0.25, 1e-2);
  }
}
}  // namespace
}  // namespace qcraft::planner

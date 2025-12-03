#include "onboard/planner/assist/alcc_target_selector.h"

#include <array>
#include <memory>
#include <vector>

#include "absl/status/statusor.h"
#include "absl/time/clock.h"
#include "absl/time/time.h"
#include "absl/types/span.h"

#include "gtest/gtest.h"

#include "common/proto/qalc.pb.h"

#include "onboard/async/thread_pool.h"
#include "onboard/base/macros.h"
#include "onboard/container/strong_int.h"
#include "onboard/global/buffered_logger.h"
#include "onboard/maps/lane_path.h"
#include "onboard/maps/map_selector.h"
#include "onboard/maps/semantic_map_defs.h"
#include "onboard/math/piecewise_linear_function.h"
#include "onboard/math/vec.h"
#include "onboard/params/param_manager.h"
#include "onboard/params/vehicle_param_api.h"
#include "onboard/planner/assist/alcc_scheduler.h"
#include "onboard/planner/assist/lcc_map_builder.h"
#include "onboard/planner/common/driving_map_topo.h"
#include "onboard/planner/object/planner_object.h"
#include "onboard/planner/planner_defs.h"
#include "onboard/planner/planner_flags.h"
#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/planner/router/route_sections.h"
#include "onboard/planner/scheduler/driving_map_topo_builder.h"
#include "onboard/planner/scheduler/scheduler_plot_util.h"
#include "onboard/planner/test_util/load_psmm_util.h"
#include "onboard/planner/test_util/object_prediction_builder.h"
#include "onboard/planner/test_util/perception_object_builder.h"
#include "onboard/planner/test_util/planner_object_builder.h"
#include "onboard/planner/test_util/predicted_trajectory_builder.h"
#include "onboard/planner/test_util/util.h"
#include "onboard/proto/autonomy_state.pb.h"
#include "onboard/proto/perception/fusion/object.pb.h"
#include "onboard/proto/positioning.pb.h"
#include "onboard/utils/history_buffer.h"
#include "onboard/utils/status_macros.h"
#include "onboard/utils/time_util.h"

namespace qcraft {
namespace planner {
namespace {

std::vector<PlannerObject> BuildPlannerObjects() {
  const Vec2d obj_pos = Vec2d(67.0, -3.6);
  const double v = 5.0;
  const double heading = 0.05;
  PerceptionObjectBuilder percep_builder;
  const auto obj_1 = percep_builder.set_id("1")
                         .set_pos(obj_pos)
                         .set_speed(Vec2d::FastUnitFromAngle(heading) * v)
                         .set_accel(Vec2d::Zero())
                         .set_length_width(2.0, 1.0)
                         .Build();
  PlannerObjectBuilder obj_builder;
  obj_builder.set_type(OT_VEHICLE)
      .set_pos(obj_pos)
      .set_v(v)
      .set_theta(heading)
      .set_object(obj_1);
  auto* object_pred_builder = obj_builder.get_object_prediction_builder();
  object_pred_builder->add_predicted_trajectory()->set_straight_line(
      obj_pos, heading, /*duration=*/10.0, v, /*acc=*/0.0);
  const auto planner_obj = obj_builder.Build();
  return {planner_obj};
}

TEST(AlccTargetSelectorTest, LaneKeeping) {
  auto param_manager = CreateParamManagerFromCarId("Q0001");

  RunParamsProtoV2 run_params;
  param_manager->GetRunParams(&run_params);
  const auto& vehicle_geom =
      run_params.vehicle_params().vehicle_geometry_params();

  SetMap("dojo");
  const auto& psmm = CreateDojoTestPSMM();

  // Construct origin lane path.
  const auto origin_lane_path =
      mapping::LanePath(psmm.semantic_map_manager(), /*lane_ids=*/
                        {mapping::ElementId(2448), mapping::ElementId(1),
                         mapping::ElementId(34), mapping::ElementId(2471)},
                        /*start_fraction=*/0.5, /*end_fraction=*/1.0);
  ASSIGN_OR_DIE(const auto dm, BuildDrivingMapByRouteOnOfflineMap(
                                   psmm, RouteSections::BuildFromLanePath(
                                             psmm, origin_lane_path)));
  // Construct sdc pose.
  absl::Time plan_time = absl::Now();
  const PoseProto sdc_pose = CreatePose(
      ToUnixDoubleSeconds(plan_time), Vec2d(33.0, 0.0), 0.0, Vec2d(11.0, 0.0));
  const auto start_point = ConvertToTrajPointProto(sdc_pose);

  // Construct pre alc state.
  const auto prev_alc_state = QALCState::ALC_STANDBY_ENABLE;
  const auto prev_lc_direction = LaneChangeDirection::LCD_NONE;

  // Build local lane map.
  ASSIGN_OR_DIE(
      const auto candidate_lanes,
      BuildLocalLaneMap(BuildLocalMapInput{
          .psmm = &psmm,
          .driving_map_topo = &dm,
          .origin_lane_path = &origin_lane_path,
          .target_lane_path = nullptr,
          .alc_state = prev_alc_state,
          .lc_direction = prev_lc_direction,
          .cut_off_length = kMinLcLaneLength + kDrivePassageKeepBehindLength,
          .projection_range =
              kMaxTravelDistanceBetweenFrames + kDrivePassageKeepBehindLength,
          .keep_behind_length = kDrivePassageKeepBehindLength}));
  // Show local lane map in vantage.
  SendLocalLaneMapToCanvas(candidate_lanes, psmm, "SBE/local_lane_map");

  auto thread_pool =
      std::make_unique<ThreadPool>(FLAGS_planner_thread_pool_size);

  // Build objects.
  const auto planner_objects = BuildPlannerObjects();
  SpacetimeTrajectoryManager st_traj_mgr(planner_objects, thread_pool.get());
  EXPECT_EQ(st_traj_mgr.trajectories().size(), 1);

  HistoryBufferAbslTime<PiecewiseLinearFunction<double>>
      online_map_drift_buffer;

  // ALC_STANDBY_ENABLE + LC_CMD_NONE scheduler.
  ASSIGN_OR_DIE(const auto scheduler_results,
                RunAlccScheduler(
                    AlccSchedulerInput{
                        .psmm = &psmm,
                        .vehicle_geom = &vehicle_geom,
                        .plan_start_point = &start_point,
                        .st_traj_mgr = &st_traj_mgr,
                        .candidate_lanes = &candidate_lanes,
                        .lc_cmd = DriverAction::LC_CMD_CANCEL,
                        .prev_alc_state = prev_alc_state,
                        .lcc_cruising_speed_mps = 11.0,
                        .online_map_drift_buffer = &online_map_drift_buffer,
                    },
                    thread_pool.get()));
  EXPECT_EQ(scheduler_results.size(), 1);
  const auto& lane_keep_scheduler_result = scheduler_results[0];
  const auto& map_path_sl = lane_keep_scheduler_result.sl_boundary;
  const auto& dp = lane_keep_scheduler_result.drive_passage;

  const auto select_target_status = SelectAlccTarget(
      map_path_sl, dp, start_point, vehicle_geom, &st_traj_mgr);
  // Should filter out the slow cut in object.
  EXPECT_EQ(st_traj_mgr.trajectories().size(), 0);
  EXPECT_OK(select_target_status);
}

TEST(AlccTargetSelectorTest, LaneChange) {
  auto param_manager = CreateParamManagerFromCarId("Q0001");

  RunParamsProtoV2 run_params;
  param_manager->GetRunParams(&run_params);
  const auto& vehicle_geom =
      run_params.vehicle_params().vehicle_geometry_params();

  SetMap("dojo");
  const auto& psmm = CreateDojoTestPSMM();

  // Construct origin lane path.
  const auto origin_lane_path =
      mapping::LanePath(psmm.semantic_map_manager(), /*lane_ids=*/
                        {mapping::ElementId(2448), mapping::ElementId(1),
                         mapping::ElementId(34), mapping::ElementId(2471)},
                        /*start_fraction=*/0.5, /*end_fraction=*/1.0);
  ASSIGN_OR_DIE(const auto dm, BuildDrivingMapByRouteOnOfflineMap(
                                   psmm, RouteSections::BuildFromLanePath(
                                             psmm, origin_lane_path)));
  // Construct sdc pose.
  absl::Time plan_time = absl::Now();
  const PoseProto sdc_pose = CreatePose(
      ToUnixDoubleSeconds(plan_time), Vec2d(33.0, 0.0), 0.0, Vec2d(11.0, 0.0));
  const auto start_point = ConvertToTrajPointProto(sdc_pose);

  // Construct pre alc state.
  const auto prev_alc_state = QALCState::ALC_STANDBY_ENABLE;
  const auto prev_lc_direction = LaneChangeDirection::LCD_NONE;

  // Build local lane map.
  ASSIGN_OR_DIE(
      const auto candidate_lanes,
      BuildLocalLaneMap(BuildLocalMapInput{
          .psmm = &psmm,
          .driving_map_topo = &dm,
          .origin_lane_path = &origin_lane_path,
          .target_lane_path = nullptr,
          .alc_state = prev_alc_state,
          .lc_direction = prev_lc_direction,
          .cut_off_length = kMinLcLaneLength + kDrivePassageKeepBehindLength,
          .projection_range =
              kMaxTravelDistanceBetweenFrames + kDrivePassageKeepBehindLength,
          .keep_behind_length = kDrivePassageKeepBehindLength}));
  // Show local lane map in vantage.
  SendLocalLaneMapToCanvas(candidate_lanes, psmm, "SBE/local_lane_map");

  auto thread_pool =
      std::make_unique<ThreadPool>(FLAGS_planner_thread_pool_size);

  // Build objects.
  const auto planner_objects = BuildPlannerObjects();
  SpacetimeTrajectoryManager st_traj_mgr(planner_objects, thread_pool.get());
  EXPECT_EQ(st_traj_mgr.trajectories().size(), 1);

  HistoryBufferAbslTime<PiecewiseLinearFunction<double>>
      online_map_drift_buffer;

  // ALC_STANDBY_ENABLE + LC_CMD_RIGHT.
  ASSIGN_OR_DIE(const auto scheduler_results,
                RunAlccScheduler(
                    AlccSchedulerInput{
                        .psmm = &psmm,
                        .vehicle_geom = &vehicle_geom,
                        .plan_start_point = &start_point,
                        .st_traj_mgr = &st_traj_mgr,
                        .candidate_lanes = &candidate_lanes,
                        .lc_cmd = DriverAction::LC_CMD_RIGHT,
                        .prev_alc_state = prev_alc_state,
                        .lcc_cruising_speed_mps = 11.0,
                        .online_map_drift_buffer = &online_map_drift_buffer,
                    },
                    thread_pool.get()));
  EXPECT_EQ(scheduler_results.size(), 2);
  const auto& lane_change_scheduler_result = scheduler_results[1];
  const auto& map_path_sl = lane_change_scheduler_result.sl_boundary;
  const auto& dp = lane_change_scheduler_result.drive_passage;

  const auto select_target_status = SelectAlccTarget(
      map_path_sl, dp, start_point, vehicle_geom, &st_traj_mgr);
  // Should not filter out the slow cut in object since it's on target lane.
  EXPECT_EQ(st_traj_mgr.trajectories().size(), 1);
  EXPECT_OK(select_target_status);
}

}  // namespace
}  // namespace planner
}  // namespace qcraft

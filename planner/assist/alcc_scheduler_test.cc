#include "onboard/planner/assist/alcc_scheduler.h"

#include <initializer_list>
#include <memory>
#include <string>

#include "absl/time/clock.h"
#include "absl/time/time.h"

#include "gtest/gtest.h"

#include "onboard/container/strong_int.h"
#include "onboard/global/buffered_logger.h"
#include "onboard/maps/map_selector.h"
#include "onboard/maps/semantic_map_defs.h"
#include "onboard/math/vec.h"
#include "onboard/params/param_manager.h"
#include "onboard/params/vehicle_param_api.h"
#include "onboard/planner/assist/lcc_map_builder.h"
#include "onboard/planner/common/driving_map_topo.h"
#include "onboard/planner/planner_defs.h"
#include "onboard/planner/planner_flags.h"
#include "onboard/planner/router/plot_util.h"
#include "onboard/planner/router/route_sections.h"
#include "onboard/planner/scheduler/driving_map_topo_builder.h"
#include "onboard/planner/scheduler/scheduler_plot_util.h"
#include "onboard/planner/test_util/load_psmm_util.h"
#include "onboard/planner/test_util/util.h"
#include "onboard/proto/positioning.pb.h"
#include "onboard/utils/status_macros.h"
#include "onboard/utils/time_util.h"

namespace qcraft {
namespace planner {
namespace {

TEST(AlccSchedulerTest, AlcStandbyEnableTest) {
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

  SpacetimeTrajectoryManager st_traj_mgr;

  auto thread_pool =
      std::make_unique<ThreadPool>(FLAGS_planner_thread_pool_size);

  HistoryBufferAbslTime<PiecewiseLinearFunction<double>>
      online_map_drift_buffer;

  // ALC_STANDBY_ENABLE + LC_CMD_RIGHT + Right Neighbor Lane.
  {
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

    for (int i = 0; i < scheduler_results.size(); ++i) {
      SendDrivePassageToCanvas(
          scheduler_results[i].drive_passage,
          "SBE/R/drive_passage_" +
              QALCState_Name(scheduler_results[i].alc_state));
    }
    EXPECT_EQ(scheduler_results.size(), 2);
    const auto& prepare_output = scheduler_results[0];
    EXPECT_EQ(prepare_output.alc_state, ALC_PREPARE);
    EXPECT_EQ(prepare_output.lc_direction, LaneChangeDirection::LCD_NONE);
    const auto& ongoing_output = scheduler_results[1];
    EXPECT_EQ(ongoing_output.alc_state, ALC_ONGOING);
    EXPECT_EQ(ongoing_output.lc_direction, LaneChangeDirection::LCD_RIGHT);
    EXPECT_EQ(ongoing_output.drive_passage.lane_path().lane_ids().front(),
              mapping::ElementId(3));
  }

  // ALC_STANDBY_ENABLE + LC_CMD_LEFT + Left Neighbor Lane.
  {
    ASSIGN_OR_DIE(const auto scheduler_results,
                  RunAlccScheduler(
                      AlccSchedulerInput{
                          .psmm = &psmm,
                          .vehicle_geom = &vehicle_geom,
                          .plan_start_point = &start_point,
                          .st_traj_mgr = &st_traj_mgr,
                          .candidate_lanes = &candidate_lanes,
                          .lc_cmd = DriverAction::LC_CMD_LEFT,
                          .prev_alc_state = prev_alc_state,
                          .lcc_cruising_speed_mps = 11.0,
                          .online_map_drift_buffer = &online_map_drift_buffer,
                      },
                      thread_pool.get()));

    for (int i = 0; i < scheduler_results.size(); ++i) {
      SendDrivePassageToCanvas(
          scheduler_results[i].drive_passage,
          "SBE/L/drive_passage_" +
              QALCState_Name(scheduler_results[i].alc_state));
    }
    EXPECT_EQ(scheduler_results.size(), 2);
    const auto& prepare_output = scheduler_results[0];
    EXPECT_EQ(prepare_output.alc_state, ALC_PREPARE);
    EXPECT_EQ(prepare_output.lc_direction, LaneChangeDirection::LCD_NONE);
    const auto& ongoing_output = scheduler_results[1];
    EXPECT_EQ(ongoing_output.alc_state, ALC_ONGOING);
    EXPECT_EQ(ongoing_output.lc_direction, LaneChangeDirection::LCD_LEFT);
    EXPECT_EQ(ongoing_output.drive_passage.lane_path().lane_ids().front(),
              mapping::ElementId(2));
  }

  // ALC_STANDBY_ENABLE + LC_CMD_RIGHT + No Right Neighbor Lane.
  {
    const auto temp_candidate_lanes = std::array<mapping::LanePath, 3>{
        candidate_lanes[0], candidate_lanes[1], mapping::LanePath()};
    ASSIGN_OR_DIE(const auto scheduler_results,
                  RunAlccScheduler(
                      AlccSchedulerInput{
                          .psmm = &psmm,
                          .vehicle_geom = &vehicle_geom,
                          .plan_start_point = &start_point,
                          .st_traj_mgr = &st_traj_mgr,
                          .candidate_lanes = &temp_candidate_lanes,
                          .lc_cmd = DriverAction::LC_CMD_RIGHT,
                          .prev_alc_state = prev_alc_state,
                          .lcc_cruising_speed_mps = 11.0,
                          .online_map_drift_buffer = &online_map_drift_buffer,
                      },
                      thread_pool.get()));

    EXPECT_EQ(scheduler_results.size(), 1);
    const auto& prepare_output = scheduler_results[0];
    EXPECT_EQ(prepare_output.alc_state, ALC_PREPARE);
    EXPECT_EQ(prepare_output.lc_direction, LaneChangeDirection::LCD_NONE);
  }

  // ALC_STANDBY_ENABLE + LC_CMD_LEFT + No Left Neighbor Lane.
  {
    const auto temp_candidate_lanes = std::array<mapping::LanePath, 3>{
        mapping::LanePath(), candidate_lanes[1], candidate_lanes[2]};
    ASSIGN_OR_DIE(const auto scheduler_results,
                  RunAlccScheduler(
                      AlccSchedulerInput{
                          .psmm = &psmm,
                          .vehicle_geom = &vehicle_geom,
                          .plan_start_point = &start_point,
                          .st_traj_mgr = &st_traj_mgr,
                          .candidate_lanes = &temp_candidate_lanes,
                          .lc_cmd = DriverAction::LC_CMD_LEFT,
                          .prev_alc_state = prev_alc_state,
                          .lcc_cruising_speed_mps = 11.0,
                          .online_map_drift_buffer = &online_map_drift_buffer,
                      },
                      thread_pool.get()));

    EXPECT_EQ(scheduler_results.size(), 1);
    const auto& prepare_output = scheduler_results[0];
    EXPECT_EQ(prepare_output.alc_state, ALC_PREPARE);
    EXPECT_EQ(prepare_output.lc_direction, LaneChangeDirection::LCD_NONE);
  }

  // ALC_STANDBY_ENABLE + LC_CMD_NONE
  {
    ASSIGN_OR_DIE(const auto scheduler_results,
                  RunAlccScheduler(
                      AlccSchedulerInput{
                          .psmm = &psmm,
                          .vehicle_geom = &vehicle_geom,
                          .plan_start_point = &start_point,
                          .st_traj_mgr = &st_traj_mgr,
                          .candidate_lanes = &candidate_lanes,
                          .lc_cmd = DriverAction::LC_CMD_NONE,
                          .prev_alc_state = prev_alc_state,
                          .lcc_cruising_speed_mps = 11.0,
                          .online_map_drift_buffer = &online_map_drift_buffer,
                      },
                      thread_pool.get()));
    EXPECT_EQ(scheduler_results.size(), 1);
    const auto& prepare_output = scheduler_results[0];
    EXPECT_EQ(prepare_output.alc_state, ALC_STANDBY_ENABLE);
    EXPECT_EQ(prepare_output.lc_direction, LaneChangeDirection::LCD_NONE);
  }

  // ALC_STANDBY_ENABLE + LC_CMD_CANCEL
  {
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
    const auto& prepare_output = scheduler_results[0];
    EXPECT_EQ(prepare_output.alc_state, ALC_STANDBY_ENABLE);
    EXPECT_EQ(prepare_output.lc_direction, LaneChangeDirection::LCD_NONE);
  }
}

TEST(AlccSchedulerTest, AlcPrepareTest) {
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
      ToUnixDoubleSeconds(plan_time), Vec2d(42.0, 0.0), 0.0, Vec2d(11.0, 0.0));
  const auto start_point = ConvertToTrajPointProto(sdc_pose);

  // Construct pre alc state.
  const auto prev_alc_state = QALCState::ALC_PREPARE;
  auto prev_lc_direction = LaneChangeDirection::LCD_LEFT;

  SpacetimeTrajectoryManager st_traj_mgr;
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

  auto thread_pool =
      std::make_unique<ThreadPool>(FLAGS_planner_thread_pool_size);

  HistoryBufferAbslTime<PiecewiseLinearFunction<double>>
      online_map_drift_buffer;

  // ALC_PREPARE + LCD_LEFT + LC_CMD_CANCEL.
  {
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
    const auto& prepare_output = scheduler_results[0];
    EXPECT_EQ(prepare_output.alc_state, ALC_STANDBY_ENABLE);
    EXPECT_EQ(prepare_output.lc_direction, LaneChangeDirection::LCD_NONE);
  }

  // ALC_PREPARE + LCD_LEFT + LC_CMD_LEFT.
  {
    ASSIGN_OR_DIE(const auto scheduler_results,
                  RunAlccScheduler(
                      AlccSchedulerInput{
                          .psmm = &psmm,
                          .vehicle_geom = &vehicle_geom,
                          .plan_start_point = &start_point,
                          .st_traj_mgr = &st_traj_mgr,
                          .candidate_lanes = &candidate_lanes,
                          .lc_cmd = DriverAction::LC_CMD_LEFT,
                          .prev_alc_state = prev_alc_state,
                          .lcc_cruising_speed_mps = 11.0,
                          .online_map_drift_buffer = &online_map_drift_buffer,
                      },
                      thread_pool.get()));
    EXPECT_EQ(scheduler_results.size(), 2);
    const auto& prepare_output = scheduler_results[0];
    EXPECT_EQ(prepare_output.alc_state, ALC_PREPARE);
    EXPECT_EQ(prepare_output.lc_direction, LaneChangeDirection::LCD_NONE);
    const auto& ongoing_output = scheduler_results[1];
    EXPECT_EQ(ongoing_output.alc_state, ALC_ONGOING);
    EXPECT_EQ(ongoing_output.lc_direction, LaneChangeDirection::LCD_LEFT);
    EXPECT_EQ(ongoing_output.drive_passage.lane_path().lane_ids().front(),
              mapping::ElementId(2));
  }

  // ALC_PREPARE + LCD_LEFT + LC_CMD_LEFT + No left
  // neighbor lane.
  {
    const auto temp_candidate_lanes = std::array<mapping::LanePath, 3>{
        mapping::LanePath(), candidate_lanes[1], candidate_lanes[2]};
    ASSIGN_OR_DIE(const auto scheduler_results,
                  RunAlccScheduler(
                      AlccSchedulerInput{
                          .psmm = &psmm,
                          .vehicle_geom = &vehicle_geom,
                          .plan_start_point = &start_point,
                          .st_traj_mgr = &st_traj_mgr,
                          .candidate_lanes = &temp_candidate_lanes,
                          .lc_cmd = DriverAction::LC_CMD_LEFT,
                          .prev_alc_state = prev_alc_state,
                          .lcc_cruising_speed_mps = 11.0,
                          .online_map_drift_buffer = &online_map_drift_buffer,
                      },
                      thread_pool.get()));
    EXPECT_EQ(scheduler_results.size(), 1);
    const auto& prepare_output = scheduler_results[0];
    EXPECT_EQ(prepare_output.alc_state, ALC_PREPARE);
    EXPECT_EQ(prepare_output.lc_direction, LaneChangeDirection::LCD_NONE);
  }

  prev_lc_direction = LaneChangeDirection::LCD_RIGHT;

  // ALC_PREPARE + LCD_RIGHT + LC_CMD_CANCEL.
  {
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
    const auto& prepare_output = scheduler_results[0];
    EXPECT_EQ(prepare_output.alc_state, ALC_STANDBY_ENABLE);
    EXPECT_EQ(prepare_output.lc_direction, LaneChangeDirection::LCD_NONE);
  }

  // ALC_PREPARE + LCD_RIGHT + LC_CMD_RIGHT.
  {
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
    const auto& prepare_output = scheduler_results[0];
    EXPECT_EQ(prepare_output.alc_state, ALC_PREPARE);
    EXPECT_EQ(prepare_output.lc_direction, LaneChangeDirection::LCD_NONE);
    const auto& ongoing_output = scheduler_results[1];
    EXPECT_EQ(ongoing_output.alc_state, ALC_ONGOING);
    EXPECT_EQ(ongoing_output.lc_direction, LaneChangeDirection::LCD_RIGHT);
    EXPECT_EQ(ongoing_output.drive_passage.lane_path().lane_ids().front(),
              mapping::ElementId(3));
  }

  // ALC_PREPARE + LCD_RIGHT + LC_CMD_RIGHT + No right
  // neighbor lane.
  {
    const auto temp_candidate_lanes = std::array<mapping::LanePath, 3>{
        candidate_lanes[0], candidate_lanes[1], mapping::LanePath()};
    ASSIGN_OR_DIE(const auto scheduler_results,
                  RunAlccScheduler(
                      AlccSchedulerInput{
                          .psmm = &psmm,
                          .vehicle_geom = &vehicle_geom,
                          .plan_start_point = &start_point,
                          .st_traj_mgr = &st_traj_mgr,
                          .candidate_lanes = &temp_candidate_lanes,
                          .lc_cmd = DriverAction::LC_CMD_RIGHT,
                          .prev_alc_state = prev_alc_state,
                          .lcc_cruising_speed_mps = 11.0,
                          .online_map_drift_buffer = &online_map_drift_buffer,
                      },
                      thread_pool.get()));
    EXPECT_EQ(scheduler_results.size(), 1);
    const auto& prepare_output = scheduler_results[0];
    EXPECT_EQ(prepare_output.alc_state, ALC_PREPARE);
    EXPECT_EQ(prepare_output.lc_direction, LaneChangeDirection::LCD_NONE);
  }
}

TEST(AlccSchedulerTest, AlcOngoingTest) {
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
  // Construct target lane path on left.
  auto target_lane_path =
      mapping::LanePath(psmm.semantic_map_manager(), /*lane_ids=*/
                        {mapping::ElementId(2), mapping::ElementId(938),
                         mapping::ElementId(940), mapping::ElementId(2470)},
                        /*start_fraction=*/0.5, /*end_fraction=*/1.0);

  // Construct sdc pose.
  absl::Time plan_time = absl::Now();
  const PoseProto sdc_pose = CreatePose(
      ToUnixDoubleSeconds(plan_time), Vec2d(42.0, 0.0), 0.0, Vec2d(11.0, 0.0));
  const auto start_point = ConvertToTrajPointProto(sdc_pose);

  // Construct pre alc state.
  const auto prev_alc_state = QALCState::ALC_ONGOING;
  auto prev_lc_direction = LaneChangeDirection::LCD_LEFT;

  // Build lane change left local lane map.
  ASSIGN_OR_DIE(
      auto candidate_lanes,
      BuildLocalLaneMap(BuildLocalMapInput{
          .psmm = &psmm,
          .driving_map_topo = &dm,
          .origin_lane_path = &origin_lane_path,
          .target_lane_path = &target_lane_path,
          .alc_state = prev_alc_state,
          .lc_direction = prev_lc_direction,
          .cut_off_length = kMinLcLaneLength + kDrivePassageKeepBehindLength,
          .projection_range =
              kMaxTravelDistanceBetweenFrames + kDrivePassageKeepBehindLength,
          .keep_behind_length = kDrivePassageKeepBehindLength}));
  auto thread_pool =
      std::make_unique<ThreadPool>(FLAGS_planner_thread_pool_size);

  SpacetimeTrajectoryManager st_traj_mgr;

  HistoryBufferAbslTime<PiecewiseLinearFunction<double>>
      online_map_drift_buffer;

  // ALC_ONGOING + LCD_LEFT + LC_CMD_CANCEL.
  {
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
    const auto& returning_output = scheduler_results[0];
    EXPECT_EQ(returning_output.alc_state, ALC_RETURNING);
    EXPECT_EQ(returning_output.lc_direction, LaneChangeDirection::LCD_NONE);
  }

  // ALC_ONGOING + LCD_LEFT + LC_CMD_LEFT.
  {
    ASSIGN_OR_DIE(const auto scheduler_results,
                  RunAlccScheduler(
                      AlccSchedulerInput{
                          .psmm = &psmm,
                          .vehicle_geom = &vehicle_geom,
                          .plan_start_point = &start_point,
                          .st_traj_mgr = &st_traj_mgr,
                          .candidate_lanes = &candidate_lanes,
                          .lc_cmd = DriverAction::LC_CMD_LEFT,
                          .prev_alc_state = prev_alc_state,
                          .lcc_cruising_speed_mps = 11.0,
                          .online_map_drift_buffer = &online_map_drift_buffer,
                      },
                      thread_pool.get()));
    EXPECT_EQ(scheduler_results.size(), 2);
    const auto& returning_output = scheduler_results[0];
    EXPECT_EQ(returning_output.alc_state, ALC_RETURNING);
    EXPECT_EQ(returning_output.lc_direction, LaneChangeDirection::LCD_NONE);
    const auto& ongoing_output = scheduler_results[1];
    EXPECT_EQ(ongoing_output.alc_state, ALC_ONGOING);
    EXPECT_EQ(ongoing_output.lc_direction, LaneChangeDirection::LCD_LEFT);
    EXPECT_EQ(ongoing_output.drive_passage.lane_path().lane_ids().front(),
              mapping::ElementId(2));
  }

  prev_lc_direction = LaneChangeDirection::LCD_RIGHT;

  // Construct target lane path on right.
  target_lane_path =
      mapping::LanePath(psmm.semantic_map_manager(), /*lane_ids=*/
                        {mapping::ElementId(3), mapping::ElementId(950),
                         mapping::ElementId(35), mapping::ElementId(2472)},
                        /*start_fraction=*/0.5, /*end_fraction=*/1.0);
  // Build lane change right local lane map.
  ASSIGN_OR_DIE(
      candidate_lanes,
      BuildLocalLaneMap(BuildLocalMapInput{
          .psmm = &psmm,
          .driving_map_topo = &dm,
          .origin_lane_path = &origin_lane_path,
          .target_lane_path = &target_lane_path,
          .alc_state = prev_alc_state,
          .lc_direction = prev_lc_direction,
          .cut_off_length = kMinLcLaneLength + kDrivePassageKeepBehindLength,
          .projection_range =
              kMaxTravelDistanceBetweenFrames + kDrivePassageKeepBehindLength,
          .keep_behind_length = kDrivePassageKeepBehindLength}));

  // ALC_ONGOING + LCD_RIGHT + LC_CMD_CANCEL.
  {
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
    const auto& returning_output = scheduler_results[0];
    EXPECT_EQ(returning_output.alc_state, ALC_RETURNING);
    EXPECT_EQ(returning_output.lc_direction, LaneChangeDirection::LCD_NONE);
  }

  // ALC_ONGOING + LCD_RIGHT + LC_CMD_RIGHT.
  {
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
    const auto& returning_output = scheduler_results[0];
    EXPECT_EQ(returning_output.alc_state, ALC_RETURNING);
    EXPECT_EQ(returning_output.lc_direction, LaneChangeDirection::LCD_NONE);
    const auto& ongoing_output = scheduler_results[1];
    EXPECT_EQ(ongoing_output.alc_state, ALC_ONGOING);
    EXPECT_EQ(ongoing_output.lc_direction, LaneChangeDirection::LCD_RIGHT);
    EXPECT_EQ(ongoing_output.drive_passage.lane_path().lane_ids().front(),
              mapping::ElementId(3));
  }
}

TEST(AlccSchedulerTest, AlcCrossingLaneTest) {
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
      ToUnixDoubleSeconds(plan_time), Vec2d(42.0, 0.0), 0.0, Vec2d(11.0, 0.0));
  const auto start_point = ConvertToTrajPointProto(sdc_pose);

  // Construct pre alc state.
  const auto prev_alc_state = QALCState::ALC_CROSSING_LANE;

  auto thread_pool =
      std::make_unique<ThreadPool>(FLAGS_planner_thread_pool_size);

  SpacetimeTrajectoryManager st_traj_mgr;

  HistoryBufferAbslTime<PiecewiseLinearFunction<double>>
      online_map_drift_buffer;

  // ALC_CROSSING_LANE + LCD_LEFT + LC_CMD_LEFT.
  {
    const auto prev_lc_direction = LaneChangeDirection::LCD_LEFT;

    // Construct target lane path on left.
    const auto target_lane_path =
        mapping::LanePath(psmm.semantic_map_manager(), /*lane_ids=*/
                          {mapping::ElementId(2), mapping::ElementId(938),
                           mapping::ElementId(940), mapping::ElementId(2470)},
                          /*start_fraction=*/0.5, /*end_fraction=*/1.0);

    // Build lane change left local lane map.
    ASSIGN_OR_DIE(
        const auto candidate_lanes,
        BuildLocalLaneMap(BuildLocalMapInput{
            .psmm = &psmm,
            .driving_map_topo = &dm,
            .origin_lane_path = &origin_lane_path,
            .target_lane_path = &target_lane_path,
            .alc_state = prev_alc_state,
            .lc_direction = prev_lc_direction,
            .cut_off_length = kMinLcLaneLength + kDrivePassageKeepBehindLength,
            .projection_range =
                kMaxTravelDistanceBetweenFrames + kDrivePassageKeepBehindLength,
            .keep_behind_length = kDrivePassageKeepBehindLength}));

    ASSIGN_OR_DIE(const auto scheduler_results,
                  RunAlccScheduler(
                      AlccSchedulerInput{
                          .psmm = &psmm,
                          .vehicle_geom = &vehicle_geom,
                          .plan_start_point = &start_point,
                          .st_traj_mgr = &st_traj_mgr,
                          .candidate_lanes = &candidate_lanes,
                          .lc_cmd = DriverAction::LC_CMD_LEFT,
                          .prev_alc_state = prev_alc_state,
                          .lcc_cruising_speed_mps = 11.0,
                          .online_map_drift_buffer = &online_map_drift_buffer,
                      },
                      thread_pool.get()));
    EXPECT_EQ(scheduler_results.size(), 1);
    const auto& output = scheduler_results[0];
    EXPECT_EQ(output.alc_state, ALC_CROSSING_LANE);
    EXPECT_EQ(output.lc_direction, LaneChangeDirection::LCD_LEFT);
    EXPECT_EQ(output.drive_passage.lane_path().lane_ids().front(),
              mapping::ElementId(2));
  }

  // ALC_CROSSING_LANE + LCD_RIGHT + LC_CMD_RIGHT.
  {
    const auto prev_lc_direction = LaneChangeDirection::LCD_RIGHT;

    // Construct target lane path on right.
    const auto target_lane_path =
        mapping::LanePath(psmm.semantic_map_manager(), /*lane_ids=*/
                          {mapping::ElementId(3), mapping::ElementId(950),
                           mapping::ElementId(35), mapping::ElementId(2472)},
                          /*start_fraction=*/0.5, /*end_fraction=*/1.0);
    // Build lane change right local lane map.
    ASSIGN_OR_DIE(
        const auto candidate_lanes,
        BuildLocalLaneMap(BuildLocalMapInput{
            .psmm = &psmm,
            .driving_map_topo = &dm,
            .origin_lane_path = &origin_lane_path,
            .target_lane_path = &target_lane_path,
            .alc_state = prev_alc_state,
            .lc_direction = prev_lc_direction,
            .cut_off_length = kMinLcLaneLength + kDrivePassageKeepBehindLength,
            .projection_range =
                kMaxTravelDistanceBetweenFrames + kDrivePassageKeepBehindLength,
            .keep_behind_length = kDrivePassageKeepBehindLength}));

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
    EXPECT_EQ(scheduler_results.size(), 1);
    const auto& output = scheduler_results[0];
    EXPECT_EQ(output.alc_state, ALC_CROSSING_LANE);
    EXPECT_EQ(output.lc_direction, LaneChangeDirection::LCD_RIGHT);
    EXPECT_EQ(output.drive_passage.lane_path().lane_ids().front(),
              mapping::ElementId(3));
  }
}

TEST(AlccSchedulerTest, AlcCompletedTest) {
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
  const auto target_lane_path =
      mapping::LanePath(psmm.semantic_map_manager(), /*lane_ids=*/
                        {mapping::ElementId(3), mapping::ElementId(950),
                         mapping::ElementId(35), mapping::ElementId(2472)},
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
  const auto prev_alc_state = QALCState::ALC_COMPLETED;
  const auto prev_lc_direction = LaneChangeDirection::LCD_LEFT;

  // Build local lane map.
  ASSIGN_OR_DIE(
      const auto candidate_lanes,
      BuildLocalLaneMap(BuildLocalMapInput{
          .psmm = &psmm,
          .driving_map_topo = &dm,
          .origin_lane_path = &origin_lane_path,
          .target_lane_path = &target_lane_path,
          .alc_state = prev_alc_state,
          .lc_direction = prev_lc_direction,
          .cut_off_length = kMinLcLaneLength + kDrivePassageKeepBehindLength,
          .projection_range =
              kMaxTravelDistanceBetweenFrames + kDrivePassageKeepBehindLength,
          .keep_behind_length = kDrivePassageKeepBehindLength}));

  SpacetimeTrajectoryManager st_traj_mgr;

  auto thread_pool =
      std::make_unique<ThreadPool>(FLAGS_planner_thread_pool_size);

  HistoryBufferAbslTime<PiecewiseLinearFunction<double>>
      online_map_drift_buffer;

  ASSIGN_OR_DIE(const auto scheduler_results,
                RunAlccScheduler(
                    AlccSchedulerInput{
                        .psmm = &psmm,
                        .vehicle_geom = &vehicle_geom,
                        .plan_start_point = &start_point,
                        .st_traj_mgr = &st_traj_mgr,
                        .candidate_lanes = &candidate_lanes,
                        .lc_cmd = DriverAction::LC_CMD_NONE,
                        .prev_alc_state = prev_alc_state,
                        .lcc_cruising_speed_mps = 11.0,
                        .online_map_drift_buffer = &online_map_drift_buffer,
                    },
                    thread_pool.get()));
  EXPECT_EQ(scheduler_results.size(), 1);
  const auto& output = scheduler_results[0];
  EXPECT_EQ(output.alc_state, ALC_STANDBY_ENABLE);
  EXPECT_EQ(output.lc_direction, LaneChangeDirection::LCD_NONE);
  EXPECT_EQ(output.drive_passage.lane_path().lane_ids().front(),
            mapping::ElementId(3));
}

TEST(AlccSchedulerTest, AlcReturnCompletedTest) {
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
  const auto prev_alc_state = QALCState::ALC_RETURN_COMPLETED;
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

  SpacetimeTrajectoryManager st_traj_mgr;

  auto thread_pool =
      std::make_unique<ThreadPool>(FLAGS_planner_thread_pool_size);

  HistoryBufferAbslTime<PiecewiseLinearFunction<double>>
      online_map_drift_buffer;

  ASSIGN_OR_DIE(const auto scheduler_results,
                RunAlccScheduler(
                    AlccSchedulerInput{
                        .psmm = &psmm,
                        .vehicle_geom = &vehicle_geom,
                        .plan_start_point = &start_point,
                        .st_traj_mgr = &st_traj_mgr,
                        .candidate_lanes = &candidate_lanes,
                        .lc_cmd = DriverAction::LC_CMD_NONE,
                        .prev_alc_state = prev_alc_state,
                        .lcc_cruising_speed_mps = 11.0,
                        .online_map_drift_buffer = &online_map_drift_buffer,
                    },
                    thread_pool.get()));
  EXPECT_EQ(scheduler_results.size(), 1);
  const auto& output = scheduler_results[0];
  EXPECT_EQ(output.alc_state, ALC_STANDBY_ENABLE);
  EXPECT_EQ(output.lc_direction, LaneChangeDirection::LCD_NONE);
  EXPECT_EQ(output.drive_passage.lane_path().lane_ids().front(),
            mapping::ElementId(2448));
}

TEST(AlccSchedulerTest, NoneStateTransformTest) {
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

  // Build local lane map.
  ASSIGN_OR_DIE(
      const auto candidate_lanes,
      BuildLocalLaneMap(BuildLocalMapInput{
          .psmm = &psmm,
          .driving_map_topo = &dm,
          .origin_lane_path = &origin_lane_path,
          .target_lane_path = nullptr,
          .alc_state = QALCState::ALC_STANDBY_ENABLE,
          .lc_direction = LaneChangeDirection::LCD_NONE,
          .cut_off_length = kMinLcLaneLength + kDrivePassageKeepBehindLength,
          .projection_range =
              kMaxTravelDistanceBetweenFrames + kDrivePassageKeepBehindLength,
          .keep_behind_length = kDrivePassageKeepBehindLength}));

  auto thread_pool =
      std::make_unique<ThreadPool>(FLAGS_planner_thread_pool_size);

  SpacetimeTrajectoryManager st_traj_mgr;

  HistoryBufferAbslTime<PiecewiseLinearFunction<double>>
      online_map_drift_buffer;

  for (const auto prev_alc_state : {ALC_OFF, ALC_STANDBY, ALC_RETURNING}) {
    ASSIGN_OR_DIE(const auto scheduler_results,
                  RunAlccScheduler(
                      AlccSchedulerInput{
                          .psmm = &psmm,
                          .vehicle_geom = &vehicle_geom,
                          .plan_start_point = &start_point,
                          .st_traj_mgr = &st_traj_mgr,
                          .candidate_lanes = &candidate_lanes,
                          .lc_cmd = DriverAction::LC_CMD_NONE,
                          .prev_alc_state = prev_alc_state,
                          .lcc_cruising_speed_mps = 11.0,
                          .online_map_drift_buffer = &online_map_drift_buffer,
                      },
                      thread_pool.get()));
    EXPECT_EQ(scheduler_results.size(), 1);
    const auto& output = scheduler_results[0];
    EXPECT_EQ(output.alc_state, prev_alc_state);
    EXPECT_EQ(output.lc_direction, LaneChangeDirection::LCD_NONE);
    EXPECT_EQ(output.drive_passage.lane_path().lane_ids().front(),
              mapping::ElementId(2448));
  }
}
}  // namespace
}  // namespace planner
}  // namespace qcraft

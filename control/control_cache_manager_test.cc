#include "onboard/control/control_cache_manager.h"

#include <algorithm>
#include <utility>
#include <vector>

#include "gtest/gtest.h"

#include "onboard/control/control_defs.h"

namespace qcraft {
namespace control {
namespace {

constexpr double kEpsilon = 1e-6;

std::vector<double> Step2TimeRange(const std::vector<int>& steps,
                                   double interval) {
  std::vector<double> time_range;
  time_range.reserve(steps.size());
  for (const int& step : steps) {
    time_range.push_back(static_cast<double>(step * interval));
  }

  return time_range;
}

TEST(ControlCacheManagerTest, DefaultCacheTest) {
  ControlCacheManager control_cache_mgr;
  for (int i = 0; i < kCacheSize; ++i) {
    EXPECT_NEAR(control_cache_mgr.QueryKappaCmd(/*step*/ i), 0.0, kEpsilon);
    EXPECT_NEAR(control_cache_mgr.QueryPlannerKappa(/*step*/ i), 0.0, kEpsilon);
    EXPECT_NEAR(control_cache_mgr.QuerySteerPctChassis(/*step*/ i), 0.0,
                kEpsilon);
    EXPECT_NEAR(control_cache_mgr.QuerySteerPctControl(/*step*/ i), 0.0,
                kEpsilon);
    EXPECT_NEAR(control_cache_mgr.QueryAccTarget(/*step*/ i), 0.0, kEpsilon);
    EXPECT_NEAR(control_cache_mgr.QuerySpeedPose(/*step*/ i), 0.0, kEpsilon);
    EXPECT_NEAR(control_cache_mgr.IntegrateLatErrorCanbus(/*step*/ i), 0.0,
                kEpsilon);
  }
}

TEST(ControlCacheManagerTest, UpdateCacheTest) {
  ControlCacheManager control_cache_mgr;
  VehicleStateProto vehicle_state;
  vehicle_state.set_is_auto_mode(true);
  ControlCommand control_cmd;
  VehPoseProto veh_predicted_pose;
  constexpr double kGradient = 0.01;
  for (int i = 0; i < 300; ++i) {
    control_cmd.set_curvature(i * kGradient);
    veh_predicted_pose.set_x(2 * i * kGradient);
    control_cache_mgr.UpdateCacheData(
        control_cmd, vehicle_state,
        {.veh_predicted_pose = &veh_predicted_pose});

    const double front_cache_expected =
        std::max((i + 1 - kCacheSize) * kGradient, 0.0);
    const double back_cache_expected = i * kGradient;

    EXPECT_NEAR(control_cache_mgr.QueryKappaCmd(/*step*/ 0),
                back_cache_expected, kEpsilon);
    EXPECT_NEAR(control_cache_mgr.QueryKappaCmd(/*step*/ kCacheSize - 1),
                front_cache_expected, kEpsilon);
    const double front_cache_x_expected =
        2 * std::max((i + 1 - kCacheSize) * kGradient, 0.0);
    const double back_cache_x_expected = 2 * i * kGradient;
    EXPECT_NEAR(control_cache_mgr.QueryPredictPose(0).x(),
                back_cache_x_expected, kEpsilon);
    EXPECT_NEAR(control_cache_mgr.QueryPredictPose(/*step*/ kCacheSize - 1).x(),
                front_cache_x_expected, kEpsilon);
  }
}

TEST(ControlCacheManagerTest, QueryMaxAndMin) {
  ControlCacheManager state_manager;
  VehicleStateProto vehicle_state;
  vehicle_state.set_is_auto_mode(true);
  vehicle_state.set_is_auto_steer(true);
  vehicle_state.set_is_auto_speed(true);
  constexpr double kKappaPose = 2.5;
  vehicle_state.set_kappa(kKappaPose);
  ControlCommand control_cmd;
  constexpr double kMaxAcc = 4.0;
  constexpr double kMaxKappa = 1.0;
  constexpr double kMinSteerControl = 80.0;
  constexpr double kMinSteerChassis = 70.0;

  VehPoseProto veh_predicted_pose;
  for (int i = 1; i < kCacheSize + 1; ++i) {
    const double acc = kMaxAcc * i / kCacheSize;
    const double kappa = kMaxKappa * i / kCacheSize;
    control_cmd.set_acceleration(acc);
    control_cmd.set_curvature(kappa);
    control_cmd.set_steering_target(
        std::max(kMinSteerControl, 100.0 * i / kCacheSize - 10.0));
    vehicle_state.set_chassis_steering_percentage(
        std::max(kMinSteerChassis, 100.0 * i / kCacheSize - 20.0));

    state_manager.UpdateCacheData(control_cmd, vehicle_state, {});
  }

  constexpr int kStep = 100;

  EXPECT_NEAR(state_manager.QueryMaxControlAcc(kStep), kMaxAcc, kEpsilon);
  EXPECT_NEAR(state_manager.QueryMaxKappaCmd(kStep), kMaxKappa, kEpsilon);

  EXPECT_NEAR(state_manager.QueryMinControlAcc(kStep),
              kMaxAcc * (kCacheSize - kStep + 1) / kCacheSize, kEpsilon);
  EXPECT_NEAR(state_manager.QueryMinKappaCmd(kStep),
              kMaxKappa * (kCacheSize - kStep + 1) / kCacheSize, kEpsilon);
  EXPECT_NEAR(state_manager.QueryMinControlSteerPct(kStep), kMinSteerControl,
              kEpsilon);
  EXPECT_NEAR(state_manager.QueryMinChassisSteerPct(kStep), kMinSteerChassis,
              kEpsilon);
  EXPECT_NEAR(state_manager.QueryMeanKappaPose(kStep), kKappaPose, kEpsilon);
}

TEST(ControlCacheManagerTest, IsInAlwaysAutoModeTest) {
  const std::vector<int> auto_size{0, 1, 20, 50};
  constexpr int kMaxTestSize = 100;

  for (size_t i = 0; i < auto_size.size(); ++i) {
    ControlCacheManager state_manager;
    VehicleStateProto vehicle_state;
    vehicle_state.set_is_auto_mode(true);
    vehicle_state.set_is_auto_steer(true);
    vehicle_state.set_is_auto_speed(true);
    ControlCommand control_cmd;

    for (int j = 0; j < auto_size[i]; ++j) {
      state_manager.UpdateCacheData(control_cmd, vehicle_state, {});
    }

    for (int j = 1; j < kMaxTestSize; ++j) {
      EXPECT_EQ(state_manager.IsInAlwaysAutoMode(j), j <= auto_size[i]);
      EXPECT_EQ(state_manager.IsInAlwaysSteerMode(j), j <= auto_size[i]);
      EXPECT_EQ(state_manager.IsInAlwaysSpeedMode(j), j <= auto_size[i]);
    }
  }
}

TEST(ControlCacheManagerTest, QuerySingleData) {
  // Load data;
  double kappa_cmd = 0.0;
  const double kappa_rate = 0.1;
  ControlCacheManager state_manager;
  VehicleStateProto vehicle_state;
  // Set kappa_cmd cache from 0.0 to 0.1 * kCacheSize;
  for (int i = 0; i < kCacheSize; ++i) {
    ControlCommand control_cmd;
    control_cmd.set_curvature(kappa_cmd);
    state_manager.UpdateCacheData(control_cmd, vehicle_state, {});
    kappa_cmd += kappa_rate;
  }

  double expect_kappa = kappa_cmd - kappa_rate;
  for (int i = 0; i < kCacheSize; ++i) {
    EXPECT_NEAR(state_manager.QueryKappaCmd(i), expect_kappa, kEpsilon);
    expect_kappa -= kappa_rate;
  }
}

TEST(DecModeStatTest, OperatorTest) {
  ControlCacheManager::DecModeStat stat1;
  ControlCacheManager::DecModeStat stat2;

  // Test the case where both stats are empty.
  EXPECT_TRUE(stat1 == stat2);

  // Test the case where only the brake durations are different.
  stat1.brake_duration = {1.0, 2.0, 3.0};
  EXPECT_FALSE(stat1 == stat2);
  stat2.brake_duration = {1.0, 2.0, 3.0};
  EXPECT_TRUE(stat1 == stat2);

  // Test the case where only the brake intervals are different.
  stat1.brake_interval = {1.0, 2.0, 3.0};
  stat2.brake_interval = {1.0, 2.0, 3.0};
  EXPECT_TRUE(stat1 == stat2);
  stat2.brake_interval[2] += 1e-12;
  EXPECT_FALSE(stat1 == stat2);
}

TEST(ControlCacheManagerTest, CountDecModeTest) {
  // Set speed_mode results and expected result;
  // first: expected result; second: speed mode data.
  std::vector<
      std::pair<ControlCacheManager::DecModeStat, std::vector<SpeedMode>>>
      speed_mode_tests;
  constexpr double kAcceleration = -0.25;

  // test 0: brake all the way.
  const std::vector<SpeedMode> speed_mode_0(kCacheSize, SpeedMode::DEC_MODE);
  const ControlCacheManager::DecModeStat expected_result_0{
      .count = 0, .brake_duration = {}, .brake_interval = {}};
  speed_mode_tests.push_back(std::make_pair(expected_result_0, speed_mode_0));

  // test 1: accelerate all the way.
  const std::vector<SpeedMode> speed_mode_1(kCacheSize, SpeedMode::ACC_MODE);
  const ControlCacheManager::DecModeStat expected_result_1{
      .count = 0, .brake_duration = {}, .brake_interval = {}};
  speed_mode_tests.push_back(std::make_pair(expected_result_1, speed_mode_1));

  // test 2: brake 10 steps at the beginning.
  constexpr int kBrakeSteps2 = 10;
  std::vector<SpeedMode> speed_mode_2(kBrakeSteps2, SpeedMode::DEC_MODE);
  const std::vector<SpeedMode> speed_mode_2_tail(kCacheSize - kBrakeSteps2,
                                                 SpeedMode::ACC_MODE);
  speed_mode_2.insert(speed_mode_2.end(), speed_mode_2_tail.begin(),
                      speed_mode_2_tail.end());
  const ControlCacheManager::DecModeStat expected_result_2{
      .count = 1,
      .brake_duration = {kBrakeSteps2 * kControlInterval},
      .brake_interval = {0.0},
      .avg_accl_cmd = {kAcceleration}};
  speed_mode_tests.push_back(std::make_pair(expected_result_2, speed_mode_2));

  // test 3: brake 10 steps at the end.
  constexpr int kBrakeSteps3 = 50;
  std::vector<SpeedMode> speed_mode_3(kCacheSize - kBrakeSteps3,
                                      SpeedMode::ACC_MODE);
  const std::vector<SpeedMode> speed_mode_3_tail(kBrakeSteps3,
                                                 SpeedMode::DEC_MODE);
  speed_mode_3.insert(speed_mode_3.end(), speed_mode_3_tail.begin(),
                      speed_mode_3_tail.end());
  const ControlCacheManager::DecModeStat expected_result_3{
      .count = 0, .brake_duration = {}, .brake_interval = {}};
  speed_mode_tests.push_back(std::make_pair(expected_result_3, speed_mode_3));

  // test 4: brake 10 steps at the end.
  const std::vector<int> brake4_duration{30, 40, 50};
  const std::vector<int> brake4_interval{0, 25, 100};

  std::vector<SpeedMode> speed_mode_4;
  for (size_t i = 0; i < brake4_duration.size(); ++i) {
    const std::vector<SpeedMode> brake_interval(brake4_interval[i],
                                                SpeedMode::ACC_MODE);
    const std::vector<SpeedMode> brake_duration(brake4_duration[i],
                                                SpeedMode::DEC_MODE);
    speed_mode_4.insert(speed_mode_4.end(), brake_interval.begin(),
                        brake_interval.end());
    speed_mode_4.insert(speed_mode_4.end(), brake_duration.begin(),
                        brake_duration.end());
  }
  speed_mode_4.insert(speed_mode_4.end(), kCacheSize - speed_mode_4.size(),
                      SpeedMode::ACC_MODE);

  const ControlCacheManager::DecModeStat expected_result_4{
      .count = static_cast<int>(brake4_duration.size()),
      .brake_duration = Step2TimeRange(brake4_duration, kControlInterval),
      .brake_interval = Step2TimeRange(brake4_interval, kControlInterval),
      .avg_accl_cmd =
          std::vector<double>(brake4_duration.size(), kAcceleration)};
  speed_mode_tests.push_back(std::make_pair(expected_result_4, speed_mode_4));

  // Verify results;
  VehicleStateProto vehicle_state;
  vehicle_state.set_is_auto_speed(true);
  ControlCommand control_cmd;
  control_cmd.set_acceleration(kAcceleration);

  for (const auto& speed_mode_test : speed_mode_tests) {
    const auto& expected_result = speed_mode_test.first;
    const auto& speed_mode_vector = speed_mode_test.second;
    ControlCacheManager state_manager;

    // Load data;
    for (int i = 0; i < kCacheSize; ++i) {
      control_cmd.set_speed_mode(speed_mode_vector[i]);
      state_manager.UpdateCacheData(control_cmd, vehicle_state, {});
    }

    ControlCacheManager::DecModeStat result = state_manager.CountDecMode();

    EXPECT_TRUE(result == expected_result);
  }
}

TEST(ControlCacheManagerTest, QueryVectorTest) {
  // Load data;
  constexpr double kInitKappa = 0.0;
  constexpr double kKappaRate = 0.1;
  constexpr double kInitAcc = -1.0;
  constexpr double kAccRate = 0.01;

  ControlCacheManager state_manager;
  VehicleStateProto vehicle_state;

  double kappa_cmd = kInitKappa;
  double acc_target = kInitAcc;
  for (int i = 0; i < kCacheSize; ++i) {
    ControlCommand control_cmd;
    control_cmd.set_curvature(kappa_cmd);
    control_cmd.mutable_debug()
        ->mutable_simple_mpc_debug()
        ->set_acceleration_cmd_closeloop(acc_target);
    state_manager.UpdateCacheData(control_cmd, vehicle_state, {});
    kappa_cmd += kKappaRate;
    acc_target += kAccRate;
  }

  constexpr int kQuerySteps = 1;
  std::vector<double> kappa_vector =
      state_manager.QueryKappaCmdVector(kQuerySteps);
  std::vector<double> acc_vector =
      state_manager.QueryAccTargetVector(kQuerySteps);

  for (int i = 0; i < kQuerySteps; ++i) {
    const double kappa_check = kInitKappa + (kCacheSize - 1 - i) * kKappaRate;
    const double acc_check = kInitAcc + (kCacheSize - 1 - i) * kAccRate;
    EXPECT_NEAR(kappa_vector[i], kappa_check, kEpsilon);
    EXPECT_NEAR(acc_vector[i], acc_check, kEpsilon);
  }
}

TEST(ControlCacheManagerTest, IsVehicleIntentedToMoveTest) {
  ControlCacheManager state_manager;
  VehicleStateProto vehicle_state;
  ControlCommand control_cmd;
  ControllerDebugProto control_debug;

  constexpr int kHasStartSteps = 100;
  for (int i = 0; i < kCacheSize; ++i) {
    control_debug.mutable_speed_mode_debug_proto()->set_is_full_stop(
        i < (kCacheSize - kHasStartSteps));
    state_manager.UpdateCacheData(control_cmd, vehicle_state,
                                  {.control_debug = &control_debug});
  }

  for (int i = 0; i < kCacheSize; ++i) {
    if (state_manager.IsVehicleIntentedToMove(i)) {
      EXPECT_EQ(i, kHasStartSteps);
    }
  }
}

TEST(ControlCacheManagerTest, CalculateStepsMoveOffTest) {
  ControlCacheManager state_manager;
  VehicleStateProto vehicle_state;
  ControlCommand control_cmd;
  ControllerDebugProto control_debug;

  constexpr int kHasMovedOff = 50;
  for (int i = 0; i < kCacheSize; ++i) {
    control_debug.mutable_speed_mode_debug_proto()->set_standstill(
        i < (kCacheSize - kHasMovedOff));
    state_manager.UpdateCacheData(control_cmd, vehicle_state,
                                  {.control_debug = &control_debug});
  }

  EXPECT_EQ(kHasMovedOff, state_manager.CalculateStepsMoveOff());
}

}  // namespace
}  // namespace control
}  // namespace qcraft

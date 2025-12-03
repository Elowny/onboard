#include "onboard/control/control_monitoring.h"

#include <cmath>
#include <memory>  // for unique_ptr
#include <vector>

#include "google/protobuf/repeated_ptr_field.h"

#include "gtest/gtest.h"

#include "onboard/control/control_cache_manager.h"
#include "onboard/control/control_defs.h"
#include "onboard/control/controllers/predict_vehicle_pose.h"
#include "onboard/control/proto/vehicle_state.pb.h"
#include "onboard/control/steering_converter.h"
#include "onboard/eval/collectors/qevent_collector.h"
#include "onboard/eval/collectors/qevent_log_collector.h"
#include "onboard/eval/proto/qevent.pb.h"
#include "onboard/eval/proto/qevents_lite.pb.h"
#include "onboard/global/counter.h"
#include "onboard/math/util.h"
#include "onboard/proto/control_cmd.pb.h"
#include "onboard/proto/counter.pb.h"

namespace qcraft::control {
namespace {

TEST(ControlMonitoringTest, QCounterControlErrorTest) {
  ControlError control_error;
  constexpr double kLateralError = 0.1;  // m.
  control_error.set_lateral_error(kLateralError);
  VehicleStateProto vehicle_state;
  vehicle_state.set_is_auto_steer(true);

  QCounterControlError(vehicle_state, control_error);
  auto counter_report = qcraft::Counter::Instance()->GetCounterOutput();
  EXPECT_EQ(counter_report->item_size(), 4);
  EXPECT_EQ(counter_report->item(0).max(),
            static_cast<int>(kLateralError * 100));
  EXPECT_EQ(counter_report->item(0).min(),
            static_cast<int>(kLateralError * 100));
}

TEST(ControlMonitoringTest, QEventBrakeInfoTest) {
  struct BrakeInfo {
    int index_start;
    int duration;
    double acceleration;
  };

  ControlCacheManager control_cache_manager;
  // brake 5 times.
  const std::vector<BrakeInfo> brakes_info{{10, 10, -0.5},  {50, 10, -0.5},
                                           {100, 20, -1.0}, {200, 1, -3.0},
                                           {300, 10, -2.0}, {400, 10, -2.5}};

  for (int i = 0; i < kCacheSize; ++i) {
    ControlCommand control_cmd;
    control_cmd.set_speed_mode(ACC_MODE);
    control_cmd.set_acceleration(1.0);
    for (const auto& brake : brakes_info) {
      if (i >= brake.index_start && i < (brake.index_start + brake.duration)) {
        control_cmd.set_acceleration(brake.acceleration);
        control_cmd.set_speed_mode(DEC_MODE);
      }
    }

    VehicleStateProto vehicle_state;
    vehicle_state.set_is_auto_speed(true);
    control_cache_manager.UpdateCacheData(control_cmd, vehicle_state, {});
  }

  QEventLogCollector collector;
  GlobalQEventCollector::SetCollector(&collector);

  QEventControlCache(/*speed*/ 3.0, control_cache_manager);

  auto qevent_result_protos = collector.GetQEventsOutput()->items();

  EXPECT_EQ(qevent_result_protos.size(), 2);
  EXPECT_EQ(qevent_result_protos[0].name(), "control_brake_too_frequent");
  EXPECT_EQ(qevent_result_protos[1].name(), "control_brake_uncomfortable");
}

TEST(ControlMonitoringTest, QEventMinRadiusTest) {
  constexpr double kRadius = 5.0;  // m.

  ControlCacheManager control_cache_manager;
  ControlCommand control_cmd;
  control_cmd.set_steering_target(99.99);
  VehicleStateProto vehicle_state;
  vehicle_state.set_is_auto_mode(true);
  vehicle_state.set_kappa(1.0 / kRadius);
  vehicle_state.set_chassis_steering_percentage(95.0);

  for (int i = 0; i < kCacheSize; ++i) {
    control_cache_manager.UpdateCacheData(control_cmd, vehicle_state, {});
  }

  QEventLogCollector collector;
  GlobalQEventCollector::SetCollector(&collector);

  QEventControlCache(/*speed*/ 3.0, control_cache_manager);
  auto qevent_result_protos = collector.GetQEventsOutput()->items();

  EXPECT_EQ(qevent_result_protos.size(), 2);
  EXPECT_EQ(qevent_result_protos[0].name(), "minimum_turning_radius");
}

TEST(ControlMonitoringTest, QEventVehicleMoveOffTest) {
  ControlCacheManager control_cache_manager;
  ControlCommand control_cmd;
  VehicleStateProto vehicle_state;
  ControllerDebugProto control_debug;

  const int trigger_step =
      kCacheSize - RoundToInt(kMoveOffMonitoringTime * kControlFrequency);
  for (int i = 0; i < kCacheSize; ++i) {
    control_debug.mutable_speed_mode_debug_proto()->set_is_full_stop(
        i < trigger_step);
    control_debug.mutable_speed_mode_debug_proto()->set_standstill(
        i < (trigger_step + /*response step*/ 50));
    control_cache_manager.UpdateCacheData(control_cmd, vehicle_state,
                                          {.control_debug = &control_debug});
  }

  QEventLogCollector collector;
  GlobalQEventCollector::SetCollector(&collector);

  QEventControlCache(/*speed*/ 10.0, control_cache_manager);
  auto qevent_result_protos = collector.GetQEventsOutput()->items();
  EXPECT_EQ(qevent_result_protos.size(), 1);
  EXPECT_EQ(qevent_result_protos[0].name(), "vehicle_move_off");
}

TEST(ControlMonitoringTest, QCounterLonControlErrorTest) {
  ControlError control_error;
  constexpr double kError = 1.0;
  control_error.set_lateral_error(kError);
  control_error.set_heading_error(kError);
  control_error.set_speed_error(kError);
  control_error.set_station_error(kError);

  VehicleStateProto vehicle_state;
  vehicle_state.set_is_auto_speed(true);
  vehicle_state.set_is_auto_steer(true);

  QCounterControlError(vehicle_state, control_error);

  auto output = Counter::Instance()->GetCounterOutput();
  EXPECT_EQ(output->item_size(), 9);
}

TEST(ControlMonitoringTest, QCounterPoseTest) {
  PoseProto pose_proto;
  QCounterPose(pose_proto);

  auto output = Counter::Instance()->GetCounterOutput();
  EXPECT_EQ(output->item_size(), 5);
}

TEST(ControlMonitoringTest, QEventDiscomfortableAndHardBrakeTest) {
  ControlCacheManager control_cache_manager;
  SteeringConverter steering_converter(/*wheel_base*/ 4.0,
                                       /*steer_ratio*/ 16.0,
                                       /*max_steer_angle*/ 3.0 * M_PI);
  PoseProto pose;

  QEventLogCollector collector;
  GlobalQEventCollector::SetCollector(&collector);

  QEventDiscomfortable(control_cache_manager, steering_converter, pose);
  QEventHardBrake(/*control_cmd_accel*/ -2.0, /*traj_min_accel*/ -2.1,
                  /*traj_ref_accel*/ -2.2, control_cache_manager);
  auto qevent_result_protos = collector.GetQEventsOutput()->items();
  EXPECT_EQ(qevent_result_protos.size(), 2);
  EXPECT_EQ(qevent_result_protos[0].name(), "discomfort");
  EXPECT_EQ(qevent_result_protos[1].name(), "control_hard_brake");
}

TEST(ControlMonitoringTest, QEventTrackingErrorTest) {
  ControlCacheManager control_cache_manager;
  ControlCommand control_cmd;
  VehicleStateProto vehicle_state;

  ControlError control_error;
  constexpr double kError = 1.5;
  control_error.set_lateral_error(kError);
  control_error.set_heading_error(kError);
  control_error.set_speed_error(kError);
  control_error.set_station_error(kError);
  for (int i = 0; i < kCacheSize; ++i) {
    control_cmd.mutable_debug()->mutable_control_error()->CopyFrom(
        control_error);
    control_cache_manager.UpdateCacheData(control_cmd, vehicle_state, {});
  }

  QEventLogCollector collector;
  GlobalQEventCollector::SetCollector(&collector);

  QEventTrackingError(/*steer_ref_pct*/ 10.0, /*speed_ref*/ 5.0,
                      control_cache_manager);
  auto qevent_result_protos = collector.GetQEventsOutput()->items();
  EXPECT_EQ(qevent_result_protos.size(), 4);
  EXPECT_EQ(qevent_result_protos[0].name(), "large_lat_error");
  EXPECT_EQ(qevent_result_protos[1].name(), "large_lon_error");
  EXPECT_EQ(qevent_result_protos[2].name(), "continuous_large_lat_error");
  EXPECT_EQ(qevent_result_protos[3].name(), "continuous_large_lon_error");
}

TEST(ControlMonitoringTest, VehPoseDiffTest) {
  VehPose vehpos_km, vehpos_dm;

  QEventLogCollector collector;
  GlobalQEventCollector::SetCollector(&collector);

  QEventLatVehPosDiff(vehpos_km, vehpos_dm);
  auto qevent_result_protos = collector.GetQEventsOutput()->items();
  EXPECT_EQ(qevent_result_protos.size(), 0);

  vehpos_km.x = 10.0;
  vehpos_km.y = 10.0;
  vehpos_dm.x = 10.05;
  vehpos_dm.y = 10.05;

  QEventLatVehPosDiff(vehpos_km, vehpos_dm);
  qevent_result_protos = collector.GetQEventsOutput()->items();
  EXPECT_EQ(qevent_result_protos.size(), 1);
  EXPECT_EQ(qevent_result_protos[0].name(), "veh_pose_predict_diff_too_large");
}

}  // namespace
}  // namespace qcraft::control

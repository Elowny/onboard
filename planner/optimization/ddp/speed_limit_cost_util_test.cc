#include "onboard/planner/optimization/ddp/speed_limit_cost_util.h"

// IWYU pragma: no_include <ext/alloc_traits.h>
//              for __alloc_traits<>::value_type

#include <memory>

#include "gtest/gtest.h"

namespace qcraft {
namespace planner {
namespace {

using SpeedlimitInfoPoint = optimizer::SpeedlimitInfoPoint;
using SpeedZoneInfo = optimizer::SpeedZoneInfo;

void GenerateStationsSpeedLimitInfo(std::vector<Vec2d>* station_points,
                                    std::vector<double>* station_points_s,
                                    std::vector<double>* station_speed_limits) {
  *station_points = {
      {0.0, 0.0}, {1.0, 0.0}, {2.0, 0.0}, {3.0, 0.0}, {4.0, 0.0}};
  *station_speed_limits = {10.0, 9.0, 5.0, 8.0, 10.0};
  station_points_s->reserve(station_points->size());
  station_points_s->push_back(0.0);
  for (int i = 1; i < station_points->size(); ++i) {
    const double s_diff =
        ((*station_points)[i] - (*station_points)[i - 1]).norm();
    station_points_s->push_back(station_points_s->back() + s_diff);
  }
}

void CheckSpeedLimitResult(
    const std::vector<double>& station_speed_limits,
    const std::vector<std::vector<SpeedlimitInfoPoint>>&
        additional_speed_limits,
    const std::vector<double>& station_speed_limits_result,
    const std::vector<std::vector<SpeedlimitInfoPoint>>&
        additional_speed_limits_result) {
  constexpr double kEpsilon = 1e-9;
  for (int idx = 0; idx < station_speed_limits.size(); ++idx) {
    EXPECT_NEAR(station_speed_limits[idx], station_speed_limits_result[idx],
                kEpsilon);
  }

  for (int idx = 0; idx < additional_speed_limits.size(); ++idx) {
    const auto& speed_limit = additional_speed_limits[idx];
    const auto& speed_limit_result = additional_speed_limits_result[idx];
    EXPECT_EQ(speed_limit.size(), speed_limit_result.size());
    for (int k = 0; k < speed_limit.size(); ++k) {
      EXPECT_NEAR(speed_limit[k].alpha, speed_limit_result[k].alpha, kEpsilon);
      EXPECT_NEAR(speed_limit[k].speed_limit, speed_limit_result[k].speed_limit,
                  kEpsilon);
    }
  }
}

TEST(MergeSpeedInfoWithSpeedZoneTest, SpeedZonesLinked) {
  std::vector<SpeedZoneInfo> speed_zones = {
      {2.000001, 2.2, {2.000001, 0.0}, {2.2, 0.0}, 6.0},
      {2.2, 2.3, {2.2, 0.0}, {2.3, 0.0}, 4.0},
      {2.3, 4.2, {2.3, 0.0}, {4.2, 0.0}, 11.0}};
  std::vector<Vec2d> station_points;
  std::vector<double> station_points_s;
  std::vector<double> station_speed_limits;
  GenerateStationsSpeedLimitInfo(&station_points, &station_points_s,
                                 &station_speed_limits);
  std::vector<std::vector<SpeedlimitInfoPoint>> additional_speed_limits;
  optimizer::speedlimit::MergeSpeedLimitWithSpeedZones(
      speed_zones, station_points, &station_speed_limits,
      &additional_speed_limits);

  // Check.
  const std::vector<double> station_speed_limits_result = {10.0, 9.0, 5.0, 8.0,
                                                           10.0};
  const std::vector<std::vector<SpeedlimitInfoPoint>>
      additional_speed_limits_result = {{}, {}, {{0.2, 4.0}, {0.3, 5.0}}, {}};
  CheckSpeedLimitResult(station_speed_limits, additional_speed_limits,
                        station_speed_limits_result,
                        additional_speed_limits_result);
}

TEST(MergeSpeedInfoWithSpeedZoneTest, SpeedZonesCrossRegion) {
  std::vector<SpeedZoneInfo> speed_zones = {
      {0.1, 2.2, {0.1, 0.0}, {2.2, 0.0}, 8.0},
      {2.9, 3.4, {2.9, 0.0}, {3.4, 0.0}, 4.0}};
  std::vector<Vec2d> station_points;
  std::vector<double> station_points_s;
  std::vector<double> station_speed_limits;
  GenerateStationsSpeedLimitInfo(&station_points, &station_points_s,
                                 &station_speed_limits);
  std::vector<std::vector<SpeedlimitInfoPoint>> additional_speed_limits;
  optimizer::speedlimit::MergeSpeedLimitWithSpeedZones(
      speed_zones, station_points, &station_speed_limits,
      &additional_speed_limits);

  // Check.
  const std::vector<double> station_speed_limits_result = {10.0, 8.0, 5.0, 4.0,
                                                           10.0};
  const std::vector<std::vector<SpeedlimitInfoPoint>>
      additional_speed_limits_result = {
          {{0.1, 8.0}}, {}, {{0.2, 5.0}, {0.9, 4.0}}, {{0.4, 8.0}}};
  CheckSpeedLimitResult(station_speed_limits, additional_speed_limits,
                        station_speed_limits_result,
                        additional_speed_limits_result);
}

TEST(MergeSpeedInfoWithSpeedZoneTest, SpeedZoneCoverRnage) {
  std::vector<SpeedZoneInfo> speed_zones = {
      {-0.1, 4.2, {-0.1, 0.0}, {4.2, 0.0}, 7.0}};
  std::vector<Vec2d> station_points;
  std::vector<double> station_points_s;
  std::vector<double> station_speed_limits;
  GenerateStationsSpeedLimitInfo(&station_points, &station_points_s,
                                 &station_speed_limits);
  std::vector<std::vector<SpeedlimitInfoPoint>> additional_speed_limits;
  optimizer::speedlimit::MergeSpeedLimitWithSpeedZones(
      speed_zones, station_points, &station_speed_limits,
      &additional_speed_limits);

  // Check.
  const std::vector<double> station_speed_limits_result = {7.0, 7.0, 5.0, 7.0,
                                                           7.0};
  const std::vector<std::vector<SpeedlimitInfoPoint>>
      additional_speed_limits_result = {{}, {}, {}, {}};
  CheckSpeedLimitResult(station_speed_limits, additional_speed_limits,
                        station_speed_limits_result,
                        additional_speed_limits_result);
}

TEST(MergeSpeedInfoWithStopLineTest, StopLineBeforeStations) {
  std::vector<Vec2d> station_points;
  std::vector<double> station_points_s;
  std::vector<double> station_speed_limits;
  GenerateStationsSpeedLimitInfo(&station_points, &station_points_s,
                                 &station_speed_limits);
  std::vector<std::vector<SpeedlimitInfoPoint>> additional_speed_limits = {
      {}, {}, {{0.4, 5.0}}, {{0.4, 8.0}}};
  const double first_stop_line_s = -0.1;
  const Vec2d first_stop_point = {-0.1, 0.0};
  optimizer::speedlimit::MergeSpeedLimitWithFirstStopLine(
      first_stop_line_s, first_stop_point, station_points_s, station_points,
      &station_speed_limits, &additional_speed_limits);

  // Check.
  const std::vector<double> station_speed_limits_result = {0.0, 0.0, 0.0, 0.0,
                                                           0.0};
  const std::vector<std::vector<SpeedlimitInfoPoint>>
      additional_speed_limits_result = {{}, {}, {}, {}};
  CheckSpeedLimitResult(station_speed_limits, additional_speed_limits,
                        station_speed_limits_result,
                        additional_speed_limits_result);
}

TEST(MergeSpeedInfoWithStopLineTest, StopLineAfterStations) {
  std::vector<Vec2d> station_points;
  std::vector<double> station_points_s;
  std::vector<double> station_speed_limits;
  GenerateStationsSpeedLimitInfo(&station_points, &station_points_s,
                                 &station_speed_limits);
  std::vector<std::vector<SpeedlimitInfoPoint>> additional_speed_limits = {
      {}, {}, {{0.4, 5.0}}, {{0.4, 8.0}}};
  const double first_stop_line_s = 4.2;
  const Vec2d first_stop_point = {4.2, 0.0};
  optimizer::speedlimit::MergeSpeedLimitWithFirstStopLine(
      first_stop_line_s, first_stop_point, station_points_s, station_points,
      &station_speed_limits, &additional_speed_limits);

  // Check.
  const std::vector<double> station_speed_limits_result = {10.0, 9.0, 5.0, 8.0,
                                                           10.0};
  const std::vector<std::vector<SpeedlimitInfoPoint>>
      additional_speed_limits_result = {{}, {}, {{0.4, 5.0}}, {{0.4, 8.0}}};
  CheckSpeedLimitResult(station_speed_limits, additional_speed_limits,
                        station_speed_limits_result,
                        additional_speed_limits_result);
}

TEST(MergeSpeedInfoWithStopLineTest, StopLineCloseToStation) {
  std::vector<Vec2d> station_points;
  std::vector<double> station_points_s;
  std::vector<double> station_speed_limits;
  GenerateStationsSpeedLimitInfo(&station_points, &station_points_s,
                                 &station_speed_limits);
  std::vector<std::vector<SpeedlimitInfoPoint>> additional_speed_limits = {
      {}, {}, {{0.4, 5.0}}, {{0.4, 8.0}}};
  const double first_stop_line_s = 2.0000001;
  const Vec2d first_stop_point = {2.0000001, 0.0};
  optimizer::speedlimit::MergeSpeedLimitWithFirstStopLine(
      first_stop_line_s, first_stop_point, station_points_s, station_points,
      &station_speed_limits, &additional_speed_limits);

  // Check.
  const std::vector<double> station_speed_limits_result = {10.0, 9.0, 0.0, 0.0,
                                                           0.0};
  const std::vector<std::vector<SpeedlimitInfoPoint>>
      additional_speed_limits_result = {{}, {}, {}, {}};
  CheckSpeedLimitResult(station_speed_limits, additional_speed_limits,
                        station_speed_limits_result,
                        additional_speed_limits_result);
}

TEST(MergeSpeedInfoWithStopLineTest, StopLineBetweenStation) {
  std::vector<Vec2d> station_points;
  std::vector<double> station_points_s;
  std::vector<double> station_speed_limits;
  GenerateStationsSpeedLimitInfo(&station_points, &station_points_s,
                                 &station_speed_limits);
  std::vector<std::vector<SpeedlimitInfoPoint>> additional_speed_limits = {
      {}, {}, {{0.4, 5.0}}, {{0.4, 8.0}}};
  const double first_stop_line_s = 2.5;
  const Vec2d first_stop_point = {2.5, 0.0};
  optimizer::speedlimit::MergeSpeedLimitWithFirstStopLine(
      first_stop_line_s, first_stop_point, station_points_s, station_points,
      &station_speed_limits, &additional_speed_limits);

  // Check.
  const std::vector<double> station_speed_limits_result = {10.0, 9.0, 5.0, 0.0,
                                                           0.0};
  const std::vector<std::vector<SpeedlimitInfoPoint>>
      additional_speed_limits_result = {{}, {}, {{0.4, 5.0}, {0.5, 0.0}}, {}};
  CheckSpeedLimitResult(station_speed_limits, additional_speed_limits,
                        station_speed_limits_result,
                        additional_speed_limits_result);
}

constexpr double kRefMaxA = 1.0;
constexpr double kRefMinA = -1.0;
constexpr double kLeadingSOffset = 0.0;

void GeneratesStartPointAndSpeedLimitInfo(
    TrajectoryPoint* plan_start_point, std::vector<Vec2d>* station_points,
    std::vector<double>* station_points_s,
    std::vector<double>* station_speed_limits,
    std::vector<std::vector<SpeedlimitInfoPoint>>* additional_speed_limits) {
  plan_start_point->set_pos(Vec2d(0.0, 0.0));
  plan_start_point->set_v(8.0);
  plan_start_point->set_a(0.0);
  plan_start_point->set_theta(0.0);

  constexpr int kStationCount = 100;

  station_points->reserve(kStationCount);
  station_speed_limits->reserve(kStationCount);
  station_points_s->reserve(kStationCount);
  additional_speed_limits->reserve(kStationCount);
  for (int i = 0; i < kStationCount; ++i) {
    station_points->push_back({static_cast<double>(i), 0.0});
    station_speed_limits->push_back(9.9);
    station_points_s->push_back(static_cast<double>(i));
    additional_speed_limits->push_back({});
  }
}

void CheckSpatialRefResult(
    const std::vector<double>& s_ref,
    const std::vector<optimizer::LeadingInfo>& leading_min_s) {
  for (int i = 1; i < s_ref.size(); ++i) {
    QCHECK_GE(s_ref[i], s_ref[i - 1]);
    if (i < leading_min_s.size()) {
      QCHECK_LE(s_ref[i], leading_min_s[i].s);
    }
  }
}

TEST(CreateSpatialRefTest, Acceleration) {
  TrajectoryPoint plan_start_point;
  std::vector<Vec2d> station_points;
  std::vector<double> station_speed_limits;
  std::vector<double> station_points_s;
  std::vector<std::vector<SpeedlimitInfoPoint>> additional_speed_limits;
  GeneratesStartPointAndSpeedLimitInfo(&plan_start_point, &station_points,
                                       &station_points_s, &station_speed_limits,
                                       &additional_speed_limits);

  const std::vector<optimizer::LeadingInfo> leading_min_s = {};
  std::vector<double> s_ref = optimizer::speedlimit::CreateSpatialRef(
      /*trajectory_time_step=*/0.2, plan_start_point, station_points,
      station_speed_limits, additional_speed_limits, leading_min_s,
      /*step_count=*/20, kRefMaxA, kRefMinA, kLeadingSOffset);
  CheckSpatialRefResult(s_ref, leading_min_s);
}

TEST(CreateSpatialRefTest, AccelerationAndDeceleration) {
  TrajectoryPoint plan_start_point;
  std::vector<Vec2d> station_points;
  std::vector<double> station_speed_limits;
  std::vector<double> station_points_s;
  std::vector<std::vector<SpeedlimitInfoPoint>> additional_speed_limits;
  GeneratesStartPointAndSpeedLimitInfo(&plan_start_point, &station_points,
                                       &station_points_s, &station_speed_limits,
                                       &additional_speed_limits);
  const int speed_step_index = 55;
  for (int i = 0; i < station_speed_limits.size(); ++i) {
    if (i > speed_step_index) station_speed_limits[i] *= 0.5;
  }

  const std::vector<optimizer::LeadingInfo> leading_min_s = {};
  std::vector<double> s_ref = optimizer::speedlimit::CreateSpatialRef(
      /*trajectory_time_step=*/0.2, plan_start_point, station_points,
      station_speed_limits, additional_speed_limits, leading_min_s,
      /*step_count=*/20, kRefMaxA, kRefMinA, kLeadingSOffset);
  CheckSpatialRefResult(s_ref, leading_min_s);
}

TEST(CreateSpatialRefTest, StaticLeading) {
  TrajectoryPoint plan_start_point;
  std::vector<Vec2d> station_points;
  std::vector<double> station_speed_limits;
  std::vector<double> station_points_s;
  std::vector<std::vector<SpeedlimitInfoPoint>> additional_speed_limits;
  GeneratesStartPointAndSpeedLimitInfo(&plan_start_point, &station_points,
                                       &station_points_s, &station_speed_limits,
                                       &additional_speed_limits);
  int step_count = 20;
  std::vector<optimizer::LeadingInfo> leading_min_s;
  leading_min_s.reserve(step_count);
  for (int i = 0; i < step_count; ++i) {
    leading_min_s.push_back({20.0, 0.0});
  }

  std::vector<double> s_ref = optimizer::speedlimit::CreateSpatialRef(
      /*trajectory_time_step=*/0.2, plan_start_point, station_points,
      station_speed_limits, additional_speed_limits, leading_min_s, step_count,
      kRefMaxA, kRefMinA, kLeadingSOffset);
  CheckSpatialRefResult(s_ref, leading_min_s);
}

TEST(CreateSpatialRefTest, AccelerateLeading) {
  TrajectoryPoint plan_start_point;
  std::vector<Vec2d> station_points;
  std::vector<double> station_speed_limits;
  std::vector<double> station_points_s;
  std::vector<std::vector<SpeedlimitInfoPoint>> additional_speed_limits;
  GeneratesStartPointAndSpeedLimitInfo(&plan_start_point, &station_points,
                                       &station_points_s, &station_speed_limits,
                                       &additional_speed_limits);

  const double a = 0.6;
  const double v_base = 0.0;
  const double s_base = 20.0;
  const int step_count = 20;
  std::vector<optimizer::LeadingInfo> leading_min_s;
  leading_min_s.reserve(step_count);
  for (int i = 0; i < step_count * 0.75; ++i) {
    const double t = 0.2 * a * static_cast<double>(i);
    leading_min_s.push_back(
        {s_base + v_base * t + 0.5 * a * Sqr(t), v_base + a * t});
  }
  std::vector<double> s_ref = optimizer::speedlimit::CreateSpatialRef(
      /*trajectory_time_step=*/0.2, plan_start_point, station_points,
      station_speed_limits, additional_speed_limits, leading_min_s, step_count,
      kRefMaxA, kRefMinA, kLeadingSOffset);
  CheckSpatialRefResult(s_ref, leading_min_s);
}

TEST(CreateSpatialRefTest, DecelAndAccelLeading) {
  TrajectoryPoint plan_start_point;
  std::vector<Vec2d> station_points;
  std::vector<double> station_speed_limits;
  std::vector<double> station_points_s;
  std::vector<std::vector<SpeedlimitInfoPoint>> additional_speed_limits;
  GeneratesStartPointAndSpeedLimitInfo(&plan_start_point, &station_points,
                                       &station_points_s, &station_speed_limits,
                                       &additional_speed_limits);

  const double a = -0.5;
  const double v_base = 2.5;
  const double s_base = 20.0;
  int step_count = 40;
  std::vector<optimizer::LeadingInfo> leading_min_s;
  leading_min_s.reserve(step_count);
  for (int i = 0; i < step_count * 0.5; ++i) {
    const double t = 0.2 * static_cast<double>(i);
    leading_min_s.push_back(
        {s_base + v_base * t + 0.5 * a * Sqr(t), v_base + a * t});
  }
  const double s_base2 = leading_min_s.back().s;
  const double v_base2 = leading_min_s.back().v;
  const int step_index2 = static_cast<double>(step_count * 0.5);
  const double a2 = 1.5;
  for (int i = step_index2; i < step_count; ++i) {
    const double t = 0.2 * static_cast<double>(i - step_index2);
    leading_min_s.push_back(
        {s_base2 + v_base2 * t + 0.5 * a2 * Sqr(t), v_base2 + a2 * t});
  }
  std::vector<double> s_ref = optimizer::speedlimit::CreateSpatialRef(
      /*trajectory_time_step=*/0.2, plan_start_point, station_points,
      station_speed_limits, additional_speed_limits, leading_min_s, step_count,
      kRefMaxA, kRefMinA, kLeadingSOffset);
  CheckSpatialRefResult(s_ref, leading_min_s);
}

TEST(CreateSpatialRefTest, BackLeading) {
  TrajectoryPoint plan_start_point;
  std::vector<Vec2d> station_points;
  std::vector<double> station_speed_limits;
  std::vector<double> station_points_s;
  std::vector<std::vector<SpeedlimitInfoPoint>> additional_speed_limits;
  GeneratesStartPointAndSpeedLimitInfo(&plan_start_point, &station_points,
                                       &station_points_s, &station_speed_limits,
                                       &additional_speed_limits);

  const double a = -0.5;
  const double v_base = 0.0;
  const double s_base = 20.0;
  int step_count = 40;
  std::vector<optimizer::LeadingInfo> leading_min_s;
  leading_min_s.reserve(step_count);
  for (int i = 0; i < step_count * 0.5; ++i) {
    const double t = 0.2 * static_cast<double>(i);
    leading_min_s.push_back(
        {s_base + v_base * t + 0.5 * a * Sqr(t), v_base + a * t});
  }
  const double s_base2 = leading_min_s.back().s;
  const double v_base2 = leading_min_s.back().v;
  const int step_index2 = static_cast<double>(step_count * 0.5);
  const double a2 = 0.6;
  for (int i = step_index2; i < step_count * 0.95; ++i) {
    const double t = 0.2 * static_cast<double>(i - step_index2);
    leading_min_s.push_back(
        {s_base2 + v_base2 * t + 0.5 * a2 * Sqr(t), v_base2 + a2 * t});
  }

  std::vector<double> s_ref = optimizer::speedlimit::CreateSpatialRef(
      /*trajectory_time_step=*/0.2, plan_start_point, station_points,
      station_speed_limits, additional_speed_limits, leading_min_s, step_count,
      kRefMaxA, kRefMinA, kLeadingSOffset);
  CheckSpatialRefResult(s_ref, leading_min_s);
}

}  // namespace
}  // namespace planner
}  // namespace qcraft

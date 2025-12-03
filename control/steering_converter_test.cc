#include "onboard/control/steering_converter.h"

#include <vector>

#include "gtest/gtest.h"

namespace qcraft::control {
namespace {

constexpr double kEpsilon = 1e-3;
constexpr double kDt = 0.01;  // s.

constexpr double kMaxSteerAngle = 9.42;  // rad.
constexpr double kWheelBase = 2.84;      // m.
constexpr double kSteerRatio = 16.0;

SteeringConverter BuildSteeringConverter() {
  VehicleGeometryParamsProto vehicle_geometry_params;
  VehicleDriveParamsProto vehicle_drive_params;

  vehicle_geometry_params.set_wheel_base(kWheelBase);
  vehicle_drive_params.set_steer_ratio(kSteerRatio);
  vehicle_drive_params.set_max_steer_angle(kMaxSteerAngle);

  return SteeringConverter(vehicle_geometry_params, vehicle_drive_params);
}

TEST(SteeringConverterTest, KappaToOthers) {
  SteeringConverter steering_converter = BuildSteeringConverter();
  const std::vector<double> kappa{-0.1, 0.0, 0.1, 0.2};
  const std::vector<double> frontwheel_expected{-0.276714037, 0.0, 0.276714037,
                                                0.516557682};

  for (size_t i = 0; i < kappa.size(); ++i) {
    const double front_wheel_angle =
        steering_converter.KappaToFrontWheelAngle(kappa[i]);
    EXPECT_NEAR(front_wheel_angle, frontwheel_expected[i], kEpsilon);

    const double steer_angle = steering_converter.KappaToSteerAngle(kappa[i]);
    const double steer_angle_expected = frontwheel_expected[i] * kSteerRatio;
    EXPECT_NEAR(steer_angle, steer_angle_expected, kEpsilon);

    const double steer_pct = steering_converter.KappaToSteerPct(kappa[i]);
    const double steer_pct_expected =
        steer_angle_expected / kMaxSteerAngle * 100.0;
    EXPECT_NEAR(steer_pct, steer_pct_expected, kEpsilon);
  }
}

TEST(SteeringConverterTest, SteerPctToFrontWheelAngle) {
  SteeringConverter steering_converter = BuildSteeringConverter();
  const std::vector<double> steer_pct{-30.0, 0.0, 50.0, 110.0};

  for (size_t i = 0; i < steer_pct.size(); ++i) {
    const double front_wheel_angle =
        steer_pct[i] * 0.01 * kMaxSteerAngle / kSteerRatio;
    EXPECT_NEAR(steering_converter.SteerPctToFrontWheelAngle(steer_pct[i]),
                front_wheel_angle, kEpsilon);
    EXPECT_EQ(front_wheel_angle * kSteerRatio > kMaxSteerAngle,
              steer_pct[i] > 100.0);
  }
}

TEST(SteeringConverterTest, OthersToKappa) {
  SteeringConverter steering_converter = BuildSteeringConverter();
  const std::vector<double> steer_pct{-30.0, 0.0, 50.0, 110.0};

  for (size_t i = 0; i < steer_pct.size(); ++i) {
    const double steer_angle = steer_pct[i] * 0.01 * kMaxSteerAngle;
    const double front_wheel_angle = steer_angle / kSteerRatio;

    const double kappa_a =
        steering_converter.FrontWheelAngleToKappa(front_wheel_angle);
    const double kappa_b = steering_converter.SteerAngleToKappa(steer_angle);
    EXPECT_NEAR(kappa_a, kappa_b, kEpsilon);
  }
}

TEST(SteeringConverterTest, OthersToKappaRate) {
  SteeringConverter steering_converter = BuildSteeringConverter();
  const std::vector<double> steer_pct{-30.0, 0.0, 50.0, 110.0};

  for (size_t i = 0; i < steer_pct.size(); ++i) {
    const double steer_angle = steer_pct[i] * 0.01 * kMaxSteerAngle;
    const double front_wheel_angle = steer_angle / kSteerRatio;

    const double kappa_a =
        steering_converter.FrontWheelAngleToKappa(front_wheel_angle);
    const double kappa_b = steering_converter.SteerAngleToKappa(steer_angle);
    EXPECT_NEAR(kappa_a, kappa_b, kEpsilon);
  }
}

TEST(SteeringConverterTest, SteerRateTest) {
  SteeringConverter steering_converter = BuildSteeringConverter();

  const std::vector<double> kappa_set{0.015,  0.009,  0.003,  0.0,
                                      -0.002, -0.007, -0.011, -0.013};

  for (size_t i = 0; i < kappa_set.size(); ++i) {
    for (size_t j = i; j < kappa_set.size(); ++j) {
      const double kappa_a = kappa_set[i];
      const double kappa_b = kappa_set[j];
      const double kappa_rate = (kappa_b - kappa_a) / kDt;

      const double steer_angle_a =
          steering_converter.KappaToSteerAngle(kappa_a);
      const double steer_angle_b =
          steering_converter.KappaToSteerAngle(kappa_b);

      const double steer_angle_rate_expected =
          (steer_angle_b - steer_angle_a) / kDt;
      const double steer_angle_rate = steering_converter.KappaRateToSteerRate(
          kappa_rate, 0.5 * (kappa_a + kappa_b));

      EXPECT_NEAR(steer_angle_rate_expected, steer_angle_rate, 0.1)
          << "i = " << i << ", j = " << j
          << ", steer_angle_rate_expected = " << steer_angle_rate_expected
          << ", steer_angle_rate = " << steer_angle_rate;
    }
  }
}

TEST(SteeringConverterTest, KappaRateTest) {
  SteeringConverter steering_converter = BuildSteeringConverter();

  const std::vector<double> steer_pct_set{1.0, 0.5, 0.3, 0.0, -0.2, -0.5, -1.0};

  for (size_t i = 0; i < steer_pct_set.size(); ++i) {
    for (size_t j = i; j < steer_pct_set.size(); ++j) {
      const double steer_angle_a = steer_pct_set[i] * 0.01 * kMaxSteerAngle;
      const double steer_angle_b = steer_pct_set[j] * 0.01 * kMaxSteerAngle;
      const double steer_rate = (steer_angle_b - steer_angle_a) / kDt;

      const double kappa_a =
          steering_converter.SteerAngleToKappa(steer_angle_a);
      const double kappa_b =
          steering_converter.SteerAngleToKappa(steer_angle_b);

      const double kappa_rate_expected = (kappa_b - kappa_a) / kDt;
      const double kappa_rate = steering_converter.SteerRateToKappaRate(
          steer_rate, 0.5 * (steer_angle_a + steer_angle_b));

      EXPECT_NEAR(kappa_rate_expected, kappa_rate, kEpsilon)
          << "i = " << i << ", j = " << j
          << ", kappa_rate_expected = " << kappa_rate_expected
          << ", kappa_rate = " << kappa_rate;
    }
  }
}

TEST(SteeringConverterTest, ClampKappaOrSteerTest) {
  SteeringConverter steering_converter = BuildSteeringConverter();

  const std::vector<double> kappa{-2.0, -1.0, -0.2, 0.0, 0.1,
                                  0.3,  0.5,  10.0, 20.0};

  for (size_t i = 0; i < kappa.size(); ++i) {
    const double max_kappa =
        steering_converter.SteerAngleToKappa(kMaxSteerAngle);

    EXPECT_GE(max_kappa,
              steering_converter.ClampKappaByMaxSteerAngle(kappa[i]));
    EXPECT_EQ(kappa[i] > max_kappa,
              steering_converter.KappaToSteerPct(kappa[i]) > 100.0);
  }
}

}  // namespace
}  // namespace qcraft::control

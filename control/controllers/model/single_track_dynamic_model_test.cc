#include "onboard/control/controllers/model/single_track_dynamic_model.h"

#include "Eigen/Core"

#include "gtest/gtest.h"

namespace qcraft::control {
namespace {

constexpr int kHorizon = 5;

DynamicModelMeasurement NoSlipDynamicModelMeasurement() {
  DynamicModelMeasurement measurement;
  measurement.yaw = 1.0;
  measurement.v = std::vector<double>(kHorizon, 10.0);

  return measurement;
}

DynamicModelConfProto DefaultDynamicModelConfProto() {
  DynamicModelConfProto conf;

  conf.set_mass(2500.0);
  conf.mutable_geo_params()->set_cg_ratio(0.35);
  conf.mutable_tire_params()->set_c_af(40000.0);
  conf.mutable_tire_params()->set_c_ar(50000.0);

  return conf;
}

TEST(VehicleDynamicModelTest, NoSlipModelTest) {
  DynamicModelMeasurement measurement = NoSlipDynamicModelMeasurement();
  DynamicModelConfProto conf = DefaultDynamicModelConfProto();

  TimeVaryingDiscreteStateSpace state_space = BuildSingleTrackDynamicModel(
      /*time_step*/ 1.0, /*wheel_base*/ 3.0, /*vehicle_length*/ 4.5,
      measurement, conf);

  EXPECT_EQ(state_space.Steps(), kHorizon);
  EXPECT_EQ(state_space.StateSize(), 6);
  EXPECT_EQ(state_space.InputSize(), 1);

  const Eigen::VectorXd init_state = Eigen::VectorXd::Zero(6);
  const std::vector<Eigen::VectorXd> input_vector =
      std::vector<Eigen::VectorXd>(kHorizon, Eigen::VectorXd::Zero(1));

  std::vector<Eigen::VectorXd> state_iteration =
      EvaluateTvdStateSpace(init_state, state_space, input_vector);

  for (const auto& state : state_iteration) {
    EXPECT_NE(state(0), init_state(0));
    EXPECT_NE(state(1), init_state(1));
    EXPECT_EQ(state(2), init_state(2));
    EXPECT_EQ(state(3), init_state(3));
    EXPECT_EQ(state(4), init_state(4));
    EXPECT_EQ(state(5), init_state(5));
  }
}

}  // namespace
}  // namespace qcraft::control

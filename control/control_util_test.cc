#include "onboard/control/control_util.h"

#include <vector>

#include "gtest/gtest.h"

#include "onboard/params/v2/proto/vehicle/common.pb.h"

namespace qcraft::control {
namespace {

TEST(ControlUtilTest, HasPhysicalSteeringWheelTest) {
  const std::vector<VehicleModel> vehicle_models{
      VEHICLE_ZHONGTONG55, VEHICLE_DONGFENG, VEHICLE_LINCOLN_MKZ,
      VEHICLE_UNKNOWN};
  const std::vector<bool> expect_res{false, false, true, true};

  EXPECT_EQ(vehicle_models.size(), expect_res.size());

  for (size_t i = 0; i < vehicle_models.size(); ++i) {
    const bool has_steering_wheel = HasPhysicalSteeringWheel(vehicle_models[i]);
    EXPECT_EQ(has_steering_wheel, expect_res[i]);
  }
}

}  // namespace
}  // namespace qcraft::control

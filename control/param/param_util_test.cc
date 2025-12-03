#include "onboard/control/param/param_util.h"

#include "gtest/gtest.h"

namespace qcraft::control {
namespace {

TEST(ClassifyVehicleTest, ReturnsCorrectType) {
  EXPECT_EQ(ClassifyVehicle(VEHICLE_LINCOLN_MKZ),
            VehicleClassification::kPassengerCar);
  EXPECT_EQ(ClassifyVehicle(VEHICLE_JINLV_MINIBUS),
            VehicleClassification::kMiniBus);
  EXPECT_EQ(ClassifyVehicle(VEHICLE_ZHONGTONG),
            VehicleClassification::kShuttle);
  EXPECT_EQ(ClassifyVehicle(VEHICLE_PIXLOOP),
            VehicleClassification::kLogisticsVehicle);
}

}  // namespace
}  // namespace qcraft::control

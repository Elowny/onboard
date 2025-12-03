#include "onboard/planner/util/vehicle_util.h"

#include "gtest/gtest.h"

namespace qcraft {
namespace planner {
namespace {

TEST(VehicleUtilTest, IsBusTest) {
  EXPECT_TRUE(IsBus(VEHICLE_ZHONGXING));
  EXPECT_TRUE(IsBus(VEHICLE_JINLV_MINIBUS));
  EXPECT_TRUE(IsBus(VEHICLE_HIGER));

  EXPECT_FALSE(IsBus(VEHICLE_UNKNOWN));
  EXPECT_FALSE(IsBus(VEHICLE_PIXLOOP));
  EXPECT_FALSE(IsBus(VEHICLE_SKYWELL));
}

}  // namespace
}  // namespace planner
}  // namespace qcraft

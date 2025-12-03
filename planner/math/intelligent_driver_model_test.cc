
#include "onboard/planner/math/intelligent_driver_model.h"

#include <limits>
#include <memory>

#include "gtest/gtest.h"

namespace qcraft::planner {
namespace {
TEST(IDM, Free) {
  idm::Parameters params{.v_desire = 10.0,
                         .s_min = 5.0,
                         .t_desire = 1.5,
                         .acc_max = 2.0,
                         .comfortable_brake = 3.0,
                         .brake_max = 6.0,
                         .delta = 4.0};

  EXPECT_NEAR(ComputeIDMAcceleration(10.0, std::numeric_limits<double>::max(),
                                     0.0, params),
              0.0, 1e-3);
  EXPECT_NEAR(ComputeIDMAcceleration(0.0, std::numeric_limits<double>::max(),
                                     0.0, params),
              2.0, 1e-3);
  EXPECT_NEAR(ComputeIDMAcceleration(20.0, std::numeric_limits<double>::max(),
                                     0.0, params),
              -6.0, 1e-3);
}

TEST(IDM, OBJ) {
  idm::Parameters params{.v_desire = 10.0,
                         .s_min = 5.0,
                         .t_desire = 1.5,
                         .acc_max = 2.0,
                         .comfortable_brake = 3.0,
                         .brake_max = 6.0,
                         .delta = 4.0};
  EXPECT_NEAR(ComputeIDMAcceleration(10.0, 0.0, 0.0, params), -params.brake_max,
              1e-3);
}

}  // namespace
}  // namespace qcraft::planner

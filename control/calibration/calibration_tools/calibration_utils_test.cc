#include "onboard/control/calibration/calibration_tools/calibration_utils.h"

#include <memory>

#include "gtest/gtest.h"

TEST(PolyFit, one_order) {
  constexpr double kEpsilon = 1e-5;
  constexpr int kOrder = 1;
  auto ployfit_ptr = std::make_unique<qcraft::control::PolyFit>(kOrder);

  std::vector<double> x{1.0, 2.0, 3.0};
  std::vector<double> y{2.0, 4.0, 6.0};
  const auto status = ployfit_ptr->ComputerCoff(x, y);
  EXPECT_EQ(status, true);

  const auto out = ployfit_ptr->GetPolyVal(4.0);
  EXPECT_NEAR(out, 8.0, kEpsilon);
}

TEST(PolyFit, two_order) {
  constexpr double kEpsilon = 1e-5;
  constexpr int kOrder = 2;
  auto ployfit_ptr = std::make_unique<qcraft::control::PolyFit>(kOrder);

  std::vector<double> x{1.0, 2.0, 3.0};
  std::vector<double> y{3.5, 10.5, 21.5};
  const auto status = ployfit_ptr->ComputerCoff(x, y);
  EXPECT_EQ(status, true);

  const auto out = ployfit_ptr->GetPolyVal(4.0);
  EXPECT_NEAR(out, 36.5, kEpsilon);
}

TEST(FindNearst, find) {
  constexpr double kEpsilon = 1e-5;

  std::vector<double> x{1.0, 2.0, 3.0};
  const double out = qcraft::control::FindNearst(x, 1.5);
  EXPECT_NEAR(out, 1.0, kEpsilon);
}

#include "onboard/control/controllers/controller_util.h"

#include <algorithm>

#include "gtest/gtest.h"

namespace qcraft {
namespace control {
namespace {

TEST(ControllerUtilTest, IsCmdBackToZeroTest) {
  std::vector<std::vector<double>> test_set;
  test_set.push_back({1, 2, 3, 4, 5});       // Expect 0
  test_set.push_back({5, 4, 3, 2, 1});       // Expect 1
  test_set.push_back({-5, -4, -3, -2, -1});  // Expect 1
  test_set.push_back({5, -4, 3, -2, 1});     // Expect 0
  test_set.push_back({-5, 4, -3, 2, -1});    // Expect 0
  test_set.push_back({1, 1, 1, 1, 0});       // Expect 1
  test_set.push_back({1, 1, 1, 1, 2});       // Expect 0

  std::vector<bool> result;
  result.reserve(test_set.size());
  for (const auto& test : test_set) {
    result.push_back(IsCmdBackToZero(test));
  }

  EXPECT_FALSE(result[0]);
  EXPECT_TRUE(result[1]);
  EXPECT_TRUE(result[2]);
  EXPECT_FALSE(result[3]);
  EXPECT_FALSE(result[4]);
  EXPECT_TRUE(result[5]);
  EXPECT_FALSE(result[6]);
}

}  // namespace
}  // namespace control
}  // namespace qcraft

#include "onboard/planner/speed/path_speed_combiner.h"

#include <algorithm>
#include <utility>
#include <vector>

#include "absl/status/status.h"

#include "gtest/gtest.h"

#include "onboard/planner/speed/speed_point.h"

namespace qcraft::planner {
namespace {
TEST(PathSpeedCombinerTest, SimpleTest) {
  constexpr int kPathSize = 50;
  constexpr int kSpeedSize = 60;
  constexpr int kTrajectorySize = 590;
  std::vector<ApolloTrajectoryPointProto> trajectory_data;

  std::vector<PathPoint> path_points;
  path_points.reserve(kPathSize);
  for (int i = 0; i < kPathSize; ++i) {
    PathPoint temp;
    temp.set_x(0.2 * i);
    temp.set_y(0.0);
    temp.set_s(0.2 * i);
    path_points.push_back(temp);
  }

  std::vector<SpeedPoint> speed_points;
  speed_points.reserve(kSpeedSize);
  for (int j = 0; j < kSpeedSize; ++j) {
    SpeedPoint temp;
    temp.set_s(0.15 * j);
    temp.set_t(j);
    temp.set_v(0.1);
    temp.set_a(0.1);
    temp.set_j(0.1);
    speed_points.push_back(temp);
  }

  DiscretizedPath path_data(std::move(path_points));
  SpeedVector speed_data(std::move(speed_points));

  EXPECT_EQ(path_data.size(), kPathSize);
  EXPECT_EQ(speed_data.size(), kSpeedSize);

  absl::Status status = CombinePathAndSpeed(path_data, /*forward=*/true,
                                            speed_data, &trajectory_data);

  EXPECT_TRUE(status.ok());
  EXPECT_EQ(trajectory_data.size(), kTrajectorySize);
}

}  // namespace
}  // namespace qcraft::planner

#include "onboard/control/lka_control/path_to_trajectory.h"

#include <algorithm>

#include "gtest/gtest.h"

namespace qcraft::control {
namespace {

TEST(PathToTrajectory, compute) {
  std::vector<PathPoint> path;
  for (int i = 0; i < 10; i++) {
    PathPoint path_point;
    path_point.set_x(1.0 * i);
    path_point.set_y(2.0 * i);
    path_point.set_z(0.0);
    path_point.set_kappa(0.1);
    path.push_back(path_point);
  }
  auto trajectory_or = PathToTrajectory(path);
  if (trajectory_or.ok()) {
    for (int i = 0; i < path.size(); i++) {
      auto trajectory = trajectory_or.value();
      EXPECT_NEAR(trajectory.trajectory_point().Get(i).path_point().x(),
                  path[i].x(), 1e-5);
      EXPECT_NEAR(trajectory.trajectory_point().Get(i).path_point().y(),
                  path[i].y(), 1e-5);
      EXPECT_NEAR(trajectory.trajectory_point().Get(i).path_point().z(),
                  path[i].z(), 1e-5);
      EXPECT_NEAR(trajectory.trajectory_point().Get(i).path_point().kappa(),
                  path[i].kappa(), 1e-5);
    }
  }
}

}  // namespace
}  // namespace qcraft::control

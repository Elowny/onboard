#include "onboard/planner/min_length_path_extension.h"

#include <algorithm>
#include <memory>
#include <vector>

#include "gtest/gtest.h"
#include "onboard/math/vec.h"
#include "onboard/planner/util/path_util.h"

namespace qcraft {
namespace planner {
namespace {

TEST(PathExtensionTest, ExtendPathAndDeleteUnreasonablePartTest) {
  std::vector<ApolloTrajectoryPointProto> trajecatory;

  constexpr int kZigzagcount = 2;
  constexpr int kForwardStep = 20;
  constexpr int kBackwardStep = 3;

  const auto generate_trajectory =
      [&](const ApolloTrajectoryPointProto& start, double theta, double v,
          double h, int steps,
          std::vector<ApolloTrajectoryPointProto>* trajectory) {
        const Vec2d tangent = Vec2d::UnitFromAngle(theta);
        for (int k = 0; k < steps; ++k) {
          const double t = k * h;
          const double delta_s = t * v;
          ApolloTrajectoryPointProto temp;
          temp.mutable_path_point()->set_x(start.path_point().x() +
                                           delta_s * tangent.x());
          temp.mutable_path_point()->set_y(start.path_point().y() +
                                           delta_s * tangent.y());
          temp.mutable_path_point()->set_theta(theta);
          temp.mutable_path_point()->set_s(start.path_point().s() + delta_s);
          temp.set_v(v);
          trajectory->push_back(temp);
        }
      };

  ApolloTrajectoryPointProto start;
  start.mutable_path_point()->set_x(0.0);
  start.mutable_path_point()->set_y(0.0);
  start.mutable_path_point()->set_theta(0.0);
  start.mutable_path_point()->set_s(0.0);
  start.set_v(0.0);

  trajecatory.reserve((kZigzagcount + 1) * kForwardStep +
                      kZigzagcount * kBackwardStep + 1);
  trajecatory.push_back(start);
  for (int stage = 0; stage < kZigzagcount; ++stage) {
    const double theta = 0.0;
    const double v = 1.0;
    const double h = 0.2;
    generate_trajectory(trajecatory.back(), theta, v, h, kForwardStep,
                        &trajecatory);
    generate_trajectory(trajecatory.back(), theta, -v, h, kBackwardStep,
                        &trajecatory);
  }
  const double theta = 0.0;
  const double v = 1.0;
  const double h = 0.2;
  generate_trajectory(trajecatory.back(), theta, v, h, kForwardStep,
                      &trajecatory);

  constexpr double kRequiredMinPathLength = 5.0;
  constexpr double kMaxCurvature = 0.2;

  const auto extend_path_output = ExtendPathAndDeleteUnreasonablePart(
      trajecatory, kRequiredMinPathLength, kMaxCurvature);

  EXPECT_TRUE(extend_path_output.ok());

  const std::vector<PathPoint>& path = *extend_path_output;
  // Check if path s is in ascending order.
  for (int k = 0; (k + 1) < path.size(); ++k) {
    EXPECT_LE(path[k].s(), path[k + 1].s());
  }
  // next point is in front of last one.
  for (int k = 0; (k + 1) < path.size(); ++k) {
    const auto& pt_vec = ToVec2d(path[k]);
    const auto& pt_next_vec = ToVec2d(path[k + 1]);
    const Vec2d tangent = Vec2d::UnitFromAngle(path[k].theta());
    EXPECT_GE((pt_next_vec - pt_vec).Dot(tangent), 0.0);
  }
}

}  // namespace
}  // namespace planner
}  // namespace qcraft

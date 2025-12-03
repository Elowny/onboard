#include "onboard/planner/plan/acc/acc_corridor_util.h"

#include <algorithm>
#include <vector>

#include "gtest/gtest.h"

#include "onboard/global/buffered_logger.h"
#include "onboard/lite/check.h"
#include "onboard/math/piecewise_linear_function.h"

namespace qcraft {

namespace planner {
namespace {

TEST(RunAccCorridorUtilTest, LatPointTest) {
  const Vec2d pos(0.0, 0.0);
  const Vec2d tangent = Vec2d(1.0, 0.0).normalized();
  const double l = 5.0;
  const Vec2d res = LatPoint(pos, tangent, l);
  QCHECK_NEAR(res.x(), 0.0, 1e-6);
  QCHECK_NEAR(res.y(), 5.0, 1e-6);
}

TEST(RunAccCorridorUtilTest, ComputeExtendedSmoothPathTest0) {
  std::vector<Vec2d> ref_center_xy;
  std::vector<double> ref_s_vec;

  const Vec2d start_pt(2.5, 3.0);
  const Vec2d tangent = Vec2d(2.0, 1.0).normalized();
  const double ref_step_s = 0.3;
  constexpr int kRefPointsCount = 100;
  ref_center_xy.reserve(kRefPointsCount);
  ref_s_vec.reserve(kRefPointsCount);

  for (int k = 0; k < kRefPointsCount; ++k) {
    const double s = static_cast<double>(k) * ref_step_s;
    ref_center_xy.push_back(start_pt + s * tangent);
    ref_s_vec.push_back(s);
  }

  const Vec2d heading = Vec2d(1.0, 1.0).normalized();
  const double step_s = 0.5;
  const double resample_step_s = 0.2;
  const double extended_length = 5.0;

  const DiscretizedPath path =
      ComputeExtendedSmoothPath(ref_center_xy, ref_s_vec, heading, step_s,
                                resample_step_s, extended_length);

  QCHECK_GE(path.length(), 0.0);
}

TEST(RunAccCorridorUtilTest, ComputeExtendedSmoothPathTest1) {
  std::vector<Vec2d> ref_center_xy;
  std::vector<double> ref_s_vec;

  const Vec2d start_pt(2.5, 3.0);
  const Vec2d tangent = Vec2d(2.0, 1.0).normalized();
  const double ref_step_s = 0.3;
  constexpr int kRefPointsCount = 100;
  ref_center_xy.reserve(kRefPointsCount);
  ref_s_vec.reserve(kRefPointsCount);

  double s = 0.0;
  for (int k = 0; k < kRefPointsCount; ++k) {
    ref_center_xy.push_back(start_pt + s * tangent);
    ref_s_vec.push_back(s);
    if (k == kRefPointsCount / 2) {
      s += 1e-4;
    } else {
      s += ref_step_s;
    }
  }

  const Vec2d heading = Vec2d(1.0, 1.0).normalized();
  const double step_s = 0.5;
  const double resample_step_s = 0.2;
  const double extended_length = 5.0;

  const DiscretizedPath path =
      ComputeExtendedSmoothPath(ref_center_xy, ref_s_vec, heading, step_s,
                                resample_step_s, extended_length);

  QCHECK_GE(path.length(), 0.0);
}

TEST(RunAccCorridorUtilTest, ComputeSmoothedCurvatureProfileTest0) {
  std::vector<Vec2d> xy;
  constexpr int kRefPointsCount = 100;
  xy.reserve(kRefPointsCount);
  for (int k = 0; k < kRefPointsCount / 2; ++k) {
    xy.push_back(Vec2d(static_cast<double>(k) * 1.0, 1.0));
  }
  for (int k = kRefPointsCount / 2; k < kRefPointsCount; ++k) {
    xy.push_back(Vec2d(static_cast<double>(k) * 1.0, 0.0));
  }
  const PiecewiseLinearFunction<double> curv =
      ComputeSmoothedCurvatureProfile(xy);
  QCHECK_GE(curv.x().size(), 0);
  for (const double y : curv.y()) {
    QCHECK(std::isfinite(y));
  }
}

TEST(RunAccCorridorUtilTest, ComputeSmoothedCurvatureProfileTest1) {
  std::vector<Vec2d> xy;
  xy.push_back(Vec2d(0.0, 0.0));
  const PiecewiseLinearFunction<double> curv =
      ComputeSmoothedCurvatureProfile(xy);
  QCHECK_EQ(curv.x().size(), 2);
  QCHECK_EQ(curv.y().size(), 2);
  for (const double y : curv.y()) {
    QCHECK_EQ(y, 0.0);
  }
}

TEST(RunAccCorridorUtilTest, ComputeSmoothedCurvatureProfileTest2) {
  std::vector<Vec2d> xy;
  xy.push_back(Vec2d(0.0, 0.0));
  xy.push_back(Vec2d(1.0, 0.0));
  const PiecewiseLinearFunction<double> curv =
      ComputeSmoothedCurvatureProfile(xy);
  QCHECK_GE(curv.x().size(), 0);
  for (const double y : curv.y()) {
    QCHECK_EQ(y, 0.0);
  }
}

TEST(RunAccCorridorUtilTest, EvaluatePathPointOnCubicSplinesTest) {
  const std::vector<double> x_s_x = {0.0, 1.0, 2.1, 3.3, 4.5, 5.0};
  const std::vector<double> y_s_x = {0.0, 0.2, 1.0, 0.7, 0.3, 0.1};
  std::vector<double> s;
  s.reserve(x_s_x.size());
  s.push_back(0.0);
  for (int i = 0; i + 1 < x_s_x.size(); ++i) {
    s.push_back(s.back() +
                std::hypot(x_s_x[i + 1] - x_s_x[i], y_s_x[i + 1] - y_s_x[i]));
  }

  const CubicSpline x_s(s, x_s_x);
  const CubicSpline y_s(s, y_s_x);

  const double cur_s = 4.88179;

  const PathPoint ppt = EvaluatePathPointOnCubicSplines(x_s, y_s, cur_s);
  QCHECK_NEAR(ppt.x(), 4.5, 1e-3);
  QCHECK_NEAR(ppt.y(), 0.3, 1e-3);
  QCHECK_NEAR(ppt.s(), cur_s, 1e-3);
  QCHECK_LE(ppt.theta(), 0.0);
  QCHECK_LE(ppt.kappa(), 0.0);
}

TEST(RunAccCorridorUtilTest, ComputeCurvatureTest0) {
  const double dx = 0.1;
  const double dy = 0.1;
  const double ddx = 0.05;
  const double ddy = 0.05;
  const double curv = ComputeCurvature(dx, dy, ddx, ddy);
  QCHECK_NEAR(curv, 0.0, 1e-6);
}

TEST(RunAccCorridorUtilTest, ComputeCurvatureTest1) {
  const double dx = 0.0;
  const double dy = 0.0;
  const double ddx = 0.05;
  const double ddy = 0.05;
  const double curv = ComputeCurvature(dx, dy, ddx, ddy);
  QCHECK_NEAR(curv, 0.0, 1e-6);
}

}  // namespace
}  // namespace planner
}  // namespace qcraft

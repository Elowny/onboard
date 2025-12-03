#include "onboard/planner/util/min_segment_distance_problem.h"

#include <stdlib.h>

#include <cmath>
#include <utility>

#include "gtest/gtest.h"
// IWYU pragma: no_include "Eigen/Core"
// IWYU pragma: no_include "Eigen/SparseCore"

namespace qcraft::planner {
namespace {

TEST(MinSegmentDistanceProblemTest, MinSegmentDistanceProblemTestQTFM) {
  constexpr double kEpsilon = 1e-9;
  constexpr double kDerivativeRatioEpsilon = 0.01;
  constexpr double kDerivativeAbsoluteEpsilon = 1e-3;
  {
    std::vector<MsdProblemWithBuffer::SegmentType> named_segments = {
        {"1", {Vec2d{0, 0}, Vec2d{0, 1}}},
        {"2", {Vec2d{0, 1}, Vec2d{1, 1}}},
        {"3", {Vec2d{1, 1}, Vec2d{1, 0}}},
        {"4", {Vec2d{1, 0}, Vec2d{0, 0}}}};
    MsdProblemWithBuffer problem(std::move(named_segments),
                                 /*cutoff_distance=*/3.0, std::vector<int>{0});

    EXPECT_NEAR(problem.Evaluate(Vec2d{0.5, 0.5}), 0.5, kEpsilon);
    EXPECT_NEAR(problem.Evaluate(Vec2d{1.5, 1.5}), std::sqrt(2.0) * 0.5,
                kEpsilon);
    EXPECT_NEAR(problem.Evaluate(Vec2d{-0.5, -0.5}), std::sqrt(2.0) * 0.5,
                kEpsilon);

    std::string debug;
    problem.EvaluateWithNearestSegmentId(Vec2d{1.5, 0.5}, &debug);
    EXPECT_EQ(debug, "3");

    problem.EvaluateWithNearestSegmentId(Vec2d{-0.5, 0.5}, &debug);
    EXPECT_EQ(debug, "1");

    problem.EvaluateWithNearestSegmentId(Vec2d{0.5, -1.0}, &debug);
    EXPECT_EQ(debug, "4");
  }

  {
    // Random generators.
    // Please don't modify the random seed, it is used to escape bad numerical
    // derivative.
    unsigned int seed = 55;
    const double width = 10.0;
    const double buffer = 10.0;
    auto random_segment = [&seed, &width]() {
      const double sx = (rand_r(&seed) * 1.0) / RAND_MAX;
      const double sy = (rand_r(&seed) * 1.0) / RAND_MAX;
      const double ex = (rand_r(&seed) * 1.0) / RAND_MAX;
      const double ey = (rand_r(&seed) * 1.0) / RAND_MAX;
      return Segment2d{Vec2d{width * sx, width * sy},
                       Vec2d{width * ex, width * ey}};
    };

    auto random_sample = [&seed, &width, &buffer]() {
      const double rx = (rand_r(&seed) * 1.0) / RAND_MAX;
      const double ry = (rand_r(&seed) * 1.0) / RAND_MAX;
      return Vec2d{rx * (width + 2 * buffer) - buffer,
                   ry * (width + 2 * buffer) - buffer};
    };

    // Create a random MSD problem.
    const int num_segments = 50;
    std::vector<std::pair<std::string, Segment2d>> named_segments;
    named_segments.reserve(num_segments);
    for (int i = 0; i < num_segments; ++i) {
      named_segments.emplace_back(std::to_string(i), random_segment());
    }

    std::vector<int> start_segment_ids;
    start_segment_ids.reserve(50);
    for (int i = 0; i < num_segments; ++i) {
      start_segment_ids.push_back(i);
    }

    MinSegmentDistanceProblem problem(
        std::move(named_segments),
        /*cut off distance = */ buffer + width * std::sqrt(2.0),
        start_segment_ids);

    const double delta = 0.0001;

    auto check_derivative_near = [](double a, double b) {
      // Derivatives shall either:
      // near in absolute value.
      // or near in ratio.
      if (std::abs(a - b) < kDerivativeAbsoluteEpsilon) {
        // For small derivatives only check near 0.0.
        return;
      }
      EXPECT_NEAR(a / b, 1.0, kDerivativeRatioEpsilon);
    };

    const int num_samples = 1000;
    for (int i = 0; i < num_samples; ++i) {
      const Vec2d sample00 = random_sample();
      const Vec2d sample01 = sample00 + Vec2d{0.0, delta};
      const Vec2d sample11 = sample00 + Vec2d{delta, delta};
      const Vec2d sample10 = sample00 + Vec2d{delta, 0.0};
      const Vec2d sample02 = sample00 + Vec2d{0.0, 2 * delta};
      const Vec2d sample20 = sample00 + Vec2d{2 * delta, 0.0};

      const double f00 = problem.Evaluate(sample00);
      const double f01 = problem.Evaluate(sample01);
      const double f11 = problem.Evaluate(sample11);
      const double f10 = problem.Evaluate(sample10);
      const double f02 = problem.Evaluate(sample02);
      const double f20 = problem.Evaluate(sample20);

      const double df_dx = (f10 - f00) / delta;
      const double df_dy = (f01 - f00) / delta;
      const double d2f_dx2 = ((f20 - f10) - (f10 - f00)) / (delta * delta);
      const double d2f_dy2 = ((f02 - f01) - (f01 - f00)) / (delta * delta);
      const double d2f_dx_dy = ((f11 - f01) - (f10 - f00)) / (delta * delta);

      MinSegmentDistanceProblem::SecondOrderDerivativeType derivative;
      const double f_result =
          problem.EvaluateWithSecondOrderDerivatives(sample00, &derivative);

      EXPECT_NEAR(f00, f_result, kEpsilon);
      check_derivative_near(df_dx, derivative.df_dx);
      check_derivative_near(df_dy, derivative.df_dy);
      check_derivative_near(d2f_dx2, derivative.d2f_dx_dx);
      check_derivative_near(d2f_dy2, derivative.d2f_dy_dy);
      check_derivative_near(d2f_dx_dy, derivative.d2f_dx_dy);
    }
  }
}

}  // namespace

}  // namespace qcraft::planner

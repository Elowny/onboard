#include "onboard/planner/math/quintic_spiral_boundary_value_problem.h"

#include <cmath>
#include <memory>
#include <ostream>
#include <string>
#include <vector>

#include "glog/logging.h"
#include "gtest/gtest.h"

namespace qcraft::planner {
namespace {
TEST(QuinticSpiralBoundaryValueProblem, StraightLine) {
  const SpiralPoint sp0 = {.x = 0.0,
                           .y = 0.0,
                           .theta = 0.0,
                           .k = 0.0,
                           .s = 0.0,
                           .dk = 0.0,
                           .ddk = 0.0};
  const SpiralPoint sp1 = {
      .x = 1.0, .y = 0.0, .theta = 0.0, .k = 0.0, .s = 0.0};
  const auto spirals = QuinticSpiralBoundaryValueSolver::solve(sp0, sp1);
  EXPECT_EQ(spirals.size(), 1);
  const auto end = spirals[0].Eval(/*s=*/1.0, /*num_steps=*/8);
  EXPECT_LT(fabs(end.x - 1.0), 1e-3);
  EXPECT_LT(fabs(end.y - 0.0), 1e-3);
  EXPECT_LT(fabs(end.k - 0.0), 1e-6);
  EXPECT_LT(fabs(end.theta), 1e-6);
}

TEST(QuinticSpiralBoundaryValueProblem, OneFourthCircle) {
  const SpiralPoint sp0 = {.x = 0.0,
                           .y = 0.0,
                           .theta = 0.0,
                           .k = 1.0,
                           .s = 0.0,
                           .dk = 0.0,
                           .ddk = 0.0};
  const SpiralPoint sp1 = {
      .x = 1.0, .y = 1.0, .theta = M_PI / 2.0, .k = 1.0, .s = 0.0};
  const auto spirals = QuinticSpiralBoundaryValueSolver::solve(sp0, sp1);
  EXPECT_EQ(spirals.size(), 1);
  const auto end = spirals[0].Eval(/*s=*/M_PI / 2.0, /*num_steps=*/8);
  EXPECT_LT(fabs(end.x - 1.0), 1e-2);
  EXPECT_LT(fabs(end.y - 1.0), 1e-2);
  EXPECT_LT(fabs(end.k - 1.0), 1e-3);
  EXPECT_LT(fabs(end.theta - M_PI / 2.0), 1e-3);
}

TEST(QuinticSpiralBoundaryValueProblem, HalfCircle) {
  const SpiralPoint sp0 = {.x = 0.0,
                           .y = 0.0,
                           .theta = 0.0,
                           .k = 1.0,
                           .s = 0.0,
                           .dk = 0.0,
                           .ddk = 0.0};
  const SpiralPoint sp1 = {
      .x = 0.0, .y = 2.0, .theta = M_PI, .k = 1.0, .s = 0.0};
  const auto spirals = QuinticSpiralBoundaryValueSolver::solve(sp0, sp1);
  EXPECT_EQ(spirals.size(), 2);
  const auto end = spirals[0].Eval(/*s=*/M_PI, /*num_steps=*/8);
  EXPECT_LT(fabs(end.x - 0.0), 1e-2);
  EXPECT_LT(fabs(end.y - 2.0), 5e-2);
  EXPECT_LT(fabs(end.k - 1.0), 1e-3);
  EXPECT_LT(fabs(end.theta - M_PI), 1e-3);
}

TEST(QuinticSpiralBoundaryValuePRoblem, ShiftedRotatedStraightLine) {
  const SpiralPoint sp0 = {.x = 1.0,
                           .y = 1.0,
                           .theta = M_PI / 4.0,
                           .k = 0.0,
                           .s = 0.0,
                           .dk = 0.0,
                           .ddk = 0.0};
  const SpiralPoint sp1 = {
      .x = 2.0, .y = 2.0, .theta = M_PI / 4.0, .k = 0.0, .s = 0.0};
  const auto spirals = QuinticSpiralBoundaryValueSolver::solve(sp0, sp1);
  EXPECT_EQ(spirals.size(), 1);
  const auto end = spirals[0].Eval(/*s=*/sqrt(2.0), /*num_steps=*/8);
  EXPECT_LT(fabs(end.x - 2.0), 1e-3);
  EXPECT_LT(fabs(end.y - 2.0), 1e-3);
  EXPECT_LT(fabs(end.k - 0.0), 1e-6);
  EXPECT_LT(fabs(end.theta - M_PI / 4.0), 1e-6);
}

TEST(QuinticSpiralBoundaryValueProblem, RotateCircle) {
  const SpiralPoint sp0 = {.x = 0.0,
                           .y = 0.0,
                           .theta = M_PI / 2.0,
                           .k = 1.0,
                           .s = 0.0,
                           .dk = 0.0,
                           .ddk = 0.0};
  const SpiralPoint sp1 = {
      .x = -2.0, .y = 0.0, .theta = 3.0 * M_PI / 2.0, .k = 1.0, .s = 0.0};
  const auto spirals = QuinticSpiralBoundaryValueSolver::solve(sp0, sp1);
  EXPECT_EQ(spirals.size(), 2);
  const auto end = spirals[0].Eval(/*s=*/M_PI, /*num_steps=*/8);
  EXPECT_LT(fabs(end.x - (-2.0)), 5e-2);
  EXPECT_LT(fabs(end.y - 0.0), 1e-2);
  EXPECT_LT(fabs(end.k - 1.0), 1e-3);
  EXPECT_LT(fabs(sin(end.theta - (-M_PI / 2.0))), 1e-3);
}

TEST(QuinticSpiralBoundaryValueProblem, ShiftRotateCircle) {
  const SpiralPoint sp0 = {.x = 1.0,
                           .y = 1.0,
                           .theta = M_PI / 2.0,
                           .k = 1.0,
                           .s = 0.0,
                           .dk = 0.0,
                           .ddk = 0.0};
  const SpiralPoint sp1 = {
      .x = -1.0, .y = 1.0, .theta = 3.0 * M_PI / 2.0, .k = 1.0, .s = 0.0};
  const auto spirals = QuinticSpiralBoundaryValueSolver::solve(sp0, sp1);
  EXPECT_EQ(spirals.size(), 2);
  const auto end = spirals[0].Eval(/*s=*/M_PI, /*num_steps=*/8);
  EXPECT_LT(fabs(end.x - (-1.0)), 5e-2);
  EXPECT_LT(fabs(end.y - 1.0), 1e-2);
  EXPECT_LT(fabs(end.k - 1.0), 1e-3);
  EXPECT_LT(fabs(sin(end.theta - (-M_PI / 2.0))), 1e-3);
}

TEST(QuinticSpiralBoundaryValueProblem, NotContinuousAngleShiftRotateCircle) {
  const SpiralPoint sp0 = {.x = 1.0,
                           .y = 1.0,
                           .theta = M_PI / 2.0,
                           .k = 1.0,
                           .s = 0.0,
                           .dk = 0.0,
                           .ddk = 0.0};
  const SpiralPoint sp1 = {
      .x = -1.0, .y = 1.0, .theta = -M_PI / 2.0, .k = 1.0, .s = 0.0};
  const auto spirals = QuinticSpiralBoundaryValueSolver::solve(sp0, sp1);
  EXPECT_EQ(spirals.size(), 2);
  const auto end = spirals[0].Eval(/*s=*/M_PI, /*num_steps=*/8);
  EXPECT_LT(fabs(end.x - (-1.0)), 5e-2);
  EXPECT_LT(fabs(end.y - 1.0), 1e-2);
  EXPECT_LT(fabs(end.k - 1.0), 1e-3);
  EXPECT_LT(fabs(sin(end.theta - (-M_PI / 2.0))), 1e-3);
}

TEST(QuinticSpiralBoundaryValueProblem, OneEighthCircle) {
  // One eighth circle radius = 2, from (0, 0) to (sqrt(2), 2 - sqrt(2)).
  const SpiralPoint sp0 = {.x = 0.0,
                           .y = 0.0,
                           .theta = 0.0,
                           .k = 0.5,
                           .s = 0.0,
                           .dk = 0.0,
                           .ddk = 0.0};
  const SpiralPoint sp1 = {.x = std::sqrt(2.0),
                           .y = 2.0 - std::sqrt(2.0),
                           .theta = M_PI / 4.0,
                           .k = 0.5,
                           .s = 0.0};
  const auto spirals = QuinticSpiralBoundaryValueSolver::solve(sp0, sp1);
  EXPECT_GE(spirals.size(), 1);
  const auto end = spirals[0].Eval(/*s=*/M_PI / 2.0, /*num_steps=*/8);
  EXPECT_LT(fabs(end.x - (std::sqrt(2.0))), 5e-2);
  EXPECT_LT(fabs(end.y - (2.0 - std::sqrt(2.0))), 1e-2);
  EXPECT_LT(fabs(end.k - 0.5), 1e-3);
  EXPECT_LT(fabs(sin(end.theta - (M_PI / 4.0))), 1e-3);
}

TEST(QuinticSpiralBoundaryValueProblem,
     SymNotContinuousAngleShiftRotateCircle) {
  const SpiralPoint sp0 = {.x = 1.0,
                           .y = 1.0,
                           .theta = -M_PI / 2.0,
                           .k = -1.0,
                           .s = 0.0,
                           .dk = 0.0,
                           .ddk = 0.0};
  const SpiralPoint sp1 = {
      .x = -1.0, .y = 1.0, .theta = M_PI / 2.0, .k = -1.0, .s = 0.0};
  const auto spirals = QuinticSpiralBoundaryValueSolver::solve(sp0, sp1);
  EXPECT_EQ(spirals.size(), 2);
  const auto end = spirals[1].Eval(/*s=*/M_PI, /*num_steps=*/8);
  LOG(INFO) << "End point: " << end.DebugString();
  EXPECT_LT(fabs(end.x - (-1.0)), 5e-2);
  EXPECT_LT(fabs(end.y - 1.0), 1e-2);
  EXPECT_LT(fabs(end.k - (-1.0)), 1e-3);
  EXPECT_LT(fabs(sin(end.theta - (M_PI / 2.0))), 1e-3);
}

}  // namespace
}  // namespace qcraft::planner

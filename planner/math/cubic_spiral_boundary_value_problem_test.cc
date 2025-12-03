#include "onboard/planner/math/cubic_spiral_boundary_value_problem.h"

#include <cmath>
#include <memory>
#include <vector>

#include "gtest/gtest.h"

namespace qcraft::planner {
namespace {
TEST(CubicSpiralBoundaryValueProblem, StraightLine) {
  const SpiralPoint sp0 = {
      .x = 0.0, .y = 0.0, .theta = 0.0, .k = 0.0, .s = 0.0};
  const SpiralPoint sp1 = {
      .x = 1.0, .y = 0.0, .theta = 0.0, .k = 0.0, .s = 0.0};
  const auto spirals = CubicSpiralBoundaryValueSolver::solve(sp0, sp1);
  EXPECT_EQ(spirals.size(), 1);
  const auto end = spirals[0].Eval(/*s=*/1.0, /*num_steps=*/8);
  EXPECT_LT(fabs(end.x - 1.0), 1e-3);
  EXPECT_LT(fabs(end.y - 0.0), 1e-3);
  EXPECT_LT(fabs(end.k - 0.0), 1e-6);
  EXPECT_LT(fabs(end.theta), 1e-6);
}

TEST(CubicSpiralBoundaryValueProblem, OneFourthCircle) {
  const SpiralPoint sp0 = {
      .x = 0.0, .y = 0.0, .theta = 0.0, .k = 1.0, .s = 0.0};
  const SpiralPoint sp1 = {
      .x = 1.0, .y = 1.0, .theta = M_PI / 2.0, .k = 1.0, .s = 0.0};
  const auto spirals = CubicSpiralBoundaryValueSolver::solve(sp0, sp1);
  EXPECT_EQ(spirals.size(), 1);
  const auto end = spirals[0].Eval(/*s=*/M_PI / 2.0, /*num_steps=*/8);
  EXPECT_LT(fabs(end.x - 1.0), 1e-2);
  EXPECT_LT(fabs(end.y - 1.0), 1e-2);
  EXPECT_LT(fabs(end.k - 1.0), 1e-3);
  EXPECT_LT(fabs(end.theta - M_PI / 2.0), 1e-3);
}

TEST(CubicSpiralBoundaryValueProblem, HalfCircle) {
  const SpiralPoint sp0 = {
      .x = 0.0, .y = 0.0, .theta = 0.0, .k = 1.0, .s = 0.0};
  const SpiralPoint sp1 = {
      .x = 0.0, .y = 2.0, .theta = M_PI, .k = 1.0, .s = 0.0};
  const auto spirals = CubicSpiralBoundaryValueSolver::solve(sp0, sp1);
  EXPECT_EQ(spirals.size(), 2);
  const auto end = spirals[0].Eval(/*s=*/M_PI, /*num_steps=*/8);
  EXPECT_LT(fabs(end.x - 0.0), 1e-2);
  EXPECT_LT(fabs(end.y - 2.0), 5e-2);
  EXPECT_LT(fabs(end.k - 1.0), 1e-3);
  EXPECT_LT(fabs(end.theta - M_PI), 1e-3);
}

TEST(CubicSpiralBoundaryValueProblem, ShiftedRotatedStraightLine) {
  const SpiralPoint sp0 = {
      .x = 1.0, .y = 1.0, .theta = M_PI / 4.0, .k = 0.0, .s = 0.0};
  const SpiralPoint sp1 = {
      .x = 2.0, .y = 2.0, .theta = M_PI / 4.0, .k = 0.0, .s = 0.0};
  const auto spirals = CubicSpiralBoundaryValueSolver::solve(sp0, sp1);
  EXPECT_EQ(spirals.size(), 1);
  const auto end = spirals[0].Eval(/*s=*/sqrt(2.0), /*num_steps=*/8);
  EXPECT_LT(fabs(end.x - 2.0), 1e-3);
  EXPECT_LT(fabs(end.y - 2.0), 1e-3);
  EXPECT_LT(fabs(end.k - 0.0), 1e-6);
  EXPECT_LT(fabs(end.theta - M_PI / 4.0), 1e-6);
}

TEST(CubicSpiralBoundaryValueProblem, RotateCircle) {
  const SpiralPoint sp0 = {
      .x = 0.0, .y = 0.0, .theta = M_PI / 2.0, .k = 1.0, .s = 0.0};
  const SpiralPoint sp1 = {
      .x = -2.0, .y = 0.0, .theta = 3.0 * M_PI / 2.0, .k = 1.0, .s = 0.0};
  const auto spirals = CubicSpiralBoundaryValueSolver::solve(sp0, sp1);
  EXPECT_EQ(spirals.size(), 2);
  const auto end = spirals[0].Eval(/*s=*/M_PI, /*num_steps=*/8);
  EXPECT_LT(fabs(end.x - (-2.0)), 5e-2);
  EXPECT_LT(fabs(end.y - 0.0), 1e-2);
  EXPECT_LT(fabs(end.k - 1.0), 1e-3);
  EXPECT_LT(fabs(sin(end.theta - (-M_PI / 2.0))), 1e-3);
}

TEST(CubicSpiralBoundaryValueProblem, ShiftRotateCircle) {
  const SpiralPoint sp0 = {
      .x = 1.0, .y = 1.0, .theta = M_PI / 2.0, .k = 1.0, .s = 0.0};
  const SpiralPoint sp1 = {
      .x = -1.0, .y = 1.0, .theta = 3.0 * M_PI / 2.0, .k = 1.0, .s = 0.0};
  const auto spirals = CubicSpiralBoundaryValueSolver::solve(sp0, sp1);
  EXPECT_EQ(spirals.size(), 2);
  const auto end = spirals[0].Eval(/*s=*/M_PI, /*num_steps=*/8);
  EXPECT_LT(fabs(end.x - (-1.0)), 5e-2);
  EXPECT_LT(fabs(end.y - 1.0), 1e-2);
  EXPECT_LT(fabs(end.k - 1.0), 1e-3);
  EXPECT_LT(fabs(sin(end.theta - (-M_PI / 2.0))), 1e-3);
}

TEST(CubicSpiralBoundaryValueProblem, NotContinuousAngleShiftRotateCircle) {
  const SpiralPoint sp0 = {
      .x = 1.0, .y = 1.0, .theta = M_PI / 2.0, .k = 1.0, .s = 0.0};
  const SpiralPoint sp1 = {
      .x = -1.0, .y = 1.0, .theta = -M_PI / 2.0, .k = 1.0, .s = 0.0};
  const auto spirals = CubicSpiralBoundaryValueSolver::solve(sp0, sp1);
  EXPECT_EQ(spirals.size(), 2);
  const auto end = spirals[0].Eval(/*s=*/M_PI, /*num_steps=*/8);
  EXPECT_LT(fabs(end.x - (-1.0)), 5e-2);
  EXPECT_LT(fabs(end.y - 1.0), 1e-2);
  EXPECT_LT(fabs(end.k - 1.0), 1e-3);
  EXPECT_LT(fabs(sin(end.theta - (-M_PI / 2.0))), 1e-3);
}

TEST(CubicSpiralBoundaryValueProblem, SymNotContinuousAngleShiftRotateCircle) {
  const SpiralPoint sp0 = {
      .x = 1.0, .y = 1.0, .theta = -M_PI / 2.0, .k = -1.0, .s = 0.0};
  const SpiralPoint sp1 = {
      .x = -1.0, .y = 1.0, .theta = M_PI / 2.0, .k = -1.0, .s = 0.0};
  const auto spirals = CubicSpiralBoundaryValueSolver::solve(sp0, sp1);
  EXPECT_EQ(spirals.size(), 2);
  const auto end = spirals[1].Eval(/*s=*/M_PI, /*num_steps=*/8);
  EXPECT_LT(fabs(end.x - (-1.0)), 5e-2);
  EXPECT_LT(fabs(end.y - 1.0), 1e-2);
  EXPECT_LT(fabs(end.k - (-1.0)), 1e-3);
  EXPECT_LT(fabs(sin(end.theta - (M_PI / 2.0))), 1e-3);
}

}  // namespace
}  // namespace qcraft::planner

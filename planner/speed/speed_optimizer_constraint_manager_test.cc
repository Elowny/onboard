#include "onboard/planner/speed/speed_optimizer_constraint_manager.h"

#include <vector>

#include "gtest/gtest.h"

namespace qcraft::planner {
namespace {

using ConstraintMgr = SpeedOptimizerConstraintManager;

const std::vector<double> kTimeRange = {0.0, 10.0};
constexpr double kDeltaT = 0.2;  // s.
constexpr int kKnotNum = 51;
constexpr double kEps = 1e-6;

TEST(ConstraintManagerTest, FilterSoftSConstraintTest) {
  SpeedOptimizerConstraintManager constraint_mgr(kTimeRange, kDeltaT, kKnotNum);
  // Add upper soft s constraint.
  constraint_mgr.AddSoftSUpperConstraint(
      ConstraintMgr::BoundStrType::STRONG,
      {.knot_idx = 0, .weight = 10.0, .bound = 100.0});
  constraint_mgr.AddSoftSUpperConstraint(
      ConstraintMgr::BoundStrType::MEDIUM,
      {.knot_idx = 0, .weight = 8.0, .bound = 110.0});
  constraint_mgr.AddSoftSUpperConstraint(
      ConstraintMgr::BoundStrType::WEAK,
      {.knot_idx = 0, .weight = 7.0, .bound = 120.0});

  // Add lower soft s constraint.
  constraint_mgr.AddSoftSLowerConstraint(
      ConstraintMgr::BoundStrType::STRONG,
      {.knot_idx = 0, .weight = 10.0, .bound = 120.0});
  constraint_mgr.AddSoftSLowerConstraint(
      ConstraintMgr::BoundStrType::MEDIUM,
      {.knot_idx = 0, .weight = 8.0, .bound = 110.0});
  constraint_mgr.AddSoftSLowerConstraint(
      ConstraintMgr::BoundStrType::WEAK,
      {.knot_idx = 0, .weight = 7.0, .bound = 100.0});

  // Filter different types of soft s constraints.
  constraint_mgr.FilterObjectSConstraint(/*knot_idx=*/0);

  const auto upper_constraint = constraint_mgr.GetUpperConstraintOfSoftS();
  EXPECT_EQ(upper_constraint.size(), 1);
  EXPECT_NEAR(upper_constraint.front().bound, 100.0, kEps);

  const auto lower_constraint = constraint_mgr.GetLowerConstraintOfSoftS();
  EXPECT_EQ(lower_constraint.size(), 1);
  EXPECT_NEAR(lower_constraint.front().bound, 120.0, kEps);
}

TEST(ConstraintManagerTest, FilterSoftSpeedConstraintTest) {
  SpeedOptimizerConstraintManager constraint_mgr(kTimeRange, kDeltaT, kKnotNum);
  // Add upper soft speed constraint.
  constraint_mgr.AddSoftSpeedUpperConstraints(
      SpeedLimitLevelProto::STRONG,
      {.knot_idx = 0, .weight = 10.0, .bound = 100.0});
  constraint_mgr.AddSoftSpeedUpperConstraints(
      SpeedLimitLevelProto::MEDIUM,
      {.knot_idx = 0, .weight = 8.0, .bound = 100.0});
  constraint_mgr.AddSoftSpeedUpperConstraints(
      SpeedLimitLevelProto::WEAK,
      {.knot_idx = 0, .weight = 7.0, .bound = 120.0});

  // Add lower soft speed constraint.
  constraint_mgr.AddSoftSpeedLowerConstraints(
      SpeedLimitLevelProto::STRONG,
      {.knot_idx = 0, .weight = 10.0, .bound = 120.0});
  constraint_mgr.AddSoftSpeedLowerConstraints(
      SpeedLimitLevelProto::MEDIUM,
      {.knot_idx = 0, .weight = 8.0, .bound = 110.0});
  constraint_mgr.AddSoftSpeedLowerConstraints(
      SpeedLimitLevelProto::WEAK,
      {.knot_idx = 0, .weight = 7.0, .bound = 100.0});

  // Filter different level of soft speed constraints.
  constraint_mgr.FilterObjectSpeedConstraint(/*knot_idx=*/0);

  const auto upper_constraint = constraint_mgr.GetUpperConstraintOfSpeed();
  EXPECT_EQ(upper_constraint.size(), 1);
  EXPECT_NEAR(upper_constraint.front().bound, 100.0, kEps);
  const auto upper_weight = constraint_mgr.GetUpperSlackWeightOfSpeed();
  EXPECT_EQ(upper_weight.size(), 1);
  EXPECT_NEAR(upper_weight.front().weight, 10.0 * kKnotNum, kEps);

  const auto lower_constraint = constraint_mgr.GetLowerConstraintOfSpeed();
  EXPECT_EQ(lower_constraint.size(), 1);
  EXPECT_NEAR(lower_constraint.front().bound, 120.0, kEps);
  const auto lower_weight = constraint_mgr.GetLowerSlackWeightOfSpeed();
  EXPECT_EQ(lower_weight.size(), 1);
  EXPECT_NEAR(lower_weight.front().weight, 10.0 * kKnotNum, kEps);
}

TEST(ConstraintManagerTest, SlackVarNumTest) {
  SpeedOptimizerConstraintManager constraint_mgr(kTimeRange, kDeltaT, kKnotNum);
  // Add soft s constraints for different knot_idx.
  constraint_mgr.AddSoftSUpperConstraint(
      ConstraintMgr::BoundStrType::STRONG,
      {.knot_idx = 0, .weight = 10.0, .bound = 100.0});
  constraint_mgr.AddSoftSUpperConstraint(
      ConstraintMgr::BoundStrType::WEAK,
      {.knot_idx = 0, .weight = 7.0, .bound = 90.0});

  constraint_mgr.AddSoftSUpperConstraint(
      ConstraintMgr::BoundStrType::STRONG,
      {.knot_idx = 1, .weight = 10.0, .bound = 100.0});
  constraint_mgr.AddSoftSUpperConstraint(
      ConstraintMgr::BoundStrType::MEDIUM,
      {.knot_idx = 1, .weight = 8.0, .bound = 90.0});

  constraint_mgr.AddSoftSUpperConstraint(
      ConstraintMgr::BoundStrType::STRONG,
      {.knot_idx = 2, .weight = 10.0, .bound = 120.0});
  constraint_mgr.AddSoftSUpperConstraint(
      ConstraintMgr::BoundStrType::WEAK,
      {.knot_idx = 2, .weight = 7.0, .bound = 120.0});

  constraint_mgr.AddSoftSLowerConstraint(
      ConstraintMgr::BoundStrType::STRONG,
      {.knot_idx = 0, .weight = 10.0, .bound = 100.0});
  constraint_mgr.AddSoftSLowerConstraint(
      ConstraintMgr::BoundStrType::WEAK,
      {.knot_idx = 0, .weight = 7.0, .bound = 90.0});

  constraint_mgr.AddSoftSLowerConstraint(
      ConstraintMgr::BoundStrType::STRONG,
      {.knot_idx = 1, .weight = 10.0, .bound = 100.0});
  constraint_mgr.AddSoftSLowerConstraint(
      ConstraintMgr::BoundStrType::MEDIUM,
      {.knot_idx = 1, .weight = 8.0, .bound = 90.0});

  constraint_mgr.AddSoftSLowerConstraint(
      ConstraintMgr::BoundStrType::STRONG,
      {.knot_idx = 2, .weight = 10.0, .bound = 120.0});
  constraint_mgr.AddSoftSLowerConstraint(
      ConstraintMgr::BoundStrType::WEAK,
      {.knot_idx = 2, .weight = 7.0, .bound = 120.0});

  EXPECT_EQ(constraint_mgr.GetSlackNumOfUpperS(), 3);
  EXPECT_EQ(constraint_mgr.GetSlackNumOfLowerS(), 3);
}

}  // namespace
}  // namespace qcraft::planner

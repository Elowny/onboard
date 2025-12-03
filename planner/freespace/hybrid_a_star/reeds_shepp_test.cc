#include "onboard/planner/freespace/hybrid_a_star/reeds_shepp.h"

#include <cmath>
#include <ostream>

#include "glog/logging.h"

#include "gtest/gtest.h"

#include "onboard/math/vec.h"
#include "onboard/vis/canvas/canvas.h"
#include "onboard/vis/common/color.h"

#include "offboard/vis/vantage/vantage_server/vantage_canvas_client.h"

namespace qcraft {
namespace planner {
namespace {

constexpr double kEpsilon = 1e-5;
constexpr double kMaxKappa = 0.17;

TEST(ReedsSheppTest, GeneralTest) {
  Node3d start(0.0, 0.0, 0.0);
  Node3d end(11.8, 7.0, -M_PI * 0.5);

  const auto result_status =
      GetShortestReedsShepp(start, end, /*max_kappa=*/0.17);
  ASSERT_TRUE(result_status.ok());
  EXPECT_NEAR(result_status->total_length, 20.00896, kEpsilon);

  const auto samples_path = GetSampledShortestReedsShepp(
      start, end, /*max_kappa=*/0.17, /*resolution=*/0.2);
  ASSERT_TRUE(!samples_path.empty());

  const auto& result = *result_status;
  for (const auto& x : result.segs_lengths) {
    LOG(INFO) << "segs_lengths: " << x;
  }
  for (const auto& x : result.segs_types) {
    LOG(INFO) << "segs_types: " << x;
  }
  LOG(INFO) << "total_length: " << result.total_length;

  vis::Canvas& canvas_path =
      vis::vantage::GetCanvasClient()->GetCanvas("reeds_shepp_test");
  for (int i = 0; i + 1 < samples_path.size(); ++i) {
    const auto color =
        samples_path[i + 1].forward ? vis::Color::kGreen : vis::Color::kRed;
    canvas_path.DrawLine(
        Vec3d(samples_path[i].x, samples_path[i].y, 0.0),
        Vec3d(samples_path[i + 1].x, samples_path[i + 1].y, 0.0), color);
    canvas_path.DrawCircle(Vec3d(samples_path[i].x, samples_path[i].y, 0.0),
                           0.05, color);
  }
  vis::vantage::GetCanvasClient()->FlushAll();
}

TEST(ReedsSheppTest, StraightForwardTest) {
  Node3d start(0.0, 0.0, 0.0);
  Node3d end(10.0, 0.0, 0.0);

  const auto result_status = GetShortestReedsShepp(start, end, kMaxKappa);
  ASSERT_TRUE(result_status.ok());
  EXPECT_NEAR(result_status->total_length, 10.0, kEpsilon);
}

TEST(ReedsSheppTest, TurnQuarterCircleTest) {
  Node3d start(0.0, 0.0, 0.0);
  Node3d end(1.0 / kMaxKappa, 1.0 / kMaxKappa, 0.5 * M_PI);

  const auto result_status = GetShortestReedsShepp(start, end, kMaxKappa);
  ASSERT_TRUE(result_status.ok());
  EXPECT_NEAR(result_status->total_length, 0.5 * M_PI / kMaxKappa, kEpsilon);
}

TEST(ReedsSheppTest, TurnHalfCircleTest) {
  Node3d start(0.0, 0.0, 0.0);
  Node3d end(0.0, 2.0 / kMaxKappa, M_PI);

  const auto result_status = GetShortestReedsShepp(start, end, kMaxKappa);
  ASSERT_TRUE(result_status.ok());
  EXPECT_NEAR(result_status->total_length, M_PI / kMaxKappa, kEpsilon);
}

TEST(ReedsSheppTest, Straight2BackwardTest) {
  Node3d start(0.0, 0.0, 0.0);
  Node3d end(30.0, 0.0, M_PI);

  const auto result_status = GetShortestReedsShepp(start, end, kMaxKappa);
  ASSERT_TRUE(result_status.ok());
}
}  // namespace
}  // namespace planner
}  // namespace qcraft

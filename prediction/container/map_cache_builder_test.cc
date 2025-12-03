#include "onboard/prediction/container/map_cache_builder.h"

#include "absl/container/flat_hash_map.h"

#include "gtest/gtest.h"

#include "onboard/container/strong_int.h"
#include "onboard/maps/semantic_map_defs.h"
#include "onboard/math/piecewise_linear_function.h"
#include "onboard/planner/test_util/load_psmm_util.h"
#include "onboard/utils/map_util.h"

namespace qcraft {
namespace prediction {
namespace {
constexpr double kFrontScanDistance = 40;
constexpr double kBackScanDistance = 40;
constexpr double kSideScanHalfWidth = 20;
constexpr double kDesireBoundarySamplingStep = 4.0;
constexpr double kDpCacheFrontScanDistance = 100;
constexpr double kDpCacheBackScanDistance = 40;
constexpr double kDpCacheSideScanHalfWidth = 40;
constexpr int kDpCacheStepS = 4.0;
constexpr int kDpMaxNum = 16;
TEST(MapCacheBuilder, LaneBoundaryCacheTest) {
  const auto& psmm = planner::CreateDojoTestPSMM();
  const Vec2d ego_pos(0.0, 0.0);
  const double ego_heading = 0.0;
  const auto cache = BuildLaneBoundaryCache(
      psmm, ego_pos, ego_heading, kFrontScanDistance, kBackScanDistance,
      kSideScanHalfWidth, kDesireBoundarySamplingStep);

  EXPECT_GT(cache.size(), 0);
  const auto* plf = FindOrNull(cache, mapping::ElementId(266));
  EXPECT_TRUE(plf != nullptr);
  const auto bound = (*plf)(0.5);
  EXPECT_NEAR(bound.left_bound, 1.62, 4e-2);
  EXPECT_NEAR(bound.right_bound, -1.79, 4e-2);
  const auto* plf2 = FindOrNull(cache, mapping::ElementId(214));
  EXPECT_TRUE(plf2 != nullptr);
  const auto bound2 = (*plf2)(0.5);
  EXPECT_NEAR(bound2.left_bound, 1.77, 4e-2);
  EXPECT_NEAR(bound2.right_bound, -1.65, 4e-2);
}
TEST(MapCacheBuilder, DrivePassageCacheTest) {
  const auto& psmm = planner::CreateDojoTestPSMM();
  const Vec2d ego_pos(0.0, 0.0);
  const double ego_heading = 0.0;
  const planner::LaneBoundaryCache lb_cache;
  auto [passages, cache] = BuildDrivePassageCache(
      psmm, lb_cache, ego_pos, ego_heading, kDpCacheFrontScanDistance,
      kDpCacheBackScanDistance, kDpCacheSideScanHalfWidth, kDpCacheStepS,
      kDpMaxNum, /*filter_virtual=*/false);
  EXPECT_GT(cache.size(), 0);
  EXPECT_EQ(passages.size(), kDpMaxNum);
  const auto* dps = FindOrNull(cache, mapping::ElementId(2448));
  EXPECT_TRUE(dps != nullptr);
  EXPECT_EQ(dps->size(), 2);
}

}  // namespace
}  // namespace prediction
}  // namespace qcraft

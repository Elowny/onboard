#include "onboard/prediction/feature_extractor/map_sampler.h"

// IWYU pragma: no_include <ext/alloc_traits.h>
//              for __alloc_traits<>::value_type
#include <cmath>
#include <memory>

#include "gtest/gtest.h"

#include "onboard/maps/maps_common.h"
#include "onboard/planner/test_util/load_psmm_util.h"

namespace qcraft {
namespace prediction {
namespace {
constexpr double kSampleInterval = 2.5;
constexpr double kEpsilon = 0.1;
TEST(MapSamplerTest, PointTypeCastTest) {
  EXPECT_EQ(static_cast<int>(SampledPointType::kNone), 0);
  EXPECT_EQ(static_cast<int>(SampledPointType::kBrokenWhite), 16);
  EXPECT_EQ(static_cast<int>(SampledPointType::kBrokenYellow), 17);
}

TEST(MapSamplerTest, LaneSampleTest) {
  const auto& psmm = planner::CreateDojoTestPSMM();
  TrafficLightManager::TLStateHashMap tl_state;
  const int desire_seg_num = 5;
  MapSampler map_sampler(psmm, tl_state, kSampleInterval, desire_seg_num,
                         MapSampler::SampleType::EQUAL_DIST);
  const Vec2d center(-133.109, -27.871);
  Box2d region(1.0, 1.0, center, 0.0);
  auto lcs = map_sampler.GetNearestLaneCenterPolylinesInBox2d(region, 1);
  EXPECT_EQ(lcs.size(), 1);
  const auto* poly = lcs[0];
  EXPECT_EQ(poly->id.value(), 679);
  const auto& seg0 = poly->segs[0];
  const auto& pt0 = poly->segs[0].seg.start();
  const auto& pt1 = poly->segs[1].seg.start();
  EXPECT_NEAR(pt0.y(), -52.484, kEpsilon);
  EXPECT_NEAR(pt0.DistanceTo(pt1), kSampleInterval, kEpsilon);
  EXPECT_EQ(poly->segs.size(), 20);
  EXPECT_EQ(seg0.type, SampledPointType::kCenterlineGeneral);
  EXPECT_EQ(seg0.speed_limit_kph, 40);
  for (const auto& light : seg0.lights) {
    EXPECT_EQ(light, 0);
  }
}

TEST(MapSamplerTest, AdaptiveLaneSampleTest) {
  const auto& psmm = planner::CreateDojoTestPSMM();

  TrafficLightManager::TLStateHashMap tl_state;
  const int desire_seg_num = 5;
  MapSampler map_sampler(psmm, tl_state, kSampleInterval, desire_seg_num,
                         MapSampler::SampleType::ADAPTIVE);
  const Vec2d center(-133.109, -27.871);
  Box2d region(1.0, 1.0, center, 0.0);
  auto lcs = map_sampler.GetNearestLaneCenterPolylinesInBox2d(region, 1);
  EXPECT_EQ(lcs.size(), 1);
  const auto* poly = lcs[0];
  const auto* lane_info = psmm.FindLaneInfoOrNull(poly->id);
  EXPECT_EQ(poly->id.value(), 679);
  EXPECT_EQ(poly->segs.size(),
            static_cast<int>(desire_seg_num *
                             std::ceil(lane_info->length() /
                                       (desire_seg_num * kSampleInterval))));
}

TEST(MapSamplerTest, JunctionLaneSampleTest) {
  const auto& psmm = planner::CreateDojoTestPSMM();

  TrafficLightManager::TLStateHashMap tl_state;
  const int desire_seg_num = 5;
  MapSampler map_sampler(psmm, tl_state, kSampleInterval, desire_seg_num,
                         MapSampler::SampleType::EQUAL_DIST);
  const Vec2d center(97.328, 10.371);
  Box2d region(1.0, 1.0, center, 0.0);
  auto lcs = map_sampler.GetNearestLaneCenterPolylinesInBox2d(region, 1);
  EXPECT_EQ(lcs.size(), 1);
  const auto* poly = lcs[0];
  const auto& seg0 = poly->segs[0];
  EXPECT_EQ(seg0.lights[3], 1);
}

TEST(MapSamplerTest, CurbLaneSampleTest) {
  const auto& psmm = planner::CreateDojoTestPSMM();

  TrafficLightManager::TLStateHashMap tl_state;
  const int desire_seg_num = 5;
  MapSampler map_sampler(psmm, tl_state, kSampleInterval, desire_seg_num,
                         MapSampler::SampleType::EQUAL_DIST);
  const Vec2d center(265.150, -9.086);
  Box2d region(2, 2, center, 0.0);
  auto lbs = map_sampler.GetNearestSolidLaneBoundaryPolylinesInBox2d(region, 1);
  EXPECT_EQ(lbs.size(), 1);
  const auto* poly = lbs[0];
  EXPECT_EQ(poly->id.value(), 392);
  const auto& seg0 = poly->segs[0];
  EXPECT_EQ(seg0.type, SampledPointType::kCurb);
  EXPECT_EQ(seg0.speed_limit_kph, 0);
}

TEST(MapSamplerTest, BrokenLaneSampleTest) {
  const auto& psmm = planner::CreateDojoTestPSMM();

  TrafficLightManager::TLStateHashMap tl_state;
  const int desire_seg_num = 5;
  MapSampler map_sampler(psmm, tl_state, kSampleInterval, desire_seg_num,
                         MapSampler::SampleType::EQUAL_DIST);
  const Vec2d center(0.0, 1.75);
  Box2d region(2, 2, center, 0.0);
  auto lbs =
      map_sampler.GetNearestBrokenLaneBoundaryPolylinesInBox2d(region, 1);
  EXPECT_EQ(lbs.size(), 1);
  const auto* poly = lbs[0];
  EXPECT_EQ(poly->id.value(), 319);
  const auto& seg0 = poly->segs[0];
  EXPECT_EQ(seg0.speed_limit_kph, 0);
}

TEST(MapSamplerTest, CrosswalkSampleTest) {
  const auto& psmm = planner::CreateDojoTestPSMM();

  TrafficLightManager::TLStateHashMap tl_state;
  const int desire_seg_num = 5;
  MapSampler map_sampler(psmm, tl_state, kSampleInterval, desire_seg_num,
                         MapSampler::SampleType::EQUAL_DIST);
  const Vec2d center(290.462, 47.688);
  Box2d region(10, 10, center, 0.0);
  auto lbs = map_sampler.GetNearestCrosswalkPolylinesInBox2d(region, 1);
  EXPECT_EQ(lbs.size(), 1);
  const auto* poly = lbs[0];
  EXPECT_EQ(poly->id.value(), 535);
  const auto& seg0 = poly->segs[0];
  EXPECT_EQ(seg0.type, SampledPointType::kCrosswalk);
  EXPECT_EQ(seg0.speed_limit_kph, 0);
}

}  // namespace
}  // namespace prediction
}  // namespace qcraft

#include "onboard/planner/planner_semantic_map_manager.h"

#include <algorithm>
#include <memory>
#include <tuple>
#include <vector>

#include "google/protobuf/repeated_ptr_field.h"

#include "gmock/gmock.h"
#include "gtest/gtest.h"

#include "onboard/async/future.h"
#include "onboard/container/strong_int.h"
#include "onboard/maps/lane_path.h"
#include "onboard/maps/map_selector.h"
#include "onboard/maps/semantic_map_manager.h"
#include "onboard/maps/v2/semantic_map_loader.h"
#include "onboard/math/coordinate_converter.h"

namespace qcraft::planner {
namespace {

class PlannerSemanticMapManagerTest : public testing::Test {
 public:
  void SetUp() override {
    SetMap("dojo");
    smm_.LoadWholeMap().Build();

    auto loader = mapping::v2::SemanticMapLoader::MakeShared();
    smm_v2_ = loader->PreloadWholeMap().Get();
    smmsi_ =
        mapping::v2::SemanticMapMultilevelSpatialIndex::MakeShared(smm_v2_);
    psmm_ = std::make_shared<PlannerSemanticMapManager>(
        PlannerSemanticMapManager(smmsi_));
    psmm_->UpdateCoordinateConverter(cc_);
    std::ignore = psmm_->BuildSemanticMapInfo(/*thread_pool=*/nullptr);
  }

  SemanticMapManager smm_;
  std::shared_ptr<mapping::v2::SemanticMapManager> smm_v2_;
  std::shared_ptr<mapping::v2::SemanticMapMultilevelSpatialIndex> smmsi_;
  std::shared_ptr<PlannerSemanticMapManager> psmm_;
  CoordinateConverter cc_ = CoordinateConverter::FromMap("dojo");
};

TEST_F(PlannerSemanticMapManagerTest, GetElementsTest) {
  // Lane proto.
  {
    const auto* lane_proto_ptr =
        psmm_->FindLaneByIdOrNull(mapping::ElementId(2448));
    EXPECT_TRUE(lane_proto_ptr != nullptr);
    EXPECT_EQ(lane_proto_ptr->id(), 2448);
  }
  // Lane info.
  {
    const auto* lane_info_ptr =
        psmm_->FindLaneInfoOrNull(mapping::ElementId(2448));
    EXPECT_TRUE(lane_info_ptr != nullptr);
    EXPECT_EQ(lane_info_ptr->id, mapping::ElementId(2448));
  }
  // TODO(zuowei): Add more UNITTEST.
}

TEST_F(PlannerSemanticMapManagerTest, GetLanesInfoAtLevelTest) {
  const auto lanes =
      psmm_->GetLanesInfoAtLevel(mapping::LevelId(0), Vec2d(0.0, 0.0), 10.0);

  const auto smm_res =
      smm_.GetLanesInfoAtLevel(mapping::LevelId(0), Vec2d(0.0, 0.0), 10.0);

  EXPECT_EQ(lanes.size(), smm_res.size());
  std::vector<mapping::ElementId> expect_res;
  expect_res.reserve(smm_res.size());
  for (const auto* lane_info : smm_res) {
    expect_res.push_back(lane_info->id);
  }
  std::sort(expect_res.begin(), expect_res.end());

  std::vector<mapping::ElementId> actual_res;
  actual_res.reserve(lanes.size());
  for (const auto* lane_info : lanes) {
    actual_res.push_back(lane_info->id);
  }
  std::sort(actual_res.begin(), actual_res.end());

  EXPECT_THAT(actual_res, testing::ElementsAreArray(expect_res));
}

TEST_F(PlannerSemanticMapManagerTest,
       GetNearestLaneInfoWithHeadingAtLevelTest) {
  const auto* lane_info = psmm_->GetNearestLaneInfoWithHeadingAtLevel(
      mapping::LevelId(0), Vec2d(-15.773, 1.481), 0.11, 10.0, 0.2);
  EXPECT_TRUE(lane_info != nullptr);

  const auto* smm_res = smm_.GetNearestLaneInfoWithHeadingAtLevel(
      mapping::LevelId(0), Vec2d(-15.773, 1.481), 0.11, 10.0, 0.2);
  EXPECT_TRUE(smm_res != nullptr);
  EXPECT_EQ(lane_info->id, smm_res->id);
}

TEST_F(PlannerSemanticMapManagerTest, GetLanesInfoWithHeadingAtLevelTest) {
  const auto lanes = psmm_->GetLanesInfoWithHeadingAtLevel(
      mapping::LevelId(0), Vec2d(-15.773, 1.481), 0.11, 10.0, 0.2);

  const auto smm_res = smm_.GetLanesInfoWithHeadingAtLevel(
      mapping::LevelId(0), Vec2d(-15.773, 1.481), 0.11, 10.0, 0.2);

  EXPECT_EQ(lanes.size(), smm_res.size());
  std::vector<mapping::ElementId> expect_res;
  expect_res.reserve(smm_res.size());
  for (const auto* lane_info : smm_res) {
    expect_res.push_back(lane_info->id);
  }

  std::vector<mapping::ElementId> actual_res;
  actual_res.reserve(lanes.size());
  for (const auto* lane_info : lanes) {
    actual_res.push_back(lane_info->id);
  }

  EXPECT_THAT(actual_res, testing::ElementsAreArray(expect_res));
}

TEST_F(PlannerSemanticMapManagerTest, GetLaneProjectionTest) {
  double psmm_fraction, psmm_min_dist;
  Vec2d psmm_point;
  Segment2d psmm_seg;
  const bool psmm_res = psmm_->GetLaneProjection(
      Vec2d(17.552, -1.022), mapping::ElementId(2448), &psmm_fraction,
      &psmm_point, &psmm_min_dist, &psmm_seg);
  EXPECT_TRUE(psmm_res);

  double smm_fraction, smm_min_dist;
  Vec2d smm_point;
  Segment2d smm_seg;
  const bool smm_res = smm_.GetLaneProjectionAtLevel(
      mapping::LevelId(0), Vec2d(17.552, -1.022), mapping::ElementId(2448),
      &smm_fraction, &smm_point, &smm_min_dist, &smm_seg);
  EXPECT_TRUE(smm_res);

  EXPECT_NEAR(psmm_fraction, smm_fraction, 1e-3);
  EXPECT_NEAR(psmm_fraction, smm_fraction, 1e-3);
  EXPECT_NEAR(psmm_point.DistanceTo(smm_point), 0.0, 1e-3);
  EXPECT_NEAR(psmm_seg.min_x(), smm_seg.min_x(), 1e-3);
  EXPECT_NEAR(psmm_seg.min_y(), smm_seg.min_y(), 1e-3);
  EXPECT_NEAR(psmm_seg.max_x(), smm_seg.max_x(), 1e-3);
  EXPECT_NEAR(psmm_seg.max_y(), smm_seg.max_y(), 1e-3);
}

TEST_F(PlannerSemanticMapManagerTest, GetNearestLaneInfoAtLevelTest) {
  const auto* expect_res = smm_.GetNearestLaneInfoAtLevel(
      mapping::LevelId(0), Vec2d(-15.752, 3.368));
  EXPECT_TRUE(expect_res != nullptr);

  const auto* res = psmm_->GetNearestLaneInfoAtLevel(mapping::LevelId(0),
                                                     Vec2d(-15.752, 3.368));
  EXPECT_TRUE(res != nullptr);

  EXPECT_EQ(expect_res->id, res->id);
}

TEST_F(PlannerSemanticMapManagerTest,
       GetNearestNamedImpassableBoundaryAtLevelTest) {
  Segment2d expect_seg;
  std::string expect_id;

  const bool expect_res = smm_.GetNearestNamedImpassableBoundaryAtLevel(
      mapping::LevelId(0), Vec2d(31.647, 0.941), &expect_seg, &expect_id);
  EXPECT_TRUE(expect_res);

  Segment2d seg;
  std::string id;
  const bool res = psmm_->GetNearestNamedImpassableBoundaryAtLevel(
      mapping::LevelId(0), Vec2d(31.647, 0.941), &seg, &id);
  EXPECT_TRUE(res);

  EXPECT_NEAR(expect_seg.start().DistanceTo(seg.start()), 0.0, 1e-2);
  EXPECT_NEAR(expect_seg.end().DistanceTo(seg.end()), 0.0, 1e-2);
  EXPECT_EQ(expect_id, id);
}

TEST_F(PlannerSemanticMapManagerTest, GetNearestIntersectionInfoAtLevelTest) {
  const auto* expect_res = smm_.GetNearestIntersectionInfoAtLevel(
      mapping::LevelId(0), Vec2d(62.919, 0.545));
  EXPECT_TRUE(expect_res != nullptr);

  const auto* res = psmm_->GetNearestIntersectionInfoAtLevel(
      mapping::LevelId(0), Vec2d(62.919, 0.545));
  EXPECT_TRUE(res != nullptr);

  EXPECT_EQ(expect_res->id, res->id);
}

TEST_F(PlannerSemanticMapManagerTest, GetNearestBusStationStopAreaAtLevelTest) {
  const auto* query_stop_area = psmm_->GetNearestBusStationStopAreaAtLevel(
      mapping::LevelId(0), Vec2d(1034.796, -467.924));

  EXPECT_TRUE(query_stop_area != nullptr);
  EXPECT_EQ(query_stop_area->id(), 12938);

  EXPECT_FALSE(query_stop_area->lane_paths().empty());
  const mapping::LanePath lane_path(&smm_, query_stop_area->lane_paths(0));
  const auto& lane_ids = lane_path.lane_ids();
  EXPECT_EQ(lane_ids.size(), 2);
  EXPECT_EQ(lane_ids[0], mapping::ElementId(12888));
  EXPECT_EQ(lane_ids[1], mapping::ElementId(12889));
}

TEST_F(PlannerSemanticMapManagerTest, QueryLaneSpeedLimitByFractionTest) {
  const double res = psmm_->QueryLaneSpeedLimitByFraction(
      mapping::ElementId(2448), /*fraction=*/0.3);
  EXPECT_NEAR(res, 13.33, 0.01);

  const double res1 =
      psmm_->QueryAverageLaneSpeedLimitById(mapping::ElementId(2448));
  EXPECT_NEAR(res1, 13.33, 0.01);

  const double res2 =
      psmm_->QueryMaxLaneSpeedLimitById(mapping::ElementId(2448));
  EXPECT_NEAR(res2, 13.33, 0.01);

  const double res3 =
      psmm_->QueryMinLaneSpeedLimitById(mapping::ElementId(2448));
  EXPECT_NEAR(res3, 13.33, 0.01);
}

TEST_F(PlannerSemanticMapManagerTest, DataSource) {
  EXPECT_EQ(psmm_->GetDataSource(), OnlineMapProto::QCRAFT_HDMAP);
  EXPECT_FALSE(psmm_->IsOnVisionMap());
  EXPECT_FALSE(psmm_->IsThirdPartyMap());
}
}  // namespace
}  // namespace qcraft::planner

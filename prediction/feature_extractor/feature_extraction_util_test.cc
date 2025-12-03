#include "onboard/prediction/feature_extractor/feature_extraction_util.h"

// IWYU pragma: no_include <ext/alloc_traits.h>
//              for __alloc_traits<>::value_type
#include <cmath>
#include <ostream>
#include <string>

#include "glog/logging.h"

#include "gtest/gtest.h"

#include "onboard/math/geometry/proto/affine_transformation.pb.h"
#include "onboard/planner/test_util/load_psmm_util.h"
#include "onboard/planner/test_util/util.h"
#include "onboard/prediction/prediction_util.h"
#include "onboard/proto/trajectory_point.pb.h"

namespace qcraft {
namespace prediction {
namespace {
constexpr double kEps = 1e-5;
TEST(FeatureExtractionUtilTest, TrajectoryProtoToPredictedPointTest) {
  ApolloTrajectoryPointProto pt;
  pt.mutable_path_point()->set_x(1.0);
  pt.mutable_path_point()->set_y(2.0);
  pt.mutable_path_point()->set_theta(3.0);
  pt.mutable_path_point()->set_s(4.0);
  pt.set_v(6.0);
  pt.set_relative_time(0.1);
  const auto vehicle_geom = planner::DefaultVehicleGeometry();
  TrajectoryProto traj;
  *traj.add_trajectory_point() = pt;
  const auto pred_pts =
      ConvertTrajectoryProtoToPredictedTrajectoryPoints(traj, vehicle_geom);
  const auto& pred_pt = pred_pts[0];
  EXPECT_NEAR(pt.relative_time(), pred_pt.t(), kEps);
  EXPECT_NEAR(pt.path_point().x() + 1.5 * std::cos(pt.path_point().theta()),
              pred_pt.pos().x(), kEps);
  EXPECT_NEAR(pt.path_point().y() + 1.5 * std::sin(pt.path_point().theta()),
              pred_pt.pos().y(), kEps);
  EXPECT_NEAR(pt.path_point().theta(), pred_pt.theta(), kEps);
  EXPECT_NEAR(pt.path_point().s(), pred_pt.s(), kEps);
  EXPECT_NEAR(pt.v(), pred_pt.v(), kEps);
}
TEST(FeatureExtractionUtilTest, AlighOnePointTest) {
  PredictedTrajectoryPointProto pt;
  pt.mutable_pos()->set_x(0.0);
  pt.mutable_pos()->set_y(0.0);
  pt.set_theta(0.0);
  pt.set_v(0.0);
  pt.set_t(0.0);
  const auto res = AlignPredictedTrajectoryPoints(
      {pt}, /*prev_time*/ 0.0,
      /*cur_time*/ 0.1, /*new_horizon=*/1.0, /*dt=*/1.0);
  EXPECT_EQ(res.size(), 1);
}

TEST(FeatureExtractionUtilTest, AlignTwoPointTest) {
  PredictedTrajectoryPointProto pt;
  pt.mutable_pos()->set_x(0.0);
  pt.mutable_pos()->set_y(0.0);
  pt.set_theta(0.0);
  pt.set_v(0.0);
  pt.set_t(0.0);

  PredictedTrajectoryPointProto pt1;
  pt1.mutable_pos()->set_x(1.0);
  pt1.mutable_pos()->set_y(1.0);
  pt1.set_theta(1.0);
  pt1.set_v(1.0);
  pt1.set_t(1.0);
  const auto res = AlignPredictedTrajectoryPoints(
      {pt, pt1}, /*prev_time*/ 0.0,
      /*cur_time*/ 0.5, /*new_horizon=*/1.0, /*dt*/ 1.0);
  LOG(INFO) << res[0].DebugString() << " " << res[1].DebugString();
  EXPECT_EQ(res.size(), 2);
  EXPECT_NEAR(res[0].t(), 0.0, kEps);
  EXPECT_NEAR(res[0].v(), 0.5, kEps);
  EXPECT_NEAR(res[0].theta(), 0.5, kEps);
  EXPECT_NEAR(res[0].pos().x(), 0.5, kEps);
  EXPECT_NEAR(res[0].pos().y(), 0.5, kEps);
  EXPECT_NEAR(res[0].s(), 0.0, kEps);

  EXPECT_NEAR(res[1].t(), 1.0, kEps);
  EXPECT_NEAR(res[1].v(), 1.5, kEps);
  EXPECT_NEAR(res[1].theta(), 1.5, kEps);
  EXPECT_NEAR(res[1].pos().x(), 1.5, kEps);
  EXPECT_NEAR(res[1].pos().y(), 1.5, kEps);
  EXPECT_NEAR(res[1].s(), std::sqrt(2), kEps);
}

TEST(FeatureExtractionUtilTest, GetRegionBox) {
  const auto box = GetRegionBox(Vec2d::Zero(), 0.0, 112.0, 67.2, 51.2);
  EXPECT_NEAR(box.length(), 179.2, kEps);
  EXPECT_NEAR(box.width(), 102.4, kEps);
}

TEST(FeatureExtractionUtilTest, OneHotObjectTypeNormalValue) {
  const auto one_hot_encoding = OneHotObjectType(1.0, 2);
  EXPECT_EQ(one_hot_encoding.size(), 2);
  EXPECT_EQ(one_hot_encoding[1], 1.0);
}

TEST(FeatureExtractionUtilTest, OneHotObjectTypeOutUpperBound) {
  const auto one_hot_encoding = OneHotObjectType(3.0, 3);
  EXPECT_EQ(one_hot_encoding.size(), 3);
  EXPECT_EQ(one_hot_encoding[2], 1.0);
}

TEST(FeatureExtractionUtilTest, OneHotObjectTypeOutLowerBound) {
  const auto one_hot_encoding = OneHotObjectType(-1.0, 2);
  EXPECT_EQ(one_hot_encoding.size(), 2);
  EXPECT_EQ(one_hot_encoding[0], 1.0);
}

TEST(FeatureExtractionUtilTest, GetExitsOfIntersection) {
  const auto& psmm = planner::CreateDojoTestPSMM();
  const auto intersection_ptr = psmm.GetNearestIntersectionInfoAtLevel(
      mapping::LevelId(0), Vec2d::Zero());
  EXPECT_TRUE(intersection_ptr != nullptr);
  const auto exits = GetExitsOfIntersection(psmm, *intersection_ptr, 100.0);
  EXPECT_FALSE(exits.empty());
}

}  // namespace
}  // namespace prediction
}  // namespace qcraft

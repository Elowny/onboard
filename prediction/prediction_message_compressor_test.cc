#include "onboard/prediction/prediction_message_compressor.h"

#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>

#include "gtest/gtest.h"

#include "onboard/math/geometry/proto/affine_transformation.pb.h"
#include "onboard/math/vec.h"
#include "onboard/planner/test_util/predicted_trajectory_builder.h"
#include "onboard/prediction/predicted_trajectory.h"
#include "onboard/prediction/prediction_defs.h"

namespace qcraft {
namespace prediction {
namespace {
TEST(PredictionMessageCompressor, Compress) {
  constexpr int kNumPoints = 2;
  std::vector<PredictedTrajectoryPoint> points;
  points.reserve(kNumPoints);
  for (int idx = 0; idx < kNumPoints; ++idx) {
    prediction::PredictedTrajectoryPoint pt;

    pt.set_pos(Vec2d(0.0, 1.0 * idx));
    pt.set_s(1.0 * idx);
    pt.set_theta(M_PI * idx / 3.0);
    pt.set_kappa(0.0);

    pt.set_t(idx * prediction::kPredictionTimeStep);
    pt.set_v(10.0);
    pt.set_a(0.0);
    points.push_back(std::move(pt));
  }
  planner::PredictedTrajectoryBuilder builder;
  builder.set_probability(1.0).set_index(0);
  builder.set_points(points);
  auto traj = builder.Build();
  PredictedTrajectoryProto traj_proto;
  traj.ToProto(&traj_proto, /*compress_traj=*/false);
  CompressPredictedTrajectoryProtoByDownSampling(&traj_proto);
  EXPECT_EQ(traj_proto.compressed_points().x_size(), kNumPoints);
  EXPECT_NEAR(traj_proto.compressed_points().x(1), 0.0, 1e-4);
  EXPECT_NEAR(traj_proto.compressed_points().y(1), 1.0, 1e-4);
  EXPECT_NEAR(traj_proto.compressed_points().t(1), 0.1, 1e-4);
}

TEST(PredictionMessageCompressor, CompressAndDecompress) {
  constexpr int kNumPoints = 3;
  std::vector<PredictedTrajectoryPoint> points;
  points.reserve(kNumPoints);
  for (int idx = 0; idx < kNumPoints; ++idx) {
    prediction::PredictedTrajectoryPoint pt;

    pt.set_pos(Vec2d(0.0, 1.0 * idx));
    pt.set_s(1.0 * idx);
    pt.set_theta(M_PI * idx / 3.0);
    pt.set_kappa(0.0);

    pt.set_t(idx * prediction::kPredictionTimeStep);
    pt.set_v(10.0);
    pt.set_a(0.0);
    points.push_back(std::move(pt));
  }
  planner::PredictedTrajectoryBuilder builder;
  builder.set_probability(1.0).set_index(0);
  builder.set_points(points);
  auto traj = builder.Build();
  PredictedTrajectoryProto traj_proto;
  traj.ToProto(&traj_proto, /*compress_traj=*/false);
  CompressPredictedTrajectoryProtoByDownSampling(&traj_proto);
  EXPECT_EQ(traj_proto.compressed_points().x_size(), 2);
  EXPECT_NEAR(traj_proto.compressed_points().x(1), 0.0, 1e-4);
  EXPECT_NEAR(traj_proto.compressed_points().y(1), 2.0, 1e-4);
  EXPECT_NEAR(traj_proto.compressed_points().t(1), 0.2, 1e-4);
  DecompressPredictedTrajectoryProto(&traj_proto);
  EXPECT_EQ(traj_proto.points_size(), 3);
  EXPECT_EQ(traj_proto.compressed_points().x_size(), 0);
  EXPECT_NEAR(traj_proto.points(2).pos().y(), 2.0, 1e-4);
}
TEST(PredictionMessageCompressor, Stationary) {
  constexpr int kNumPoints = 1;
  std::vector<PredictedTrajectoryPoint> points;
  points.reserve(kNumPoints);
  for (int idx = 0; idx < kNumPoints; ++idx) {
    prediction::PredictedTrajectoryPoint pt;

    pt.set_pos(Vec2d(0.0, 1.0 * idx));
    pt.set_s(1.0 * idx);
    pt.set_theta(M_PI * idx / 3.0);
    pt.set_kappa(0.0);

    pt.set_t(idx * prediction::kPredictionTimeStep);
    pt.set_v(0.0);
    pt.set_a(0.0);
    points.push_back(std::move(pt));
  }
  planner::PredictedTrajectoryBuilder builder;
  builder.set_probability(1.0).set_index(0).set_type(PT_STATIONARY);
  builder.set_points(points);
  auto traj = builder.Build();
  PredictedTrajectoryProto traj_proto;
  traj.ToProto(&traj_proto, /*compress_traj=*/false);
  CompressPredictedTrajectoryProtoByDownSampling(&traj_proto);
  EXPECT_EQ(traj_proto.compressed_points().x_size(), 0);
  EXPECT_EQ(traj_proto.points_size(), kNumPoints);
}
TEST(PredictionMessageCompressor, OnePoint) {
  constexpr int kNumPoints = 1;
  std::vector<PredictedTrajectoryPoint> points;
  points.reserve(kNumPoints);
  for (int idx = 0; idx < kNumPoints; ++idx) {
    prediction::PredictedTrajectoryPoint pt;

    pt.set_pos(Vec2d(0.0, 1.0 * idx));
    pt.set_s(1.0 * idx);
    pt.set_theta(M_PI * idx / 3.0);
    pt.set_kappa(0.0);

    pt.set_t(idx * prediction::kPredictionTimeStep);
    pt.set_v(10.0);
    pt.set_a(0.0);
    points.push_back(std::move(pt));
  }
  planner::PredictedTrajectoryBuilder builder;
  builder.set_probability(1.0).set_index(0);
  builder.set_points(points);
  auto traj = builder.Build();
  PredictedTrajectoryProto traj_proto;
  traj.ToProto(&traj_proto, /*compress_traj=*/false);
  CompressPredictedTrajectoryProtoByDownSampling(&traj_proto);
  DecompressPredictedTrajectoryProto(&traj_proto);
  EXPECT_EQ(traj_proto.points_size(), 1);
  EXPECT_EQ(traj_proto.compressed_points().x_size(), 0);
  EXPECT_NEAR(traj_proto.points(0).v(), 10.0, 1e-4);
}
TEST(PredictionMessageCompressor, Empty) {
  planner::PredictedTrajectoryBuilder builder;
  builder.set_probability(1.0).set_index(0);
  auto traj = builder.Build();
  PredictedTrajectoryProto traj_proto;
  traj.ToProto(&traj_proto, /*compress_traj=*/false);
  CompressPredictedTrajectoryProtoByDownSampling(&traj_proto);
  EXPECT_FALSE(traj_proto.has_compressed_points());
  DecompressPredictedTrajectoryProto(&traj_proto);
  EXPECT_EQ(traj_proto.compressed_points().x_size(), 0);
  EXPECT_EQ(traj_proto.points_size(), 0);
}
}  // namespace
}  // namespace prediction
}  // namespace qcraft

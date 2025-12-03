#include "onboard/planner/util/online_map_converter.h"

#include <memory>

#include "gtest/gtest.h"

#include "onboard/global/buffered_logger.h"
#include "onboard/global/clock.h"
#include "onboard/math/geometry/proto/affine_transformation.pb.h"
#include "onboard/planner/test_util/load_psmm_util.h"
#include "onboard/proto/positioning.pb.h"
#include "onboard/utils/status_macros.h"
#include "onboard/utils/time_util.h"

namespace qcraft::planner {
namespace {
TEST(OnlineMapConverterTest, RunOnlineSemanticMapConverterTest) {
  const auto& psmm = CreateDojoTestPSMM();

  PoseProto pose_proto;
  pose_proto.mutable_pos_smooth()->set_x(10.0);
  pose_proto.mutable_pos_smooth()->set_y(0.0);
  pose_proto.set_yaw(0.0);

  ASSIGN_OR_DIE(const auto online_map_proto,
                RunOnlineSemanticMapConverter(
                    psmm, OnlineSemanticMapConverterOption{
                              .timestamp_s = ToUnixDoubleSeconds(Clock::Now()),
                              .smooth_x = pose_proto.pos_smooth().x(),
                              .smooth_y = pose_proto.pos_smooth().y(),
                              .smooth_yaw = pose_proto.yaw(),
                              .look_ahead_distance = 30.0,
                              .look_back_distance = 20.0,
                              .lane_sample_interval = 1.0,
                              .boundary_sample_interval = 1.0}));

  EXPECT_EQ(online_map_proto.lanes().size(), 9);
  EXPECT_EQ(online_map_proto.boundaries().size(), 2);
  EXPECT_EQ(online_map_proto.lane_ids_at_ego_pos().size(),
            online_map_proto.point_index_of_lane_at_ego_pos().size());
  EXPECT_EQ(online_map_proto.lane_ids_at_ego_pos(0), 2);
  EXPECT_EQ(online_map_proto.lane_ids_at_ego_pos(1), 2448);
  EXPECT_EQ(online_map_proto.lane_ids_at_ego_pos(2), 3);
}

TEST(OnlineMapConverterTest, RunOnlineSemanticMapPredictionConverterTest) {
  const auto& psmm = CreateDojoTestPSMM();

  PoseProto pose_proto;
  pose_proto.mutable_pos_smooth()->set_x(10.0);
  pose_proto.mutable_pos_smooth()->set_y(0.0);
  pose_proto.set_yaw(0.0);

  ASSIGN_OR_DIE(const auto online_map_proto,
                RunOnlineSemanticMapPredictionConverter(
                    psmm, OnlineSemanticMapConverterOption{
                              .timestamp_s = ToUnixDoubleSeconds(Clock::Now()),
                              .smooth_x = pose_proto.pos_smooth().x(),
                              .smooth_y = pose_proto.pos_smooth().y(),
                              .smooth_yaw = pose_proto.yaw(),
                              .look_ahead_distance = 30.0,
                              .look_back_distance = 20.0,
                              .lane_sample_interval = 1.0,
                              .boundary_sample_interval = 1.0}));

  EXPECT_EQ(online_map_proto.lanes().size(), 9);
  EXPECT_EQ(online_map_proto.boundaries().size(), 2);
  EXPECT_EQ(online_map_proto.lane_ids_at_ego_pos().size(),
            online_map_proto.point_index_of_lane_at_ego_pos().size());
}
}  // namespace
}  // namespace qcraft::planner

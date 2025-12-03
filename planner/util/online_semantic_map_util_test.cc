#include "onboard/planner/util/online_semantic_map_util.h"

#include <memory>
#include <ostream>

#include "absl/container/flat_hash_set.h"
#include "absl/time/clock.h"
#include "absl/time/time.h"
#include "glog/logging.h"

#include "gtest/gtest.h"

#include "onboard/container/strong_int.h"
#include "onboard/global/buffered_logger.h"
#include "onboard/maps/lane_path.h"
#include "onboard/maps/lane_path_data.h"
#include "onboard/planner/test_util/load_psmm_util.h"
#include "onboard/planner/util/lane_path_util.h"
#include "onboard/planner/util/online_map_converter.h"
#include "onboard/planner/util/planner_semantic_map_manager_builder.h"
#include "onboard/planner/util/planner_semantic_map_util.h"
#include "onboard/utils/status_macros.h"

namespace qcraft {
namespace planner {
namespace {

TEST(FindClosestLanePathPointsTest, BasicTest) {
  const Vec2d ego_pos(0.0, 0.0);
  // Construct online map and psmm.
  const auto& whole_psmm = CreateDojoTestPSMM();
  ASSIGN_OR_DIE(const auto online_smm_proto,
                RunOnlineSemanticMapConverter(whole_psmm,
                                              OnlineSemanticMapConverterOption{
                                                  .smooth_x = ego_pos.x(),
                                                  .smooth_y = ego_pos.y(),
                                                  .smooth_yaw = 0.0,
                                                  .look_ahead_distance = 90.0,
                                                  .look_back_distance = 10.0}));

  ASSIGN_OR_DIE(const auto psmm_ptr, BuildOnlineMapPsmm(online_smm_proto));
  const auto& psmm = *psmm_ptr;

  constexpr double kEpsilon = 0.1;  // m.
  {
    ASSIGN_OR_DIE(const auto origin_lane_path,
                  BuildLanePathFromData(
                      mapping::LanePathData(
                          /*start_fraction=*/0.0, /*end_fraction=*/1.0,
                          {mapping::ElementId(2448), mapping::ElementId(1),
                           mapping::ElementId(34)}),
                      psmm));

    const auto origin_lane_path_points =
        SampleLanePathPoints(psmm, origin_lane_path);
    ASSIGN_OR_DIE(const auto frenet_frame,
                  BuildKdTreeFrenetFrame(origin_lane_path_points,
                                         /*down_sample_raw_points=*/true));

    auto start_time = absl::Now();
    ASSIGN_OR_DIE(
        const auto new_lane_path_points,
        FindClosestLanePathPoints(psmm, online_smm_proto, frenet_frame, ego_pos,
                                  /*valid_lane_length=*/50.0,
                                  /*max_lat_offset_thres=*/0.5,
                                  /*avg_lat_offset_thres=*/0.5));
    LOG(INFO) << (absl::Now() - start_time) / absl::Microseconds(1) * 1e-3
              << " ms consumed in FindClosestLanePathPoints";

    EXPECT_FALSE(new_lane_path_points.empty());
    EXPECT_NEAR(new_lane_path_points.front().x(), 0.0, kEpsilon);
    EXPECT_NEAR(new_lane_path_points.front().y(), 0.0, kEpsilon);
    EXPECT_NEAR(new_lane_path_points.back().x(), 50.0, kEpsilon);
    EXPECT_NEAR(new_lane_path_points.back().y(), 0.0, kEpsilon);
  }

  {
    ASSIGN_OR_DIE(const auto origin_lane_path,
                  BuildLanePathFromData(
                      mapping::LanePathData(
                          /*start_fraction=*/0.0, /*end_fraction=*/1.0,
                          {mapping::ElementId(3), mapping::ElementId(951),
                           mapping::ElementId(43)}),
                      psmm));

    const auto origin_lane_path_points =
        SampleLanePathPoints(psmm, origin_lane_path);
    ASSIGN_OR_DIE(const auto frenet_frame,
                  BuildKdTreeFrenetFrame(origin_lane_path_points,
                                         /*down_sample_raw_points=*/true));

    auto start_time = absl::Now();
    ASSIGN_OR_DIE(
        const auto new_lane_path_points,
        FindClosestLanePathPoints(psmm, online_smm_proto, frenet_frame, ego_pos,
                                  /*valid_lane_length=*/70.0,
                                  /*max_lat_offset_thres=*/0.5,
                                  /*avg_lat_offset_thres=*/0.5));
    LOG(INFO) << (absl::Now() - start_time) / absl::Microseconds(1) * 1e-3
              << " ms consumed in FindClosestLanePathPoints";

    EXPECT_FALSE(new_lane_path_points.empty());
    EXPECT_NEAR(new_lane_path_points.front().x(), 0.0, kEpsilon);
    EXPECT_NEAR(new_lane_path_points.front().y(), -3.5, kEpsilon);
    EXPECT_NEAR(new_lane_path_points.back().x(), 68.94, kEpsilon);
    EXPECT_NEAR(new_lane_path_points.back().y(), -3.86, kEpsilon);
  }
}

TEST(ComputeStartLanesByPosTest, BasicTest) {
  const Vec2d ego_pos(0.0, 0.0);
  // Construct online map and psmm.
  const auto& whole_psmm = CreateDojoTestPSMM();
  ASSIGN_OR_DIE(const auto online_smm_proto,
                RunOnlineSemanticMapConverter(whole_psmm,
                                              OnlineSemanticMapConverterOption{
                                                  .smooth_x = ego_pos.x(),
                                                  .smooth_y = ego_pos.y(),
                                                  .smooth_yaw = 0.0,
                                                  .look_ahead_distance = 90.0,
                                                  .look_back_distance = 10.0}));

  ASSIGN_OR_DIE(const auto psmm_ptr, BuildOnlineMapPsmm(online_smm_proto));
  const auto& psmm = *psmm_ptr;

  {
    const auto starting_lane_ids = ComputeStartLanesByPos(
        psmm, online_smm_proto, ego_pos, /*proj_thres=*/1.0,
        /*max_backward_thres=*/1.0);

    const absl::flat_hash_set<mapping::ElementId> start_lanes_gt(
        {mapping::ElementId(2), mapping::ElementId(2448),
         mapping::ElementId(3)});
    EXPECT_EQ(starting_lane_ids.size(), start_lanes_gt.size());
    for (int i = 0; i < starting_lane_ids.size(); ++i) {
      EXPECT_TRUE(start_lanes_gt.contains(starting_lane_ids[i]));
    }
  }

  {
    const auto starting_lane_ids = ComputeStartLanesByPos(
        psmm, online_smm_proto, {-2.0, 0.0}, /*proj_thres=*/1.0,
        /*max_backward_thres=*/1.0);

    const absl::flat_hash_set<mapping::ElementId> start_lanes_gt(
        {mapping::ElementId(96), mapping::ElementId(55), mapping::ElementId(97),
         mapping::ElementId(56), mapping::ElementId(57),
         mapping::ElementId(267)});
    EXPECT_EQ(starting_lane_ids.size(), start_lanes_gt.size());
    for (int i = 0; i < starting_lane_ids.size(); ++i) {
      EXPECT_TRUE(start_lanes_gt.contains(starting_lane_ids[i]));
    }
  }

  {
    const auto starting_lane_ids = ComputeStartLanesByPos(
        psmm, online_smm_proto, {-2.0, 0.0}, /*proj_thres=*/1.0,
        /*max_backward_thres=*/3.0);

    const absl::flat_hash_set<mapping::ElementId> start_lanes_gt(
        {mapping::ElementId(96), mapping::ElementId(55), mapping::ElementId(97),
         mapping::ElementId(56), mapping::ElementId(57),
         mapping::ElementId(267)});
    EXPECT_EQ(starting_lane_ids.size(), start_lanes_gt.size());
    for (int i = 0; i < starting_lane_ids.size(); ++i) {
      EXPECT_TRUE(start_lanes_gt.contains(starting_lane_ids[i]));
    }
  }

  {
    const auto starting_lane_ids = ComputeStartLanesByPos(
        psmm, online_smm_proto, {-2.0, 0.0}, /*proj_thres=*/3.0,
        /*max_backward_thres=*/3.0);
    const absl::flat_hash_set<mapping::ElementId> start_lanes_gt(
        {mapping::ElementId(2), mapping::ElementId(2448),
         mapping::ElementId(3)});
    EXPECT_EQ(starting_lane_ids.size(), start_lanes_gt.size());
    for (int i = 0; i < starting_lane_ids.size(); ++i) {
      EXPECT_TRUE(start_lanes_gt.contains(starting_lane_ids[i]));
    }
  }
}

}  // namespace
}  // namespace planner
}  // namespace qcraft

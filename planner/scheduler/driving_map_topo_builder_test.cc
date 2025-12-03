#include "onboard/planner/scheduler/driving_map_topo_builder.h"

#include <math.h>

#include <memory>
#include <vector>

#include "absl/types/span.h"

#include "gtest/gtest.h"

#include "onboard/base/macros.h"
#include "onboard/container/strong_int.h"
#include "onboard/global/buffered_logger.h"
#include "onboard/maps/lane_point.h"
#include "onboard/maps/map_selector.h"
#include "onboard/maps/semantic_map_defs.h"
#include "onboard/planner/test_util/load_psmm_util.h"
#include "onboard/planner/util/online_map_converter.h"
#include "onboard/planner/util/planner_semantic_map_manager_builder.h"
#include "onboard/utils/status_macros.h"

namespace qcraft::planner {

namespace {

TEST(DrivingMapBuilder, BuildDrivingMapByRouteOnOfflineMap) {
  SetMap("dojo");
  const auto& psmm = CreateDojoTestPSMM();

  std::vector<mapping::SectionId> section_ids = {mapping::SectionId(13334),
                                                 mapping::SectionId(13365)};
  RouteSections sections(0.5, 0.9, section_ids, mapping::LanePoint());

  auto dm_or = BuildDrivingMapByRouteOnOfflineMap(psmm, sections);

  EXPECT_OK(dm_or);

  const auto lanes = dm_or->lanes();
  EXPECT_EQ(lanes.size(), 6);
  EXPECT_EQ(lanes[0].id, mapping::ElementId(2182));
  EXPECT_EQ(lanes[1].id, mapping::ElementId(2180));
  EXPECT_EQ(lanes[2].id, mapping::ElementId(2185));
  EXPECT_EQ(lanes[3].id, mapping::ElementId(13326));
  EXPECT_EQ(lanes[4].id, mapping::ElementId(13328));
  EXPECT_EQ(lanes[5].id, mapping::ElementId(13332));

  EXPECT_EQ(lanes[0].outgoing_lane_ids[0], mapping::ElementId(13326));
  EXPECT_TRUE(lanes[0].incoming_lane_ids.empty());

  EXPECT_EQ(lanes[1].outgoing_lane_ids[0], mapping::ElementId(13328));
  EXPECT_TRUE(lanes[1].incoming_lane_ids.empty());

  EXPECT_EQ(lanes[2].outgoing_lane_ids[0], mapping::ElementId(13332));
  EXPECT_TRUE(lanes[2].incoming_lane_ids.empty());
}

TEST(DrivingMapBuilder, BuildDrivingMapByOnlineMap) {
  const Vec2d ego_pos(20.0, -4.0);
  // Build online semantic map according whole map.
  const auto& whole_psmm = CreateDojoTestPSMM();
  ASSIGN_OR_DIE(const auto online_smm_proto,
                RunOnlineSemanticMapConverter(whole_psmm,
                                              OnlineSemanticMapConverterOption{
                                                  .smooth_x = ego_pos.x(),
                                                  .smooth_y = ego_pos.y(),
                                                  .smooth_yaw = 0.0,
                                                  .look_ahead_distance = 50.0,
                                                  .look_back_distance = 10.0,
                                              }));

  // Build psmm according online semantic map.
  ASSIGN_OR_DIE(const auto psmm_ptr, BuildOnlineMapPsmm(online_smm_proto));
  const auto& psmm = *psmm_ptr;
  // Build driving map topo according semantic map manager and online semantic
  // map.
  ASSIGN_OR_DIE(const auto dm,
                BuildDrivingMapByOnlineMap(psmm, online_smm_proto, ego_pos));

  const auto lanes = dm.lanes();
  EXPECT_EQ(lanes.size(), 5);
  EXPECT_EQ(lanes[0].id, mapping::ElementId(2448));
  EXPECT_EQ(lanes[0].outgoing_lane_ids[0], mapping::ElementId(1));
  EXPECT_TRUE(lanes[0].incoming_lane_ids.empty());

  EXPECT_EQ(lanes[1].id, mapping::ElementId(3));
  EXPECT_EQ(lanes[1].outgoing_lane_ids[0], mapping::ElementId(950));
  EXPECT_EQ(lanes[1].outgoing_lane_ids[1], mapping::ElementId(951));
  EXPECT_TRUE(lanes[1].incoming_lane_ids.empty());

  EXPECT_EQ(lanes[2].id, mapping::ElementId(1));
  EXPECT_TRUE(lanes[2].outgoing_lane_ids.empty());
  EXPECT_EQ(lanes[2].incoming_lane_ids[0], mapping::ElementId(2448));

  EXPECT_EQ(lanes[3].id, mapping::ElementId(950));
  EXPECT_EQ(lanes[3].incoming_lane_ids[0], mapping::ElementId(3));
  EXPECT_TRUE(lanes[3].outgoing_lane_ids.empty());

  EXPECT_EQ(lanes[4].id, mapping::ElementId(951));
  EXPECT_EQ(lanes[4].incoming_lane_ids[0], mapping::ElementId(3));
  EXPECT_TRUE(lanes[4].outgoing_lane_ids.empty());
}

TEST(DrivingMapBuilder, BuildDrivingMapAtPosAndHeadingInRadiusWithMaxLength) {
  SetMap("dojo");
  const auto& psmm = CreateDojoTestPSMM();
  const Vec2d pos(0.1, 0.0);
  const auto dm_or = BuildDrivingMapAtPosAndHeadingInRadiusWithMaxLength(
      psmm, pos, /*heading=*/0.0, /*radius=*/3.0, /*max_forward_length=*/20.0,
      /*max_heading_diff=*/M_PI_4, /*allow_virtual=*/false);
  EXPECT_OK(dm_or);
  const auto& start_ids = dm_or->starting_lane_ids();
  EXPECT_EQ(start_ids.size(), 2);
  EXPECT_EQ(start_ids[0], mapping::ElementId(56));
  EXPECT_EQ(start_ids[1], mapping::ElementId(97));
  const auto& lanes = dm_or->lanes();
  EXPECT_EQ(lanes.size(), 3);

  // Intersection start with non-virtual lanes.
  const Vec2d pos2(772.2, -464.1);
  const auto dm_2_or = BuildDrivingMapAtPosAndHeadingInRadiusWithMaxLength(
      psmm, pos2, /*heading=*/0.0, /*radius=*/10.0, /*max_forward_length=*/50.0,
      /*max_heading_diff=*/M_PI_4, /*allow_virtual=*/false);
  const auto& start_ids_2 = dm_2_or->starting_lane_ids();
  EXPECT_EQ(start_ids_2.size(), 2);
  const auto& lanes_2 = dm_2_or->lanes();
  EXPECT_EQ(lanes_2.size(), 4);
  EXPECT_EQ(lanes_2[3].id, mapping::ElementId(7018));
  EXPECT_EQ(lanes_2[2].id, mapping::ElementId(7022));
  EXPECT_TRUE(lanes_2[3].outgoing_lane_ids.empty());
  EXPECT_TRUE(lanes_2[2].outgoing_lane_ids.empty());

  EXPECT_EQ(lanes_2[0].start_fraction, 0.0);
  EXPECT_EQ(lanes_2[1].start_fraction, 0.0);

  const Vec2d pos3(603.6, -465.4);
  const auto dm_3_or = BuildDrivingMapAtPosAndHeadingInRadiusWithMaxLength(
      psmm, pos3, /*heading=*/0.0, /*radius=*/3.0, /*max_forward_length=*/10.0,
      /*max_heading_diff=*/M_PI_4, /*allow_virtual=*/false);

  EXPECT_EQ(dm_3_or->starting_lane_ids().size(), 1);
  EXPECT_EQ(dm_3_or->starting_lane_ids()[0], mapping::ElementId(6781));
  const auto* lane = dm_3_or->GetLaneById(mapping::ElementId(6781));
  EXPECT_EQ(lane->outgoing_lane_ids.size(), 0);
}

}  // namespace

}  // namespace qcraft::planner

#include "onboard/planner/assist/vision_lane_path_filter.h"

// IWYU pragma: no_include <google/protobuf/repeated_ptr_field.h>

#include <memory>
#include <ostream>
#include <utility>
#include <vector>

#include "absl/time/clock.h"
#include "absl/time/time.h"
#include "glog/logging.h"

#include "gtest/gtest.h"

#include "onboard/base/macros.h"
#include "onboard/container/strong_int.h"
#include "onboard/global/buffered_logger.h"
#include "onboard/maps/map_selector.h"
#include "onboard/maps/online_map_utils.h"
#include "onboard/maps/semantic_map_defs.h"
#include "onboard/maps/semantic_map_manager.h"
#include "onboard/planner/test_util/load_psmm_util.h"
#include "onboard/planner/util/online_map_converter.h"
#include "onboard/planner/util/planner_semantic_map_manager_builder.h"
#include "onboard/proto/localization.pb.h"
#include "onboard/utils/status_macros.h"

namespace qcraft {
namespace planner {
namespace {

TEST(AssistUtilTest, ProjectLanePathToCurrentOnlineMap) {
  const Vec2d ego_pos(0.0, 0.0);
  const double ego_v = 10.0;
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

  auto new_online_smm_proto = online_smm_proto;
  new_online_smm_proto.set_update_id(online_smm_proto.update_id() + 1);
  for (auto& lane : *new_online_smm_proto.mutable_lanes()) {
    lane.set_id(lane.id() + 10000);
    lane.set_left_lane_id(lane.left_lane_id() + 10000);
    lane.set_right_lane_id(lane.right_lane_id() + 10000);
    for (auto& outgoing_id : *lane.mutable_outgoing_lane_ids()) {
      outgoing_id += 10000;
    }
    for (auto& incoming_id : *lane.mutable_incoming_lane_ids()) {
      incoming_id += 10000;
    }
  }
  for (auto& boundary : *new_online_smm_proto.mutable_boundaries()) {
    boundary.set_left_lane_id(boundary.left_lane_id() + 10000);
    boundary.set_right_lane_id(boundary.right_lane_id() + 10000);
  }

  // Build semantic map manager according online semantic map.
  auto semantic_map_manager =
      std::make_shared<SemanticMapManager>(GetMap(), /*ignore_level=*/true);
  auto* smm = semantic_map_manager.get();
  {
    auto [semantic_map, meta] = mapping::ToSemanticMapProtos(online_smm_proto);
    smm->Emplace(std::move(semantic_map), meta).Build();
    auto lt = online_smm_proto.localization_transform();
    lt.set_level_id(0);
    smm->UpdateLocalizationTransform(lt);
  }

  auto new_smm =
      std::make_shared<SemanticMapManager>(GetMap(), /*ignore_level=*/true);
  {
    auto [semantic_map, meta] =
        mapping::ToSemanticMapProtos(new_online_smm_proto);
    new_smm->Emplace(std::move(semantic_map), meta).Build();
    auto lt = online_smm_proto.localization_transform();
    lt.set_level_id(0);
    new_smm->UpdateLocalizationTransform(lt);
  }

  ASSIGN_OR_DIE(const auto psmm_ptr, BuildOnlineMapPsmm(online_smm_proto));
  const auto& psmm = *psmm_ptr;
  ASSIGN_OR_DIE(const auto new_psmm_ptr,
                BuildOnlineMapPsmm(new_online_smm_proto));
  const auto& new_psmm = *new_psmm_ptr;

  {
    const mapping::LanePath original_lane_path(
        smm,
        {mapping::ElementId(2448), mapping::ElementId(1),
         mapping::ElementId(34)},
        /*start_fraction=*/0.0, /*end_fraction=*/1.0);

    auto start_time = absl::Now();
    const auto new_lane_path_or = ProjectLanePathToCurrentOnlineMap(
        new_psmm, new_online_smm_proto, psmm, original_lane_path, ego_pos,
        ego_v, /*check_preview_length=*/0.0, /*thread_pool=*/nullptr);
    LOG(INFO) << (absl::Now() - start_time) / absl::Microseconds(1) * 1e-3
              << " ms consumed in ProjectLanePathToCurrentOnlineMap";

    EXPECT_OK(new_lane_path_or);
    EXPECT_EQ(new_lane_path_or->size(), 4);
    EXPECT_EQ(new_lane_path_or->lane_ids()[1].value(), 12448);
    EXPECT_EQ(new_lane_path_or->lane_ids()[2].value(), 10001);
    EXPECT_EQ(new_lane_path_or->lane_ids()[3].value(), 10034);
  }

  {
    const mapping::LanePath original_lane_path(
        smm,
        {mapping::ElementId(3), mapping::ElementId(951),
         mapping::ElementId(43)},
        /*start_fraction=*/0.0, /*end_fraction=*/1.0);

    auto start_time = absl::Now();
    const auto new_lane_path_or = ProjectLanePathToCurrentOnlineMap(
        new_psmm, new_online_smm_proto, psmm, original_lane_path, ego_pos,
        ego_v, /*check_preview_length=*/0.0, /*thread_pool=*/nullptr);
    LOG(INFO) << (absl::Now() - start_time) / absl::Microseconds(1) * 1e-3
              << " ms consumed in ProjectLanePathToCurrentOnlineMap";

    EXPECT_OK(new_lane_path_or);
    EXPECT_EQ(new_lane_path_or->size(), 4);
    EXPECT_EQ(new_lane_path_or->lane_ids()[1].value(), 10003);
    EXPECT_EQ(new_lane_path_or->lane_ids()[2].value(), 10950);
    EXPECT_EQ(new_lane_path_or->lane_ids()[3].value(), 10035);
  }
}

}  // namespace
}  // namespace planner
}  // namespace qcraft

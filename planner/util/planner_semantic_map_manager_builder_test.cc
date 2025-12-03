#include "onboard/planner/util/planner_semantic_map_manager_builder.h"

#include "absl/time/clock.h"
#include "absl/time/time.h"

#include "gtest/gtest.h"

#include "onboard/global/buffered_logger.h"
#include "onboard/maps/map_selector.h"
#include "onboard/maps/v2/semantic_map_gflags.h"
#include "onboard/maps/v2/semantic_map_loader.h"
#include "onboard/planner/test_util/load_psmm_util.h"
#include "onboard/planner/util/online_map_converter.h"
#include "onboard/proto/adasis.pb.h"
#include "onboard/proto/online_map.pb.h"
#include "onboard/utils/status_macros.h"
#include "onboard/utils/time_util.h"

namespace qcraft::planner {
namespace {
TEST(PlannerSemanticMapManagerBuilderTest, BuildOnlineMapPsmmTest) {
  const auto& psmm = CreateDojoTestPSMM();
  absl::Time plan_time = absl::Now();
  ASSIGN_OR_DIE(const auto online_smm_proto,
                RunOnlineSemanticMapConverter(
                    psmm, OnlineSemanticMapConverterOption{
                              .timestamp_s = ToUnixDoubleSeconds(plan_time),
                              .smooth_x = 0.0,
                              .smooth_y = 0.0,
                              .smooth_yaw = 0.0,
                              .look_ahead_distance = 150.0,
                              .look_back_distance = 20.0}));

  ASSIGN_OR_DIE(const auto fake_psmm, BuildOnlineMapPsmm(online_smm_proto));
  EXPECT_NE(fake_psmm, nullptr);
}

TEST(PlannerSemanticMapManagerBuilderTest,
     AsyncLoadPlannerSemanticMapManagerTest) {
  SetMap("dojo");
  auto loader_v2 = mapping::v2::SemanticMapLoader::MakeShared();
  const auto semantic_map_manager_v2 =
      loader_v2->PreloadAsync(/*lon=*/0.0, /*lat=*/0.0, /*edge_count=*/3).Get();
  const auto semantic_map_index_v2 =
      mapping::v2::SemanticMapMultilevelSpatialIndex::MakeShared(
          semantic_map_manager_v2);

  auto psmm_future = AsyncLoadPlannerSemanticMapManager(
      semantic_map_index_v2, CoordinateConverter::FromMap("dojo"),
      static_cast<ThreadPool*>(nullptr));
  EXPECT_TRUE(psmm_future.IsValid());

  psmm_future.Wait();
  EXPECT_TRUE(psmm_future.IsReady());

  const auto psmm = psmm_future.Get();
  EXPECT_NE(psmm, nullptr);
  psmm_future = Future<std::shared_ptr<PlannerSemanticMapManager>>();
  EXPECT_FALSE(psmm_future.IsValid());
}

TEST(PlannerSemanticMapManagerBuilderTest, UpdateHdMapAlongRouteTest) {
  FLAGS_use_local_qcraft_hdmap = false;
  const auto navinfo_hdmap_listener =
      mapping::v2::SemanticMapMultilevelSpatialIndexListenerAsnyc::MakeShared(
          {OnlineMapProto_DataSource_NAVINFO_HDMAP});
  navinfo_hdmap_listener->EnableRouteFilter(/*enable=*/true);

  RouteSectionSequenceProto sections_proto;
  sections_proto.set_start_fraction(0.1);
  sections_proto.set_end_fraction(0.8);
  auto* section_ids_ptr = sections_proto.mutable_section_id();
  section_ids_ptr->Add(12401);
  section_ids_ptr->Add(12400);
  section_ids_ptr->Add(12408);

  PlannerState::HdMapState map_state;
  map_state.load_distance = 150.0;
  map_state.has_destination = false;

  const auto hd_map_state =
      std::make_optional<PlannerState::HdMapState>(map_state);

  const auto smmsi = UpdateHdSemanticMapManagerAlongRoute(
      {0.0, 0.0}, sections_proto, hd_map_state,
      /*::google::protobuf::RepeatedPtrField<OverlappingSection>=*/nullptr,
      navinfo_hdmap_listener.get());

  EXPECT_EQ(navinfo_hdmap_listener->GetLatestMppSections().ego_section_id(),
            12401);
  EXPECT_NEAR(
      navinfo_hdmap_listener->GetLatestMppSections().ego_section_fraction(),
      0.1, 1e-8);
  EXPECT_EQ(smmsi, nullptr);
}
}  // namespace
}  // namespace qcraft::planner

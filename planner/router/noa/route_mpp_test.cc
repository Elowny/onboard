#include "onboard/planner/router/noa/route_mpp.h"

#include <algorithm>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

#include "gtest/gtest.h"

#include "onboard/lite/qissue_trans.h"

// IWYU pragma: no_include <google/protobuf/repeated_ptr_field.h>

#include "absl/container/flat_hash_map.h"

#include "onboard/async/future.h"
#include "onboard/base/macros.h"
#include "onboard/container/strong_int.h"
#include "onboard/global/logging.h"
#include "onboard/maps/ehr/proto/extension.pb.h"
#include "onboard/maps/ehr/proto/odd.pb.h"
#include "onboard/maps/map_or_die_macros.h"
#include "onboard/maps/proto/semantic_map.pb.h"
#include "onboard/maps/proto/semantic_map_meta.pb.h"
#include "onboard/maps/semantic_map_defs.h"
#include "onboard/maps/v2/semantic_map_definition.h"
#include "onboard/maps/v2/semantic_map_loader.h"
#include "onboard/maps/v2/semantic_map_manager.h"
#include "onboard/maps/v2/semantic_map_object.h"
#include "onboard/math/coordinate_converter.h"
#include "onboard/planner/router/noa/hd_hmi_adapter.h"
#include "onboard/proto/hmi_content.pb.h"
#include "onboard/proto/route_navigation.pb.h"
#include "onboard/utils/source_location.h"
namespace qcraft::planner::route::noa {
namespace {

TEST(RouteMpp, MppToNoaRoute) {
  // Add v2smm test
  auto loader =
      mapping::v2::SemanticMapLoader::MakeShared({.map_name = "dojo"});
  auto map_fut = loader->PreloadWholeMap();
  const auto* v2smm = map_fut.Get().get();
  std::vector<int64_t> sections = {12401, 12400, 12408, 12412};
  std::int64_t update_id = 0;
  auto set_route_fn = [v2smm](int64_t section_id, bool value) {
    const auto* const_section =
        v2smm->FindSection(mapping::SectionId(section_id)).get();
    auto* section = const_cast<mapping::v2::Section*>(const_section);
    auto* proto = const_cast<mapping::SectionProto*>(&section->proto());
    proto->mutable_third_party_section_extend()
        ->set_is_part_of_calculated_route(value);
  };
  // Inject route guidance property
  {
    for (auto section_id : sections) {
      set_route_fn(section_id, true);
    }
    auto noa_route_mpp = MppToNoaRoute(*v2smm, sections, update_id,
                                       /*route_trunc_mpp_by_nav=*/true);

    const auto& target =
        noa_route_mpp.route.route_section_sequence().section_id();
    ASSERT_TRUE(std::equal(sections.begin(), sections.end(), target.begin()));
  }

  MppSectionsProto mpp;
  mpp.mutable_section_ids()->Reserve(sections.size());
  for (const auto& section : sections) {
    mpp.add_section_ids(section);
  }
  mpp.set_ego_section_id(sections.front());
  std::shared_ptr<mapping::v2::SemanticMapSpatialIndex> smm_index =
      mapping::v2::SemanticMapSpatialIndex::MakeShared(map_fut.Get());
  absl::StatusOr<RouteManagerOutputProto> rm_output_proto = CalcNavInfoFromMpp(
      *smm_index, mpp, /*global=*/{0.0, 0.0},
      /*heading=*/std::make_optional<double>(0.0), update_id,
      /*av_speed=*/std::make_optional<double>(10.0), /*look_ahead_time=*/3.0,
      /*route_trunc_mpp_by_nav=*/true, /*restrict_proto=*/{});
  ASSERT_OK(rm_output_proto);
  ASSERT_TRUE(rm_output_proto->has_route_sections_from_current());
  ASSERT_TRUE(rm_output_proto->has_route_navi_info());
  ASSERT_TRUE(rm_output_proto->has_update_id());
  ASSERT_EQ(rm_output_proto->update_id(), update_id);
  ASSERT_TRUE(rm_output_proto->has_recommend_lane_speed_limit());

  // Test UpdateNavAction
  RouteSectionSequenceProto rss;
  rss.set_start_fraction(0.3);
  rss.set_end_fraction(0.8);
  std::for_each(sections.cbegin(), sections.cend(),
                [&rss](auto section_id) { rss.add_section_id(section_id); });

  {
    // Set LaneProto::type JCT,sections: {12401, 12400, 12408, 12412};
    // 12412:{2470}

    SMM_SECTION_PROTO_OR_RETURN(section_proto, *v2smm,
                                mapping::SectionId(sections.back()), void());

    mapping::LaneProto* jct_lane = nullptr;
    SMM_LANE_PROTO_OR_RETURN(lane, *v2smm, mapping::ElementId(2470), void());
    jct_lane = const_cast<mapping::LaneProto*>(lane);
    jct_lane->set_type(mapping::LaneProto::RAMP_JCT);
    mapping::LaneProto* last_exit_lane = nullptr;
    for (auto ic_lane_id : jct_lane->incoming_lanes()) {
      SMM_LANE_PROTO_OR_RETURN(exit_lane, *v2smm,
                               mapping::ElementId(ic_lane_id), void());
      if (exit_lane->section_id() == sections[sections.size() - 2]) {
        last_exit_lane = const_cast<mapping::LaneProto*>(exit_lane);
        last_exit_lane->set_type(mapping::LaneProto::EXIT);
      }
    }
    mapping::LaneProto* first_exit_lane = nullptr;
    for (auto ic_lane_id : last_exit_lane->incoming_lanes()) {
      SMM_LANE_PROTO_OR_RETURN(exit_lane, *v2smm,
                               mapping::ElementId(ic_lane_id), void());
      if (exit_lane->section_id() == sections[sections.size() - 3]) {
        first_exit_lane = const_cast<mapping::LaneProto*>(exit_lane);
        first_exit_lane->set_type(mapping::LaneProto::EXIT);
      }
    }
    NaviInstructionProto nav_instruction;
    UpdateNavInstruction(*v2smm, rss,
                         /*horizon=*/200, &nav_instruction);
    ASSERT_EQ(nav_instruction.navi_action().sub_action_type(),
              NaviActionProto::ENTER_RAMP);
    ASSERT_EQ(nav_instruction.navi_action().section_id(),
              sections[sections.size() - 3]);
    EXPECT_NEAR(nav_instruction.distance(), 48, 1.0);
    nav_instruction.Clear();
    RouteSectionSequenceProto one_rss;
    one_rss.add_section_id(12401);
    UpdateNavInstruction(*v2smm, one_rss,
                         /*horizon=*/200, &nav_instruction);
    ASSERT_TRUE(nav_instruction.is_last_navi_action());
  }

  // Test UpdateHmiOddEvent
  std::vector<double> sec_lens;
  {
    {
      // Passed
      SMM_SECTION_PROTO_OR_RETURN(section_proto0, *v2smm, sections[0], void());
      auto* section0 = const_cast<mapping::SectionProto*>(section_proto0);
      auto* ev0 = section0->mutable_odd()->add_custom_events();
      ev0->set_end_fraction(0.25);
      ev0->set_type(mapping::CustomEvent::FORCED_DOWNGRADE);
      sec_lens.push_back(section_proto0->average_length() * (1 - 0.3));

      SMM_SECTION_PROTO_OR_RETURN(section_proto1, *v2smm, sections[1], void());
      auto* section1 = const_cast<mapping::SectionProto*>(section_proto1);
      sec_lens.push_back(section_proto1->average_length());

      auto* ev1 = section1->mutable_odd()->add_custom_events();
      ev1->set_start_fraction(0.0);
      ev1->set_end_fraction(0.8);
      ev1->set_type(mapping::CustomEvent::ROAD_MODEL_CHANGE);

      SMM_SECTION_PROTO_OR_RETURN(section_proto2, *v2smm, sections[2], void());
      auto* section2 = const_cast<mapping::SectionProto*>(section_proto2);
      sec_lens.push_back(section_proto2->average_length());

      auto* ev2 = section2->mutable_odd()->add_traffic_events();
      ev2->set_type(mapping::TrafficEvent::TRAFFIC_ACCIDENT);

      auto* ev3 = section2->mutable_odd()->add_traffic_events();
      ev3->set_type(mapping::TrafficEvent::TRAFFIC_CONTROL);
      ev3->set_start_fraction(0.6);

      auto* ev4 = section2->mutable_odd()->add_traffic_events();
      ev4->set_type(mapping::TrafficEvent::ROAD_CLOSURE);
      ev4->set_end_fraction(0.7);
    }
    NcaOdcProto nca_odc_proto;
    UpdateHmiOddEvent(*v2smm, rss,
                      /*horizon=*/500, &nca_odc_proto);
    ASSERT_EQ(nca_odc_proto.odd_traffic_events_size(), 3);
    ASSERT_EQ(nca_odc_proto.odd_custom_events_size(), 1);

    EXPECT_NEAR(nca_odc_proto.odd_custom_events(0).distance(), sec_lens[0],
                1.0);  // ev1
    EXPECT_NEAR(nca_odc_proto.odd_traffic_events(0).distance(),
                sec_lens[0] + sec_lens[1], 1.0);  // ev2

    ASSERT_EQ(nca_odc_proto.odd_traffic_events(1).type(),
              mapping::TrafficEvent::TRAFFIC_CONTROL);
    EXPECT_NEAR(nca_odc_proto.odd_traffic_events(1).distance(),
                sec_lens[0] + sec_lens[1] + sec_lens[2] * 0.6, 1.0);  // ev3
  }
}

TEST(RouteMppTest, ParseDynamicOddAlongMpp) {
  mapping::SemanticMapProto semantic_map_proto;
  auto* section1 = semantic_map_proto.mutable_sections()->Add();
  section1->set_id(10086);
  section1->mutable_lanes()->Add(1);
  section1->mutable_lanes()->Add(2);
  section1->mutable_lanes()->Add(3);

  auto* odd = section1->mutable_odd();
  auto* custom_event = odd->mutable_custom_events()->Add();
  custom_event->set_type(mapping::CustomEvent::AVOID_LANE_SEGMENTS);
  custom_event->set_start_fraction(0.1);
  custom_event->set_end_fraction(0.5);
  auto* lane_number = custom_event->mutable_lane_number();
  lane_number->Add(1);
  lane_number->Add(3);

  mapping::SemanticMapMetaProto semantic_map_meta_proto;

  std::shared_ptr<mapping::v2::SemanticMapManager> smm =
      mapping::v2::SemanticMapManager::MakeShared(
          mapping::v2::kInvalidUpdateId, {}, CoordinateConverter::FromLocale(),
          std::move(semantic_map_proto), std::move(semantic_map_meta_proto));

  const auto res = ParseDynamicOddAlongMpp(*smm, {10086});
  EXPECT_EQ(res.lane_segments_map.size(), 2);
  EXPECT_TRUE(res.ContainsLane(mapping::ElementId(1)));
  EXPECT_TRUE(res.ContainsLane(mapping::ElementId(3)));
  EXPECT_DOUBLE_EQ(
      res.lane_segments_map.at(mapping::ElementId(1)).start_fraction, 0.1);
  EXPECT_DOUBLE_EQ(res.lane_segments_map.at(mapping::ElementId(1)).end_fraction,
                   0.5);
}

}  // namespace

}  // namespace qcraft::planner::route::noa

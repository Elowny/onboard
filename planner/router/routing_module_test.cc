#include "onboard/planner/router/routing_module.h"

#include <memory>
#include <ostream>
#include <string>
#include <vector>

#include "glog/logging.h"
#include "google/protobuf/repeated_ptr_field.h"

#include "gtest/gtest.h"

#include "common/proto/drive_mission.pb.h"
#include "common/proto/map_geometry.pb.h"

#include "onboard/base/macros.h"
#include "onboard/global/logging.h"
#include "onboard/lite/proto/lite_common.pb.h"
#include "onboard/lite/qissue_trans.h"
#include "onboard/lite/unittest/lite2_unittest_helper.h"
#include "onboard/maps/map_or_die_macros.h"
#include "onboard/maps/map_selector.h"
#include "onboard/maps/proto/semantic_map.pb.h"
#include "onboard/maps/proto/semantic_map_meta.pb.h"
#include "onboard/maps/v2/semantic_map_gflags.h"
#include "onboard/math/coordinate_converter.h"
#include "onboard/math/geometry/proto/affine_transformation.pb.h"
#include "onboard/math/vec.h"
#include "onboard/planner/router/route_test_util.h"
#include "onboard/planner/router/router_flags.h"
#include "onboard/proto/online_map.pb.h"
#include "onboard/proto/q_run_event.pb.h"
#include "onboard/proto/route_navigation.pb.h"
#include "onboard/utils/source_location.h"

namespace qcraft::planner {
namespace {
// This test is not a real UT, but much like the staging/smoke test.
// Only to test the code branch coverage.
TEST(RoutingModule, NoaMainLoop) {
  FLAGS_map = "dojo";
  const auto cc = CoordinateConverter::FromMap("dojo");

  Lite2UnitTestHelper lite2_ut_helper;
  lite2_ut_helper.Init("RoutingModule", ROUTING_MODULE, /*is_async=*/false);
  RoutingModule* routing_module =
      static_cast<RoutingModule*>(lite2_ut_helper.GetLiteModule());

  {
    FLAGS_route_noa_mode = true;
    // need to call `DisableInputOutputChecker` before re-init
    // TODO(xycai) this is not expected usage of LiteModule. But this UT fails
    // if we replace `routing_module->Init` in each curly braced block by
    // `lite2_ut_helper.Init` and `lite2_ut_helper.Destroy`
    routing_module->DisableInputOutputChecker();
    routing_module->Init(lite2_ut_helper.GetLiteContext());
    Vec2d pos = {2.10505583997, 0.547665799936};
    const auto cc = CoordinateConverter::FromMap("dojo");
    auto smooth_pos = cc.GlobalToSmooth(pos);
    PoseProto pose_proto;
    pose_proto.mutable_pos_smooth()->set_x(smooth_pos.x());
    pose_proto.mutable_pos_smooth()->set_y(smooth_pos.y());
    routing_module->HandlePose(std::make_shared<const PoseProto>(pose_proto));
    routing_module->HandleGnssRawReadingProto(
        std::make_shared<const GnssRawReadingProto>());
    auto s = routing_module->NoaMainLoop();
    ASSERT_TRUE(s.ok());
  }

  // Test mpp
  RoutingResultProto routing_result;
  {
    LOG(INFO) << "Test mpp proto";
    FLAGS_route_fix_6 = false;
    FLAGS_route_snapshot_sim_mode = false;
    FLAGS_use_local_qcraft_hdmap = false;
    // need to do this before re-init
    routing_module->DisableInputOutputChecker();
    routing_module->Init(lite2_ut_helper.GetLiteContext());

    const auto& smm = LoadDojoMap();
    routing_module->semantic_map_listener_ =
        mapping::v2::SemanticMapSpatialIndexListenerAsnyc::MakeShared(
            {OnlineMapProto_DataSource_QCRAFT_HDMAP});

    std::shared_ptr<OnlineMapProto> online_map_proto =
        std::make_shared<OnlineMapProto>();
    online_map_proto->set_data_source(smm.GetDataSource());
    *(online_map_proto->mutable_semantic_map_meta()) =
        smm.semantic_map_meta_proto();
    *(online_map_proto->mutable_semantic_map()) = smm.semantic_map_proto();

    routing_module->semantic_map_listener_->UpdateOnlineMap(online_map_proto);
    routing_module->HandleLocalizationTransform(
        std::make_shared<const LocalizationTransformProto>(
            cc.localization_transform()));
    std::vector<int> section_ids = {12401, 12400, 12317, 12399};
    MppSectionsProto mpp;
    for (auto id : section_ids) {
      mpp.add_section_ids(id);
    }
    mpp.set_ego_section_id(section_ids.front());
    // Set routing result & route manager state.
    routing_result.set_update_id(123L);
    routing_result.set_success(true);

    float total_length = 0.0f;
    Vec2d pos = {0, 0};
    bool first_set_point = false;
    for (auto id : section_ids) {
      SMM_SECTION_PROTO_OR_BREAK(section_proto, smm, id);
      SMM_LANE_PROTO_OR_BREAK(lane_proto, smm, section_proto->lanes()[0]);
      for (const auto& geo_pt : lane_proto->polyline().points()) {
        auto* pt = routing_result.add_path_points_from_current();
        pt->set_x(geo_pt.longitude());
        pt->set_y(geo_pt.latitude());
        if (!first_set_point) {
          pos = {pt->x(), pt->y()};
        }
      }
      total_length += section_proto->length();
    }

    routing_result.mutable_navi_preview()->set_update_id(123L);
    auto* nav_seg = routing_result.mutable_navi_preview()->add_navi_segments();
    auto* nav_action = nav_seg->add_navi_actions();
    nav_action->set_navi_action_type(NaviActionProto::STRAIGHT);
    nav_seg->set_length(total_length);
    auto* instruciton = routing_result.mutable_current_navi_instruction();
    instruciton->set_distance(total_length);
    instruciton->set_is_last_navi_action(true);
    instruciton->set_update_id(123L);

    const auto cc = CoordinateConverter::FromMap("dojo");
    auto smooth_pos = cc.GlobalToSmooth(pos);
    PoseProto pose_proto;
    pose_proto.mutable_pos_smooth()->set_x(smooth_pos.x());
    pose_proto.mutable_pos_smooth()->set_y(smooth_pos.y());

    routing_module->HandleRoutingResultProto(
        std::make_shared<const RoutingResultProto>(routing_result));
    routing_module->HandlePose(std::make_shared<const PoseProto>(pose_proto));
    routing_module->HandleMppProto(
        std::make_shared<const MppSectionsProto>(mpp));
    ASSERT_OK(routing_module->NoaMainLoop());
  }
  {
    // Test HandleSdRouteProto
    LOG(INFO) << "Test HandleSdRouteProto ";
    FLAGS_route_use_sdroute = true;
    FLAGS_route_noa_mode = true;
    FLAGS_route_snapshot_sim_mode = false;
    FLAGS_use_local_qcraft_hdmap = false;
    // need to do this before re-init
    routing_module->DisableInputOutputChecker();
    routing_module->Init(lite2_ut_helper.GetLiteContext());
    QRunEventsProto run_events;
    auto* run_event = run_events.add_run_events();
    run_event->set_key(QRunEvent::KEY_QEVENT_SET_SD_ROUTE);
    run_event->set_value_type(QRunEvent::TYPE_MSG);
    auto& sd_route = *run_event->mutable_sd_route();
    sd_route.set_route_id(100);
    sd_route.set_length(100);
    sd_route.set_status(SDRouteProto::CREATE);
    auto* section = sd_route.add_sd_sections();
    const auto& last_points = routing_result.path_points_from_current();
    auto xy_to_geo_fn = [](const auto& pt) {
      mapping::GeoPointProto geo;
      geo.set_longitude(pt.x());
      geo.set_latitude(pt.y());
      return geo;
    };
    {
      auto* link1 = section->add_sd_links();
      link1->set_length(50);
      link1->set_link_id(1);
      *link1->add_points() = xy_to_geo_fn(last_points[0]);
      *link1->add_points() = xy_to_geo_fn(last_points[1]);
    }
    {
      auto* link2 = section->add_sd_links();
      link2->set_length(50);
      link2->set_link_id(2);
      *link2->add_points() = xy_to_geo_fn(last_points[2]);
      *link2->add_points() = xy_to_geo_fn(last_points[3]);
    }

    LOG(INFO) << run_event->DebugString();
    auto q_run_events =
        std::make_shared<QRunEventsProto>(std::move(run_events));
    routing_module->HandleRunEvents(q_run_events);
    ASSERT_OK(routing_module->NoaMainLoop());
    LOG(INFO) << "Handle sd route proto exit";
  }
  lite2_ut_helper.Destroy();
}

TEST(RoutingModule, L4MainLoop) {
  FLAGS_map = "dojo";
  FLAGS_route_noa_mode = false;

  Lite2UnitTestHelper lite2_ut_helper;
  lite2_ut_helper.Init("RoutingModule", ROUTING_MODULE, /*is_async=*/false);
  RoutingModule* routing_module =
      static_cast<RoutingModule*>(lite2_ut_helper.GetLiteModule());

  {
    Vec2d pos = {0.0, 0.0};
    const auto cc = CoordinateConverter::FromMap("dojo");
    auto smooth_pos = cc.GlobalToSmooth(pos);
    PoseProto pose_proto;
    pose_proto.mutable_pos_smooth()->set_x(smooth_pos.x());
    pose_proto.mutable_pos_smooth()->set_y(smooth_pos.y());
    routing_module->HandlePose(
        std::make_shared<const PoseProto>(std::move(pose_proto)));
    routing_module->HandleLocalizationTransform(
        std::make_shared<const LocalizationTransformProto>(
            cc.localization_transform()));
    routing_module->HandleGnssRawReadingProto(
        std::make_shared<const GnssRawReadingProto>());
    auto autonomy_state_proto = std::make_shared<AutonomyStateProto>();
    autonomy_state_proto->set_autonomy_state(
        AutonomyStateProto::READY_TO_AUTO_DRIVE);
    routing_module->HandleAutonomyState(std::move(autonomy_state_proto));
  }

  ReroutingRequestProto rerouting_request;
  auto& des1 = *rerouting_request.mutable_routing_request()->add_destinations();
  des1.mutable_global_point()->set_longitude(-1.8552155302746883e-08);
  des1.mutable_global_point()->set_latitude(1.6905497148730788e-09);
  des1.mutable_global_point()->set_altitude(0.0);
  auto& des2 = *rerouting_request.mutable_routing_request()->add_destinations();
  des2.mutable_global_point()->set_longitude(0.00005076);
  des2.mutable_global_point()->set_latitude(0.0000006);
  des2.mutable_global_point()->set_altitude(0.0);
  routing_module->HandleReroutingRequest(
      std::make_shared<const ReroutingRequestProto>(
          std::move(rerouting_request)));
  LOG(INFO) << "Routing request processed.";
  routing_module->MainLoop();
  {
    // Test Update route
    LOG(INFO) << "Test Update route";
    Vec2d pos = {9E-8, 1.56E-8};
    const auto cc = CoordinateConverter::FromMap("dojo");
    auto smooth_pos = cc.GlobalToSmooth(pos);
    PoseProto pose_proto;
    pose_proto.mutable_pos_smooth()->set_x(smooth_pos.x());
    pose_proto.mutable_pos_smooth()->set_y(smooth_pos.y());
    routing_module->HandlePose(
        std::make_shared<const PoseProto>(std::move(pose_proto)));
    routing_module->MainLoop();
  }
  {
    // Test reroute
    LOG(INFO) << "Test reroute process main loop.";
    Vec2d pos = {-1.6380597E-6, 5.959405E-6};  //// id=1191
    const auto cc = CoordinateConverter::FromMap("dojo");
    auto smooth_pos = cc.GlobalToSmooth(pos);
    PoseProto pose_proto;
    pose_proto.mutable_pos_smooth()->set_x(smooth_pos.x());
    pose_proto.mutable_pos_smooth()->set_y(smooth_pos.y());
    pose_proto.set_yaw(/*yaw=*/1.6);

    routing_module->HandlePose(
        std::make_shared<const PoseProto>(std::move(pose_proto)));
    routing_module->MainLoop();  // No reroute.

    routing_module->route_mgr_->data_.rms_debug.set_last_update_time(0L);
    auto autonomy_state_proto = std::make_shared<AutonomyStateProto>();
    autonomy_state_proto->set_autonomy_state(AutonomyStateProto::REMOTE_DRIVE);
    routing_module->HandleAutonomyState(std::move(autonomy_state_proto));
    routing_module->MainLoop();  // Should reroute
  }

  lite2_ut_helper.Destroy();
}

}  // namespace

}  // namespace qcraft::planner

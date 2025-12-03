#include "onboard/planner/router/noa/noa_main_loop.h"

#include <memory>
#include <optional>
#include <utility>

#include "google/protobuf/repeated_ptr_field.h"

#include "gtest/gtest.h"

#include "common/proto/map_geometry.pb.h"

#include "onboard/base/macros.h"
#include "onboard/math/geometry/proto/affine_transformation.pb.h"
#include "onboard/proto/adasis.pb.h"
#include "onboard/proto/hmi_content.pb.h"
#include "onboard/proto/positioning.pb.h"
#include "onboard/proto/route_navigation.pb.h"
namespace qcraft::planner::route::noa {
namespace {
SDRouteProto CreateTestSdRouteProto() {
  SDRouteProto sd_route;
  sd_route.set_route_id(100);
  sd_route.set_length(100);
  sd_route.set_status(SDRouteProto::CREATE);

  auto* section = sd_route.add_sd_sections();
  {
    auto* link1 = section->add_sd_links();
    link1->set_length(50);
    link1->set_link_id(1);
    auto* geo_pt_1 = link1->mutable_points()->Add();
    geo_pt_1->set_longitude(2.0);
    geo_pt_1->set_latitude(0.500001);

    auto* geo_pt_2 = link1->mutable_points()->Add();
    geo_pt_2->set_longitude(2.0);
    geo_pt_2->set_latitude(0.500002);
  }
  {
    auto* link2 = section->add_sd_links();
    link2->set_length(50);
    link2->set_link_id(2);
    auto* geo_pt_1 = link2->mutable_points()->Add();
    geo_pt_1->set_longitude(2.0);
    geo_pt_1->set_latitude(0.500002);

    auto* geo_pt_2 = link2->mutable_points()->Add();
    geo_pt_2->set_longitude(2.0);
    geo_pt_2->set_latitude(0.500003);
  }
  return sd_route;
}

TEST(NoaMainLoopTest, NoaIncrementalUpdateLoop) {
  Vec2d pos = {2.0, 0.500001};
  const CoordinateConverter cc(pos);
  auto smooth_pos = cc.GlobalToSmooth(pos);
  PoseProto pose_proto;
  pose_proto.mutable_pos_smooth()->set_x(smooth_pos.x());
  pose_proto.mutable_pos_smooth()->set_y(smooth_pos.y());
  pose_proto.set_yaw(/*yaw=*/1.6);

  RoutingInput routing_input = {
      .pose = std::make_shared<PoseProto>(),
      .localization_transform = std::make_shared<LocalizationTransformProto>(
          cc.localization_transform()),
      .sd_route_proto =
          std::make_shared<SDRouteProto>(CreateTestSdRouteProto()),
  };
  RouteManagerOutputProto route_mgr_output_proto;
  ExternalRouteManager external_route_manager;
  IncrementalNoaOutput output{
      .external_route_manager = &external_route_manager,
      .rms = external_route_manager.mutable_route_manager_state(),
      .route_mgr_output_proto = &route_mgr_output_proto,
  };
  const auto noa_status = NoaIncrementalUpdateLoop(
      routing_input,
      /*smm_listener=*/nullptr, RouteParamProto(),
      NoaStartupConfig{
          .route_use_sdroute = true,
      },
      /*av_global2d=*/pos, /*heading=*/std::nullopt, RestrictProto(), &output);
  ASSERT_NOT_OK(noa_status.route_nav_status);
  ASSERT_OK(noa_status.sd_route_status);
  ASSERT_NE(output.route_mgr_output_proto, nullptr);
  ASSERT_EQ(output.route_mgr_output_proto->route_status(),
            RouteManagerOutputProto::INVALID);
  ASSERT_EQ(external_route_manager.route_manager_state()
                .route_content_proto.nav_mode(),
            NavMode::ON_SD_ROUTE);
  // Test Delete sd route
  {
    SDRouteProto del_sd_route;
    del_sd_route.set_route_id(100);
    del_sd_route.set_length(100);
    del_sd_route.set_status(SDRouteProto::DELETE);
    routing_input.sd_route_proto =
        std::make_shared<SDRouteProto>(std::move(del_sd_route));
    const auto del_noa_status =
        NoaIncrementalUpdateLoop(routing_input, nullptr, RouteParamProto(),
                                 NoaStartupConfig{
                                     .route_use_sdroute = true,
                                 },
                                 /*av_global2d=*/pos, /*heading=*/std::nullopt,
                                 RestrictProto(), &output);
    ASSERT_NOT_OK(del_noa_status.sd_route_status);
    ASSERT_FALSE(external_route_manager.has_route());
  }
}

}  // namespace
}  // namespace qcraft::planner::route::noa

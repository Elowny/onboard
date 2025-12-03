#include "onboard/planner/router/task/route_reset_action.h"

#include <memory>

#include "gtest/gtest.h"

#include "onboard/planner/router/proto/route_manager_output.pb.h"
#include "onboard/planner/router/route_manager_state.h"
#include "onboard/proto/hmi_content.pb.h"
#include "onboard/proto/route.pb.h"

namespace qcraft::planner::route {
namespace {
TEST(RouteResetAction, ResetProtoContent) {
  RouteResetAction route_reset_action(nullptr);
  RouteManagerState rms;
  rms.request_id = 1234;

  RouteManagerOutputProto last_route_mgr_output_proto;
  last_route_mgr_output_proto.set_reset(false);
  last_route_mgr_output_proto.set_update_id(0);
  last_route_mgr_output_proto.mutable_route_sections_from_current()
      ->set_start_fraction(0.5);
  last_route_mgr_output_proto.mutable_route_sections_from_current()
      ->add_section_id(1);
  route_reset_action.ResetProtoContent(rms, &last_route_mgr_output_proto);
  ASSERT_EQ(last_route_mgr_output_proto.update_id(), rms.request_id);
  ASSERT_TRUE(last_route_mgr_output_proto.route_status() ==
              RouteManagerOutputProto::RESET);
  ASSERT_EQ(last_route_mgr_output_proto.route_sections_from_current()
                .section_id_size(),
            1);
  ASSERT_EQ(
      last_route_mgr_output_proto.route_sections_from_current().section_id(0),
      1);
  const HmiContentProto& hmi = route_reset_action.hmi_content_proto();
  ASSERT_EQ(hmi.route_content().routing_request_id(), "");
  ASSERT_TRUE(route_reset_action.routing_result_proto().success());
  ASSERT_EQ(route_reset_action.routing_result_proto().update_id(),
            rms.request_id);
  route_reset_action.HandleAction(rms,
                                  &last_route_mgr_output_proto);  // No crash.
  route_reset_action.set_max_keep_publish_times(100);
  ASSERT_EQ(route_reset_action.max_keep_publish_times(), 100);
  route_reset_action.set_max_keep_publish_times(0);
  route_reset_action.ResetProtoContent(rms, &last_route_mgr_output_proto);
}
}  // namespace
}  // namespace qcraft::planner::route

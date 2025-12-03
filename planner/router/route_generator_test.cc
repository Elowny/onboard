#include "onboard/planner/router/route_generator.h"

#include <memory>
#include <optional>
#include <utility>

#include "absl/hash/hash.h"
#include "absl/status/statusor.h"
#include "google/protobuf/repeated_ptr_field.h"

#include "gtest/gtest.h"

#include "common/proto/drive_mission.pb.h"
#include "common/proto/map_geometry.pb.h"

#include "onboard/base/macros.h"
#include "onboard/container/strong_int.h"
#include "onboard/maps/proto/semantic_map.pb.h"
#include "onboard/maps/semantic_map_defs.h"
#include "onboard/maps/smm_proto_util.h"
#include "onboard/maps/v2/semantic_map_manager.h"
#include "onboard/math/geometry/proto/affine_transformation.pb.h"
#include "onboard/planner/router/proto/route_params.pb.h"
#include "onboard/planner/router/request/routing_request_context.h"
#include "onboard/planner/router/route_core.h"
#include "onboard/planner/router/route_test_util.h"
#include "onboard/planner/router/route_util.h"
#include "onboard/planner/router/router_flags.h"
#include "onboard/proto/positioning.pb.h"
#include "onboard/proto/route.pb.h"
#include "onboard/utils/file_util.h"

namespace qcraft::planner::route {
namespace {
TEST(RouteGeneratorTest, InitRoute) {
  FLAGS_router_enable_off_road_search = true;
  const auto& smm = LoadDojoMap();
  const auto* map_index = LoadDojoIndex();
  RouteParamProto route_params;
  file_util::TextFileToProto(FLAGS_route_default_params_file, &route_params);
  RouteCore route_core(&smm, &route_params);
  RoutingRequestContext routing_request_context = {
      .semantic_map_manager = &smm,
      .map_index = map_index,
      .route_param_proto = &route_params,
      .request_id = 1,
      .pred_trajectory_point = std::nullopt,
  };

  auto cc = CoordinateConverter::FromMap("dojo");
  PlannerRoutingRequestProto::InitRouteRequest init_request;
  RoutingRequestProto* routing_request = init_request.mutable_routing_request();

  const auto* lane_proto =
      mapping::FindLaneProto(smm, mapping::ElementId(3834));
  auto xyz = planner::ComputePointOnLane(lane_proto->polyline().points(),
                                         /*fraction=*/0.5);
  auto* smooth = init_request.mutable_pose()->mutable_pos_smooth();
  smooth->set_x(xyz.x());
  smooth->set_y(xyz.y());
  RoutingDestinationProto dest1;
  auto* offroad = dest1.mutable_off_road();
  auto* spot = offroad->mutable_parking_spot();
  spot->add_specified_parking_spot_ids(4042);
  routing_request->mutable_destinations()->Add(std::move(dest1));
  ASSERT_EQ(routing_request->destinations_size(), 1);
  ASSERT_TRUE(HasOnlyOneOffRoadRequest(*routing_request));
  auto result =
      InitRoute(routing_request_context, cc, /*RouteRestrictDistrict*/ {},
                init_request, /*is_bus=*/false, &route_core);
  ASSERT_OK(result);
  ASSERT_TRUE(result->success());
}
}  // namespace
}  // namespace qcraft::planner::route

#include <string>
#include <utility>

#include "google/protobuf/text_format.h"
#include "gtest/gtest.h"
#include "onboard/planner/router/proto/route_params.pb.h"
#include "onboard/utils/proto_util.h"

namespace qcraft::planner {
namespace {
TEST(RouteParamTest, WriteFile) {
  RouteParamProto src;

  RouteMatchFilter match_filter;
  RouteMatchSort match_sorter;
  match_filter.set_radius_error(5.0);
  match_filter.set_heading_error(1.4);
  match_filter.set_motoway_only(true);
  match_sorter.set_dist_weight(1.0);
  match_sorter.set_heading_weight(0.7);

  RouteMapMatchParam map_match_param;
  *map_match_param.mutable_filter() = std::move(match_filter);
  *map_match_param.mutable_sorter() = std::move(match_sorter);
  map_match_param.set_match_type(RouteMatchType::STATIONARY);
  *src.mutable_poi_match_param() = map_match_param;

  map_match_param.mutable_filter()->set_heading_error(3.1415 / 4);
  map_match_param.mutable_sorter()->set_dist_weight(1.0);
  map_match_param.mutable_sorter()->set_heading_weight(0.2);
  map_match_param.set_match_type(RouteMatchType::DRIVING);

  src.mutable_cost_param()->mutable_turn_cost()->set_right_turn(10.0);
  src.mutable_cost_param()->mutable_turn_cost()->set_infeasible_u_turn(1000.0);
  src.set_default_lane_width(3.5);

  *src.mutable_on_driving_param() = map_match_param;

  std::string content;
  google::protobuf::TextFormat::PrintToString(src, &content);

  RouteParamProto target;
  google::protobuf::TextFormat::ParseFromString(content, &target);

  ASSERT_TRUE(ProtoEquals(src, target));
}
}  // namespace
}  // namespace qcraft::planner

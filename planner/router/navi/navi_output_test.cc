#include "onboard/planner/router/navi/navi_output.h"

#include <algorithm>

#include "gtest/gtest.h"

#include "onboard/base/macros.h"
#include "onboard/planner/router/route_test_util.h"
#include "onboard/utils/file_util.h"

namespace qcraft::planner::route {

namespace {

TEST(NaviOutputTest, CreateNaviPreview) {
  const auto& semantic_map_manager = LoadDojoMap();
  {
    const auto clp_str = R"(
    lane_paths {
    lane_ids: 3
    lane_ids: 950
    lane_ids: 35
    lane_ids: 2472
    lane_ids: 54
    lane_ids: 168
    lane_ids: 129
    start_fraction: 0.5
    end_fraction: 0.5
    lane_path_in_forward_direction: true
    })";
    CompositeLanePathProto proto;
    ASSERT_TRUE(file_util::StringToProto(clp_str, &proto));

    const auto navi_output_or =
        CreateNaviOutputFromRoute(semantic_map_manager, proto, 0);
    EXPECT_OK(navi_output_or);
  }

  {
    const auto clp_str = R"(
    lane_paths {
      lane_ids: 3
      start_fraction: 0.1
      end_fraction: 0.9
      lane_path_in_forward_direction: true
    })";
    CompositeLanePathProto proto;
    ASSERT_TRUE(file_util::StringToProto(clp_str, &proto));
    const auto navi_output_or =
        CreateNaviOutputFromRoute(semantic_map_manager, proto, 0);
    EXPECT_OK(navi_output_or);
    ASSERT_EQ(navi_output_or->current_navi_instruction.navi_action()
                  .navi_action_type(),
              NaviActionProto::DESTINATION);
  }
}
}  // namespace

}  // namespace qcraft::planner::route

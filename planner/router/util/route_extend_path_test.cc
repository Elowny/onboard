#include "onboard/planner/router/util/route_extend_path.h"

#include <iterator>
#include <memory>

#include "gtest/gtest.h"

#include "onboard/maps/semantic_map_defs.h"
#include "onboard/planner/router/route_test_util.h"
namespace qcraft::planner::route {

namespace {

TEST(RouteExtendPath, FindConnectedLanePathWithinTest) {
  const mapping::LanePoint origin = {mapping::ElementId(2448), 0.5};
  const mapping::LanePoint destination = {mapping::ElementId(1), 0.3};

  // 1: OK: two ids connected directly
  auto lane_path_or =
      FindConnectedLanePathWithin(LoadDojoMap(), origin, destination);
  EXPECT_EQ(lane_path_or->lane_ids().size(), 2);
  EXPECT_EQ(mapping::ElementId(*lane_path_or->lane_ids().begin()),
            origin.lane_id());
  EXPECT_EQ(lane_path_or->start_fraction(), origin.fraction());
  EXPECT_EQ(mapping::ElementId(*lane_path_or->lane_ids().rbegin()),
            destination.lane_id());
  EXPECT_EQ(lane_path_or->end_fraction(), destination.fraction());

  // 2 : invalid: Two ids is not adjacent.
  lane_path_or = FindConnectedLanePathWithin(LoadDojoMap(), origin,
                                             {mapping::ElementId(2), 0.4});

  // 3: fraction is invalid.
  lane_path_or = FindConnectedLanePathWithin(LoadDojoMap(), origin,
                                             {origin.lane_id(), 0.4});
  EXPECT_TRUE(!lane_path_or.ok());

  // 4: multi lanes.
  lane_path_or = FindConnectedLanePathWithin(
      LoadDojoMap(), {mapping::ElementId(95), 0.85}, {origin.lane_id(), 0.3});
  EXPECT_OK(lane_path_or);
  EXPECT_EQ(lane_path_or->lane_ids().size(), 3);

  // 5: beyond length.
  lane_path_or =
      FindConnectedLanePathWithin(LoadDojoMap(), {mapping::ElementId(95), 0.85},
                                  {mapping::ElementId(34), 0.3});
  EXPECT_NOT_OK(lane_path_or);
}

}  // namespace

}  // namespace qcraft::planner::route

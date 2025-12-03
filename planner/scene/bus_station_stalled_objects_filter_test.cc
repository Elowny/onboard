#include "onboard/planner/scene/bus_station_stalled_objects_filter.h"

#include "gtest/gtest.h"

#include "onboard/maps/map_selector.h"
#include "onboard/planner/router/plot_util.h"
#include "onboard/planner/router/route_sections_util.h"
#include "onboard/planner/test_util/load_psmm_util.h"
#include "onboard/planner/test_util/route_builder.h"
#include "onboard/planner/test_util/util.h"
#include "onboard/proto/positioning.pb.h"

namespace qcraft {
namespace planner {
namespace {
TEST(BusStationStalledObjectsFilterTest, NormalTest) {
  SetMap("dojo");
  const auto& psmm = CreateDojoTestPSMM();
  const auto* smm = psmm.semantic_map_manager();
  const auto& cc = psmm.coordinate_converter();
  // Construct route sections.
  const PoseProto sdc_pose =
      CreatePose(/*timestamp=*/0.0, Vec2d(1038.831, -379.395),
                 /*heading=*/-1.56, Vec2d(5.0, 0.0));
  const auto route_path = RoutingToNameSpot(*smm, cc, sdc_pose, "zhandian");
  const auto route_sections =
      RouteSectionsFromCompositeLanePath(*smm, route_path);
  SendRouteSectionsAreaToCanvas(psmm, route_sections, "bus_route_section");
  BusStationStalledObjectsFilter filter(psmm, route_sections);

  // Vehicle is within stop area.
  EXPECT_TRUE(filter.IsFiltered(psmm, /*obj_pos=*/Vec2d(1035.2, -457.808),
                                /*obj_type=*/OT_VEHICLE));
  // Vehicle is within a range of 20.0m from stop area along lane path.
  EXPECT_TRUE(filter.IsFiltered(psmm, /*obj_pos=*/Vec2d(1038.776, -443.029),
                                /*obj_type=*/OT_LARGE_VEHICLE));
  // Vehicle is outside stop area.
  EXPECT_FALSE(filter.IsFiltered(psmm, /*obj_pos=*/Vec2d(1038.457, -467.874),
                                 /*obj_type=*/OT_LARGE_VEHICLE));
  // Barrier within stop area.
  EXPECT_FALSE(filter.IsFiltered(psmm, /*obj_pos=*/Vec2d(1035.2, -457.808),
                                 /*obj_type=*/OT_BARRIER));
}
}  // namespace
}  // namespace planner
}  // namespace qcraft

#include "onboard/planner/scheduler/smooth_reference_line_builder.h"

#include <optional>
#include <ostream>
#include <string>
#include <utility>

#include "gtest/gtest.h"

#include "onboard/base/macros.h"
#include "onboard/container/strong_int.h"
#include "onboard/global/buffered_logger.h"
#include "onboard/lite/check.h"
#include "onboard/maps/lane_path.h"
#include "onboard/maps/lane_point.h"
#include "onboard/maps/map_selector.h"
#include "onboard/math/vec.h"
#include "onboard/planner/common/plot_util.h"
#include "onboard/planner/composite_lane_path.h"
#include "onboard/planner/router/drive_passage_builder.h"
#include "onboard/planner/router/plot_util.h"
#include "onboard/planner/router/route_sections_util.h"
#include "onboard/planner/router/route_test_util.h"
#include "onboard/planner/scheduler/path_boundary_builder.h"
#include "onboard/planner/scheduler/smooth_reference_line_result.h"
#include "onboard/planner/test_util/load_psmm_util.h"
#include "onboard/utils/status_macros.h"
#include "onboard/vis/canvas/canvas.h"
#include "onboard/vis/canvas/proto/canvas_buffer.pb.h"
#include "onboard/vis/common/color.h"

#include "offboard/vis/vantage/vantage_server/vantage_canvas_client.h"

namespace qcraft::planner {
namespace {

void SendSmoothedResultToCanvas(
    const DrivePassage& drive_passage,
    const SmoothedReferenceCenterResult& smoothed_result,
    const std::string& channel) {
  QCHECK(!channel.empty()) << "empty canvas channel name!";
  auto& canvas = vis::vantage::GetCanvasClient()->GetCanvas(channel);
  canvas.SetGroundZero(1);
  for (int i = 0; i + 1 < drive_passage.size(); ++i) {
    const double relative_s =
        drive_passage.station(StationIndex(i)).accumulated_s() -
        drive_passage.front_s();
    const double next_relative_s =
        drive_passage.station(StationIndex(i + 1)).accumulated_s() -
        drive_passage.front_s();
    const auto lane_point =
        drive_passage.extend_lane_path().ArclengthToLanePoint(relative_s);
    const auto next_lane_point =
        drive_passage.extend_lane_path().ArclengthToLanePoint(next_relative_s);
    ASSIGN_OR_CONTINUE(const auto smoothed_l,
                       smoothed_result.GetSmoothedLateralOffset(lane_point));
    ASSIGN_OR_CONTINUE(
        const auto next_smoothed_l,
        smoothed_result.GetSmoothedLateralOffset(next_lane_point));
    const auto xy =
        drive_passage.station(StationIndex(i)).lat_point(smoothed_l);
    const auto next_xy =
        drive_passage.station(StationIndex(i + 1)).lat_point(next_smoothed_l);
    canvas.DrawLine(Vec3d(xy, 0.2), Vec3d(next_xy, 0.2),
                    vis::Color::kLightGreen, 3, vis::BorderStyleProto::DASHED);
  }
}

TEST(SmoothLanePathBoundedByPathBoundary, SmootherTest) {
  const TestRouteResult route_result = CreateALeftTurnRouteInDojo();
  EXPECT_TRUE(!route_result.route_lane_path.IsEmpty());

  const auto& psmm = CreateDojoTestPSMM();
  const auto backward_extended_lane_path =
      BackwardExtendLanePathOnRouteSections(
          psmm, route_result.route_sections,
          route_result.route_lane_path.lane_paths().front(),
          /*extend_len=*/10.0);
  EXPECT_OK(backward_extended_lane_path);
  const auto dp_or =
      BuildDrivePassage(psmm, /*vision_map_ptr=*/nullptr,
                        route_result.route_lane_path.lane_paths().front(),
                        *backward_extended_lane_path,
                        /*anchor_point=*/mapping::LanePoint(),
                        route_result.route_sections.planning_horizon(psmm),
                        route_result.route_sections.destination(),
                        /*all_lanes_virtual=*/false,
                        /*override_speed_limit_mps=*/std::nullopt);

  EXPECT_OK(dp_or);
  SendDrivePassageToCanvas(dp_or.value(), "left_turn_drive_passage");

  const auto path_bound_or =
      BuildPathBoundaryFromDrivePassage(psmm, dp_or.value());
  EXPECT_OK(path_bound_or);

  DrawPathSlBoundaryToCanvas(path_bound_or.value(), "left_turn_path_bound");

  const auto smoothed_line_or = SmoothLanePathBoundedByPathBoundary(
      psmm, dp_or.value(), path_bound_or.value(), {mapping::ElementId(193)},
      /*half_av_width=*/0.0);
  EXPECT_OK(smoothed_line_or);

  SendSmoothedResultToCanvas(dp_or.value(), smoothed_line_or.value(),
                             "smoothed_left_turn");
}

TEST(FindLanesToSmoothFromRoute, FindLaneIdsTest) {
  {
    const TestRouteResult route_result =
        CreateALeftTurnWithDirectionInfoRouteInDojo();
    EXPECT_TRUE(!route_result.route_lane_path.IsEmpty());

    const auto& psmm = CreateDojoTestPSMM();

    const auto lane_ids_vec =
        FindLanesToSmoothFromRoute(psmm, route_result.route_sections);
    EXPECT_TRUE(lane_ids_vec.ok());
    EXPECT_EQ(lane_ids_vec->size(), 1);
    for (const auto& lane_ids : *lane_ids_vec) {
      EXPECT_EQ(lane_ids.size(), 1);
      for (const auto& id : lane_ids) {
        EXPECT_EQ(id, mapping::ElementId(7814));
      }
    }
  }

  {
    const TestRouteResult route_result =
        CreateARightTurnWithDirectionInfoRouteInDojo();
    EXPECT_TRUE(!route_result.route_lane_path.IsEmpty());

    const auto& psmm = CreateDojoTestPSMM();

    const auto lane_ids_vec =
        FindLanesToSmoothFromRoute(psmm, route_result.route_sections);

    EXPECT_TRUE(lane_ids_vec.ok());
    EXPECT_EQ(lane_ids_vec->size(), 2);
    for (const auto& lane_ids : *lane_ids_vec) {
      EXPECT_EQ(lane_ids.size(), 1);
      for (const auto& id : lane_ids) {
        EXPECT_TRUE(id == mapping::ElementId(7807) ||
                    id == mapping::ElementId(7806));
      }
    }
  }
}

TEST(BuildSmoothedResultMapFromRouteSections, UpdateSmoothResultTest) {
  SetMap("dojo");
  const auto& psmm = CreateDojoTestPSMM();

  SmoothedReferenceLineResultMap results;

  const mapping::ElementId id(7814);
  const std::vector<mapping::ElementId> lane_ids = {id};
  const auto smoothed_line_or =
      SmoothLanePathByLaneIds(psmm, lane_ids, /*half_av_width=*/0.0);

  EXPECT_OK(smoothed_line_or);
  EXPECT_TRUE(!results.Contains(lane_ids));
  results.AddResult(lane_ids, *smoothed_line_or);
  EXPECT_TRUE(results.Contains(lane_ids));

  const TestRouteResult route_result =
      CreateARightTurnWithDirectionInfoRouteInDojo();
  EXPECT_TRUE(!route_result.route_lane_path.IsEmpty());

  const auto new_results = BuildSmoothedResultMapFromRouteSections(
      psmm, route_result.route_sections,
      /*half_av_width=*/0.0, std::move(results));
  EXPECT_OK(new_results);
  EXPECT_TRUE(!new_results->Contains(lane_ids));
  EXPECT_TRUE(new_results->Contains({mapping::ElementId(7806)}));
  EXPECT_TRUE(new_results->Contains({mapping::ElementId(7807)}));
}

}  // namespace
}  // namespace qcraft::planner

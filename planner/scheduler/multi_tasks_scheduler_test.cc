#include "onboard/planner/scheduler/multi_tasks_scheduler.h"

#include <iterator>
#include <optional>
#include <tuple>
#include <utility>
#include <vector>

#include "absl/hash/hash.h"
#include "google/protobuf/repeated_ptr_field.h"

#include "gtest/gtest.h"

#include "onboard/base/macros.h"
#include "onboard/container/strong_int.h"
#include "onboard/maps/lane_point.h"
#include "onboard/maps/proto/lane_path.pb.h"
#include "onboard/maps/semantic_map_defs.h"
#include "onboard/planner/common/path_sl_boundary.h"
#include "onboard/planner/decision/traffic_light/tl_info.h"
#include "onboard/planner/object/planner_object_manager.h"
#include "onboard/planner/router/drive_passage_builder.h"
#include "onboard/planner/router/navi/route_navi_info_builder.h"
#include "onboard/planner/router/route_sections.h"
#include "onboard/planner/router/route_sections_util.h"
#include "onboard/planner/scheduler/lane_graph/lane_graph_builder.h"
#include "onboard/planner/scheduler/lane_graph/lane_path_finder.h"
#include "onboard/planner/scheduler/proto/scheduler.pb.h"
#include "onboard/planner/scheduler/scheduler_util.h"
#include "onboard/planner/scheduler/smooth_reference_line_builder.h"
#include "onboard/planner/test_util/load_psmm_util.h"
#include "onboard/planner/test_util/util.h"

namespace qcraft::planner {

namespace {

TEST(SchedulerTest, MakeSchedulerOutput) {
  const auto& psmm = CreateDojoTestPSMM();
  const auto* smm = psmm.semantic_map_manager();

  const mapping::LanePoint destination(mapping::ElementId(7076), 0.6);
  const RouteSections route_sections(
      0.3, 0.6,
      {mapping::SectionId(12574), mapping::SectionId(12573),
       mapping::SectionId(12623), mapping::SectionId(12587)},
      destination);
  const RouteSectionsInfo sections_info(psmm, &route_sections);

  const mapping::LanePath lane_path(
      smm,
      {mapping::ElementId(6786), mapping::ElementId(7141),
       mapping::ElementId(7128), mapping::ElementId(7082)},
      0.3, 0.6);

  const auto backward_extended_lane_path =
      *BackwardExtendLanePathOnRouteSections(psmm, route_sections, lane_path,
                                             /*extend_len=*/10.0);

  auto passage = *BuildDrivePassage(
      psmm, /*vision_map_ptr=*/nullptr, lane_path, backward_extended_lane_path,
      /*anchor_point=*/mapping::LanePoint(),
      route_sections.planning_horizon(psmm), route_sections.destination(),
      /*all_lanes_virtual=*/false, /*override_speed_limit_mps=*/std::nullopt);

  const auto route_navi_info = *CalcNaviInfoByLaneGraph(
      *smm, sections_info, /*avoid_lanes=*/{}, /*preview_dist=*/1000.0);

  const auto lane_graph = BuildLaneGraph(
      psmm, sections_info, PlannerObjectManager(),
      /*stalled_objects=*/{}, /*avoid_lanes=*/{}, route_navi_info);
  const auto lp_infos =
      *FindBestLanePathsFromStart(psmm, sections_info, route_navi_info,
                                  lane_graph, /*thread_pool=*/nullptr);
  const LanePathInfo* target_lp_info = nullptr;
  for (const auto& lp_info : lp_infos) {
    if (lp_info.start_lane_id() == lane_path.front().lane_id()) {
      target_lp_info = &lp_info;
      break;
    }
  }
  ASSERT_NE(target_lp_info, nullptr);

  const auto vehicle_geom = DefaultVehicleGeometry();
  const auto smooth_result_map_or = BuildSmoothedResultMapFromRouteSections(
      psmm, route_sections, vehicle_geom.width() * 0.5,
      SmoothedReferenceLineResultMap());
  ASSERT_OK(smooth_result_map_or);
  const auto& smooth_result_map = *smooth_result_map_or;

  const mapping::LanePath prev_target_lane_path_from_start(
      smm, {mapping::ElementId(6785), mapping::ElementId(7139)}, 0.3, 1.0);

  ApolloTrajectoryPointProto plan_start_point;
  plan_start_point.mutable_path_point()->set_x(665.2);
  plan_start_point.mutable_path_point()->set_y(-465.4);
  plan_start_point.mutable_path_point()->set_theta(0.0);
  plan_start_point.set_v(10.0);

  const bool should_smooth = ShouldSmoothRefLane(TrafficLightInfoMap(), passage,
                                                 /*prev_smooth_state=*/false);

  const auto scheduler_output_or = MakeSchedulerOutput(
      psmm, sections_info, lp_infos, std::move(passage), *target_lp_info,
      vehicle_geom, SpacetimeTrajectoryManager(), plan_start_point,
      smooth_result_map, prev_target_lane_path_from_start,
      /*prev_lane_path_before_lc_from_start=*/mapping::LanePath(),
      MakeNoneLaneChangeState(), route_navi_info, /*borrow=*/false,
      should_smooth, /*planner_is_l4_mode=*/true,
      AutonomyStateProto::AUTO_DRIVE);
  ASSERT_OK(scheduler_output_or);
  const auto& scheduler_output = *scheduler_output_or;

  const auto hash = scheduler_output.Hash();
  EXPECT_EQ(std::get<0>(hash), mapping::ElementId(6786));
  EXPECT_FALSE(std::get<1>(hash));
  EXPECT_FALSE(std::get<2>(hash));

  SchedulerOutputProto proto;
  ToSchedulerOutputProto(scheduler_output, &proto);
  EXPECT_EQ(proto.target_lane_path().lane_ids().size(),
            scheduler_output.drive_passage.lane_path().size());
  EXPECT_EQ(*proto.backward_extended_lane_path().lane_ids().begin(),
            scheduler_output.drive_passage.extend_lane_path()
                .front()
                .lane_id()
                .value());
  EXPECT_EQ(
      *proto.backward_extended_lane_path().lane_ids().rbegin(),
      scheduler_output.drive_passage.lane_path().back().lane_id().value());
  EXPECT_EQ(scheduler_output.sl_boundary.size(),
            proto.path_boundary().reference_center().size());
  EXPECT_EQ(proto.lc_state().stage(),
            scheduler_output.lane_change_state.stage());
  EXPECT_EQ(proto.length_along_route(), scheduler_output.length_along_route);
  EXPECT_EQ(proto.borrow_lane(), scheduler_output.borrow_lane);
  EXPECT_EQ(proto.max_reach_length(), scheduler_output.max_reach_length);
  EXPECT_EQ(proto.should_smooth(), scheduler_output.should_smooth);
  EXPECT_EQ(proto.request_help_lane_change_by_route(),
            scheduler_output.request_help_lane_change_by_route);
  EXPECT_EQ(proto.standard_congestion_factor(),
            scheduler_output.standard_congestion_factor);
  EXPECT_EQ(proto.traffic_congestion_factor(),
            scheduler_output.traffic_congestion_factor);
}

}  // namespace
}  // namespace qcraft::planner

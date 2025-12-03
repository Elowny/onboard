#include "onboard/planner/util/hmi_content_util.h"

#include <utility>

#include "absl/status/statusor.h"
#include "google/protobuf/repeated_ptr_field.h"

#include "gtest/gtest.h"

#include "onboard/container/strong_int.h"
#include "onboard/global/buffered_logger.h"
#include "onboard/lite/check.h"
#include "onboard/maps/map_selector.h"
#include "onboard/maps/semantic_map_defs.h"
#include "onboard/math/vec.h"
#include "onboard/planner/router/drive_passage_builder.h"
#include "onboard/planner/test_util/load_psmm_util.h"

namespace qcraft::planner {
namespace {
std::vector<ApolloTrajectoryPointProto> CreateTrajectoryByStartAndEndPoint(
    const Vec2d& start_point, const Vec2d& end_point, int size) {
  QCHECK_GE(size, 2);
  std::vector<ApolloTrajectoryPointProto> traj_points;
  const double step = end_point.DistanceTo(start_point) / (size - 1);
  const auto unit = (end_point - start_point).Unit();
  for (int i = 0; i < size; ++i) {
    const auto pt = start_point + i * step * unit;
    ApolloTrajectoryPointProto traj_point;
    traj_point.mutable_path_point()->set_x(pt.x());
    traj_point.mutable_path_point()->set_y(pt.y());
    traj_points.push_back(std::move(traj_point));
  }
  return traj_points;
}

TEST(HmiContentUtilTest, ReportHmiContent) {
  // Load planner semantic map.
  SetMap("dojo");
  const auto& psmm = CreateDojoTestPSMM();

  // Borrow opposite lane test.
  {
    const auto lane_path =
        mapping::LanePath(psmm.semantic_map_manager(), /*lane_ids=*/
                          {mapping::ElementId(2), mapping::ElementId(938)},
                          /*start_fraction=*/0.0, /*end_fraction=*/1.0);
    const auto drive_passage = *BuildDrivePassageFromLanePath(
        psmm, lane_path, /*step_s=*/1.0,
        /*avoid_loop=*/true, /*backward_extend_len=*/0.0,
        /*required_planning_horizon=*/0.0,
        /*required_backward_len=*/0.0,
        /*override_speed_limit_mps=*/std::nullopt);

    // Enter left opposite lane test.
    {
      const auto traj_points = CreateTrajectoryByStartAndEndPoint(
          /*start_point=*/Vec2d(4.0, 3.5), /*end_point=*/Vec2d(44.0, 7.0),
          /*size=*/10);
      const auto hmi_proto = ReportHmiContent(HmiContentInput{
          &psmm, &lane_path, &drive_passage, &traj_points,
          /*alerted_front_vehicle_ptr==*/nullptr,
          /*collision_warning_request=*/false,
          /*lane_change_type=*/LaneChangeType::NO_CHANGE,
          /*lane_change_general_type=*/LaneChangeGeneralType::LCGT_NO_CHANGE,
          /*lane_change_stage=*/LaneChangeStage::LCS_NONE,
          /*borrow_lane=*/true,
          /*request_help_lane_change_by_route=*/false,
          /*distance_to_traffic_light_stop_line=*/std::nullopt,
          /*distance_to_roadblock=*/std::nullopt,
          /*unsafe_object_ids=*/nullptr,
          /*plc_target_lp=*/nullptr});
      EXPECT_EQ(hmi_proto.driving_states_size(), 1);
      EXPECT_EQ(hmi_proto.driving_states(0),
                HmiContentProto::DS_BORROW_OPPOSITE_LANE);
    }

    // All trajectory points are located in left opposite lane test.
    {
      const auto traj_points = CreateTrajectoryByStartAndEndPoint(
          /*start_point=*/Vec2d(5.0, 7.0), /*end_point=*/Vec2d(44.0, 7.0),
          /*size=*/10);
      const double distance_to_roadblock = 50.0;
      const mapping::LanePath plc_target_lane_path;
      const auto hmi_proto = ReportHmiContent(HmiContentInput{
          &psmm, &lane_path, &drive_passage, &traj_points,
          /*alerted_front_vehicle_ptr=*/nullptr,
          /*collision_warning_request=*/false,
          /*lane_change_type=*/LaneChangeType::NO_CHANGE,
          /*lane_change_general_type=*/LaneChangeGeneralType::LCGT_NO_CHANGE,
          /*lane_change_stage=*/LaneChangeStage::LCS_NONE,
          /*borrow_lane=*/true,
          /*request_help_lane_change_by_route=*/false,
          /*distance_to_traffic_light_stop_line=*/std::nullopt,
          /*distance_to_roadblock=*/distance_to_roadblock,
          /*unsafe_object_ids=*/nullptr,
          /*plc_target_lp=*/&plc_target_lane_path});
      EXPECT_EQ(hmi_proto.driving_states_size(), 1);
      EXPECT_EQ(hmi_proto.driving_states(0),
                HmiContentProto::DS_BORROW_OPPOSITE_LANE);
      EXPECT_FALSE(hmi_proto.has_plc_target_lane_path());
      EXPECT_NEAR(hmi_proto.distance_to_roadblock(), distance_to_roadblock,
                  1e-1);
    }

    // Leave opposite lane test.
    {
      const auto traj_points = CreateTrajectoryByStartAndEndPoint(
          /*start_point=*/Vec2d(5.0, 7.0), /*end_point=*/Vec2d(44.0, 3.5),
          /*size=*/10);
      const std::string alerted_front_vehicle{"123"};
      const std::vector<std::string> unsafe_object_ids{"456", "789"};
      const mapping::LanePath plc_target_lane_path = lane_path;
      const auto hmi_proto = ReportHmiContent(HmiContentInput{
          &psmm, &lane_path, &drive_passage, &traj_points,
          /*alerted_front_vehicle_ptr=*/&alerted_front_vehicle,
          /*collision_warning_request=*/false,
          /*lane_change_type=*/LaneChangeType::NO_CHANGE,
          /*lane_change_general_type=*/LaneChangeGeneralType::LCGT_NO_CHANGE,
          /*lane_change_stage=*/LaneChangeStage::LCS_NONE,
          /*borrow_lane=*/true,
          /*request_help_lane_change_by_route=*/false,
          /*distance_to_traffic_light_stop_line=*/std::nullopt,
          /*distance_to_roadblock=*/std::nullopt,
          /*unsafe_object_ids=*/&unsafe_object_ids,
          /*plc_target_lp=*/&plc_target_lane_path});
      EXPECT_EQ(hmi_proto.driving_states_size(), 1);
      EXPECT_EQ(hmi_proto.driving_states(0),
                HmiContentProto::DS_BORROW_OPPOSITE_LANE);
      EXPECT_EQ(hmi_proto.highlight_objects_size(), 3);
      EXPECT_EQ(hmi_proto.highlight_objects()[0].object_id(), "123");
      EXPECT_EQ(hmi_proto.highlight_objects()[0].type(),
                HmiContentProto::HighlightObjectInfo::HLO_ACC_FOLLOWING_TARGET);
      EXPECT_EQ(hmi_proto.highlight_objects()[1].object_id(), "456");
      EXPECT_EQ(hmi_proto.highlight_objects()[1].type(),
                HmiContentProto::HighlightObjectInfo::HLO_LANE_CHANGE_UNSAFE);
      EXPECT_TRUE(hmi_proto.has_plc_target_lane_path());
    }
  }
}

TEST(HmiContentUtilTest, ReportBoundaryPointsToHmiContent) {
  constexpr int kSize = 200;
  std::vector<Vec2d> points;
  points.push_back({0.0, 0.0});
  for (int i = 1; i < kSize; ++i) {
    points.push_back(points.back() + Vec2d(2.0, 0.0));
  }

  HmiContentProto hmi_content;
  *hmi_content.mutable_path_boundary() =
      ReportBoundaryPointsToHmiContent(points, /*is_left=*/true,
                                       /*style=*/
                                       HmiPathBoundaryProto::STYLE_WARN);
  EXPECT_TRUE(hmi_content.has_path_boundary());
  EXPECT_EQ(hmi_content.path_boundary().left_boundary_size(), kSize / 10 + 1);
  EXPECT_TRUE(hmi_content.path_boundary().right_boundary().empty());
  EXPECT_EQ(hmi_content.path_boundary().left_render_style(),
            HmiPathBoundaryProto::STYLE_WARN);
  EXPECT_FALSE(hmi_content.path_boundary().has_right_render_style());
}

PathSlBoundary MockPathSlBoundary() {
  constexpr int kSize = 401;
  std::vector<double> s_vec, center_l, left_l, right_l, target_left_l,
      target_right_l;
  s_vec.reserve(kSize);
  center_l.reserve(kSize);
  left_l.reserve(kSize);
  right_l.reserve(kSize);

  std::vector<Vec2d> center_xy, left_xy, right_xy, target_left_xy,
      target_right_xy;
  center_xy.reserve(kSize);
  left_xy.reserve(kSize);
  right_xy.reserve(kSize);

  constexpr double kStep = 1.0;
  for (int i = 0; i < kSize; ++i) {
    const double s = i * kStep;
    s_vec.emplace_back(s);
    center_l.emplace_back(0.0);
    left_l.emplace_back(2.0);
    right_l.emplace_back(-2.0);
    center_xy.emplace_back(s, 0.0);
    left_xy.emplace_back(s, 2.0);
    right_xy.emplace_back(s, -2.0);
  }

  target_left_l = left_l;
  target_right_l = right_l;
  target_left_xy = left_xy;
  target_right_xy = right_xy;

  return PathSlBoundary(
      std::move(s_vec), std::move(center_l), std::move(right_l),
      std::move(left_l), std::move(target_right_l), std::move(target_left_l),
      std::move(center_xy), std::move(right_xy), std::move(left_xy),
      std::move(target_right_xy), std::move(target_left_xy));
}

TEST(HmiContentUtilTest, ReportPathBoundaryToHmiContent) {
  const auto sl_boundary = MockPathSlBoundary();

  HmiContentProto hmi_content;
  *hmi_content.mutable_path_boundary() =
      ReportPathBoundaryToHmiContent(&sl_boundary, /*left_style=*/
                                     HmiPathBoundaryProto::STYLE_NORMAL,
                                     /*right_style=*/
                                     HmiPathBoundaryProto::STYLE_WARN);
  EXPECT_TRUE(hmi_content.has_path_boundary());
  EXPECT_EQ(hmi_content.path_boundary().left_boundary_size(), 7);
  EXPECT_EQ(hmi_content.path_boundary().right_boundary_size(), 7);
  EXPECT_EQ(hmi_content.path_boundary().left_render_style(),
            HmiPathBoundaryProto::STYLE_NORMAL);
  EXPECT_EQ(hmi_content.path_boundary().right_render_style(),
            HmiPathBoundaryProto::STYLE_WARN);
}

}  // namespace
}  // namespace qcraft::planner

#include "onboard/planner/plan/acc/acc_corridor_with_map.h"

#include <cmath>
#include <memory>

#include "gtest/gtest.h"

#include "onboard/container/strong_int.h"
#include "onboard/global/buffered_logger.h"
#include "onboard/lite/check.h"
#include "onboard/maps/map_selector.h"
#include "onboard/math/geometry/proto/affine_transformation.pb.h"
#include "onboard/math/util.h"
#include "onboard/planner/common/path_generator_util.h"
#include "onboard/planner/composite_lane_path.h"
#include "onboard/planner/object/planner_object.h"
#include "onboard/planner/scheduler/path_boundary_builder.h"
#include "onboard/planner/test_util/load_psmm_util.h"
#include "onboard/planner/test_util/object_prediction_builder.h"
#include "onboard/planner/test_util/perception_object_builder.h"
#include "onboard/planner/test_util/planner_object_builder.h"
#include "onboard/planner/test_util/predicted_trajectory_builder.h"
#include "onboard/planner/test_util/route_builder.h"
#include "onboard/planner/test_util/util.h"
#include "onboard/planner/util/path_util.h"
#include "onboard/planner/util/planner_semantic_map_util.h"
#include "onboard/proto/perception/fusion/object.pb.h"

namespace qcraft {

namespace planner {
namespace {

class RunAccCorridorWithMapTest : public ::testing::Test {
 public:
  void SetUp() override {
    SetMap("dojo");
    psmm_ptr_ = CreateDojoTestPSMMSharedPtr();

    PoseProto pose;
    pose.mutable_pos_smooth()->set_x(0.0);
    pose.mutable_pos_smooth()->set_y(0.0);
    pose.set_yaw(0.0);
    pose.mutable_vel_body()->set_y(Kph2Mps(5));
    pose.set_curvature(1.0 / 100.0);

    const auto route_path =
        RoutingToNameSpot(*psmm_ptr_->semantic_map_manager(),
                          psmm_ptr_->coordinate_converter(), pose, "7b_n1");
    lane_paths_ = route_path.lane_paths();
    start_pt_.set_x(0.0);
    start_pt_.set_y(0.0);
    start_pt_.set_y(0.0);
    start_pt_.set_theta(0.0);
    const auto dp_opt = BuildDrivePassageFromLaneId(
        *psmm_ptr_, mapping::ElementId(2448), start_pt_,
        /*max_length=*/100.0);
    EXPECT_TRUE(dp_opt.has_value());
    dp_ = *dp_opt;

    // Build spacetime object manager.
    PerceptionObjectBuilder perception_builder;
    const auto perception_obj = perception_builder.set_id("abc")
                                    .set_type(OT_VEHICLE)
                                    .set_timestamp(1.0)
                                    .set_velocity(2.0)
                                    .set_yaw(0.0)
                                    .set_length_width(4.0, 2.0)
                                    .set_box_center(Vec2d(10.0, 2.0))
                                    .Build();
    PlannerObjectBuilder builder;
    builder.set_type(OT_VEHICLE)
        .set_object(perception_obj)
        .get_object_prediction_builder()
        ->add_predicted_trajectory()
        ->set_probability(1.0)
        .set_straight_line(Vec2d(10.0, 2.0), Vec2d(10.5, 2.0),
                           /*init_v=*/0.0, /*last_v=*/0.1);
    const PlannerObject object = builder.Build();
    st_traj_mgr_ = SpacetimeTrajectoryManager(absl::MakeSpan(&object, 1));
    frenet_frame_ = std::make_shared<QtfmEnhancedKdTreeFrenetFrame>(
        BuildQtfmEnhancedKdTreeFrenetFrame(
            SampleLanePathPoints(*psmm_ptr_, lane_paths_.front()),
            /*down_sample_raw_points=*/false)
            .value());
  }

 protected:
  std::shared_ptr<PlannerSemanticMapManager> psmm_ptr_;
  std::vector<mapping::LanePath> lane_paths_;
  PathPoint start_pt_;
  DrivePassage dp_;
  SpacetimeTrajectoryManager st_traj_mgr_;
  std::shared_ptr<QtfmEnhancedKdTreeFrenetFrame> frenet_frame_;
};
}  // namespace

TEST_F(RunAccCorridorWithMapTest,
       FindClosestLanePointToSmoothPointOnLaneTest0) {
  const Vec2d query_point = Vec2d(0.5, 15.0);
  const mapping::ElementId lane_id = mapping::ElementId(2448);
  std::optional<double> max_allow_distance;
  max_allow_distance = 50.0;
  mapping::LanePoint lane_point;
  Vec2d pos;
  Vec2d tangent;
  double signed_distance;
  const auto status = internal::FindClosestLanePointToSmoothPointOnLane(
      *psmm_ptr_, query_point, lane_id, max_allow_distance, &lane_point, &pos,
      &tangent, &signed_distance);
  QCHECK_EQ(status.ok(), true);
}

TEST_F(RunAccCorridorWithMapTest,
       FindClosestLanePointToSmoothPointOnLaneTest1) {
  const Vec2d query_point = Vec2d(0.5, 15.0);
  const mapping::ElementId lane_id = mapping::ElementId(5009);
  std::optional<double> max_allow_distance;
  max_allow_distance = 50.0;
  mapping::LanePoint lane_point;
  Vec2d pos;
  Vec2d tangent;
  double signed_distance;
  const auto status = internal::FindClosestLanePointToSmoothPointOnLane(
      *psmm_ptr_, query_point, lane_id, max_allow_distance, &lane_point, &pos,
      &tangent, &signed_distance);
  QCHECK_EQ(status.ok(), false);
}

TEST_F(RunAccCorridorWithMapTest, EstimateMinDrivePassageDivergeLengthTest0) {
  const auto dp_opt =
      BuildDrivePassageFromLaneId(*psmm_ptr_, mapping::ElementId(2), start_pt_,
                                  /*max_length=*/100.0);
  EXPECT_TRUE(dp_opt.has_value());

  const double length = internal::EstimateMinDrivePassageDivergeLength(
      dp_, *dp_opt, /*start_point=*/Vec2d(0.0, 0.0));
  QCHECK_GE(length, 0.0);
}

TEST_F(RunAccCorridorWithMapTest, EstimateMinDrivePassageDivergeLengthTest1) {
  const auto dp_opt = BuildDrivePassageFromLaneId(
      *psmm_ptr_, mapping::ElementId(2448), start_pt_,
      /*max_length=*/100.0);
  EXPECT_TRUE(dp_opt.has_value());

  const double length = internal::EstimateMinDrivePassageDivergeLength(
      dp_, *dp_opt, /*start_point=*/Vec2d(0.0, 0.0));
  QCHECK_GE(length, 0.0);
}

TEST_F(RunAccCorridorWithMapTest, ComputeDrivePassageCostTest) {
  std::vector<UniCycleState> probing_traj;
  constexpr int kTrajSize = 100;
  probing_traj.reserve(kTrajSize);
  for (int k = 0; k < kTrajSize; ++k) {
    UniCycleState state;
    state.x = static_cast<double>(k) * 1.0;
    state.y = 0.0;
    state.heading = 0.0;
    state.kappa = 0.0;
  }
  const double cost = internal::ComputeDrivePassageCost(dp_, probing_traj);
  QCHECK_GT(cost, 0.0);
}

TEST_F(RunAccCorridorWithMapTest, SimulateUniCycleStateTest) {
  UniCycleState state;
  state.x = 0.0;
  state.y = 0.0;
  state.heading = 0.02;
  state.kappa = 0.01;
  const UniCycleState res =
      SimulateUniCycleState(state, /*s=*/1.0, /*kappa_decay=*/0.95);
  QCHECK_GE(res.x, 0.0);
  QCHECK_GE(res.y, 0.0);
}

TEST_F(RunAccCorridorWithMapTest, GenerateProbingTrajsTest) {
  PoseProto pose;
  pose.mutable_pos_smooth()->set_x(2.444);
  pose.mutable_pos_smooth()->set_y(-1.185);
  pose.set_yaw(-0.40);
  pose.mutable_vel_body()->set_y(Kph2Mps(5));
  pose.set_curvature(1.0 / 100.0);
  const std::vector<UniCycleState> res =
      internal::GenerateProbingTrajs(pose, DefaultVehicleGeometry(),
                                     /*ds=*/0.1, /*desire_length=*/10.0);
  QCHECK_GE(res.size(), 5);
  double s = 0.0;
  for (int i = 0; i + 1 < res.size(); ++i) {
    const auto& p0 = res[i];
    const auto& p1 = res[i + 1];
    QCHECK_GE(std::abs(p0.kappa), std::abs(p1.kappa));
    s += std::hypot(p1.x - p0.x, p1.y - p0.y);
  }
  QCHECK_GE(s, 10.0);
}

TEST_F(RunAccCorridorWithMapTest, GetLaneBoundaryPointsAndWidthTest) {
  const Vec2d pos(2.0, 0.2);
  const Vec2d tangent(1.0, 0.0);
  Vec2d right, left;
  double width;
  internal::GetLaneBoundaryPointsAndWidth(*psmm_ptr_, mapping::ElementId(2448),
                                          pos, tangent, &right, &left, &width);
  QCHECK_GE(width, 0.0);
}

TEST_F(RunAccCorridorWithMapTest, BuildLaneBoxApproxByProjectionTest) {
  const Box2d box = internal::BuildLaneBoxApproxByProjection(
      *psmm_ptr_, mapping::ElementId(2448), mapping::ElementId(1),
      Vec2d(0.0, 0.0), Vec2d(50.0, 0.0), Vec2d(0.0, 0.0), Vec2d(0.0, 0.0));
  QCHECK_GE(box.area(), 0.0);
}

TEST_F(RunAccCorridorWithMapTest, BuildLaneBoxApproxTest) {
  const std::optional<Box2d> box = internal::BuildLaneBoxApprox(
      *psmm_ptr_, Vec2d(0.0, 0.0), Vec2d(50.0, 0.0), mapping::ElementId(2448));

  QCHECK_EQ(box.has_value(), true);
  QCHECK_GE(box->area(), 0.0);
}

TEST_F(RunAccCorridorWithMapTest, IsAVConvergeOrNearCenterTest0) {
  QCHECK_EQ(
      internal::IsAVConvergeOrNearCenter(/*lateral_v=*/0.05, /*current_l=*/0.1),
      false);
}

TEST_F(RunAccCorridorWithMapTest, IsAVConvergeOrNearCenterTest1) {
  QCHECK_EQ(
      internal::IsAVConvergeOrNearCenter(/*lateral_v=*/0.5, /*current_l=*/-1.0),
      true);
}

TEST_F(RunAccCorridorWithMapTest,
       CanFindClosestSlowObjectOnPathInRegionBoxTest) {
  const Box2d region_box(Vec2d(0.0, 0.0), /*heading=*/0.0, /*length=*/100.0,
                         /*width=*/20.0);
  Vec2d obj_closest_pt;
  QCHECK_EQ(internal::CanFindClosestSlowObjectOnPathInRegionBox(
                st_traj_mgr_.trajectories(), region_box, *frenet_frame_,
                /*av_max_s=*/10.0,
                /*low_speed_threshold=*/1.0, &obj_closest_pt),
            true);
}

TEST_F(RunAccCorridorWithMapTest, FindSlowCutinObjectsInRegionBoxTest) {
  const auto path_sl_or = BuildPathBoundaryFromDrivePassage(*psmm_ptr_, dp_);
  QCHECK_EQ(path_sl_or.ok(), true);
  const Box2d region_box(Vec2d(0.0, 0.0), /*heading=*/0.0, /*length=*/100.0,
                         /*width=*/20.0);
  const std::set<std::string> res = internal::FindSlowCutinObjectsInRegionBox(
      st_traj_mgr_.trajectories(), region_box, *path_sl_or, *frenet_frame_,
      /*low_speed_threshold=*/1.0);
  QCHECK_EQ(res.size(), 0);
  QCHECK(res.find("abc-0") == res.end());
}

TEST_F(RunAccCorridorWithMapTest, FindClosestLanePathAtPositionOrNullTest) {
  const std::pair<const mapping::LanePath*, mapping::ElementId> res =
      internal::FindClosestLanePathAtPositionOrNull(
          *psmm_ptr_, lane_paths_,
          /*query_pos=*/Vec2d(0.0, 0.0));
  QCHECK_NOTNULL(res.first);
  QCHECK_EQ(res.second, mapping::ElementId(2448));
}

TEST_F(RunAccCorridorWithMapTest, ComputeTimeLateralOffsetPlfFromPoseTest0) {
  PoseProto pose;
  pose.mutable_pos_smooth()->set_x(2.444);
  pose.mutable_pos_smooth()->set_y(-1.185);
  pose.set_yaw(-0.40);
  pose.mutable_vel_body()->set_y(Kph2Mps(5));
  pose.set_curvature(1.0 / 100.0);
  const absl::StatusOr<PiecewiseLinearFunction<double, double>> res =
      internal::ComputeTimeLateralOffsetPlfFromPose(dp_, pose);
  QCHECK_EQ(res.ok(), true);
}

TEST_F(RunAccCorridorWithMapTest, ComputeTimeLateralOffsetPlfFromPoseTest1) {
  PoseProto pose;
  pose.mutable_pos_smooth()->set_x(-20.444);
  pose.mutable_pos_smooth()->set_y(-1.185);
  pose.set_yaw(-0.40);
  pose.mutable_vel_body()->set_y(Kph2Mps(5));
  pose.set_curvature(1.0 / 100.0);
  const absl::StatusOr<PiecewiseLinearFunction<double, double>> res =
      internal::ComputeTimeLateralOffsetPlfFromPose(dp_, pose);
  QCHECK_EQ(res.ok(), false);
}

TEST_F(RunAccCorridorWithMapTest, ComputeCurvatureCostTest) {
  std::vector<PathPoint> path_points;
  constexpr int kCounts = 50;
  path_points.reserve(kCounts);
  PathPoint pt;
  pt.set_x(0.0);
  pt.set_y(0.0);
  pt.set_z(0.0);
  pt.set_theta(0.0);
  pt.set_s(0.0);
  pt.set_kappa(0.01);  // TMP.
  pt.set_lambda(0.0);  // TMP.
  path_points.push_back(pt);

  for (int idx = 0; idx < kCounts; ++idx) {
    path_points.push_back(
        GetPathPointAlongCircle(pt, static_cast<double>(idx) * 0.1));
  }
  const double cost =
      internal::ComputeCurvatureCost(path_points, /*ref_curvature=*/0.0,
                                     /*look_forward_length=*/2.0);
  QCHECK_GE(cost, 0.0);
}

TEST_F(RunAccCorridorWithMapTest, ComputePathLengthFromPositionTest) {
  std::vector<PathPoint> path_points;
  constexpr int kCounts = 50;
  path_points.reserve(kCounts);
  for (int idx = 0; idx < kCounts; ++idx) {
    PathPoint pt;
    pt.set_x(static_cast<double>(idx));
    pt.set_y(0.0);
    pt.set_z(0.0);
    pt.set_theta(0.0);
    pt.set_s(static_cast<double>(idx));
    pt.set_kappa(0.0);  // TMP.
    path_points.push_back(pt);
  }
  const DiscretizedPath path(path_points);
  const double length =
      internal::ComputePathLengthFromPosition(path, Vec2d(20.0, 0.0));
  QCHECK_GE(length, 0.0);
}

TEST_F(RunAccCorridorWithMapTest, SampleSmoothedPathOnLanePathTest) {
  const std::optional<DiscretizedPath> path =
      internal::SampleSmoothedPathOnLanePath(*psmm_ptr_, lane_paths_.front(),
                                             /*start_s=*/0.0,
                                             /*sample_step_s=*/1.0);
  QCHECK_EQ(path.has_value(), true);
}

TEST_F(RunAccCorridorWithMapTest, ExpandLaneIdsBySemanticTest) {
  const Vec2d pos(0.0, 0.0);
  const double heading = 0.0;
  constexpr double kMaxSearchRadiusForNearestLane = 5.0;          // m.
  constexpr double kFindParallelLaneMaxHeadingDiff = M_PI / 4.0;  // 45 deg.
  const auto lanes = psmm_ptr_->GetLanesInfoWithHeadingAtLevel(
      psmm_ptr_->GetLevel(), pos, heading, kMaxSearchRadiusForNearestLane,
      kFindParallelLaneMaxHeadingDiff);
  const auto all_lanes =
      internal::ExpandLaneIdsBySemantic(absl::MakeSpan(lanes));
  QCHECK_GE(all_lanes.size(), 0);
}

}  // namespace planner
}  // namespace qcraft

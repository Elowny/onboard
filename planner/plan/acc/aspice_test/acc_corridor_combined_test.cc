#include "onboard/planner/plan/acc/acc_corridor_combined.h"

#include <algorithm>
#include <cmath>
#include <memory>
#include <utility>

#include "gtest/gtest.h"

#include "onboard/container/strong_int.h"
#include "onboard/global/buffered_logger.h"
#include "onboard/lite/check.h"
#include "onboard/maps/map_selector.h"
#include "onboard/planner/common/driving_map_topo.h"
#include "onboard/planner/scheduler/driving_map_topo_builder.h"
#include "onboard/planner/test_util/load_psmm_util.h"

namespace qcraft {

namespace planner {
namespace {

class RunAccCorridorCombinedTest : public ::testing::Test {
 public:
  void SetUp() override {
    SetMap("dojo");
    psmm_ptr_ = CreateDojoTestPSMMSharedPtr();
    lane_id_ = mapping::ElementId(2180);
    lane_info_ = psmm_ptr_->FindLaneInfoOrNull(mapping::ElementId(2180));
    std::vector<Vec2d> points = lane_info_->points_smooth;
    for (int idx = 0; idx < points.size(); ++idx) {
      points[idx] += Vec2d(-10.0, 0.3);
    }
    frenet_frame_ = std::make_shared<QtfmEnhancedKdTreeFrenetFrame>(
        BuildQtfmEnhancedKdTreeFrenetFrame(lane_info_->points_smooth,
                                           /*down_sample_raw_points=*/false)
            .value());

    std::vector<PathPoint> path_points;
    path_points.reserve(points.size());
    double s = 0.0;
    for (int idx = 0; idx < points.size() - 1; ++idx) {
      PathPoint pt;
      pt.set_x(points[idx].x());
      pt.set_y(points[idx].y());
      pt.set_z(0.0);
      pt.set_theta((points[idx + 1] - points[idx]).Angle());
      pt.set_s(s);
      s += (points[idx + 1] - points[idx]).norm();
      pt.set_kappa(0.0);  // TMP.
      path_points.push_back(pt);
    }
    path_ = DiscretizedPath(path_points);
    credible_length_ = lane_info_->length();

    corridor_step_s_ = 0.5;

    pt_.set_x(points.front().x() + 10.0);
    pt_.set_y(points.front().y());
    pt_.set_z(0.0);
    pt_.set_theta(path_points.front().theta());
    pt_.set_kappa(0.0);
    pt_.set_s(20.0);

    // Build driving map topo.
    constexpr double kDrivingMapStartLaneSearchRadius = 10.0;
    constexpr double kDrivingMapStastLaneSearchMaxHeadingDiff = M_PI_4;
    auto dm_or = BuildDrivingMapAtPosAndHeadingInRadiusWithMaxLength(
        *psmm_ptr_, Vec2d(pt_.x(), pt_.y()), pt_.theta(),
        kDrivingMapStartLaneSearchRadius, credible_length_,
        kDrivingMapStastLaneSearchMaxHeadingDiff,
        /*allow_virtual=*/false);
    if (dm_or.ok()) {
      dm_ = std::make_unique<DrivingMapTopo>(std::move(*dm_or));
    }
  }

 protected:
  std::shared_ptr<PlannerSemanticMapManager> psmm_ptr_;
  mapping::ElementId lane_id_;
  const mapping::LaneInfo* lane_info_;
  std::shared_ptr<QtfmEnhancedKdTreeFrenetFrame> frenet_frame_;
  DiscretizedPath path_;
  double credible_length_;
  PathPoint pt_;
  double corridor_step_s_;
  std::unique_ptr<DrivingMapTopo> dm_;
};
}  // namespace

TEST_F(RunAccCorridorCombinedTest, FilterLanesTest) {
  const double av_heading = M_PI_2;
  std::vector<const mapping::LaneInfo*> lane_infos;
  for (const auto& lane_info : psmm_ptr_->lane_info()) {
    lane_infos.push_back(&lane_info);
  }
  const std::vector<const mapping::LaneInfo*> filtered_lanes =
      internal::FilterLanes(*psmm_ptr_, absl::MakeSpan(lane_infos),
                            *frenet_frame_, av_heading);
  QCHECK_GE(filtered_lanes.size(), 0);
  bool has_2180 = false;
  for (const auto& lane_ptr : filtered_lanes) {
    if (lane_ptr->id == lane_id_) {
      has_2180 = true;
      break;
    }
  }
  QCHECK(!has_2180);
}

TEST_F(RunAccCorridorCombinedTest, ComputeLaneCostTest) {
  const auto cost = internal::ComputeLaneCost(
      *frenet_frame_, lane_info_->points_smooth, credible_length_);
  QCHECK_EQ(cost.has_value(), true);
  QCHECK_GE(*cost, 0.0);
}

TEST_F(RunAccCorridorCombinedTest, FindMostPossibleLaneTest) {
  const auto lane = internal::FindMostPossibleLane(
      *psmm_ptr_, *frenet_frame_, *dm_.get(), path_, credible_length_);
  QCHECK_EQ(lane.ok(), false);
}

TEST_F(RunAccCorridorCombinedTest, FindConnectingPathPointTest) {
  const auto path_point = internal::FindConnectingPathPoint(
      *psmm_ptr_, path_, credible_length_, lane_id_, /*av_v=*/0.0);
  QCHECK_EQ(path_point.ok(), true);
}

TEST_F(RunAccCorridorCombinedTest, BuildDrivePassageAndCheckValidTest0) {
  const absl::StatusOr<DrivePassage> dp =
      internal::BuildDrivePassageAndCheckValid(*psmm_ptr_, lane_id_, pt_,
                                               credible_length_);
  QCHECK_EQ(dp.ok(), true);
}

TEST_F(RunAccCorridorCombinedTest, BuildDrivePassageAndCheckValidTest1) {
  const absl::StatusOr<DrivePassage> dp =
      internal::BuildDrivePassageAndCheckValid(*psmm_ptr_, lane_id_,
                                               path_.front(), credible_length_);
  QCHECK_EQ(dp.ok(), true);
}

TEST_F(RunAccCorridorCombinedTest,
       CombineReferenceCenterFromEstimatedPathAndDpTest0) {
  const absl::StatusOr<DrivePassage> dp =
      internal::BuildDrivePassageAndCheckValid(*psmm_ptr_, lane_id_, pt_,
                                               credible_length_);  // toulan !!

  QCHECK_EQ(dp.ok(), true);

  const absl::StatusOr<std::vector<internal::ReferenceCenterPoint>>
      center_points = internal::CombineReferenceCenterFromEstimatedPathAndDp(
          path_, *dp, pt_, credible_length_, corridor_step_s_);
  QCHECK_EQ(center_points.ok(), true);
}

TEST_F(RunAccCorridorCombinedTest,
       CombineReferenceCenterFromEstimatedPathAndDpTest1) {
  const absl::StatusOr<DrivePassage> dp =
      internal::BuildDrivePassageAndCheckValid(*psmm_ptr_, lane_id_, pt_,
                                               credible_length_);  // toulan !!

  QCHECK_EQ(dp.ok(), true);

  PathPoint pt = path_.front();
  pt.set_x(pt.x() - 10.0);

  const absl::StatusOr<std::vector<internal::ReferenceCenterPoint>>
      center_points = internal::CombineReferenceCenterFromEstimatedPathAndDp(
          path_, *dp, pt, credible_length_, corridor_step_s_);
  QCHECK_EQ(center_points.ok(), false);
}

TEST_F(RunAccCorridorCombinedTest, BuildPathSlBoundaryWithSmoothedPathTest) {
  const PathSlBoundary sl_boundary =
      internal::BuildPathSlBoundaryWithSmoothedPath(path_);
  QCHECK_GE(sl_boundary.end_s(), 0.0);
}

TEST_F(RunAccCorridorCombinedTest,
       BuildPossiblePreviewLaneKeepAccCorridorTest0) {
  ApolloTrajectoryPointProto plan_start_point;

  plan_start_point.mutable_path_point()->set_x(pt_.x());
  plan_start_point.mutable_path_point()->set_y(pt_.y());
  plan_start_point.mutable_path_point()->set_z(0.0);
  plan_start_point.mutable_path_point()->set_theta(pt_.theta());

  const absl::StatusOr<AccCorridor> acc_corridor =
      BuildPossiblePreviewLaneKeepAccCorridor(
          *psmm_ptr_, plan_start_point, path_, *frenet_frame_, *dm_.get(),
          credible_length_, corridor_step_s_,
          /*total_preview_time=*/1.0);
  QCHECK_EQ(acc_corridor.ok(), false);
}

TEST_F(RunAccCorridorCombinedTest,
       BuildPossiblePreviewLaneKeepAccCorridorTest1) {
  ApolloTrajectoryPointProto plan_start_point;

  plan_start_point.mutable_path_point()->set_x(0.0);
  plan_start_point.mutable_path_point()->set_y(0.0);
  plan_start_point.mutable_path_point()->set_z(0.0);
  plan_start_point.mutable_path_point()->set_theta(M_PI_2);

  const absl::StatusOr<AccCorridor> acc_corridor =
      BuildPossiblePreviewLaneKeepAccCorridor(
          *psmm_ptr_, plan_start_point, path_, *frenet_frame_, *dm_.get(),
          credible_length_, corridor_step_s_,
          /*total_preview_time=*/1.0);
  QCHECK_EQ(acc_corridor.ok(), false);
}

}  // namespace planner
}  // namespace qcraft

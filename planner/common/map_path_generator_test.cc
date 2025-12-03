#include "onboard/planner/common/map_path_generator.h"

#include <stdint.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <optional>

#include "absl/time/clock.h"
#include "google/protobuf/repeated_ptr_field.h"

#include "gtest/gtest.h"

#include "onboard/base/macros.h"
#include "onboard/global/buffered_logger.h"
#include "onboard/maps/map_selector.h"
#include "onboard/maps/maps_common.h"
#include "onboard/maps/online_map_converter.h"
#include "onboard/maps/proto/online_semantic_map.pb.h"
#include "onboard/maps/semantic_map_manager.h"
#include "onboard/planner/common/path_generator_util.h"
#include "onboard/planner/test_util/util.h"
#include "onboard/planner/util/path_util.h"
#include "onboard/planner/util/planner_semantic_map_manager_builder.h"
#include "onboard/utils/status_macros.h"
#include "onboard/utils/time_util.h"

namespace qcraft {
namespace planner {

namespace {

constexpr double kPathPreviewTime = 5.0;     // s.
constexpr double kPathMinLength = 20.0;      // m.
constexpr double kPathMinPreviewTime = 2.0;  // s.
constexpr double kPathStep = 0.2;            // m.

constexpr double kMaxSearchRadius = 2.5;    // m.
constexpr double kMaxHeadingDiff = M_PI_4;  // rad.
constexpr double kDrivePassageLengthFactor = 1.5;

constexpr double kEps = 1e-1;
constexpr double kMaxLateralAccLimit = 3.0;

TEST(GenerateSmoothPath, SmoothPath) {
  std::vector<double> x{0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0};
  std::vector<double> y{0.0, 0.5, 0.0, 0.0, -0.5, 0.0, 0.0, 0.5, 0.0, 0.0};
  std::vector<double> s{0.0};
  s.reserve(x.size());
  for (int i = 1; i < x.size(); ++i) {
    s.push_back(std::hypot(x[i] - x[i - 1], y[i] - y[i - 1]));
  }

  const double path_length = s.back();
  const double speed = 5.0;
  const double max_check_distance = 10.0;
  const auto path_or = GenerateSmoothPath(speed, kPathStep, max_check_distance,
                                          kMaxLateralAccLimit, &x, &y, &s);

  EXPECT_OK(path_or);
  const auto& path = *path_or;
  constexpr double kEpsHighPrecision = 1e-2;
  EXPECT_NEAR(std::hypot(path[1].x() - path[0].x(), path[1].y() - path[0].y()),
              kPathStep, kEpsHighPrecision);
  EXPECT_NEAR(path[1].s() - path[0].s(), kPathStep, kEpsHighPrecision);
  EXPECT_NEAR(path.front().x(), 0.0, kEpsHighPrecision);
  EXPECT_NEAR(path.front().y(), 0.0, kEpsHighPrecision);
  EXPECT_NEAR(x.back(), 9.0, kEpsHighPrecision);
  EXPECT_NEAR(y.back(), 0.0, kEpsHighPrecision);
  EXPECT_NEAR(path.back().s(), path_length, kEpsHighPrecision);
}

TEST(GenerateLaneKeepPathFromDrivePassage, LaneKeeping) {
  const auto vehicle_param = planner::DefaultVehicleGeometry();
  SetMap("dojo");
  auto whole_smm = std::make_shared<SemanticMapManager>();
  whole_smm->LoadWholeMap().Build();

  ApolloTrajectoryPointProto start;
  start.set_v(10.0);
  start.set_a(0.0);
  start.set_j(0.0);
  start.set_relative_time(0.0);
  start.mutable_path_point()->set_x(10.0);
  start.mutable_path_point()->set_y(0.0);
  start.mutable_path_point()->set_z(0.0);
  start.mutable_path_point()->set_theta(0.0);
  start.mutable_path_point()->set_kappa(0.0);
  start.mutable_path_point()->set_s(0.0);
  start.mutable_path_point()->set_lambda(0.0);

  const double path_length = start.v() * kPathPreviewTime;
  // Construct online map.
  ASSIGN_OR_DIE(
      const auto online_smm_proto,
      mapping::RunOnlineSemanticMapConverter(
          *whole_smm, mapping::OnlineSemanticMapOption{
                          .timestamp_s = ToUnixDoubleSeconds(absl::Now()),
                          .smooth_x = start.path_point().x(),
                          .smooth_y = start.path_point().y(),
                          .smooth_yaw = start.path_point().theta(),
                          .look_ahead_distance = 150.0,
                          .look_back_distance = 20.0}));

  std::shared_ptr<planner::PlannerSemanticMapManager> psmm = nullptr;
  ASSIGN_OR_DIE(psmm, planner::BuildOnlineMapPsmm(online_smm_proto));
  EXPECT_NE(nullptr, psmm);
  const mapping::LaneInfo* lane_info =
      psmm->GetNearestLaneInfoWithHeadingAtLevel(
          psmm->GetLevel(), ToVec2d(start.path_point()),
          start.path_point().theta(), kMaxSearchRadius, kMaxHeadingDiff);
  EXPECT_NE(nullptr, lane_info);
  const auto drive_passage =
      BuildDrivePassageFromLaneId(*psmm, lane_info->id, start.path_point(),
                                  kDrivePassageLengthFactor * path_length);
  EXPECT_TRUE(drive_passage.has_value());
  const auto path = GenerateLaneKeepPathFromDrivePassage(
      *drive_passage, start, path_length, kPathStep, kPathMinLength,
      kPathMinPreviewTime);

  EXPECT_OK(path);
  // Keep on lane center.
  EXPECT_NEAR(path->back().y(), 0.0, kEps);

  ApolloTrajectoryPointProto start2;
  start2.set_v(10.0);
  start2.set_a(0.0);
  start2.set_j(0.0);
  start2.set_relative_time(0.0);
  start2.mutable_path_point()->set_x(10.0);
  start2.mutable_path_point()->set_y(1.0);
  start2.mutable_path_point()->set_z(0.0);
  start2.mutable_path_point()->set_theta(-0.1);
  start2.mutable_path_point()->set_kappa(0.0);
  start2.mutable_path_point()->set_s(0.0);
  start2.mutable_path_point()->set_lambda(0.0);

  const auto path2 = GenerateLaneKeepPathFromDrivePassage(
      *drive_passage, start2, path_length, kPathStep, kPathMinLength,
      kPathMinPreviewTime);

  EXPECT_OK(path2);
  // Return to lane center.
  EXPECT_NEAR(path2->back().y(), 0.0, kEps);
}

TEST(GenerateLaneChangePathFromDrivePassage, LaneChange) {
  const auto vehicle_param = planner::DefaultVehicleGeometry();
  SetMap("dojo");
  auto whole_smm = std::make_shared<SemanticMapManager>();
  whole_smm->LoadWholeMap().Build();

  ApolloTrajectoryPointProto start;
  start.set_v(10.0);
  start.set_a(0.0);
  start.set_j(0.0);
  start.set_relative_time(0.0);
  start.mutable_path_point()->set_x(10.0);
  start.mutable_path_point()->set_y(0.1);
  start.mutable_path_point()->set_z(0.0);
  start.mutable_path_point()->set_theta(0.05);
  start.mutable_path_point()->set_kappa(0.01);
  start.mutable_path_point()->set_s(0.0);
  start.mutable_path_point()->set_lambda(0.0);

  const double path_length = start.v() * kPathPreviewTime;
  // Construct online map.
  ASSIGN_OR_DIE(
      const auto online_smm_proto,
      mapping::RunOnlineSemanticMapConverter(
          *whole_smm, mapping::OnlineSemanticMapOption{
                          .timestamp_s = ToUnixDoubleSeconds(absl::Now()),
                          .smooth_x = start.path_point().x(),
                          .smooth_y = start.path_point().y(),
                          .smooth_yaw = start.path_point().theta(),
                          .look_ahead_distance = 150.0,
                          .look_back_distance = 20.0}));

  std::shared_ptr<planner::PlannerSemanticMapManager> psmm = nullptr;
  ASSIGN_OR_DIE(psmm, planner::BuildOnlineMapPsmm(online_smm_proto));
  EXPECT_NE(nullptr, psmm);
  const mapping::LaneInfo* lane_info =
      psmm->GetNearestLaneInfoWithHeadingAtLevel(
          psmm->GetLevel(), ToVec2d(start.path_point()),
          start.path_point().theta(), kMaxSearchRadius, kMaxHeadingDiff);
  EXPECT_NE(nullptr, lane_info);
  EXPECT_FALSE(lane_info->lane_neighbors_on_left.empty());
  const auto target_drive_passage = BuildDrivePassageFromLaneId(
      *psmm, lane_info->lane_neighbors_on_left.front().other_id,
      start.path_point(), kDrivePassageLengthFactor * path_length);
  EXPECT_TRUE(target_drive_passage.has_value());

  const auto path = GenerateLaneChangePathFromDrivePassage(
      *target_drive_passage, start, path_length, kPathStep, kPathMinLength,
      kPathMinPreviewTime);

  EXPECT_OK(path);
  // Lane change to left lane.
  EXPECT_NEAR(path->back().y(), 3.5, kEps);
}

TEST(GeneratePreviewLaneKeepPathFromStartPoint, Preview) {
  const auto vehicle_param = planner::DefaultVehicleGeometry();
  SetMap("dojo");
  auto whole_smm = std::make_shared<SemanticMapManager>();
  whole_smm->LoadWholeMap().Build();

  ApolloTrajectoryPointProto start;
  start.set_v(10.0);
  start.set_a(0.0);
  start.set_j(0.0);
  start.set_relative_time(0.0);
  start.mutable_path_point()->set_x(-30.0);
  start.mutable_path_point()->set_y(-4.5);
  start.mutable_path_point()->set_z(0.0);
  start.mutable_path_point()->set_theta(0.3);
  start.mutable_path_point()->set_kappa(-0.01);
  start.mutable_path_point()->set_s(0.0);
  start.mutable_path_point()->set_lambda(0.0);

  const double path_length = start.v() * kPathPreviewTime;
  // Construct online map.
  ASSIGN_OR_DIE(
      auto online_smm_proto,
      mapping::RunOnlineSemanticMapConverter(
          *whole_smm, mapping::OnlineSemanticMapOption{
                          .timestamp_s = ToUnixDoubleSeconds(absl::Now()),
                          .smooth_x = 0.0,
                          .smooth_y = 0.0,
                          .smooth_yaw = 0.0,
                          .look_ahead_distance = 150.0,
                          .look_back_distance = 20.0}));

  // The target lane we want to preview.
  constexpr uint64_t kTargetLaneId = 2448;
  for (int i = 0; i < online_smm_proto.lanes().size(); i++) {
    if (online_smm_proto.lanes()[i].id() == kTargetLaneId) {
      // Due to lanes in dojo map do not have correct virtual attributes, we
      // remove all lanes here except the target lane.
      online_smm_proto.mutable_lanes()->SwapElements(i, 0);
      online_smm_proto.mutable_lanes()->DeleteSubrange(
          1, online_smm_proto.lanes().size() - 1);
      break;
    }
  }
  EXPECT_EQ(online_smm_proto.lanes().size(), 1);
  EXPECT_EQ(online_smm_proto.lanes()[0].id(), kTargetLaneId);

  std::shared_ptr<planner::PlannerSemanticMapManager> psmm = nullptr;
  ASSIGN_OR_DIE(psmm, planner::BuildOnlineMapPsmm(online_smm_proto));
  EXPECT_NE(nullptr, psmm);
  const mapping::LaneInfo* lane_info =
      psmm->GetNearestLaneInfoWithHeadingAtLevel(
          psmm->GetLevel(), ToVec2d(start.path_point()),
          start.path_point().theta(), kMaxSearchRadius, kMaxHeadingDiff);
  EXPECT_EQ(nullptr, lane_info);
  const auto path = GeneratePreviewLaneKeepPathFromStartPoint(
      *psmm, start, path_length, kPathPreviewTime, kPathStep, kPathMinLength,
      kPathMinPreviewTime, kMaxSearchRadius, kMaxHeadingDiff,
      kDrivePassageLengthFactor, /*min_start_preview_distance=*/0.0,
      /*candidate_path=*/nullptr);

  EXPECT_OK(path);
  // Back to lane center.
  EXPECT_NEAR(path->back().y(), 0.0, kEps);
}

}  // namespace

}  // namespace planner
}  // namespace qcraft

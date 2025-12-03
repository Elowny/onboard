#include <algorithm>
#include <cmath>
#include <memory>
#include <optional>
#include <random>
#include <vector>

#include "absl/status/statusor.h"
#include "benchmark/benchmark.h"

#include "onboard/async/future.h"
#include "onboard/maps/lane_point.h"
#include "onboard/maps/map_selector.h"
#include "onboard/maps/v2/semantic_map_loader.h"
#include "onboard/math/coordinate_converter.h"
#include "onboard/math/geometry/box2d.h"
#include "onboard/math/geometry/proto/affine_transformation.pb.h"
#include "onboard/math/util.h"
#include "onboard/math/vec.h"
#include "onboard/planner/composite_lane_path.h"
#include "onboard/planner/object/planner_object.h"
#include "onboard/planner/object/spacetime_object_state.h"
#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/planner/router/drive_passage.h"
#include "onboard/planner/router/drive_passage_builder.h"
#include "onboard/planner/router/route_sections.h"
#include "onboard/planner/router/route_sections_util.h"
#include "onboard/planner/second_order_trajectory_point.h"
#include "onboard/planner/test_util/load_psmm_util.h"
#include "onboard/planner/test_util/object_prediction_builder.h"
#include "onboard/planner/test_util/perception_object_builder.h"
#include "onboard/planner/test_util/planner_object_builder.h"
#include "onboard/planner/test_util/predicted_trajectory_builder.h"
#include "onboard/planner/test_util/route_builder.h"
#include "onboard/prediction/predicted_trajectory.h"
#include "onboard/prediction/prediction_defs.h"
#include "onboard/proto/perception/fusion/object.pb.h"
#include "onboard/proto/positioning.pb.h"

namespace qcraft::planner {
namespace {

constexpr int kTimes = 100;

double RandomDouble(double start, double end, std::mt19937* gen) {
  std::uniform_real_distribution<> dis(start, end);
  return dis(*gen);
}

Vec2d RandomPointInUnitCircle(std::mt19937* gen) {
  constexpr int kTrialThreshold = 20;
  double x, y;
  for (int i = 0; i < kTrialThreshold; ++i) {
    x = RandomDouble(-1.0, 1.0, gen);
    y = RandomDouble(-1.0, 1.0, gen);

    if (Sqr(x) + Sqr(y) <= 1.0) {
      return Vec2d{x, y};
    }
  }
  // Probability of triggering this line is:
  // P_out^20, P_out = 1 - 3.14/4.0
  // Which evaluates to 5e-14.
  return Vec2d{0.0, 0.0};
}

std::vector<prediction::PredictedTrajectoryPoint> GenerateRandomTrajectory(
    double speed, std::mt19937* gen) {
  constexpr int kTrajSize = 100;
  const double radius = speed * prediction::kPredictionTimeStep;
  std::vector<prediction::PredictedTrajectoryPoint> points;
  points.reserve(kTrajSize);

  prediction::PredictedTrajectoryPoint pt;
  pt.set_pos(Vec2d::Zero());
  pt.set_s(0.0);
  pt.set_theta(0.0);
  pt.set_kappa(0.0);
  pt.set_t(0.0);
  pt.set_v(speed);
  pt.set_a(0.0);
  points.push_back(pt);

  for (int i = 1; i < kTrajSize; ++i) {
    pt.set_pos(pt.pos() + radius * RandomPointInUnitCircle(gen));
    pt.set_s(pt.s() + radius);
    pt.set_theta(RandomDouble(-1.0, 1.0, gen) * M_PI);
    pt.set_t(i * prediction::kPredictionTimeStep);
    points.push_back(pt);
  }

  return points;
}

void BM_SingleQueryTraj(benchmark::State& state) {  // NOLINT
  // Load map
  SetMap("dojo");

  auto loader = mapping::v2::SemanticMapLoader::MakeShared();
  auto map_fut = loader->PreloadWholeMap();
  auto semantic_map_manager = map_fut.Get();
  const auto* smm = semantic_map_manager.get();
  auto cc = CoordinateConverter::FromMap("dojo");

  const auto& psmm = planner::CreateDojoTestPSMM();

  PoseProto pose;
  pose.mutable_pos_smooth()->set_x(0.0);
  pose.mutable_pos_smooth()->set_y(0.0);
  pose.set_yaw(0.0);
  pose.mutable_vel_smooth()->set_x(0.0);
  pose.mutable_vel_smooth()->set_y(0.0);

  const auto route_path = RoutingToNameSpot(*smm, cc, pose, "b7_e2_start");
  const auto route_sections =
      RouteSectionsFromCompositeLanePath(*smm, route_path);

  const auto drive_passage_or = BuildDrivePassage(
      psmm, /*vision_map_ptr=*/nullptr, route_path.lane_paths().front(),
      route_path.lane_paths().front(),
      /*anchor_point=*/mapping::LanePoint(),
      route_sections.planning_horizon(psmm), route_sections.destination(),
      /*all_lanes_virtual=*/false, /*override_speed_limit_mps=*/std::nullopt);

  const auto& drive_passage = drive_passage_or.value();

  const double speed = state.range(0);
  PerceptionObjectBuilder perception_builder;
  const auto perception_obj = perception_builder.set_id("abc")
                                  .set_type(OT_VEHICLE)
                                  .set_pos(Vec2d::Zero())
                                  .set_timestamp(0.0)
                                  .set_velocity(speed)
                                  .set_yaw(0.0)
                                  .set_length_width(4.0, 2.0)
                                  .set_box_center(Vec2d::Zero())
                                  .Build();

  std::mt19937 gen;
  for (auto _ : state) {
    state.PauseTiming();
    const auto traj_points = GenerateRandomTrajectory(speed, &gen);
    PlannerObjectBuilder builder;
    builder.set_type(OT_VEHICLE)
        .set_object(perception_obj)
        .get_object_prediction_builder()
        ->add_predicted_trajectory()
        ->set_probability(0.5)
        .set_points(traj_points);
    const PlannerObject object = builder.Build();
    const auto& traj = object.traj(0);
    const auto states = SampleTrajectoryStates(
        traj, object.pose().pos(), object.contour(), object.bounding_box());
    std::vector<Box2d> box_vec;
    box_vec.reserve(states.size());
    for (const auto& state : states) {
      box_vec.push_back(state.box);
    }
    state.ResumeTiming();

    for (int i = 0; i < kTimes; ++i) {
      for (const auto& box : box_vec) {
        [[maybe_unused]] const auto result =
            drive_passage.QueryFrenetBoxAt(box);
      }
    }
  }
}
BENCHMARK(BM_SingleQueryTraj)->Arg(2)->Arg(5)->Arg(10)->Arg(20)->Arg(30);

void BM_BatchQueryTraj(benchmark::State& state) {  // NOLINT
  // Load map
  SetMap("dojo");

  const auto& psmm = planner::CreateDojoTestPSMM();
  const auto* smm = psmm.semantic_map_manager();
  const auto& cc = psmm.coordinate_converter();

  PoseProto pose;
  pose.mutable_pos_smooth()->set_x(0.0);
  pose.mutable_pos_smooth()->set_y(0.0);
  pose.set_yaw(0.0);
  pose.mutable_vel_smooth()->set_x(0.0);
  pose.mutable_vel_smooth()->set_y(0.0);

  const auto route_path = RoutingToNameSpot(*smm, cc, pose, "b7_e2_start");
  const auto route_sections =
      RouteSectionsFromCompositeLanePath(*smm, route_path);

  const auto drive_passage_or = BuildDrivePassage(
      psmm, /*vision_map_ptr=*/nullptr, route_path.lane_paths().front(),
      route_path.lane_paths().front(),
      /*anchor_point=*/mapping::LanePoint(),
      route_sections.planning_horizon(psmm), route_sections.destination(),
      /*all_lanes_virtual=*/false, /*override_speed_limit_mps=*/std::nullopt);

  const auto& drive_passage = drive_passage_or.value();

  const double speed = state.range(0);
  PerceptionObjectBuilder perception_builder;
  const auto perception_obj = perception_builder.set_id("abc")
                                  .set_type(OT_VEHICLE)
                                  .set_pos(Vec2d::Zero())
                                  .set_timestamp(0.0)
                                  .set_velocity(speed)
                                  .set_yaw(0.0)
                                  .set_length_width(4.0, 2.0)
                                  .set_box_center(Vec2d::Zero())
                                  .Build();

  std::mt19937 gen;
  for (auto _ : state) {
    state.PauseTiming();
    const auto traj_points = GenerateRandomTrajectory(speed, &gen);
    PlannerObjectBuilder builder;
    builder.set_type(OT_VEHICLE)
        .set_object(perception_obj)
        .get_object_prediction_builder()
        ->add_predicted_trajectory()
        ->set_probability(0.5)
        .set_points(traj_points);
    const PlannerObject object = builder.Build();
    const auto& traj = object.traj(0);
    const auto states = SampleTrajectoryStates(
        traj, object.pose().pos(), object.contour(), object.bounding_box());
    std::vector<Box2d> box_vec;
    box_vec.reserve(states.size());
    for (const auto& state : states) {
      box_vec.push_back(state.box);
    }
    state.ResumeTiming();

    for (int i = 0; i < kTimes; ++i) {
      [[maybe_unused]] const auto result = drive_passage.BatchQueryFrenetBoxes(
          box_vec, /*laterally_bounded=*/true);
    }
  }
}
BENCHMARK(BM_BatchQueryTraj)->Arg(2)->Arg(5)->Arg(10)->Arg(20)->Arg(30);

}  // namespace
}  // namespace qcraft::planner

BENCHMARK_MAIN();

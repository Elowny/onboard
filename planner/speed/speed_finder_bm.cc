#include <algorithm>
#include <array>
#include <cmath>
#include <map>
#include <optional>
#include <ostream>
#include <string>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_set.h"
#include "absl/status/statusor.h"
#include "absl/time/time.h"
#include "absl/types/span.h"
#include "benchmark/benchmark.h"

#include "onboard/global/buffered_logger.h"
#include "onboard/lite/check.h"
#include "onboard/lite/proto/lite_common.pb.h"
#include "onboard/maps/lane_path.h"
#include "onboard/maps/lane_point.h"
#include "onboard/maps/map_selector.h"
#include "onboard/math/geometry/polygon2d.h"
#include "onboard/math/vec.h"
#include "onboard/planner/common/discretized_path.h"
#include "onboard/planner/common/driving_map_topo.h"
#include "onboard/planner/common/path_sl_boundary.h"
#include "onboard/planner/composite_lane_path.h"
#include "onboard/planner/decision/constraint_manager.h"
#include "onboard/planner/decision/end_of_path_boundary.h"
#include "onboard/planner/decision/proto/constraint.pb.h"
#include "onboard/planner/object/object_vector.h"
#include "onboard/planner/object/planner_object.h"
#include "onboard/planner/object/planner_object_manager_builder.h"
#include "onboard/planner/object/spacetime_trajectory_manager.h"
#include "onboard/planner/planner_defs.h"
#include "onboard/planner/planner_semantic_map_manager.h"
#include "onboard/planner/proto/planner_params.pb.h"
#include "onboard/planner/router/drive_passage.h"
#include "onboard/planner/router/drive_passage_builder.h"
#include "onboard/planner/router/route_sections.h"
#include "onboard/planner/router/route_sections_util.h"
#include "onboard/planner/scheduler/driving_map_topo_builder.h"
#include "onboard/planner/scheduler/path_boundary_builder.h"
#include "onboard/planner/scheduler/proto/lane_change.pb.h"
#include "onboard/planner/scheduler/smooth_reference_line_result.h"
#include "onboard/planner/speed/proto/speed_finder_params.pb.h"
#include "onboard/planner/speed/speed_finder.h"
#include "onboard/planner/speed/speed_finder_input.h"
#include "onboard/planner/test_util/load_psmm_util.h"
#include "onboard/planner/test_util/object_prediction_builder.h"
#include "onboard/planner/test_util/perception_object_builder.h"
#include "onboard/planner/test_util/predicted_trajectory_builder.h"
#include "onboard/planner/test_util/route_builder.h"
#include "onboard/planner/test_util/util.h"
#include "onboard/planner/util/lane_path_util.h"
#include "onboard/prediction/object_prediction.h"
#include "onboard/proto/perception.pb.h"
#include "onboard/proto/perception/fusion/object.pb.h"
#include "onboard/proto/positioning.pb.h"
#include "onboard/proto/prediction.pb.h"
#include "onboard/proto/trajectory_point.pb.h"
#include "onboard/proto/vehicle.pb.h"
#include "onboard/utils/status_macros.h"

namespace qcraft {
namespace planner {
namespace {

Polygon2d GeneratePolygon(const Vec2d& center, int num_points, double radius) {
  std::vector<Vec2d> points;
  const double angle_step = 2.0 * M_PI / num_points;
  points.reserve(num_points);
  for (int i = 0; i < num_points; ++i) {
    points.push_back(center +
                     radius * Vec2d::FastUnitFromAngle(angle_step * i));
  }
  return Polygon2d(std::move(points), /*is_convex=*/true);
}

ObjectProto BuildObject(const std::string& id, const Polygon2d& contour,
                        double timestamp, ObjectType type) {
  return PerceptionObjectBuilder()
      .set_id(id)
      .set_type(type)
      .set_pos(contour.CircleCenter())
      .set_timestamp(timestamp)
      .set_velocity(5.0)
      .set_yaw(0.0)
      .set_length_width(4.0, 2.0)
      .set_box_center(Vec2d::Zero())
      .set_contour(contour)
      .Build();
}
constexpr int kNumOfTypes = 5;

static const std::array<ObjectType, kNumOfTypes> kTypes{
    OT_VEHICLE, OT_PEDESTRIAN, OT_CYCLIST, OT_LARGE_VEHICLE,
    OT_WARNING_TRIANGLE};

ObjectsProto BuildPerceptionObjects(int num_objects, int num_contour_points,
                                    double timestamp) {
  ObjectsProto objects;
  objects.mutable_header()->set_timestamp(timestamp);
  objects.set_scope(ObjectsProto::SCOPE_REAL);
  const Polygon2d contour = GeneratePolygon(/*center=*/Vec2d::Zero(),
                                            num_contour_points, /*radius=*/2.0);
  for (int i = 0; i < num_objects; ++i) {
    *objects.add_objects() = BuildObject(std::to_string(i), contour, timestamp,
                                         kTypes[i % kNumOfTypes]);
  }
  return objects;
}

constexpr int kNumTrajs = 2;
constexpr double kProb = (1.0 - 1e-6) / kNumTrajs;

void BuildFollowPredictionObject(int idx, double timestamp,
                                 const Polygon2d& contour,
                                 ObjectPredictionProto* proto) {
  ObjectPredictionBuilder builder;
  const Vec2d start_pos(55.0, 0.0);
  builder.set_object(BuildObject("Follow " + std::to_string(idx),
                                 contour.Shift(start_pos), timestamp,
                                 kTypes[idx % kNumOfTypes]));
  for (int i = 0; i < kNumTrajs; ++i) {
    builder.add_predicted_trajectory()
        ->set_probability(kProb)
        .set_straight_line(start_pos, /*theta=*/0.0, /*duration=*/10.0,
                           /*init_v=*/0.2, /*acc=*/0.5);
  }
  builder.Build().ToProto(proto);
}

void BuildMovingClosePredictionObject(int idx, double timestamp,
                                      const Polygon2d& contour,
                                      ObjectPredictionProto* proto) {
  ObjectPredictionBuilder builder;
  const Vec2d start_pos(25.0, -5.0);
  builder.set_object(BuildObject("Close " + std::to_string(idx),
                                 contour.Shift(start_pos), timestamp,
                                 kTypes[idx % kNumOfTypes]));
  for (int i = 0; i < kNumTrajs; ++i) {
    builder.add_predicted_trajectory()
        ->set_probability(kProb)
        .set_straight_line(start_pos, /*theta=*/0.05, /*duration=*/10.0,
                           /*init_v=*/0.2, /*acc=*/0.5);
  }
  builder.Build().ToProto(proto);
}

void BuildCrossPredictionObject(int idx, double timestamp,
                                const Polygon2d& contour,
                                ObjectPredictionProto* proto) {
  ObjectPredictionBuilder builder;
  const Vec2d start_pos(45.0, -15.0);
  builder.set_object(BuildObject("Cross " + std::to_string(idx),
                                 contour.Shift(start_pos), timestamp,
                                 kTypes[idx % kNumOfTypes]));
  for (int i = 0; i < kNumTrajs; ++i) {
    builder.add_predicted_trajectory()
        ->set_probability(kProb)
        .set_straight_line(start_pos, /*theta=*/M_PI * 0.5,
                           /*duration=*/10.0,
                           /*init_v=*/0.2, /*acc=*/0.5);
  }
  builder.Build().ToProto(proto);
}

void BuildCutInPredictionObject(int idx, double timestamp,
                                const Polygon2d& contour,
                                ObjectPredictionProto* proto) {
  ObjectPredictionBuilder builder;
  const Vec2d start_pos(25.0, 7.0);
  builder.set_object(BuildObject("CutIn " + std::to_string(idx),
                                 contour.Shift(start_pos), timestamp,
                                 kTypes[idx % kNumOfTypes]));
  for (int i = 0; i < kNumTrajs; ++i) {
    builder.add_predicted_trajectory()
        ->set_probability(kProb)
        .set_straight_line(start_pos, /*theta=*/-0.15,
                           /*duration=*/10.0,
                           /*init_v=*/2.0, /*acc=*/0.5);
  }
  builder.Build().ToProto(proto);
}

void BuildLeadPredictionObject(int idx, double timestamp,
                               const Polygon2d& contour,
                               ObjectPredictionProto* proto) {
  ObjectPredictionBuilder builder;
  const Vec2d start_pos(35.0, -15.0);
  builder.set_object(BuildObject("Lead " + std::to_string(idx),
                                 contour.Shift(start_pos), timestamp,
                                 kTypes[idx % kNumOfTypes]));
  for (int i = 0; i < kNumTrajs; ++i) {
    builder.add_predicted_trajectory()
        ->set_probability(kProb)
        .set_straight_line(start_pos, /*theta=*/M_PI * 0.5,
                           /*duration=*/10.0,
                           /*init_v=*/2.0, /*acc=*/0.1);
  }
  builder.Build().ToProto(proto);
}

void BuildInteractivePredictionObject(int idx, double timestamp,
                                      const Polygon2d& contour,
                                      ObjectPredictionProto* proto) {
  ObjectPredictionBuilder builder;
  const Vec2d start_pos(25.0, -15.0);
  builder.set_object(BuildObject("Interactive " + std::to_string(idx),
                                 contour.Shift(start_pos), timestamp,
                                 kTypes[idx % kNumOfTypes]));
  for (int i = 0; i < kNumTrajs; ++i) {
    builder.add_predicted_trajectory()
        ->set_probability(kProb)
        .set_straight_line(start_pos, /*theta=*/M_PI * 0.25,
                           /*duration=*/10.0,
                           /*init_v=*/10.0, /*acc=*/0.1);
  }
  builder.Build().ToProto(proto);
}

void BuildStationaryPredictionObject(int idx, double timestamp,
                                     const Polygon2d& contour,
                                     ObjectPredictionProto* proto) {
  ObjectPredictionBuilder builder;
  const Vec2d start_pos(50, 0.0);
  builder.set_object(BuildObject("Stationary " + std::to_string(idx),
                                 contour.Shift(start_pos), timestamp,
                                 kTypes[idx % kNumOfTypes]));
  for (int i = 0; i < kNumTrajs; ++i) {
    builder.add_predicted_trajectory()
        ->set_probability(kProb)
        .set_stationary_traj(start_pos, /*theta=*/0.0);
  }
  builder.Build().ToProto(proto);
}

void BuildStationaryClosePredictionObject(int idx, double timestamp,
                                          const Polygon2d& contour,
                                          ObjectPredictionProto* proto) {
  ObjectPredictionBuilder builder;
  const Vec2d start_pos(24, 4.0);
  builder.set_object(BuildObject("Close " + std::to_string(idx),
                                 contour.Shift(start_pos), timestamp,
                                 OT_VEHICLE));
  for (int i = 0; i < kNumTrajs; ++i) {
    builder.add_predicted_trajectory()
        ->set_probability(kProb)
        .set_stationary_traj(start_pos, /*theta=*/0.0);
  }
  builder.Build().ToProto(proto);
}

ObjectsPredictionProto BuildPredictionObjects(int num_objects,
                                              int num_contour_points,
                                              double timestamp) {
  ObjectsPredictionProto prediction_proto;
  prediction_proto.mutable_header()->set_timestamp(timestamp);

  const Polygon2d contour = GeneratePolygon(/*center=*/Vec2d::Zero(),
                                            num_contour_points, /*radius=*/2.0);
  for (int i = 0; i < num_objects; ++i) {
    BuildFollowPredictionObject(i, timestamp, contour,
                                prediction_proto.add_objects());
    BuildMovingClosePredictionObject(i, timestamp, contour,
                                     prediction_proto.add_objects());
    BuildCrossPredictionObject(i, timestamp, contour,
                               prediction_proto.add_objects());
    BuildCutInPredictionObject(i, timestamp, contour,
                               prediction_proto.add_objects());
    BuildLeadPredictionObject(i, timestamp, contour,
                              prediction_proto.add_objects());
    BuildInteractivePredictionObject(i, timestamp, contour,
                                     prediction_proto.add_objects());
    BuildStationaryPredictionObject(i, timestamp, contour,
                                    prediction_proto.add_objects());
    BuildStationaryClosePredictionObject(i, timestamp, contour,
                                         prediction_proto.add_objects());
  }
  return prediction_proto;
}

void BM_SpeedFinder(benchmark::State& state) {  // NOLINT
  const PlannerParamsProto planner_params = DefaultPlannerParams();
  const VehicleGeometryParamsProto vehicle_geometry_params =
      DefaultVehicleGeometry();
  const VehicleDriveParamsProto& vehicle_drive_params =
      DefaultVehicleDriveParams();
  const MotionConstraintParamsProto& motion_constraint_params =
      planner_params.motion_constraint_params();
  const SpeedFinderParamsProto& speed_finder_params =
      planner_params.speed_finder_params();

  constexpr double kVx = 10.0;
  constexpr double kVy = 0.0;
  const PoseProto sdc_pose =
      CreatePose(/*timestamp=*/0.0, /*pos=*/Vec2d(20.0, 0.0),
                 /*heading=*/0.0, /*speed_vec=*/Vec2d(kVx, kVy));
  // Plan start point.
  const ApolloTrajectoryPointProto plan_start_point =
      ConvertToTrajPointProto(sdc_pose);

  SetMap("dojo");
  const auto& psmm = CreateDojoTestPSMM();
  const auto* smm = psmm.semantic_map_manager();
  const auto& cc = psmm.coordinate_converter();

  // Build drive passage.
  const auto route_path = RoutingToNameSpot(*smm, cc, sdc_pose, "7b_n1");
  const auto route_sections =
      RouteSectionsFromCompositeLanePath(*smm, route_path);
  const auto target_lane_path =
      BackwardExtendTargetAlignedRouteLanePath(
          psmm, !route_path.transitions().front().lc_left,
          route_path.lane_paths()[1].front(), route_path.lane_paths().front())
          .Connect(smm, route_path.lane_paths()[1]);
  ASSIGN_OR_DIE(const auto dm, BuildDrivingMapByRouteOnOfflineMap(
                                   psmm, RouteSections::BuildFromLanePath(
                                             psmm, target_lane_path)));
  const auto backward_extended_lane_path =
      BackwardExtendLanePathOnRouteSections(psmm, route_sections,
                                            target_lane_path,
                                            kDrivePassageKeepBehindLength);
  QCHECK(backward_extended_lane_path.ok());
  const auto drive_passage = BuildDrivePassage(
      psmm, /*vision_map_ptr=*/nullptr, target_lane_path,
      *backward_extended_lane_path,
      /*anchor_point=*/mapping::LanePoint(),
      route_sections.planning_horizon(psmm), route_sections.destination(),
      /*all_lanes_virtual=*/false,
      /*override_speed_limit_mps=*/std::nullopt);
  QCHECK(drive_passage.ok() && !drive_passage->empty())
      << "Building drive passage failed!";

  // Create spacetime object manager.
  constexpr int kNumContourPoints = 16;
  const int num_perception_objects = state.range(0);
  const auto perception_objects =
      BuildPerceptionObjects(num_perception_objects, kNumContourPoints,
                             /*timestamp=*/0.0);
  const auto prediction_objects =
      BuildPredictionObjects(num_perception_objects, kNumContourPoints,
                             /*timestamp=*/0.0);
  const auto planner_objects =
      BuildPlannerObjects(&perception_objects, &prediction_objects,
                          /*align_time=*/0.0,
                          /*thread_pool=*/nullptr);
  SpacetimeTrajectoryManager st_traj_mgr(planner_objects);

  // Build path sl boundary.
  LaneChangeStateProto lane_change_state;
  lane_change_state.set_stage(LaneChangeStage::LCS_NONE);
  SmoothedReferenceLineResultMap smooth_result_map;
  const auto path_sl_boundary = BuildPathBoundaryFromPose(
      psmm, *drive_passage, plan_start_point, vehicle_geometry_params,
      st_traj_mgr, lane_change_state, smooth_result_map,
      /*borrow_lane_boundary=*/false,
      /*should_smooth=*/false, /*unsafe_object_ids=*/{});
  QCHECK(path_sl_boundary.ok());

  // Build constraint manager.
  ConstraintManager constraint_mgr;
  auto end_of_path_boundary_constraint =
      BuildEndOfPathBoundaryConstraint(*drive_passage, *path_sl_boundary);
  QCHECK(end_of_path_boundary_constraint.ok());
  constraint_mgr.AddStopLine(
      std::move(end_of_path_boundary_constraint).value());

  const std::map<std::string, ConstraintProto::LeadingObjectProto>
      leading_trajs;
  const absl::flat_hash_set<std::string> follower_set;

  const absl::flat_hash_set<std::string> stalled_objects;

  const int path_point_num = speed_finder_params.max_traj_steps();
  std::vector<PathPoint> path_points;
  path_points.reserve(path_point_num);
  for (int k = 0; k < path_point_num; ++k) {
    auto& path_point = path_points.emplace_back();
    path_point.set_x(plan_start_point.path_point().x() +
                     k * kTrajectoryTimeStep * kVx);
    path_point.set_y(plan_start_point.path_point().y() +
                     k * kTrajectoryTimeStep * kVy);
    path_point.set_z(0.0);
    path_point.set_theta(plan_start_point.path_point().theta());
    path_point.set_kappa(0.0);
    path_point.set_lambda(0.0);
    path_point.set_s(kVx * k * 0.1);
  }

  DiscretizedPath path(path_points);
  const auto resampled_path = DiscretizedPath::CreateResampledPath(
      std::move(path), kPathSampleInterval);

  const SpeedFinderInput input{.base_name = "SpeedFinderBenchmark",
                               .driving_map_topo = &dm,
                               .psmm = &psmm,
                               .traj_mgr = &st_traj_mgr,
                               .constraint_mgr = &constraint_mgr,
                               .leading_trajs = &leading_trajs,
                               .follower_set = &follower_set,
                               .drive_passage = &(*drive_passage),
                               .path_sl_boundary = &(*path_sl_boundary),
                               .stalled_objects = &stalled_objects,
                               .path = &resampled_path,
                               .st_path_points = &path_points,
                               .time_aligned_prev_traj = nullptr,
                               .plan_start_v = plan_start_point.v(),
                               .plan_start_a = plan_start_point.a(),
                               .plan_start_j = plan_start_point.j(),
                               .plan_time = absl::UnixEpoch(),
                               .planner_av_context = nullptr,
                               .real_objects = &perception_objects,
                               .virtual_objects = nullptr,
                               .planner_model_pool = nullptr,
                               .run_act_net_speed_decision = false};

  for (auto _ : state) {
    benchmark::DoNotOptimize(
        FindSpeed(input, vehicle_geometry_params, vehicle_drive_params,
                  motion_constraint_params, speed_finder_params,
                  /*thread_pool=*/nullptr));
  }
}
BENCHMARK(BM_SpeedFinder)->Arg(1)->Arg(5);

}  // namespace
}  // namespace planner
}  // namespace qcraft

BENCHMARK_MAIN();

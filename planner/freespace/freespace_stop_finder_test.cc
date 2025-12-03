#include "onboard/planner/freespace/freespace_stop_finder.h"

#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>

#include "absl/hash/hash.h"
#include "absl/status/statusor.h"
#include "absl/types/span.h"

#include "gtest/gtest.h"

#include "onboard/lite/proto/lite_common.pb.h"
#include "onboard/math/geometry/polygon2d.h"
#include "onboard/math/vec.h"
#include "onboard/planner/object/object_vector.h"
#include "onboard/planner/object/planner_cluster_object.h"
#include "onboard/planner/object/planner_cluster_object_manager.h"
#include "onboard/planner/object/planner_object.h"
#include "onboard/planner/object/planner_object_manager_builder.h"
#include "onboard/planner/test_util/load_psmm_util.h"
#include "onboard/planner/test_util/object_prediction_builder.h"
#include "onboard/planner/test_util/perception_object_builder.h"
#include "onboard/planner/test_util/predicted_trajectory_builder.h"
#include "onboard/planner/test_util/util.h"
#include "onboard/prediction/object_prediction.h"
#include "onboard/proto/perception.pb.h"
#include "onboard/proto/perception/fusion/object.pb.h"
#include "onboard/proto/positioning.pb.h"
#include "onboard/proto/prediction.pb.h"
#include "onboard/proto/trajectory_point.pb.h"

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
      .set_velocity(0.0)
      .set_yaw(0.0)
      .set_length_width(4.0, 2.0)
      .set_box_center(contour.CircleCenter())
      .set_contour(contour)
      .Build();
}

TEST(FreespaceStopFinderTest, FindFreespaceStop) {
  constexpr double kVx = 3.0;  // m/s.
  constexpr double kVy = 0.0;
  const PoseProto sdc_pose =
      CreatePose(/*timestamp=*/0.0, /*pos=*/Vec2d::Zero(),
                 /*heading=*/0.0, /*speed_vec=*/Vec2d(kVx, kVy));

  // Plan start point.
  const ApolloTrajectoryPointProto plan_start_point =
      ConvertToTrajPointProto(sdc_pose);

  const Vec2d obj_center(20.0, 0.0);
  const Polygon2d contour = GeneratePolygon(/*center=*/obj_center,
                                            /*num_points=*/10, /*radius=*/2.0);
  // Build perception object.
  ObjectsProto objects;
  objects.mutable_header()->set_timestamp(0.0);
  objects.set_scope(ObjectsProto::SCOPE_REAL);
  ObjectProto object =
      BuildObject("obj_1", contour, /*timestamp=*/0.0, ObjectType::OT_VEHICLE);
  *objects.add_objects() = object;

  // Build prediction object.
  ObjectsPredictionProto prediction_proto;
  ObjectPredictionBuilder builder;
  builder.set_object(object);
  builder.add_predicted_trajectory()->set_probability(1.0).set_stationary_traj(
      obj_center, /*theta=*/0.0);
  builder.Build().ToProto(prediction_proto.add_objects());

  // Build planner objects.
  const auto planner_objects = BuildPlannerObjects(&objects, &prediction_proto,
                                                   /*align_time=*/0.0,
                                                   /*thread_pool=*/nullptr);

  // Build stalled objects.
  const absl::flat_hash_set<std::string> stalled_objects;

  // Discretized path.
  constexpr int kPathPointsNum = 100;
  constexpr double kPathStep = 0.2;  // m.
  DiscretizedPath path;
  path.reserve(kPathPointsNum);
  for (int k = 0; k < kPathPointsNum; ++k) {
    auto& path_point = path.emplace_back();
    path_point.set_x(plan_start_point.path_point().x() + k * kPathStep);
    path_point.set_y(plan_start_point.path_point().y());
    path_point.set_z(0.0);
    path_point.set_theta(plan_start_point.path_point().theta());
    path_point.set_kappa(0.0);
    path_point.set_lambda(0.0);
    path_point.set_s(k * kPathStep);
  }

  // Manager.
  SpacetimeTrajectoryManager st_traj_mgr(planner_objects);
  ConstraintManager constraint_mgr;
  const auto& psmm = CreateDojoTestPSMM();

  // Cluster objects.
  const PlannerClusterObjectManager cluster_obj_mgr;
  const absl::flat_hash_set<PlannerClusterObject::Id> stalled_cluster_objects;

  // Param.
  const auto planner_params = DefaultPlannerParams();
  const auto vehicle_geometry_params = DefaultVehicleGeometry();
  const auto& octagon_geometry_params =
      planner_params.vehicle_models_params()
          .freespace_vehicle_octagon_model_params();
  const auto& motion_constraint_params =
      planner_params.motion_constraint_params();
  const auto& speed_finder_params = planner_params.speed_finder_params();
  absl::Time plan_time;

  // Run.
  const absl::StatusOr<FreespaceStopFinderOutput> output = FindFreespaceStop(
      path, /*forward=*/true, kVx, plan_time, &psmm, st_traj_mgr,
      cluster_obj_mgr, constraint_mgr, stalled_objects, stalled_cluster_objects,
      vehicle_geometry_params, octagon_geometry_params,
      motion_constraint_params, speed_finder_params,
      /*thread_pool=*/nullptr);

  EXPECT_TRUE(output.ok());
  if (output.ok()) {
    EXPECT_GT(output->stop_s, 0.0);
  }
}

}  // namespace
}  // namespace planner
}  // namespace qcraft

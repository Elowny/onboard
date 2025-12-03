#include "onboard/planner/plan/acc/acc_target_util.h"

#include <algorithm>
#include <memory>
#include <optional>
#include <ostream>

#include "gtest/gtest.h"

#include "onboard/container/strong_int.h"
#include "onboard/global/buffered_logger.h"
#include "onboard/lite/check.h"
#include "onboard/maps/map_selector.h"
#include "onboard/maps/semantic_map_defs.h"
#include "onboard/math/piecewise_linear_function.h"
#include "onboard/planner/common/discretized_path.h"
#include "onboard/planner/common/path_generator_util.h"
#include "onboard/planner/object/planner_object.h"
#include "onboard/planner/router/drive_passage.h"
#include "onboard/planner/scheduler/path_boundary_builder.h"
#include "onboard/planner/test_util/load_psmm_util.h"
#include "onboard/planner/test_util/object_prediction_builder.h"
#include "onboard/planner/test_util/perception_object_builder.h"
#include "onboard/planner/test_util/planner_object_builder.h"
#include "onboard/planner/test_util/predicted_trajectory_builder.h"
#include "onboard/proto/trajectory_point.pb.h"

// #include "onboard/planner/test_util/object_prediction_builder.h"

namespace qcraft {

namespace planner {
namespace {

TEST(RunAccTargetUtil, IsAccTargetTypeTest0) {
  const ObjectType type = ObjectType::OT_VEHICLE;
  QCHECK_EQ(internal::IsAccTargetType(type), true);
}

TEST(RunAccTargetUtil, IsAccTargetTypeTest1) {
  const ObjectType type = ObjectType::OT_WARNING_TRIANGLE;
  QCHECK_EQ(internal::IsAccTargetType(type), true);
}

TEST(RunAccTargetUtil, AlignSpacetimeObjectTrajectoryToCorridorTest) {
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
  const SpacetimeObjectTrajectory st_traj(object, 0,
                                          /*required_lateral_gap=*/0.0);

  // Build path boundary.
  SetMap("dojo");
  const auto& psmm = CreateDojoTestPSMM();

  PathPoint start_pt;
  start_pt.set_x(0.0);
  start_pt.set_y(0.0);
  start_pt.set_y(0.0);
  start_pt.set_theta(0.0);
  const auto dp_opt =
      BuildDrivePassageFromLaneId(psmm, mapping::ElementId(2448), start_pt,
                                  /*max_length=*/100.0);
  EXPECT_TRUE(dp_opt.has_value());
  auto path_sl_or = BuildPathBoundaryFromDrivePassage(psmm, *dp_opt);
  QCHECK_EQ(path_sl_or.ok(), true);
  std::vector<PathPoint> path_points;
  path_points.reserve(dp_opt->size());
  const double front_s = dp_opt->front_s();
  std::vector<double> s, kappa;
  std::vector<Vec2d> pts;
  s.reserve(dp_opt->size());
  kappa.reserve(dp_opt->size());
  pts.reserve(dp_opt->size());
  for (const auto& station : dp_opt->stations()) {
    PathPoint pt;
    pt.set_x(station.xy().x());
    pt.set_y(station.xy().y());
    pt.set_z(0.0);
    pt.set_s(station.accumulated_s() - front_s);
    pt.set_theta(station.tangent().Angle());
    pt.set_lambda(0.0);
    path_points.push_back(pt);
    s.push_back(station.accumulated_s() - front_s);
    pts.push_back(station.xy());
  }
  path_points.front().set_s(0.0);
  for (int k = 0; k + 1 < path_points.size(); ++k) {
    const double delta_length = path_points[k + 1].s() - path_points[k].s();
    const double theta_diff =
        NormalizeAngle(path_points[k + 1].theta() - path_points[k].theta());
    path_points[k].set_kappa(theta_diff / delta_length);
    kappa.push_back(theta_diff / delta_length);
  }
  kappa.push_back(kappa.back());
  DiscretizedPath path(path_points);
  const double credible_length = 5.0;
  PiecewiseLinearFunction<double, double> kappa_s(s, kappa);

  AccCorridor cor{.boundary = std::move(*path_sl_or),
                  .frenet_frame = BuildQtfmEnhancedKdTreeFrenetFrame(
                                      pts, /*down_sample_raw_points=*/false)
                                      .value(),
                  .path = std::move(path),
                  .credible_length = credible_length,
                  .kappa_s = std::move(kappa_s)};

  const auto res =
      internal::AlignSpacetimeObjectTrajectoryToCorridor(st_traj, cor);
  QCHECK_EQ(res.is_stationary(), false);
}

TEST(RunAccTargetUtil, ObjectInFrontOfAvFrontCenterByLengthTest) {
  const double object_length = 10.0;
  FrenetBox object_box;
  object_box.s_min = 20.0;
  object_box.s_max = 30.0;
  object_box.l_min = -1.0;
  object_box.l_max = 1.0;
  const ObjectType type = ObjectType::OT_LARGE_VEHICLE;
  const double av_front_s = 6.0;
  QCHECK_EQ(internal::ObjectInFrontOfAvFrontCenterByLength(
                object_length, object_box, type, av_front_s),
            true);
}

}  // namespace
}  // namespace planner
}  // namespace qcraft

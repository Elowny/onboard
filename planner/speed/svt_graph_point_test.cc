#include "onboard/planner/speed/svt_graph_point.h"

#include "gtest/gtest.h"

namespace qcraft::planner {
namespace {

TEST(SvtPointTest, CtorAndSetter) {
  const double s = 1.0;
  const double v = 2.0;
  const double t = 3.0;
  constexpr double kEps = 1e-6;
  const SvtPoint svt_point(s, v, t);

  SvtGraphPoint svt_graph_point(/*grid_index_s=*/0, /*grid_index_v=*/0,
                                /*index_t=*/0, svt_point);

  const double accel_cost = 5.0;
  const double edge_cost = 10.0;
  const double speed_limit_cost = 15.0;
  const double reference_speed_cost = 20.0;
  const double object_cost = 25.0;

  svt_graph_point.set_accel_cost(accel_cost);
  svt_graph_point.set_edge_cost(edge_cost);
  svt_graph_point.set_speed_limit_cost(speed_limit_cost);
  svt_graph_point.set_reference_speed_cost(reference_speed_cost);
  svt_graph_point.set_object_cost(object_cost);

  EXPECT_NEAR(svt_graph_point.accel_cost(), accel_cost, kEps);
  EXPECT_NEAR(svt_graph_point.edge_cost(), edge_cost, kEps);
  EXPECT_NEAR(svt_graph_point.speed_limit_cost(), speed_limit_cost, kEps);
  EXPECT_NEAR(svt_graph_point.reference_speed_cost(), reference_speed_cost,
              kEps);
  EXPECT_NEAR(svt_graph_point.object_cost(), object_cost, kEps);
}

}  // namespace
}  // namespace qcraft::planner

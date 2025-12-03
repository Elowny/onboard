#include "onboard/planner/trajectory_util.h"

// IWYU pragma: no_include <ext/alloc_traits.h>
//              for __alloc_traits<>::value_type

#include "gtest/gtest.h"

#include "onboard/math/vec.h"

namespace qcraft {
namespace planner {
namespace {

constexpr double kEps = 1.0e-6;

TEST(TrajectoryUtilTest, TestLerpApolloTrajectoryPointProto) {
  ApolloTrajectoryPointProto p0;
  p0.mutable_path_point()->set_x(0.0);
  p0.mutable_path_point()->set_y(1.0);
  p0.mutable_path_point()->set_theta(2.0);
  p0.mutable_path_point()->set_kappa(3.0);
  p0.mutable_path_point()->set_lambda(3.5);
  p0.mutable_path_point()->set_s(4.0);
  p0.set_v(6.0);
  p0.set_a(7.0);
  p0.set_j(7.5);
  p0.set_relative_time(8.0);

  ApolloTrajectoryPointProto p1;
  p1.mutable_path_point()->set_x(1.0);
  p1.mutable_path_point()->set_y(2.0);
  p1.mutable_path_point()->set_theta(3.0);
  p1.mutable_path_point()->set_kappa(4.0);
  p1.mutable_path_point()->set_lambda(4.5);
  p1.mutable_path_point()->set_s(5.0);
  p1.set_v(7.0);
  p1.set_a(8.0);
  p1.set_j(8.5);
  p1.set_relative_time(9.0);

  const ApolloTrajectoryPointProto p = LerpTrajectoryPoint(p0, p1, 0.5);

  EXPECT_NEAR(p.path_point().x(), 0.5, kEps);
  EXPECT_NEAR(p.path_point().y(), 1.5, kEps);
  EXPECT_NEAR(p.path_point().theta(), 2.5, kEps);
  EXPECT_NEAR(p.path_point().kappa(), 3.5, kEps);
  EXPECT_NEAR(p.path_point().lambda(), 4.0, kEps);
  EXPECT_NEAR(p.path_point().s(), 4.5, kEps);
  EXPECT_NEAR(p.v(), 6.5, kEps);
  EXPECT_NEAR(p.a(), 7.5, kEps);
  EXPECT_NEAR(p.j(), 8.0, kEps);
  EXPECT_NEAR(p.relative_time(), 8.5, kEps);
}

TEST(TrajectoryUtilTest, TestLerpTrajectoryPointProto) {
  TrajectoryPointProto p0;
  p0.mutable_pos()->set_x(0.0);
  p0.mutable_pos()->set_y(1.0);
  p0.set_theta(2.0);
  p0.set_kappa(3.0);
  p0.set_s(4.0);
  p0.set_v(6.0);
  p0.set_a(7.0);
  p0.set_t(8.0);
  p0.set_j(11.0);
  p0.set_psi(12.0);
  p0.set_chi(13.0);

  TrajectoryPointProto p1;
  p1.mutable_pos()->set_x(1.0);
  p1.mutable_pos()->set_y(2.0);
  p1.set_theta(3.0);
  p1.set_kappa(4.0);
  p1.set_s(5.0);
  p1.set_v(7.0);
  p1.set_a(8.0);
  p1.set_t(9.0);
  p1.set_j(12.0);
  p1.set_psi(13.0);
  p1.set_chi(14.0);

  const TrajectoryPointProto p = LerpTrajectoryPoint(p0, p1, 0.5);

  EXPECT_NEAR(p.pos().x(), 0.5, kEps);
  EXPECT_NEAR(p.pos().y(), 1.5, kEps);
  EXPECT_NEAR(p.theta(), 2.5, kEps);
  EXPECT_NEAR(p.kappa(), 3.5, kEps);
  EXPECT_NEAR(p.s(), 4.5, kEps);
  EXPECT_NEAR(p.v(), 6.5, kEps);
  EXPECT_NEAR(p.a(), 7.5, kEps);
  EXPECT_NEAR(p.t(), 8.5, kEps);
  EXPECT_NEAR(p.j(), 11.5, kEps);
  EXPECT_NEAR(p.psi(), 12.5, kEps);
  EXPECT_NEAR(p.chi(), 13.5, kEps);
}

TEST(TrajectoryUtilTest, TestLerpTrajectoryPoint) {
  TrajectoryPoint p0;
  p0.set_pos(Vec2d(0.0, 1.0));
  p0.set_theta(2.0);
  p0.set_kappa(3.0);
  p0.set_s(4.0);
  p0.set_v(6.0);
  p0.set_a(7.0);
  p0.set_t(8.0);
  p0.set_j(11.0);
  p0.set_psi(12.0);
  p0.set_chi(13.0);

  TrajectoryPoint p1;
  p1.set_pos(Vec2d(1.0, 2.0));
  p1.set_theta(3.0);
  p1.set_kappa(4.0);
  p1.set_s(5.0);
  p1.set_v(7.0);
  p1.set_a(8.0);
  p1.set_t(9.0);
  p1.set_j(12.0);
  p1.set_psi(13.0);
  p1.set_chi(14.0);

  const TrajectoryPoint p = LerpTrajectoryPoint(p0, p1, 0.5);

  EXPECT_NEAR(p.pos().x(), 0.5, kEps);
  EXPECT_NEAR(p.pos().y(), 1.5, kEps);
  EXPECT_NEAR(p.theta(), 2.5, kEps);
  EXPECT_NEAR(p.kappa(), 3.5, kEps);
  EXPECT_NEAR(p.s(), 4.5, kEps);
  EXPECT_NEAR(p.v(), 6.5, kEps);
  EXPECT_NEAR(p.a(), 7.5, kEps);
  EXPECT_NEAR(p.t(), 8.5, kEps);
  EXPECT_NEAR(p.j(), 11.5, kEps);
  EXPECT_NEAR(p.psi(), 12.5, kEps);
  EXPECT_NEAR(p.chi(), 13.5, kEps);
}

TEST(TrajectoryUtilTest, TestLerpSecondOrderTrajectoryPoint) {
  SecondOrderTrajectoryPoint p0;
  p0.set_pos(Vec2d(0.0, 1.0));
  p0.set_theta(2.0);
  p0.set_kappa(3.0);
  p0.set_s(4.0);
  p0.set_v(6.0);
  p0.set_a(7.0);
  p0.set_t(8.0);

  SecondOrderTrajectoryPoint p1;
  p1.set_pos(Vec2d(1.0, 2.0));
  p1.set_theta(3.0);
  p1.set_kappa(4.0);
  p1.set_s(5.0);
  p1.set_v(7.0);
  p1.set_a(8.0);
  p1.set_t(9.0);

  const SecondOrderTrajectoryPoint p = LerpTrajectoryPoint(p0, p1, 0.5);

  EXPECT_NEAR(p.pos().x(), 0.5, kEps);
  EXPECT_NEAR(p.pos().y(), 1.5, kEps);
  EXPECT_NEAR(p.theta(), 2.5, kEps);
  EXPECT_NEAR(p.kappa(), 3.5, kEps);
  EXPECT_NEAR(p.s(), 4.5, kEps);
  EXPECT_NEAR(p.v(), 6.5, kEps);
  EXPECT_NEAR(p.a(), 7.5, kEps);
  EXPECT_NEAR(p.t(), 8.5, kEps);
}

TEST(TrajectoryUtilTest, TestQueryApolloTrajectoryPointProtoByS) {
  ApolloTrajectoryPointProto p0;
  p0.mutable_path_point()->set_x(0.0);
  p0.mutable_path_point()->set_y(1.0);
  p0.mutable_path_point()->set_theta(2.0);
  p0.mutable_path_point()->set_kappa(3.0);
  p0.mutable_path_point()->set_lambda(3.5);
  p0.mutable_path_point()->set_s(4.0);
  p0.set_v(6.0);
  p0.set_a(7.0);
  p0.set_j(7.5);
  p0.set_relative_time(8.0);

  ApolloTrajectoryPointProto p1;
  p1.mutable_path_point()->set_x(1.0);
  p1.mutable_path_point()->set_y(2.0);
  p1.mutable_path_point()->set_theta(3.0);
  p1.mutable_path_point()->set_kappa(4.0);
  p1.mutable_path_point()->set_lambda(4.5);
  p1.mutable_path_point()->set_s(5.0);
  p1.set_v(7.0);
  p1.set_a(8.0);
  p1.set_j(8.5);
  p1.set_relative_time(9.0);

  ApolloTrajectoryPointProto p2;
  p2.mutable_path_point()->set_x(3.0);
  p2.mutable_path_point()->set_y(4.0);
  p2.mutable_path_point()->set_theta(5.0);
  p2.mutable_path_point()->set_kappa(6.0);
  p2.mutable_path_point()->set_lambda(6.5);
  p2.mutable_path_point()->set_s(7.0);
  p2.set_v(9.0);
  p2.set_a(10.0);
  p2.set_j(10.5);
  p2.set_relative_time(10.0);

  std::vector<ApolloTrajectoryPointProto> points = {p0, p1, p2};
  auto p = QueryApolloTrajectoryPointByS(points, 3.0);
  EXPECT_NEAR(p.path_point().y(), 1.0, kEps);
  EXPECT_NEAR(p.j(), 7.5, kEps);
  p = QueryApolloTrajectoryPointByS(points, 5.0);
  EXPECT_NEAR(p.path_point().y(), 2.0, kEps);
  EXPECT_NEAR(p.j(), 8.5, kEps);
  p = QueryApolloTrajectoryPointByS(points, 8.0);
  EXPECT_NEAR(p.path_point().y(), 4.0, kEps);
  EXPECT_NEAR(p.j(), 10.5, kEps);

  TrajectoryProto traj;
  *traj.mutable_trajectory_point() = {points.begin(), points.end()};
  p = QueryApolloTrajectoryPointByS(traj, 3.0);
  EXPECT_NEAR(p.path_point().y(), 1.0, kEps);
  EXPECT_NEAR(p.j(), 7.5, kEps);
  p = QueryApolloTrajectoryPointByS(traj, 5.0);
  EXPECT_NEAR(p.path_point().y(), 2.0, kEps);
  EXPECT_NEAR(p.j(), 8.5, kEps);
  p = QueryApolloTrajectoryPointByS(traj, 8.0);
  EXPECT_NEAR(p.path_point().y(), 4.0, kEps);
  EXPECT_NEAR(p.j(), 10.5, kEps);
}

TEST(TrajectoryUtilTest, TestQueryTrajectoryPointByS) {
  TrajectoryPoint p0;
  p0.set_pos(Vec2d(0.0, 1.0));
  p0.set_theta(2.0);
  p0.set_kappa(3.0);
  p0.set_psi(3.5);
  p0.set_chi(5.0);
  p0.set_s(4.0);
  p0.set_v(6.0);
  p0.set_a(7.0);
  p0.set_j(7.5);
  p0.set_t(8.0);

  TrajectoryPoint p1;
  p1.set_pos(Vec2d(1.0, 2.0));
  p1.set_theta(3.0);
  p1.set_kappa(4.0);
  p1.set_psi(4.5);
  p1.set_chi(6.0);
  p1.set_s(5.0);
  p1.set_v(7.0);
  p1.set_a(8.0);
  p1.set_j(8.5);
  p1.set_t(10.0);

  TrajectoryPoint p2;
  p2.set_pos(Vec2d(3.0, 4.0));
  p2.set_theta(5.0);
  p2.set_kappa(6.0);
  p2.set_psi(5.5);
  p2.set_chi(7.0);
  p2.set_s(7.0);
  p2.set_v(9.0);
  p2.set_a(10.0);
  p2.set_j(10.5);
  p2.set_t(10.0);

  std::vector<TrajectoryPoint> points = {p0, p1, p2};
  auto p = QueryTrajectoryPointByS(points, 3.0);
  EXPECT_NEAR(p.pos().y(), 1.0, kEps);
  EXPECT_NEAR(p.j(), 7.5, kEps);
  p = QueryTrajectoryPointByS(points, 5.0);
  EXPECT_NEAR(p.pos().y(), 2.0, kEps);
  EXPECT_NEAR(p.j(), 8.5, kEps);
  p = QueryTrajectoryPointByS(points, 8.0);
  EXPECT_NEAR(p.pos().y(), 4.0, kEps);
  EXPECT_NEAR(p.j(), 10.5, kEps);

  std::vector<TrajectoryPointProto> traj;
  traj.reserve(points.size());
  for (int i = 0; i < points.size(); ++i) {
    TrajectoryPointProto p;
    points[i].ToProto(&p);
    traj.push_back(p);
  }
  auto p_proto = QueryTrajectoryPointByS(traj, 3.0);
  EXPECT_NEAR(p_proto.pos().y(), 1.0, kEps);
  EXPECT_NEAR(p_proto.j(), 7.5, kEps);
  p_proto = QueryTrajectoryPointByS(traj, 5.0);
  EXPECT_NEAR(p_proto.pos().y(), 2.0, kEps);
  EXPECT_NEAR(p_proto.j(), 8.5, kEps);
  p_proto = QueryTrajectoryPointByS(traj, 8.0);
  EXPECT_NEAR(p_proto.pos().y(), 4.0, kEps);
  EXPECT_NEAR(p_proto.j(), 10.5, kEps);
}

TEST(TrajectoryUtilTest, TestTrajectoryPointProtoToApolloTrajectoryPointProto) {
  TrajectoryPointProto p0;
  p0.mutable_pos()->set_x(0.0);
  p0.mutable_pos()->set_y(1.0);
  p0.set_theta(2.0);
  p0.set_kappa(3.0);
  p0.set_s(4.0);
  p0.set_v(6.0);
  p0.set_a(7.0);
  p0.set_t(8.0);
  p0.set_j(11.0);
  p0.set_psi(12.0);
  p0.set_chi(13.0);

  const ApolloTrajectoryPointProto p = ToApolloTrajectoryPointProto(p0);

  EXPECT_NEAR(p.path_point().x(), 0.0, kEps);
  EXPECT_NEAR(p.path_point().y(), 1.0, kEps);
  EXPECT_NEAR(p.path_point().theta(), 2.0, kEps);
  EXPECT_NEAR(p.path_point().kappa(), 3.0, kEps);
  EXPECT_NEAR(p.path_point().lambda(), 2.0, kEps);
  EXPECT_NEAR(p.path_point().s(), 4.0, kEps);
  EXPECT_NEAR(p.v(), 6.0, kEps);
  EXPECT_NEAR(p.a(), 7.0, kEps);
  EXPECT_NEAR(p.j(), 11.0, kEps);
  EXPECT_NEAR(p.relative_time(), 8.0, kEps);
}

TEST(TrajectoryUtilTest, TestApolloTrajectoryPointProtoToTrajectoryPointProto) {
  ApolloTrajectoryPointProto p0;
  p0.mutable_path_point()->set_x(0.0);
  p0.mutable_path_point()->set_y(1.0);
  p0.mutable_path_point()->set_theta(2.0);
  p0.mutable_path_point()->set_kappa(3.0);
  p0.mutable_path_point()->set_lambda(3.5);
  p0.mutable_path_point()->set_s(4.0);
  p0.set_v(6.0);
  p0.set_a(7.0);
  p0.set_j(7.5);
  p0.set_relative_time(8.0);

  const TrajectoryPointProto p = ToTrajectoryPointProto(p0);

  EXPECT_NEAR(p.pos().x(), 0.0, kEps);
  EXPECT_NEAR(p.pos().y(), 1.0, kEps);
  EXPECT_NEAR(p.theta(), 2.0, kEps);
  EXPECT_NEAR(p.kappa(), 3.0, kEps);
  EXPECT_NEAR(p.s(), 4.0, kEps);
  EXPECT_NEAR(p.v(), 6.0, kEps);
  EXPECT_NEAR(p.a(), 7.0, kEps);
  EXPECT_NEAR(p.t(), 8.0, kEps);
  EXPECT_NEAR(p.j(), 7.5, kEps);
  EXPECT_NEAR(p.psi(), 21.0, kEps);
  EXPECT_NEAR(p.chi(), 0.0, kEps);
}

TEST(TrajectoryUtilTest,
     TestSecondOrderApolloTrajectoryPointProtoToTrajectoryPointProto) {
  ApolloTrajectoryPointProto p0;
  p0.mutable_path_point()->set_x(0.0);
  p0.mutable_path_point()->set_y(1.0);
  p0.mutable_path_point()->set_theta(2.0);
  p0.mutable_path_point()->set_kappa(3.0);
  p0.mutable_path_point()->set_s(4.0);
  p0.set_v(6.0);
  p0.set_a(7.0);
  p0.set_relative_time(8.0);

  const TrajectoryPointProto p =
      ToTrajectoryPointProtoFromSecondOrderApollo(p0);

  EXPECT_NEAR(p.pos().x(), 0.0, kEps);
  EXPECT_NEAR(p.pos().y(), 1.0, kEps);
  EXPECT_NEAR(p.theta(), 2.0, kEps);
  EXPECT_NEAR(p.kappa(), 3.0, kEps);
  EXPECT_NEAR(p.s(), 4.0, kEps);
  EXPECT_NEAR(p.v(), 6.0, kEps);
  EXPECT_NEAR(p.a(), 7.0, kEps);
  EXPECT_NEAR(p.t(), 8.0, kEps);
  EXPECT_NEAR(p.j(), 0.0, kEps);
  EXPECT_NEAR(p.psi(), 0.0, kEps);
  EXPECT_NEAR(p.chi(), 0.0, kEps);
}

TEST(TrajectoryUtilTest,
     TestBatchApolloTrajectoryPointProtoToTrajectoryPointProto) {
  ApolloTrajectoryPointProto p0;
  p0.mutable_path_point()->set_x(0.0);
  p0.mutable_path_point()->set_y(1.0);
  p0.mutable_path_point()->set_theta(2.0);
  p0.mutable_path_point()->set_kappa(3.0);
  p0.mutable_path_point()->set_lambda(3.5);
  p0.mutable_path_point()->set_s(4.0);
  p0.set_v(6.0);
  p0.set_a(7.0);
  p0.set_j(7.5);
  p0.set_relative_time(8.0);

  ApolloTrajectoryPointProto p1;
  p1.mutable_path_point()->set_x(1.0);
  p1.mutable_path_point()->set_y(2.0);
  p1.mutable_path_point()->set_theta(3.0);
  p1.mutable_path_point()->set_kappa(4.0);
  p1.mutable_path_point()->set_lambda(4.5);
  p1.mutable_path_point()->set_s(5.0);
  p1.set_v(7.0);
  p1.set_a(8.0);
  p1.set_j(8.5);
  p1.set_relative_time(9.0);

  ApolloTrajectoryPointProto p2;
  p2.mutable_path_point()->set_x(3.0);
  p2.mutable_path_point()->set_y(4.0);
  p2.mutable_path_point()->set_theta(5.0);
  p2.mutable_path_point()->set_kappa(6.0);
  p2.mutable_path_point()->set_lambda(6.5);
  p2.mutable_path_point()->set_s(7.0);
  p2.set_v(9.0);
  p2.set_a(10.0);
  p2.set_j(10.5);
  p2.set_relative_time(10.0);

  const std::vector<TrajectoryPointProto> res =
      ToTrajectoryPointProto({p0, p1, p2});

  EXPECT_NEAR(res[0].pos().x(), 0.0, kEps);
  EXPECT_NEAR(res[0].pos().y(), 1.0, kEps);
  EXPECT_NEAR(res[0].theta(), 2.0, kEps);
  EXPECT_NEAR(res[0].kappa(), 3.0, kEps);
  EXPECT_NEAR(res[0].s(), 4.0, kEps);
  EXPECT_NEAR(res[0].v(), 6.0, kEps);
  EXPECT_NEAR(res[0].a(), 7.0, kEps);
  EXPECT_NEAR(res[0].t(), 8.0, kEps);
  EXPECT_NEAR(res[0].j(), 7.5, kEps);
  EXPECT_NEAR(res[0].psi(), 21.0, kEps);
  EXPECT_NEAR(res[0].chi(), 10.5, kEps);

  EXPECT_NEAR(res[1].pos().x(), 1.0, kEps);
  EXPECT_NEAR(res[1].pos().y(), 2.0, kEps);
  EXPECT_NEAR(res[1].theta(), 3.0, kEps);
  EXPECT_NEAR(res[1].kappa(), 4.0, kEps);
  EXPECT_NEAR(res[1].s(), 5.0, kEps);
  EXPECT_NEAR(res[1].v(), 7.0, kEps);
  EXPECT_NEAR(res[1].a(), 8.0, kEps);
  EXPECT_NEAR(res[1].t(), 9.0, kEps);
  EXPECT_NEAR(res[1].j(), 8.5, kEps);
  EXPECT_NEAR(res[1].psi(), 31.5, kEps);
  EXPECT_NEAR(res[1].chi(), 27.0, kEps);

  EXPECT_NEAR(res[2].pos().x(), 3.0, kEps);
  EXPECT_NEAR(res[2].pos().y(), 4.0, kEps);
  EXPECT_NEAR(res[2].theta(), 5.0, kEps);
  EXPECT_NEAR(res[2].kappa(), 6.0, kEps);
  EXPECT_NEAR(res[2].s(), 7.0, kEps);
  EXPECT_NEAR(res[2].v(), 9.0, kEps);
  EXPECT_NEAR(res[2].a(), 10.0, kEps);
  EXPECT_NEAR(res[2].t(), 10.0, kEps);
  EXPECT_NEAR(res[2].j(), 10.5, kEps);
  EXPECT_NEAR(res[2].psi(), 58.5, kEps);
  EXPECT_NEAR(res[2].chi(), 27.0, kEps);
}

TEST(TrajectoryUtilTest,
     TestBatchSecondOrderApolloTrajectoryPointProtoToTrajectoryPointProto) {
  ApolloTrajectoryPointProto p0;
  p0.mutable_path_point()->set_x(0.0);
  p0.mutable_path_point()->set_y(1.0);
  p0.mutable_path_point()->set_theta(2.0);
  p0.mutable_path_point()->set_kappa(3.0);
  p0.mutable_path_point()->set_s(4.0);
  p0.set_v(6.0);
  p0.set_a(7.0);
  p0.set_relative_time(8.0);

  ApolloTrajectoryPointProto p1;
  p1.mutable_path_point()->set_x(1.0);
  p1.mutable_path_point()->set_y(2.0);
  p1.mutable_path_point()->set_theta(3.0);
  p1.mutable_path_point()->set_kappa(4.0);
  p1.mutable_path_point()->set_s(5.0);
  p1.set_v(7.0);
  p1.set_a(8.0);
  p1.set_relative_time(9.0);

  ApolloTrajectoryPointProto p2;
  p2.mutable_path_point()->set_x(3.0);
  p2.mutable_path_point()->set_y(4.0);
  p2.mutable_path_point()->set_theta(5.0);
  p2.mutable_path_point()->set_kappa(6.0);
  p2.mutable_path_point()->set_s(7.0);
  p2.set_v(9.0);
  p2.set_a(10.0);
  p2.set_relative_time(10.0);

  const std::vector<TrajectoryPointProto> res =
      ToTrajectoryPointProtoFromSecondOrderApollo({p0, p1, p2});

  EXPECT_NEAR(res[0].pos().x(), 0.0, kEps);
  EXPECT_NEAR(res[0].pos().y(), 1.0, kEps);
  EXPECT_NEAR(res[0].theta(), 2.0, kEps);
  EXPECT_NEAR(res[0].kappa(), 3.0, kEps);
  EXPECT_NEAR(res[0].s(), 4.0, kEps);
  EXPECT_NEAR(res[0].v(), 6.0, kEps);
  EXPECT_NEAR(res[0].a(), 7.0, kEps);
  EXPECT_NEAR(res[0].t(), 8.0, kEps);
  EXPECT_NEAR(res[0].j(), 1.0, kEps);
  EXPECT_NEAR(res[0].psi(), 1.0, kEps);
  EXPECT_NEAR(res[0].chi(), 1.0, kEps);

  EXPECT_NEAR(res[1].pos().x(), 1.0, kEps);
  EXPECT_NEAR(res[1].pos().y(), 2.0, kEps);
  EXPECT_NEAR(res[1].theta(), 3.0, kEps);
  EXPECT_NEAR(res[1].kappa(), 4.0, kEps);
  EXPECT_NEAR(res[1].s(), 5.0, kEps);
  EXPECT_NEAR(res[1].v(), 7.0, kEps);
  EXPECT_NEAR(res[1].a(), 8.0, kEps);
  EXPECT_NEAR(res[1].t(), 9.0, kEps);
  EXPECT_NEAR(res[1].j(), 2.0, kEps);
  EXPECT_NEAR(res[1].psi(), 2.0, kEps);
  EXPECT_NEAR(res[1].chi(), 0.0, kEps);

  EXPECT_NEAR(res[2].pos().x(), 3.0, kEps);
  EXPECT_NEAR(res[2].pos().y(), 4.0, kEps);
  EXPECT_NEAR(res[2].theta(), 5.0, kEps);
  EXPECT_NEAR(res[2].kappa(), 6.0, kEps);
  EXPECT_NEAR(res[2].s(), 7.0, kEps);
  EXPECT_NEAR(res[2].v(), 9.0, kEps);
  EXPECT_NEAR(res[2].a(), 10.0, kEps);
  EXPECT_NEAR(res[2].t(), 10.0, kEps);
  EXPECT_NEAR(res[2].j(), 2.0, kEps);
  EXPECT_NEAR(res[2].psi(), 2.0, kEps);
  EXPECT_NEAR(res[2].chi(), 0.0, kEps);
}

TEST(TrajectoryUtilTest, TestBatchApolloTrajectoryPointProtoToTrajectoryPoint) {
  ApolloTrajectoryPointProto p0;
  p0.mutable_path_point()->set_x(0.0);
  p0.mutable_path_point()->set_y(1.0);
  p0.mutable_path_point()->set_theta(2.0);
  p0.mutable_path_point()->set_kappa(3.0);
  p0.mutable_path_point()->set_lambda(3.5);
  p0.mutable_path_point()->set_s(4.0);
  p0.set_v(6.0);
  p0.set_a(7.0);
  p0.set_j(7.5);
  p0.set_relative_time(8.0);

  ApolloTrajectoryPointProto p1;
  p1.mutable_path_point()->set_x(1.0);
  p1.mutable_path_point()->set_y(2.0);
  p1.mutable_path_point()->set_theta(3.0);
  p1.mutable_path_point()->set_kappa(4.0);
  p1.mutable_path_point()->set_lambda(4.5);
  p1.mutable_path_point()->set_s(5.0);
  p1.set_v(7.0);
  p1.set_a(8.0);
  p1.set_j(8.5);
  p1.set_relative_time(9.0);

  ApolloTrajectoryPointProto p2;
  p2.mutable_path_point()->set_x(3.0);
  p2.mutable_path_point()->set_y(4.0);
  p2.mutable_path_point()->set_theta(5.0);
  p2.mutable_path_point()->set_kappa(6.0);
  p2.mutable_path_point()->set_lambda(6.5);
  p2.mutable_path_point()->set_s(7.0);
  p2.set_v(9.0);
  p2.set_a(10.0);
  p2.set_j(10.5);
  p2.set_relative_time(10.0);

  const std::vector<TrajectoryPoint> res = ToTrajectoryPoint({p0, p1, p2});

  EXPECT_NEAR(res[0].pos().x(), 0.0, kEps);
  EXPECT_NEAR(res[0].pos().y(), 1.0, kEps);
  EXPECT_NEAR(res[0].theta(), 2.0, kEps);
  EXPECT_NEAR(res[0].kappa(), 3.0, kEps);
  EXPECT_NEAR(res[0].s(), 4.0, kEps);
  EXPECT_NEAR(res[0].v(), 6.0, kEps);
  EXPECT_NEAR(res[0].a(), 7.0, kEps);
  EXPECT_NEAR(res[0].t(), 8.0, kEps);
  EXPECT_NEAR(res[0].j(), 7.5, kEps);
  EXPECT_NEAR(res[0].psi(), 21.0, kEps);
  EXPECT_NEAR(res[0].chi(), 10.5, kEps);

  EXPECT_NEAR(res[1].pos().x(), 1.0, kEps);
  EXPECT_NEAR(res[1].pos().y(), 2.0, kEps);
  EXPECT_NEAR(res[1].theta(), 3.0, kEps);
  EXPECT_NEAR(res[1].kappa(), 4.0, kEps);
  EXPECT_NEAR(res[1].s(), 5.0, kEps);
  EXPECT_NEAR(res[1].v(), 7.0, kEps);
  EXPECT_NEAR(res[1].a(), 8.0, kEps);
  EXPECT_NEAR(res[1].t(), 9.0, kEps);
  EXPECT_NEAR(res[1].j(), 8.5, kEps);
  EXPECT_NEAR(res[1].psi(), 31.5, kEps);
  EXPECT_NEAR(res[1].chi(), 27.0, kEps);

  EXPECT_NEAR(res[2].pos().x(), 3.0, kEps);
  EXPECT_NEAR(res[2].pos().y(), 4.0, kEps);
  EXPECT_NEAR(res[2].theta(), 5.0, kEps);
  EXPECT_NEAR(res[2].kappa(), 6.0, kEps);
  EXPECT_NEAR(res[2].s(), 7.0, kEps);
  EXPECT_NEAR(res[2].v(), 9.0, kEps);
  EXPECT_NEAR(res[2].a(), 10.0, kEps);
  EXPECT_NEAR(res[2].t(), 10.0, kEps);
  EXPECT_NEAR(res[2].j(), 10.5, kEps);
  EXPECT_NEAR(res[2].psi(), 58.5, kEps);
  EXPECT_NEAR(res[2].chi(), 27.0, kEps);
}

TEST(TrajectoryUtilTest,
     TestBatchSecondOrderApolloTrajectoryPointProtoToTrajectoryPoint) {
  ApolloTrajectoryPointProto p0;
  p0.mutable_path_point()->set_x(0.0);
  p0.mutable_path_point()->set_y(1.0);
  p0.mutable_path_point()->set_theta(2.0);
  p0.mutable_path_point()->set_kappa(3.0);
  p0.mutable_path_point()->set_s(4.0);
  p0.set_v(6.0);
  p0.set_a(7.0);
  p0.set_relative_time(8.0);

  ApolloTrajectoryPointProto p1;
  p1.mutable_path_point()->set_x(1.0);
  p1.mutable_path_point()->set_y(2.0);
  p1.mutable_path_point()->set_theta(3.0);
  p1.mutable_path_point()->set_kappa(4.0);
  p1.mutable_path_point()->set_s(5.0);
  p1.set_v(7.0);
  p1.set_a(8.0);
  p1.set_relative_time(9.0);

  ApolloTrajectoryPointProto p2;
  p2.mutable_path_point()->set_x(3.0);
  p2.mutable_path_point()->set_y(4.0);
  p2.mutable_path_point()->set_theta(5.0);
  p2.mutable_path_point()->set_kappa(6.0);
  p2.mutable_path_point()->set_s(7.0);
  p2.set_v(9.0);
  p2.set_a(10.0);
  p2.set_relative_time(10.0);

  const std::vector<TrajectoryPoint> res =
      ToTrajectoryPointFromSecondOrderApollo({p0, p1, p2});

  EXPECT_NEAR(res[0].pos().x(), 0.0, kEps);
  EXPECT_NEAR(res[0].pos().y(), 1.0, kEps);
  EXPECT_NEAR(res[0].theta(), 2.0, kEps);
  EXPECT_NEAR(res[0].kappa(), 3.0, kEps);
  EXPECT_NEAR(res[0].s(), 4.0, kEps);
  EXPECT_NEAR(res[0].v(), 6.0, kEps);
  EXPECT_NEAR(res[0].a(), 7.0, kEps);
  EXPECT_NEAR(res[0].t(), 8.0, kEps);
  EXPECT_NEAR(res[0].j(), 1.0, kEps);
  EXPECT_NEAR(res[0].psi(), 1.0, kEps);
  EXPECT_NEAR(res[0].chi(), 1.0, kEps);

  EXPECT_NEAR(res[1].pos().x(), 1.0, kEps);
  EXPECT_NEAR(res[1].pos().y(), 2.0, kEps);
  EXPECT_NEAR(res[1].theta(), 3.0, kEps);
  EXPECT_NEAR(res[1].kappa(), 4.0, kEps);
  EXPECT_NEAR(res[1].s(), 5.0, kEps);
  EXPECT_NEAR(res[1].v(), 7.0, kEps);
  EXPECT_NEAR(res[1].a(), 8.0, kEps);
  EXPECT_NEAR(res[1].t(), 9.0, kEps);
  EXPECT_NEAR(res[1].j(), 2.0, kEps);
  EXPECT_NEAR(res[1].psi(), 2.0, kEps);
  EXPECT_NEAR(res[1].chi(), 0.0, kEps);

  EXPECT_NEAR(res[2].pos().x(), 3.0, kEps);
  EXPECT_NEAR(res[2].pos().y(), 4.0, kEps);
  EXPECT_NEAR(res[2].theta(), 5.0, kEps);
  EXPECT_NEAR(res[2].kappa(), 6.0, kEps);
  EXPECT_NEAR(res[2].s(), 7.0, kEps);
  EXPECT_NEAR(res[2].v(), 9.0, kEps);
  EXPECT_NEAR(res[2].a(), 10.0, kEps);
  EXPECT_NEAR(res[2].t(), 10.0, kEps);
  EXPECT_NEAR(res[2].j(), 2.0, kEps);
  EXPECT_NEAR(res[2].psi(), 2.0, kEps);
  EXPECT_NEAR(res[2].chi(), 0.0, kEps);
}

TEST(TrajectoryUtilTest, TestToPathFromApolloTrajectory) {
  ApolloTrajectoryPointProto p0;
  p0.mutable_path_point()->set_x(0.0);
  p0.mutable_path_point()->set_y(1.0);
  p0.mutable_path_point()->set_theta(2.0);
  p0.mutable_path_point()->set_kappa(3.0);
  p0.mutable_path_point()->set_s(4.0);
  p0.set_v(6.0);
  p0.set_a(7.0);
  p0.set_relative_time(8.0);

  ApolloTrajectoryPointProto p1;
  p1.mutable_path_point()->set_x(1.0);
  p1.mutable_path_point()->set_y(2.0);
  p1.mutable_path_point()->set_theta(3.0);
  p1.mutable_path_point()->set_kappa(4.0);
  p1.mutable_path_point()->set_s(5.0);
  p1.set_v(7.0);
  p1.set_a(8.0);
  p1.set_relative_time(9.0);

  ApolloTrajectoryPointProto p2;
  p2.mutable_path_point()->set_x(3.0);
  p2.mutable_path_point()->set_y(4.0);
  p2.mutable_path_point()->set_theta(5.0);
  p2.mutable_path_point()->set_kappa(6.0);
  p2.mutable_path_point()->set_s(7.0);
  p2.set_v(9.0);
  p2.set_a(10.0);
  p2.set_relative_time(10.0);

  {
    TrajectoryProto proto;
    *proto.add_trajectory_point() = p0;
    *proto.add_trajectory_point() = p1;
    *proto.add_trajectory_point() = p2;

    const std::vector<PathPoint> res =
        ToPathFromApolloTrajectory(proto.trajectory_point());

    EXPECT_NEAR(res[0].x(), 0.0, kEps);
    EXPECT_NEAR(res[0].y(), 1.0, kEps);
    EXPECT_NEAR(res[0].theta(), 2.0, kEps);
    EXPECT_NEAR(res[0].kappa(), 3.0, kEps);
    EXPECT_NEAR(res[0].s(), 4.0, kEps);

    EXPECT_NEAR(res[1].x(), 1.0, kEps);
    EXPECT_NEAR(res[1].y(), 2.0, kEps);
    EXPECT_NEAR(res[1].theta(), 3.0, kEps);
    EXPECT_NEAR(res[1].kappa(), 4.0, kEps);
    EXPECT_NEAR(res[1].s(), 5.0, kEps);

    EXPECT_NEAR(res[2].x(), 3.0, kEps);
    EXPECT_NEAR(res[2].y(), 4.0, kEps);
    EXPECT_NEAR(res[2].theta(), 5.0, kEps);
    EXPECT_NEAR(res[2].kappa(), 6.0, kEps);
    EXPECT_NEAR(res[2].s(), 7.0, kEps);
  }

  {
    const std::vector<ApolloTrajectoryPointProto> traj_points = {p0, p1, p2};

    const std::vector<PathPoint> res = ToPathFromApolloTrajectory(traj_points);

    EXPECT_NEAR(res[0].x(), 0.0, kEps);
    EXPECT_NEAR(res[0].y(), 1.0, kEps);
    EXPECT_NEAR(res[0].theta(), 2.0, kEps);
    EXPECT_NEAR(res[0].kappa(), 3.0, kEps);
    EXPECT_NEAR(res[0].s(), 4.0, kEps);

    EXPECT_NEAR(res[1].x(), 1.0, kEps);
    EXPECT_NEAR(res[1].y(), 2.0, kEps);
    EXPECT_NEAR(res[1].theta(), 3.0, kEps);
    EXPECT_NEAR(res[1].kappa(), 4.0, kEps);
    EXPECT_NEAR(res[1].s(), 5.0, kEps);

    EXPECT_NEAR(res[2].x(), 3.0, kEps);
    EXPECT_NEAR(res[2].y(), 4.0, kEps);
    EXPECT_NEAR(res[2].theta(), 5.0, kEps);
    EXPECT_NEAR(res[2].kappa(), 6.0, kEps);
    EXPECT_NEAR(res[2].s(), 7.0, kEps);
  }
}

}  // namespace
}  // namespace planner
}  // namespace qcraft

#ifndef ONBOARD_PLANNER_TRAJECTORY_UTIL_H_
#define ONBOARD_PLANNER_TRAJECTORY_UTIL_H_

#include <algorithm>
#include <cmath>
#include <iterator>
#include <vector>

#include "absl/types/span.h"

#include "onboard/global/buffered_logger.h"
#include "onboard/lite/check.h"
#include "onboard/math/geometry/proto/affine_transformation.pb.h"
#include "onboard/math/util.h"
#include "onboard/planner/second_order_trajectory_point.h"
#include "onboard/planner/trajectory_point.h"
#include "onboard/planner/util/path_util.h"
#include "onboard/prediction/predicted_trajectory.h"
#include "onboard/proto/trajectory.pb.h"
#include "onboard/proto/trajectory_point.pb.h"

namespace qcraft {
namespace planner {

// *********************** Lerp and Query Functions ***********************
// Note: This is an approximate function which lerps each field between two
// ApolloTrajectoryPointProto given a lerp factor 'alpha'.
inline void LerpTrajPoint(const ApolloTrajectoryPointProto& p0,
                          const ApolloTrajectoryPointProto& p1, double alpha,
                          ApolloTrajectoryPointProto* p) {
  LerpPathPoint(p0.path_point(), p1.path_point(), alpha,
                p->mutable_path_point());
#define LERP_FIELD(field, lerp) \
  p->set_##field(lerp(p0.field(), p1.field(), alpha));
  LERP_FIELD(v, Lerp)
  LERP_FIELD(a, Lerp)
  LERP_FIELD(j, Lerp)
  LERP_FIELD(relative_time, Lerp)
#undef LERP_FIELD
}

void LerpTrajPoint(const TrajectoryPointProto& p0,
                   const TrajectoryPointProto& p1, double alpha,
                   TrajectoryPointProto* p);

void LerpTrajPoint(const TrajectoryPoint& p0, const TrajectoryPoint& p1,
                   double alpha, TrajectoryPoint* p);

void LerpTrajPoint(const SecondOrderTrajectoryPoint& p0,
                   const SecondOrderTrajectoryPoint& p1, double alpha,
                   SecondOrderTrajectoryPoint* p);

template <typename T>
inline T LerpTrajectoryPoint(const T& p0, const T& p1, double alpha) {
  T p;
  LerpTrajPoint(p0, p1, alpha, &p);
  return p;
}

// For ApolloTrajectoryPointProto and TrajectoryProto.
template <typename ApolloTrajectoryPointProtoIterator>
inline ApolloTrajectoryPointProto QueryApolloTrajectoryPointByS(
    ApolloTrajectoryPointProtoIterator begin,
    ApolloTrajectoryPointProtoIterator end, double s) {
  const auto it = std::lower_bound(begin, end, s, [](const auto& p, double s) {
    return p.path_point().s() < s;
  });

  if (it == begin) return *begin;
  if (it == end) return *(end - 1);
  constexpr double kEps = 1.0e-6;
  const double prev_s = std::prev(it)->path_point().s();
  const double curr_s = it->path_point().s();
  if (std::fabs(curr_s - prev_s) < kEps) {
    return *it;
  }
  const double alpha = (s - prev_s) / (curr_s - prev_s);
  return LerpTrajectoryPoint(*std::prev(it), *it, alpha);
}

// For ApolloTrajectoryPointProto and TrajectoryProto.
template <typename ApolloTrajectoryPointProtoIterator>
inline ApolloTrajectoryPointProto QueryApolloTrajectoryPointByT(
    ApolloTrajectoryPointProtoIterator begin,
    ApolloTrajectoryPointProtoIterator end, double t) {
  const auto it = std::lower_bound(begin, end, t, [](const auto& p, double t) {
    return p.relative_time() < t;
  });

  if (it == begin) return *begin;
  if (it == end) return *(end - 1);
  constexpr double kEps = 1.0e-6;
  const double prev_t = std::prev(it)->relative_time();
  const double curr_t = it->relative_time();
  if (std::fabs(curr_t - prev_t) < kEps) {
    return *it;
  }
  const double alpha = (t - prev_t) / (curr_t - prev_t);
  return LerpTrajectoryPoint(*std::prev(it), *it, alpha);
}

inline ApolloTrajectoryPointProto QueryApolloTrajectoryPointByS(
    absl::Span<const ApolloTrajectoryPointProto> traj_points, double s) {
  QCHECK(!traj_points.empty());

  return QueryApolloTrajectoryPointByS(traj_points.begin(), traj_points.end(),
                                       s);
}

inline ApolloTrajectoryPointProto QueryApolloTrajectoryPointByS(
    const TrajectoryProto& trajectory, double s) {
  const auto& traj_points = trajectory.trajectory_point();
  QCHECK(!traj_points.empty());

  return QueryApolloTrajectoryPointByS(traj_points.begin(), traj_points.end(),
                                       s);
}

// For TrajectoryPointProto, TrajectoryPoint and SecondOrderTrajectoryPoint.
template <typename T>
inline typename T::value_type QueryTrajectoryPointByS(const T& traj_points,
                                                      double s) {
  const auto begin = traj_points.begin();
  const auto end = traj_points.end();
  const auto it = std::lower_bound(
      begin, end, s, [](const auto& p, double s) { return p.s() < s; });

  if (it == begin) return *begin;
  if (it == end) return *(end - 1);
  constexpr double kEps = 1.0e-6;
  const double prev_s = std::prev(it)->s();
  const double curr_s = it->s();
  if (std::fabs(curr_s - prev_s) < kEps) {
    return *it;
  }
  const double alpha = (s - prev_s) / (curr_s - prev_s);
  return LerpTrajectoryPoint(*std::prev(it), *it, alpha);
}

// *********************** Conversions ***********************
// Note: States beyond third order will be lost.
inline ApolloTrajectoryPointProto ToApolloTrajectoryPointProto(
    const TrajectoryPointProto& p0) {
  ApolloTrajectoryPointProto p;
  p.mutable_path_point()->set_x(p0.pos().x());
  p.mutable_path_point()->set_y(p0.pos().y());
  p.mutable_path_point()->set_theta(p0.theta());
  p.mutable_path_point()->set_kappa(p0.kappa());
  p.mutable_path_point()->set_lambda(
      TrajectoryPoint::ComputeLambda(p0.v(), p0.psi()));
  p.mutable_path_point()->set_s(p0.s());
  p.set_relative_time(p0.t());
  p.set_v(p0.v());
  p.set_a(p0.a());
  p.set_j(p0.j());
  return p;
}

// Note: States beyond third order are not filled.
inline TrajectoryPointProto ToTrajectoryPointProto(
    const ApolloTrajectoryPointProto& p0) {
  TrajectoryPointProto p;
  p.mutable_pos()->set_x(p0.path_point().x());
  p.mutable_pos()->set_y(p0.path_point().y());
  p.set_theta(p0.path_point().theta());
  p.set_kappa(p0.path_point().kappa());
  p.set_psi(p0.path_point().lambda() * p0.v());
  p.set_s(p0.path_point().s());
  p.set_t(p0.relative_time());
  p.set_v(p0.v());
  p.set_a(p0.a());
  p.set_j(p0.j());
  return p;
}

// Note: States beyond second order are not filled.
inline TrajectoryPointProto ToTrajectoryPointProtoFromSecondOrderApollo(
    const ApolloTrajectoryPointProto& p0) {
  TrajectoryPointProto p;
  p.mutable_pos()->set_x(p0.path_point().x());
  p.mutable_pos()->set_y(p0.path_point().y());
  p.set_theta(p0.path_point().theta());
  p.set_kappa(p0.path_point().kappa());
  p.set_s(p0.path_point().s());
  p.set_t(p0.relative_time());
  p.set_v(p0.v());
  p.set_a(p0.a());
  return p;
}

inline PoseTrajectoryPointProto ToPoseTrajectoryPointProto(
    const ApolloTrajectoryPointProto& p0) {
  PoseTrajectoryPointProto p;
  p.mutable_pos()->set_x(p0.path_point().x());
  p.mutable_pos()->set_y(p0.path_point().y());
  p.set_theta(p0.path_point().theta());
  p.set_t(p0.relative_time());
  return p;
}

// Note: States beyond third order will be lost.
std::vector<ApolloTrajectoryPointProto> ToApolloTrajectoryPointProto(
    absl::Span<const TrajectoryPointProto> traj);

// Note: pos_cov will be lost
std::vector<ApolloTrajectoryPointProto> ToApolloTrajectoryPointProto(
    absl::Span<const ::qcraft::prediction::PredictedTrajectoryPoint> traj);

// Note: States beyond third order are filled by finite
// difference.
std::vector<TrajectoryPointProto> ToTrajectoryPointProto(
    absl::Span<const ApolloTrajectoryPointProto> traj);
// Note: States beyond second order are filled by finite
// difference.
std::vector<TrajectoryPointProto> ToTrajectoryPointProtoFromSecondOrderApollo(
    absl::Span<const ApolloTrajectoryPointProto> traj);

std::vector<TrajectoryPointProto> ToTrajectoryPointProto(
    absl::Span<const TrajectoryPoint> traj);

std::vector<TrajectoryPoint> ToTrajectoryPoint(
    absl::Span<const TrajectoryPointProto> traj);

// Note: States beyond third order will be lost.
std::vector<ApolloTrajectoryPointProto> ToApolloTrajectoryPointProto(
    absl::Span<const TrajectoryPoint> traj);

// Note: States beyond third order are filled by finite
// difference.
std::vector<TrajectoryPoint> ToTrajectoryPoint(
    absl::Span<const ApolloTrajectoryPointProto> traj);
// Note: States beyond second order are filled by finite
// difference.
std::vector<TrajectoryPoint> ToTrajectoryPointFromSecondOrderApollo(
    absl::Span<const ApolloTrajectoryPointProto> traj);

// Shift trajectory to integer point by time, fill point with shift number at
// the end of trajectory.
std::vector<ApolloTrajectoryPointProto> ShiftTrajectoryByTime(
    double shift_time,
    const std::vector<ApolloTrajectoryPointProto>& trajectory_points,
    double max_decel_jerk, double max_acc_jerk);

template <template <class...> class Container>
std::vector<PathPoint> ToPathFromApolloTrajectory(
    const Container<ApolloTrajectoryPointProto>& traj_points) {
  std::vector<PathPoint> path_points;
  path_points.reserve(traj_points.size());
  for (const auto& pt : traj_points) {
    path_points.push_back(pt.path_point());
  }
  return path_points;
}

// Shift trajectory by time.
// trajectory_points's size must be greater than 1.
void ShiftTrajectoryByTime(
    double shift_time,
    std::vector<TrajectoryPoint>* /*inout*/ trajectory_points);

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_TRAJECTORY_UTIL_H_

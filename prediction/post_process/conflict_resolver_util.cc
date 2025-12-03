#include "onboard/prediction/post_process/conflict_resolver_util.h"

#include <algorithm>

#include "onboard/global/trace.h"
#include "onboard/math/geometry/proto/affine_transformation.pb.h"

namespace qcraft {
namespace prediction {

namespace {

bool AreCumulativeDistanceClose(const PathPoint& p1, const PathPoint& p2) {
  constexpr double kCloseThreshold = 1e-6;
  const Vec2d p1_pos(p1.x(), p1.y());
  const Vec2d p2_pos(p2.x(), p2.y());
  return p1_pos.DistanceTo(p2_pos) < kCloseThreshold;
}

PathPoint PredictedPointToPathPoint(const PredictedTrajectoryPoint& pt) {
  PathPoint path_pt;
  path_pt.set_x(pt.pos().x());
  path_pt.set_y(pt.pos().y());
  path_pt.set_s(pt.s());
  path_pt.set_theta(pt.theta());
  path_pt.set_kappa(pt.kappa());
  return path_pt;
}

planner::SpeedPoint PredictedPointToSpeedPoint(
    const PredictedTrajectoryPoint& pt) {
  // Cant calculate jerk for now.
  return planner::SpeedPoint(pt.t(), pt.s(), pt.v(), pt.a(), /*j=*/0.0);
}

PathPoint PredictedPointProtoToPathPoint(
    const PredictedTrajectoryPointProto& pt) {
  PathPoint path_pt;
  path_pt.set_x(pt.pos().x());
  path_pt.set_y(pt.pos().y());
  path_pt.set_s(pt.s());
  path_pt.set_theta(pt.theta());
  path_pt.set_kappa(pt.kappa());
  return path_pt;
}

planner::SpeedPoint PredictedPointProtoToSpeedPoint(
    const PredictedTrajectoryPointProto& pt) {
  // Cant calculate jerk for now.
  return planner::SpeedPoint(pt.t(), pt.s(), pt.v(), pt.a(), /*j=*/0.0);
}

}  // namespace

// Resolve predicted trajectories that might have stopped and thus fail to
// provide valid segments to build segment matcher.
std::pair<planner::DiscretizedPath, planner::SpeedVector>
PredictedTrajectoryToPurePathAndSpeedVector(
    const PredictedTrajectory& trajectory) {
  FUNC_QTRACE();
  const auto& predicted_points = trajectory.points();
  planner::DiscretizedPath path;
  std::vector<planner::SpeedPoint> speed_points;
  speed_points.reserve(predicted_points.size());
  path.reserve(predicted_points.size());
  path.push_back(PredictedPointToPathPoint(predicted_points.front()));
  speed_points.push_back(PredictedPointToSpeedPoint(predicted_points.front()));

  for (int i = 1; i < predicted_points.size(); ++i) {
    const auto& predicted_point = predicted_points[i];
    const auto path_pt = PredictedPointToPathPoint(predicted_point);
    if (AreCumulativeDistanceClose(path_pt, path.back())) {
      // Do not add this point if it is close to the previous added point.
      continue;
    }
    path.push_back(path_pt);
    speed_points.push_back(PredictedPointToSpeedPoint(predicted_point));
  }
  return std::make_pair(std::move(path),
                        planner::SpeedVector(std::move(speed_points)));
}

std::pair<planner::DiscretizedPath, planner::SpeedVector>
PredictedTrajectoryPointProtoToPurePathAndSpeedVector(
    const std::vector<PredictedTrajectoryPointProto>& points) {
  // Do not decrease point size because of stationary points.
  planner::DiscretizedPath path;
  std::vector<planner::SpeedPoint> speed_points;
  speed_points.reserve(points.size());
  path.reserve(points.size());
  for (int i = 0; i < points.size(); ++i) {
    const auto& point_proto = points[i];
    path.push_back(PredictedPointProtoToPathPoint(point_proto));
    speed_points.push_back(PredictedPointProtoToSpeedPoint(point_proto));
  }
  return std::make_pair(std::move(path),
                        planner::SpeedVector(std::move(speed_points)));
}

ConflictResolverDebugProto::SimpleSpeedProfile
EdgeConnectionToSimpleSpeedProfile(
    absl::Span<const std::string> cost_names, const std::vector<SvtEdge>& edges,
    const SvtEdgeVector<SvtEdgeCost>& /*search_costs*/,
    const std::vector<SvtEdgeIndex>& edge_idxes) {
  ConflictResolverDebugProto::SimpleSpeedProfile profile;
  const auto& final_edge = edges[edge_idxes.back().value()];
  if (final_edge.edge_cost != nullptr) {
    const auto& sum_cost = final_edge.edge_cost->sum_cost;
    profile.set_total_cost(sum_cost);
    profile.set_eval_cost(sum_cost / final_edge.final_t);
  }
  profile.set_final_t(final_edge.final_t);
  profile.set_final_v(final_edge.final_v);
  if (final_edge.states != nullptr && !final_edge.states->empty()) {
    profile.set_final_s(final_edge.states->back().s);
  }
  for (const auto& edge_idx : edge_idxes) {
    *profile.add_edges() =
        edges[edge_idx.value()].ToSimpleSpeedProfileProto(cost_names);
  }
  return profile;
}

ConflictResolverDebugProto::SpeedProfile SpeedVectorToSpeedProfile(
    const planner::SpeedVector& speed_vector,
    const std::vector<SvtEdgeIndex>& edge_idxes) {
  ConflictResolverDebugProto::SpeedProfile speed_profile;
  speed_profile.mutable_points()->Reserve(speed_vector.size());
  for (const auto& point : speed_vector) {
    ConflictResolverDebugProto::SpeedProfile::SpeedPoint pt;
    pt.set_s(point.s());
    pt.set_v(point.v());
    pt.set_t(point.t());
    pt.set_a(point.a());
    *speed_profile.add_points() = std::move(pt);
  }
  for (const auto& idx : edge_idxes) {
    speed_profile.add_edge_idxes(idx.value());
  }
  return speed_profile;
}

planner::SpeedPoint SvtStateToSpeedPoint(const SvtState& state, double a) {
  return planner::SpeedPoint(state.t, state.s, state.v, a, 0.0);
}
}  // namespace prediction
}  // namespace qcraft

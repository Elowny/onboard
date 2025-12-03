#include "onboard/planner/test_util/predicted_trajectory_builder.h"

#include <memory>
#include <string>
#include <utility>

#include "absl/memory/memory.h"

#include "onboard/math/util.h"
#include "onboard/prediction/prediction_defs.h"

namespace qcraft {
namespace planner {
PredictedTrajectoryBuilder::PredictedTrajectoryBuilder() {}

PredictedTrajectoryBuilder& PredictedTrajectoryBuilder::set_probability(
    double probability) {
  probability_ = probability;
  return *this;
}

PredictedTrajectoryBuilder& PredictedTrajectoryBuilder::set_annotation(
    std::string annotation) {
  annotation_ = std::move(annotation);
  return *this;
}

PredictedTrajectoryBuilder& PredictedTrajectoryBuilder::set_type(
    PredictionType type) {
  type_ = type;
  return *this;
}

PredictedTrajectoryBuilder& PredictedTrajectoryBuilder::set_index(int index) {
  index_ = index;
  return *this;
}

PredictedTrajectoryBuilder& PredictedTrajectoryBuilder::set_points(
    std::vector<prediction::PredictedTrajectoryPoint> points) {
  points_ = std::move(points);
  return *this;
}

// Specify a straight line prediction with constant acceleration.
PredictedTrajectoryBuilder& PredictedTrajectoryBuilder::set_straight_line(
    const Vec2d& start, const Vec2d& end, double init_v, double last_v) {
  const Vec2d tangent = Vec2d(end - start).Unit();
  const double theta = tangent.FastAngle();
  const double length = Vec2d(end - start).Dot(tangent);
  const double time = length * 2.0 / (init_v + last_v);
  const double acc = (last_v - init_v) / time;

  points_.clear();
  points_.reserve(CeilToInt(time / prediction::kPredictionTimeStep));
  for (double t = 0.0; t <= time; t += prediction::kPredictionTimeStep) {
    const double s = (init_v + 0.5 * acc * t) * t;
    const Vec2d pos = start + tangent * s;
    const double v = init_v + acc * t;

    prediction::PredictedTrajectoryPoint pt;
    pt.set_pos(pos);
    pt.set_s(s);
    pt.set_theta(theta);
    pt.set_kappa(0.0);
    pt.set_t(t);
    pt.set_v(v);
    pt.set_a(acc);

    points_.push_back(pt);
  }
  return *this;
}
PredictedTrajectoryBuilder& PredictedTrajectoryBuilder::set_straight_line(
    const Vec2d& start, double theta, double duration, double init_v,
    double acc) {
  const Vec2d tangent = Vec2d::FastUnitFromAngle(theta);
  points_.clear();
  points_.reserve(CeilToInt(duration / prediction::kPredictionTimeStep));
  for (double t = 0.0; t <= duration; t += prediction::kPredictionTimeStep) {
    const double s = (init_v + 0.5 * acc * t) * t;
    const Vec2d pos = start + tangent * s;
    const double v = init_v + acc * t;

    prediction::PredictedTrajectoryPoint pt;
    pt.set_pos(pos);
    pt.set_s(s);
    pt.set_theta(theta);
    pt.set_kappa(0.0);
    pt.set_t(t);
    pt.set_v(v);
    pt.set_a(acc);

    points_.push_back(pt);
  }
  return *this;
}

PredictedTrajectoryBuilder& PredictedTrajectoryBuilder::set_stationary_traj(
    const Vec2d& pos, double theta) {
  prediction::PredictedTrajectoryPoint point;
  point.set_pos(pos);
  point.set_s(0.0);
  point.set_theta(theta);
  point.set_kappa(0.0);
  point.set_v(0.0);
  point.set_a(0.0);
  point.set_t(0.0);

  constexpr int kPredictionSteps =
      prediction::kPredictionDuration / prediction::kPredictionTimeStep;
  points_.reserve(kPredictionSteps);
  for (int i = 0; i < kPredictionSteps; ++i) {
    point.set_t(i * prediction::kPredictionTimeStep);
    points_.push_back(point);
  }
  type_ = PredictionType::PT_STATIONARY;
  return *this;
}

prediction::PredictedTrajectory PredictedTrajectoryBuilder::Build() {
  traj_ = absl::make_unique<prediction::PredictedTrajectory>();
  traj_->set_probability(probability_);
  traj_->set_annotation(annotation_);
  traj_->set_type(type_);
  traj_->set_index(index_);

  if (!points_.empty()) {
    *traj_->mutable_points() = std::move(points_);
  }

  return *traj_;
}
}  // namespace planner
}  // namespace qcraft

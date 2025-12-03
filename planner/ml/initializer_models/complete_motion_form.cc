#include "onboard/planner/ml/initializer_models/complete_motion_form.h"

#include <algorithm>
#include <cmath>
#include <iterator>

#include "onboard/math/frenet_common.h"
#include "onboard/math/util.h"
#include "onboard/math/vec.h"
#include "onboard/planner/trajectory_util.h"

namespace qcraft::planner {

CompleteMotion::CompleteMotion(
    const GeometryFormBuilder* form_builder,
    const std::vector<ApolloTrajectoryPointProto>& traj) {
  traj_ = traj;
  duration_ = traj.back().relative_time() - traj.front().relative_time();
  form_builder_ = form_builder;
  int traj_size = traj_.size();
  relative_stations_.reserve(traj_size);
  relative_stations_.push_back(0.0);
  for (int i = 1; i < traj_size; ++i) {
    relative_stations_[i] =
        relative_stations_[i - 1] +
        std::hypot(traj_[i].path_point().x() - traj_[i - 1].path_point().x(),
                   traj_[i].path_point().y() - traj_[i - 1].path_point().y());
  }
}

SampledMotionFormStates CompleteMotion::SampleStates() const {
  const double const_step_dt = prediction::kPredictionTimeStep *
                               MotionForm::kConstTimeIntervalSampleStep;
  SampledMotionFormStates res;
  res.const_interval_states = Sample(const_step_dt);

  int desired_equal_dist_steps =
      static_cast<int>(duration_ / kDesireEqualTimeInterval);
  desired_equal_dist_steps =
      std::clamp<int>(desired_equal_dist_steps, kMinEqualTimeIntervalSampleStep,
                      kMaxEqualTimeIntervalSampleStep);
  const double equal_time_dt = duration_ / (desired_equal_dist_steps - 1);
  res.equal_interval_states = Sample(equal_time_dt);
  return res;
}
std::vector<MotionState> CompleteMotion::SampleEqualIntervalStates() const {
  int desired_equal_dist_steps =
      static_cast<int>(duration_ / kDesireEqualTimeInterval) + 1;
  desired_equal_dist_steps =
      std::clamp<int>(desired_equal_dist_steps, kMinEqualTimeIntervalSampleStep,
                      kMaxEqualTimeIntervalSampleStep);
  const double equal_time_dt = duration_ / (desired_equal_dist_steps - 1);
  return Sample(equal_time_dt);
}
std::vector<MotionState> CompleteMotion::Sample(double d_t) const {
  const int num_samples = CeilToInt(duration_ / d_t) + 1;
  std::vector<MotionState> samples;
  samples.reserve(num_samples);
  for (int i = 0; i < num_samples; ++i) {
    samples.push_back(State(i * d_t));
  }
  return samples;
}

MotionState CompleteMotion::GetStartMotionState() const {
  const auto& start_point = traj_.front();
  const auto xy_pos =
      Vec2d(start_point.path_point().x(), start_point.path_point().y());
  const auto sl_pos = form_builder_->LookUpSL(xy_pos);
  MotionState motion_state{
      .xy = xy_pos,
      .h = NormalizeAngle(start_point.path_point().theta()),
      .k = start_point.path_point().kappa(),
      .ref_k = form_builder_->LookUpRefK(sl_pos.s),
      .t = 0.0,
      .v = start_point.v(),
      .a = start_point.a(),
      .accumulated_s = sl_pos.s,
      .s = relative_stations_.front(),
      .l = sl_pos.l,
  };
  return motion_state;
}

MotionState CompleteMotion::GetEndMotionState() const {
  const auto& end_point = traj_.back();
  const auto xy_pos =
      Vec2d(end_point.path_point().x(), end_point.path_point().y());
  const auto sl_pos = form_builder_->LookUpSL(xy_pos);
  MotionState motion_state{
      .xy = xy_pos,
      .h = NormalizeAngle(end_point.path_point().theta()),
      .k = end_point.path_point().kappa(),
      .ref_k = form_builder_->LookUpRefK(sl_pos.s),
      .t = duration_,
      .v = end_point.v(),
      .a = end_point.a(),
      .accumulated_s = sl_pos.s,
      .s = relative_stations_.back(),
      .l = sl_pos.l,
  };
  return motion_state;
}

MotionState CompleteMotion::State(double t) const {
  auto traj_point =
      QueryApolloTrajectoryPointByT(traj_.begin(), traj_.end(), t);
  const auto it = std::lower_bound(
      traj_.begin(), traj_.end(), t,
      [](const auto& p, double t) { return p.relative_time() < t; });
  int pre_idx =
      std::distance(traj_.begin(), it == traj_.end() ? traj_.end() - 1 : it);
  const auto xy_pos =
      Vec2d(traj_point.path_point().x(), traj_point.path_point().y());
  const auto sl_pos = form_builder_->LookUpSL(xy_pos);
  MotionState motion_state{
      .xy = xy_pos,
      .h = NormalizeAngle(traj_point.path_point().theta()),
      .k = traj_point.path_point().kappa(),
      .ref_k = form_builder_->LookUpRefK(sl_pos.s),
      .t = t,
      .v = traj_point.v(),
      .a = traj_point.a(),
      .accumulated_s = sl_pos.s,
      .s = relative_stations_[pre_idx] +
           std::hypot(xy_pos.x() - traj_[pre_idx].path_point().x(),
                      xy_pos.y() - traj_[pre_idx].path_point().y()),
      .l = sl_pos.l,
  };
  return motion_state;
}

}  // namespace qcraft::planner

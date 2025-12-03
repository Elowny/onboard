#include "onboard/planner/common/path_sl_boundary.h"

#include "onboard/math/geometry/util.h"
#include "onboard/math/piecewise_linear_function.h"
namespace qcraft::planner {
namespace {
constexpr double kDesiredPathSlBoundarySampleStep = 2.0;
constexpr int kPathSlBoundaryMinInterval = 2;
}  // namespace
void PathSlBoundary::BuildResampledData() {
  PiecewiseLinearFunction<double, double> ref_center_l_s_plf(s_vec_,
                                                             ref_center_l_vec_);
  PiecewiseLinearFunction<double, double> right_l_s_plf(s_vec_, right_l_vec_);
  PiecewiseLinearFunction<double, double> left_l_s_plf(s_vec_, left_l_vec_);
  PiecewiseLinearFunction<double, double> target_right_l_s_plf(
      s_vec_, target_right_l_vec_);
  PiecewiseLinearFunction<double, double> target_left_l_s_plf(
      s_vec_, target_left_l_vec_);
  PiecewiseLinearFunction<Vec2d, double> ref_center_xy_s_plf(
      s_vec_, ref_center_xy_vec_);
  PiecewiseLinearFunction<Vec2d, double> right_xy_s_plf(s_vec_, right_xy_vec_);
  PiecewiseLinearFunction<Vec2d, double> left_xy_s_plf(s_vec_, left_xy_vec_);
  PiecewiseLinearFunction<Vec2d, double> target_right_xy_s_plf(
      s_vec_, target_right_xy_vec_);
  PiecewiseLinearFunction<Vec2d, double> target_left_xy_s_plf(
      s_vec_, target_left_xy_vec_);
  const double start_s = s_vec_.front();
  const double end_s = s_vec_.back();
  const int num_interval = std::max<int>(
      static_cast<int>((end_s - start_s) / kDesiredPathSlBoundarySampleStep),
      kPathSlBoundaryMinInterval);
  sample_interval_ = (end_s - start_s) / num_interval;
  const int num_points = num_interval + 1;
  resampled_s_vec_.reserve(num_points);
  for (int i = 0; i < num_points; ++i) {
    const double cur_s = start_s + i * sample_interval_;
    resampled_s_vec_.push_back(cur_s);
  }
  resampled_ref_center_l_vec_ = ref_center_l_s_plf.Evaluate(resampled_s_vec_);
  resampled_right_l_vec_ = right_l_s_plf.Evaluate(resampled_s_vec_);
  resampled_left_l_vec_ = left_l_s_plf.Evaluate(resampled_s_vec_);
  resampled_target_right_l_vec_ =
      target_right_l_s_plf.Evaluate(resampled_s_vec_);
  resampled_target_left_l_vec_ = target_left_l_s_plf.Evaluate(resampled_s_vec_);
  resampled_ref_center_xy_vec_ = ref_center_xy_s_plf.Evaluate(resampled_s_vec_);
  resampled_right_xy_vec_ = right_xy_s_plf.Evaluate(resampled_s_vec_);
  resampled_left_xy_vec_ = left_xy_s_plf.Evaluate(resampled_s_vec_);
  resampled_target_right_xy_vec_ =
      target_right_xy_s_plf.Evaluate(resampled_s_vec_);
  resampled_target_left_xy_vec_ =
      target_left_xy_s_plf.Evaluate(resampled_s_vec_);
}

PathSlBoundary::PathSlBoundary(
    std::vector<double> s, std::vector<double> ref_center_l,
    std::vector<double> right_l, std::vector<double> left_l,
    std::vector<double> target_right_l, std::vector<double> target_left_l,
    std::vector<Vec2d> ref_center_xy, std::vector<Vec2d> right_xy,
    std::vector<Vec2d> left_xy, std::vector<Vec2d> target_right_xy,
    std::vector<Vec2d> target_left_xy)
    : s_vec_(std::move(s)),
      ref_center_l_vec_(std::move(ref_center_l)),
      right_l_vec_(std::move(right_l)),
      left_l_vec_(std::move(left_l)),
      target_right_l_vec_(std::move(target_right_l)),
      target_left_l_vec_(std::move(target_left_l)),
      ref_center_xy_vec_(std::move(ref_center_xy)),
      right_xy_vec_(std::move(right_xy)),
      left_xy_vec_(std::move(left_xy)),
      target_right_xy_vec_(std::move(target_right_xy)),
      target_left_xy_vec_(std::move(target_left_xy)) {
  const int vec_size = s_vec_.size();
  QCHECK_GT(vec_size, 1);
  QCHECK_EQ(vec_size, ref_center_l_vec_.size());
  QCHECK_EQ(vec_size, ref_center_xy_vec_.size());
  QCHECK_EQ(vec_size, right_l_vec_.size());
  QCHECK_EQ(vec_size, left_l_vec_.size());
  QCHECK_EQ(vec_size, target_right_l_vec_.size());
  QCHECK_EQ(vec_size, target_left_l_vec_.size());
  QCHECK_EQ(vec_size, right_xy_vec_.size());
  QCHECK_EQ(vec_size, left_xy_vec_.size());
  QCHECK_EQ(vec_size, target_right_xy_vec_.size());
  QCHECK_EQ(vec_size, target_left_xy_vec_.size());
  BuildResampledData();
}

PathSlBoundary::PathSlBoundary(
    std::vector<double> s, std::vector<double> right_l,
    std::vector<double> left_l, std::vector<double> target_right_l,
    std::vector<double> target_left_l, std::vector<Vec2d> right_xy,
    std::vector<Vec2d> left_xy, std::vector<Vec2d> target_right_xy,
    std::vector<Vec2d> target_left_xy)
    : s_vec_(std::move(s)),
      right_l_vec_(std::move(right_l)),
      left_l_vec_(std::move(left_l)),
      target_right_l_vec_(std::move(target_right_l)),
      target_left_l_vec_(std::move(target_left_l)),
      right_xy_vec_(std::move(right_xy)),
      left_xy_vec_(std::move(left_xy)),
      target_right_xy_vec_(std::move(target_right_xy)),
      target_left_xy_vec_(std::move(target_left_xy)) {
  const int vec_size = s_vec_.size();
  QCHECK_GT(vec_size, 1);
  QCHECK_EQ(vec_size, right_l_vec_.size());
  QCHECK_EQ(vec_size, left_l_vec_.size());
  QCHECK_EQ(vec_size, target_right_l_vec_.size());
  QCHECK_EQ(vec_size, target_left_l_vec_.size());
  QCHECK_EQ(vec_size, right_xy_vec_.size());
  QCHECK_EQ(vec_size, left_xy_vec_.size());
  QCHECK_EQ(vec_size, target_right_xy_vec_.size());
  QCHECK_EQ(vec_size, target_left_xy_vec_.size());

  ref_center_l_vec_.reserve(vec_size);
  ref_center_xy_vec_.reserve(vec_size);
  for (int i = 0; i < vec_size; ++i) {
    ref_center_l_vec_.emplace_back(0.5 * (right_l_vec_[i] + left_l_vec_[i]));
    ref_center_xy_vec_.emplace_back(0.5 * (right_xy_vec_[i] + left_xy_vec_[i]));
  }
  BuildResampledData();
}

void PathSlBoundary::ToProto(ScheduledPathBoundaryProto* proto) const {
  const int size = this->size();
  proto->mutable_reference_center()->Reserve(size);
  proto->mutable_left_boundary()->Reserve(size);
  proto->mutable_right_boundary()->Reserve(size);
  proto->mutable_target_left_boundary()->Reserve(size);
  proto->mutable_target_right_boundary()->Reserve(size);

  for (const auto& pt : resampled_ref_center_xy_vec_) {
    Vec2dToProto(pt, proto->add_reference_center());
  }
  for (const auto& pt : resampled_left_xy_vec_) {
    Vec2dToProto(pt, proto->add_left_boundary());
  }
  for (const auto& pt : resampled_right_xy_vec_) {
    Vec2dToProto(pt, proto->add_right_boundary());
  }
  for (const auto& pt : resampled_target_left_xy_vec_) {
    Vec2dToProto(pt, proto->add_target_left_boundary());
  }
  for (const auto& pt : resampled_target_right_xy_vec_) {
    Vec2dToProto(pt, proto->add_target_right_boundary());
  }
}

}  // namespace qcraft::planner

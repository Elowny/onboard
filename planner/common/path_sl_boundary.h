#ifndef ONBOARD_PLANNER_COMMON_PATH_SL_BOUNDARY_H_
#define ONBOARD_PLANNER_COMMON_PATH_SL_BOUNDARY_H_

// IWYU pragma: no_include <ext/alloc_traits.h>
//              for __alloc_traits<>::value_type

#include <algorithm>
#include <ostream>
#include <utility>
#include <vector>

#include "absl/types/span.h"

#include "onboard/global/buffered_logger.h"
#include "onboard/lite/check.h"
#include "onboard/math/util.h"
#include "onboard/math/vec.h"
#include "onboard/planner/scheduler/proto/scheduler.pb.h"

namespace qcraft::planner {

class PathSlBoundary {
 public:
  PathSlBoundary() = default;

  explicit PathSlBoundary(
      std::vector<double> s, std::vector<double> ref_center_l,
      std::vector<double> right_l, std::vector<double> left_l,
      std::vector<double> target_right_l, std::vector<double> target_left_l,
      std::vector<Vec2d> ref_center_xy, std::vector<Vec2d> right_xy,
      std::vector<Vec2d> left_xy, std::vector<Vec2d> target_right_xy,
      std::vector<Vec2d> target_left_xy);

  // reference centers are calculated by boundary
  explicit PathSlBoundary(std::vector<double> s, std::vector<double> right_l,
                          std::vector<double> left_l,
                          std::vector<double> target_right_l,
                          std::vector<double> target_left_l,
                          std::vector<Vec2d> right_xy,
                          std::vector<Vec2d> left_xy,
                          std::vector<Vec2d> target_right_xy,
                          std::vector<Vec2d> target_left_xy);

  bool IsEmpty() const { return s_vec_.empty(); }
  int size() const { return s_vec_.size(); }

  absl::Span<const double> s_vector() const { return s_vec_; }

  double start_s() const {
    QCHECK(!s_vec_.empty()) << "Empty path sl boundary!";
    return s_vec_.front();
  }

  double end_s() const {
    QCHECK(!s_vec_.empty()) << "Empty path sl boundary!";
    return s_vec_.back();
  }

  absl::Span<const double> reference_center_l_vector() const {
    return ref_center_l_vec_;
  }

  absl::Span<const double> right_l_vector() const { return right_l_vec_; }

  absl::Span<const double> left_l_vector() const { return left_l_vec_; }

  absl::Span<const double> target_right_l_vector() const {
    return target_right_l_vec_;
  }

  absl::Span<const double> target_left_l_vector() const {
    return target_left_l_vec_;
  }

  absl::Span<const Vec2d> reference_center_xy_vector() const {
    return ref_center_xy_vec_;
  }

  absl::Span<const Vec2d> right_xy_vector() const { return right_xy_vec_; }

  absl::Span<const Vec2d> left_xy_vector() const { return left_xy_vec_; }

  absl::Span<const Vec2d> target_right_xy_vector() const {
    return target_right_xy_vec_;
  }

  absl::Span<const Vec2d> target_left_xy_vector() const {
    return target_left_xy_vec_;
  }

  // {right_l, left_l}
  std::pair<double, double> QueryBoundaryL(double s) const;
  // {target_right_l, target_left_l}
  std::pair<double, double> QueryTargetBoundaryL(double s) const;
  // {right_xy, left_xy}
  std::pair<Vec2d, Vec2d> QueryBoundaryXY(double s) const;

  double QueryReferenceCenterL(double s) const;
  Vec2d QueryReferenceCenterXY(double s) const;

  void ToProto(ScheduledPathBoundaryProto* proto) const;

 private:
  void BuildResampledData();

  // {start_index, lerp_factor}
  std::pair<int, double> FindLerpInfo(double s) const;

  std::vector<double> s_vec_;             // Station.
  std::vector<double> ref_center_l_vec_;  // Reference center
  // Right or left l vec is always the outer boundary
  std::vector<double> right_l_vec_;
  std::vector<double> left_l_vec_;
  // Target right or left l vec is always the inner boundary
  std::vector<double> target_right_l_vec_;
  std::vector<double> target_left_l_vec_;
  std::vector<Vec2d> ref_center_xy_vec_;
  std::vector<Vec2d> right_xy_vec_;
  std::vector<Vec2d> left_xy_vec_;
  std::vector<Vec2d> target_right_xy_vec_;
  std::vector<Vec2d> target_left_xy_vec_;

  std::vector<double> resampled_s_vec_;  // Resampled Station.
  std::vector<double>
      resampled_ref_center_l_vec_;  // Resampled Reference center
  // Right or left l vec is always the outer boundary
  std::vector<double> resampled_right_l_vec_;
  std::vector<double> resampled_left_l_vec_;
  // Target right or left l vec is always the inner boundary
  std::vector<double> resampled_target_right_l_vec_;
  std::vector<double> resampled_target_left_l_vec_;
  std::vector<Vec2d> resampled_ref_center_xy_vec_;
  std::vector<Vec2d> resampled_right_xy_vec_;
  std::vector<Vec2d> resampled_left_xy_vec_;
  std::vector<Vec2d> resampled_target_right_xy_vec_;
  std::vector<Vec2d> resampled_target_left_xy_vec_;
  double sample_interval_ = 0.0;
};

inline std::pair<double, double> PathSlBoundary::QueryBoundaryL(
    double s) const {
  const auto [index, factor] = FindLerpInfo(s);
  return std::make_pair(Lerp(resampled_right_l_vec_[index],
                             resampled_right_l_vec_[index + 1], factor),
                        Lerp(resampled_left_l_vec_[index],
                             resampled_left_l_vec_[index + 1], factor));
}

inline std::pair<Vec2d, Vec2d> PathSlBoundary::QueryBoundaryXY(double s) const {
  const auto [index, factor] = FindLerpInfo(s);
  return std::make_pair(Lerp(resampled_right_xy_vec_[index],
                             resampled_right_xy_vec_[index + 1], factor),
                        Lerp(resampled_left_xy_vec_[index],
                             resampled_left_xy_vec_[index + 1], factor));
}

inline std::pair<double, double> PathSlBoundary::QueryTargetBoundaryL(
    double s) const {
  const auto [index, factor] = FindLerpInfo(s);
  return std::make_pair(Lerp(resampled_target_right_l_vec_[index],
                             resampled_target_right_l_vec_[index + 1], factor),
                        Lerp(resampled_target_left_l_vec_[index],
                             resampled_target_left_l_vec_[index + 1], factor));
}

inline double PathSlBoundary::QueryReferenceCenterL(double s) const {
  const auto [index, factor] = FindLerpInfo(s);
  return Lerp(resampled_ref_center_l_vec_[index],
              resampled_ref_center_l_vec_[index + 1], factor);
}

inline Vec2d PathSlBoundary::QueryReferenceCenterXY(double s) const {
  const auto [index, factor] = FindLerpInfo(s);
  return Lerp(resampled_ref_center_xy_vec_[index],
              resampled_ref_center_xy_vec_[index + 1], factor);
}

inline std::pair<int, double> PathSlBoundary::FindLerpInfo(double s) const {
  if (sample_interval_ == 0.0) {
    return {0, 0};
  }
  int idx = static_cast<int>(
      std::floor((s - resampled_s_vec_.front()) / sample_interval_));
  idx = std::clamp<int>(idx, 0, static_cast<int>(resampled_s_vec_.size()) - 2);
  return {idx, (s - resampled_s_vec_[idx]) / sample_interval_};
}
}  // namespace qcraft::planner

#endif

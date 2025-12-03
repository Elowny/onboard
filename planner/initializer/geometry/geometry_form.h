#ifndef ONBOARD_PLANNER_INITIALIZER_GEOMETRY_GEOMETRY_FORM_H_
#define ONBOARD_PLANNER_INITIALIZER_GEOMETRY_GEOMETRY_FORM_H_

#include <vector>
// IWYU pragma: no_include <algorithm>

#include "absl/types/span.h"
#include "onboard/math/vec.h"
#include "onboard/planner/initializer/geometry/geometry_state.h"

namespace qcraft {
namespace planner {

class GeometryForm {
 public:
  virtual double length() const = 0;
  virtual GeometryState State(double s) const = 0;
  virtual const GeometryState& FastState(double s) const = 0;
  virtual std::vector<GeometryState> Sample(double delta_s) const = 0;
  virtual absl::Span<const GeometryState> states() const = 0;
  virtual ~GeometryForm() {}
};

class StraightLineGeometry : public GeometryForm {
 public:
  StraightLineGeometry(const Vec2d& start, const Vec2d& end);

  double length() const override;

  std::vector<GeometryState> Sample(double delta_s) const override;
  GeometryState State(double s) const override;
  const GeometryState& FastState(double s) const override;
  absl::Span<const GeometryState> states() const override {
    return absl::MakeSpan(densely_discretized_states_);
  }

 private:
  Vec2d start_;
  Vec2d end_;
  Vec2d unit_;
  std::vector<GeometryState> densely_discretized_states_;
};

class PiecewiseLinearGeometry : public GeometryForm {
 public:
  explicit PiecewiseLinearGeometry(absl::Span<const GeometryState> states);
  double length() const override { return length_; }
  std::vector<GeometryState> Sample(double delta_s) const override;
  GeometryState State(double s) const override;
  const GeometryState& FastState(double s) const override;
  absl::Span<const GeometryState> states() const override {
    return absl::MakeSpan(densely_discretized_states_);
  }

 private:
  double length_;
  std::vector<GeometryState> states_;
  std::vector<GeometryState> densely_discretized_states_;
  int dense_state_size_;
  std::vector<double> vec_s_;
};

class StationaryGeometry : public GeometryForm {
 public:
  explicit StationaryGeometry(const GeometryState& state) : state_({state}) {}

  double length() const override { return 0.0; }

  GeometryState State(double /*s*/) const override { return state_[0]; }
  const GeometryState& FastState(double /*s*/) const override {
    return state_[0];
  }
  std::vector<GeometryState> Sample(double /*delta_s*/) const override {
    return state_;
  }
  absl::Span<const GeometryState> states() const override {
    return absl::MakeSpan(state_);
  }

 private:
  std::vector<GeometryState> state_;
};

}  // namespace planner
}  // namespace qcraft
#endif  // ONBOARD_PLANNER_INITIALIZER_GEOMETRY_GEOMETRY_FORM_H_

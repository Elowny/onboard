#ifndef ONBOARD_PLANNER_INITIALIZER_GEOMETRY_GEOMETRY_FORM_BUILDER_H_
#define ONBOARD_PLANNER_INITIALIZER_GEOMETRY_GEOMETRY_FORM_BUILDER_H_

#include <memory>
#include <vector>

#include "absl/status/statusor.h"

#include "onboard/math/frenet_common.h"
#include "onboard/math/frenet_frame.h"
#include "onboard/math/piecewise_linear_function.h"
#include "onboard/math/vec.h"
#include "onboard/planner/initializer/geometry/geometry_form.h"
#include "onboard/planner/initializer/geometry/geometry_state.h"
#include "onboard/planner/initializer/proto/initializer.pb.h"
#include "onboard/planner/router/drive_passage.h"

namespace qcraft::planner {

struct DrivePassageSamplePoint {
  Vec2d xy;
  double l;
  double accumulated_s;
  int station_index;
};

class GeometryFormBuilder {
 public:
  GeometryFormBuilder() = default;

  explicit GeometryFormBuilder(const DrivePassage* passage,
                               double max_sampling_acc_s,
                               double s_from_start_with_diff);

  // Use a cubic spiral to connect start point to sample point
  absl::StatusOr<PiecewiseLinearGeometry> BuildCubicSpiralGeometry(
      const GeometryState& start_state,
      const DrivePassageSamplePoint& end) const;

  // Use a cubic spiral to connect two sample point
  absl::StatusOr<PiecewiseLinearGeometry> BuildCubicSpiralGeometry(
      const DrivePassageSamplePoint& start,
      const DrivePassageSamplePoint& end) const;

  // Use a quintic spiral to connect start point to sample point
  absl::StatusOr<PiecewiseLinearGeometry> BuildQuinticSpiralGeometry(
      const GeometryState& start_state,
      const DrivePassageSamplePoint& end) const;

  // Use a quintic spiral to connect two sample point
  absl::StatusOr<PiecewiseLinearGeometry> BuildQuinticSpiralGeometry(
      const DrivePassageSamplePoint& start,
      const DrivePassageSamplePoint& end) const;

  // Use a quintic/cubic polynomial l(s) to connect start point to sample point
  absl::StatusOr<PiecewiseLinearGeometry> BuildLateralQuinticPolyGeometry(
      const GeometryState& start_state,
      const DrivePassageSamplePoint& end) const;

  // Use a quintic/cubic polynomial l(s) to build "approximately" smooth
  // geometry form that satisfies the boundary condition. Returns a lateral
  // polynomial that connects two sample points.
  absl::StatusOr<PiecewiseLinearGeometry> BuildLateralQuinticPolyGeometry(
      const DrivePassageSamplePoint& start,
      const DrivePassageSamplePoint& end) const;

  double LookUpRefK(const double station) const {
    return k_s_.Evaluate(station);
  }

  FrenetCoordinate LookUpSL(const Vec2d& xy) const {
    return smoothed_frenet_frame_->XYToSL(xy);
  }

  double smooth_dp_sampling_acc_s() const {
    return default_max_sampling_acc_s_;
  }

  void FillSmoothDrivePassage(GeometryGraphProto* proto) const;

 private:
  const DrivePassage* passage_;
  double default_max_sampling_acc_s_;
  std::vector<double> station_k_;
  // All the piecewise linear functions' "s" is the arc length on the original
  // unsmoothed drive passage.
  PiecewiseLinearFunction<double, double> k_s_;
  std::vector<Vec2d> smoothed_xy_;
  PiecewiseLinearFunction<double, double> smoothed_hsin_s_;
  PiecewiseLinearFunction<double, double> smoothed_hcos_s_;
  std::unique_ptr<FrenetFrame> smoothed_frenet_frame_;
  std::unique_ptr<FrenetFrame> frenet_frame_;
};

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_INITIALIZER_GEOMETRY_GEOMETRY_FORM_BUILDER_H_

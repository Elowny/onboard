#include "onboard/planner/initializer/geometry/geometry_form_builder.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <ostream>
#include <utility>

#include "absl/status/status.h"
#include "absl/strings/str_format.h"
#include "absl/types/span.h"
#include "google/protobuf/repeated_ptr_field.h"

#include "onboard/container/strong_int.h"
#include "onboard/lite/check.h"
#include "onboard/lite/logging.h"
#include "onboard/math/cubic_spline.h"
#include "onboard/math/geometry/proto/affine_transformation.pb.h"
#include "onboard/math/util.h"
#include "onboard/planner/math/cubic_spiral_boundary_value_problem.h"
#include "onboard/planner/math/quintic_spiral_boundary_value_problem.h"
#include "onboard/planner/math/spiral.h"
#include "onboard/utils/status_builder.h"
#include "onboard/utils/status_macros.h"

namespace qcraft::planner {

namespace {

constexpr double kSpiralDiscretizationResolution = 1.0;
constexpr double kFrenetProjectionMaxHeadingDiff = M_PI / 4.0;
constexpr double kCurvatureClip = 0.15;
constexpr int kMinQuinticSplineStationLen = 2;
constexpr int kSmoothDrivePassageResampleResolution = 5;
constexpr int kSmoothDrivePassageMinResampleStationSize = 20;

struct SmoothFeature {
  double k;
  double dk;
  double ddk;
  double theta;
};

std::vector<double> ComputeQuinticPolynomialCoefficients(
    const double x0, const double dx0, const double ddx0, const double x1,
    const double dx1, const double ddx1, const double p) {
  std::vector<double> coef(6);
  coef[0] = x0;
  coef[1] = dx0;
  coef[2] = ddx0 * 0.5;

  const double p2 = p * p;
  const double p3 = p * p2;

  const double c0 = (x1 - 0.5 * p2 * ddx0 - dx0 * p - x0) / p3;
  const double c1 = (dx1 - ddx0 * p - dx0) / p2;
  const double c2 = (ddx1 - ddx0) / p;

  coef[3] = 0.5 * (20.0 * c0 - 8.0 * c1 + c2);
  coef[4] = (-15.0 * c0 + 7.0 * c1 - c2) / p;
  coef[5] = (6.0 * c0 - 3.0 * c1 + 0.5 * c2) / p2;
  return coef;
}

double EvaluateQuinticPolynomial(const std::vector<double>& coef,
                                 double delta_s) {
  return ((((coef[5] * delta_s + coef[4]) * delta_s + coef[3]) * delta_s +
           coef[2]) *
              delta_s +
          coef[1]) *
             delta_s +
         coef[0];
}

double EvaluateQuinticPolynomialDeriv(const std::vector<double>& coef,
                                      double delta_s) {
  return (((5.0 * coef[5] * delta_s + 4.0 * coef[4]) * delta_s +
           3.0 * coef[3]) *
              delta_s +
          2.0 * coef[2]) *
             delta_s +
         coef[1];
}

double EvaluateQuinticPolynomialSecondDeriv(const std::vector<double>& coef,
                                            double delta_s) {
  return 2.0 * coef[2] + 6.0 * coef[3] * delta_s +
         12.0 * coef[4] * delta_s * delta_s +
         20.0 * coef[5] * delta_s * delta_s * delta_s;
}

double CalculateCurvature(const double& ref_k, const double& dref_k,
                          const double& l, const double& dl,
                          const double& ddl) {
  double denominator = (dl * dl + (1 - l * ref_k) * (1 - l * ref_k));
  if (std::fabs(denominator) < 1e-8) {
    return 0.0;
  }
  denominator *= std::sqrt(denominator);
  const double numerator = ref_k + ddl - 2 * l * ref_k * ref_k -
                           l * ddl * ref_k + l * l * ref_k * ref_k * ref_k +
                           l * dl * dref_k + 2 * dl * dl * ref_k;
  return numerator / denominator;
}

// Safely compute the curvature at the lateral offset point. Avoid
// singualarity at 1 - k * l = 0.
double SafeCurvatureOffset(double k, double lat) {
  constexpr double kEpsilon = 1e-3;
  const auto val = 1.0 - k * lat;
  if (fabs(val) < kEpsilon) {
    return 1.0;  // Set to a large curvature value.
  }
  return k / val;
}

// Discretize a spiral to a vector of geometry states and check curvature within
// kCurvatureClip.
std::vector<SpiralPoint> DiscretizeSpiralAndCheckCurvature(
    const Spiral& spiral) {
  constexpr int kMinSamplingSteps = 5;
  // Ensure at least 5 spiral points for better evaluation
  const int num_steps = std::max<int>(
      FloorToInt(spiral.length() / kSpiralDiscretizationResolution),
      kMinSamplingSteps);
  return spiral.BatchEval(spiral.length(), num_steps);
}

// Spiral builder may return 0 ~ 2 spirals for a pair of points, pick the
// shortest one if 2 spirals returned.
absl::StatusOr<std::vector<SpiralPoint>> ProcessSpiralsAndPickBest(
    const std::vector<Spiral>& spirals) {
  if (spirals.empty()) {
    return absl::NotFoundError("No feasible spiral.");
  }
  if (spirals.size() == 1) {
    return DiscretizeSpiralAndCheckCurvature(spirals[0]);
  }
  if (spirals.size() == 2) {
    const auto points1 = DiscretizeSpiralAndCheckCurvature(spirals[0]);
    const auto points2 = DiscretizeSpiralAndCheckCurvature(spirals[1]);
    return spirals[0].length() < spirals[1].length() ? points1 : points2;
  }
  // Should not be here.
  QLOG(FATAL) << "Should not reach here.";
  QCHECK(false);
  return absl::UnknownError("Should not be here");
}

absl::StatusOr<std::vector<FrenetCoordinate>>
BatchQueryFrenetCoordinateOnDrivePassage(
    absl::Span<const SpiralPoint> spiral_points, const FrenetFrame* passage_ff,
    const DrivePassage* passage) {
  std::vector<FrenetCoordinate> points_sl;
  points_sl.reserve(spiral_points.size());
  for (const auto& point : spiral_points) {
    const Vec2d point_xy(point.x, point.y);
    ASSIGN_OR_RETURN(
        FrenetCoordinate point_sl,
        passage_ff->XYToSLWithHeadingDiffLimit(point_xy, point.theta,
                                               kFrenetProjectionMaxHeadingDiff),
        _ << "Can not project spiral point (" << point_xy.transpose()
          << ") onto drive passage.");
    point_sl.s += passage->front_s();
    points_sl.push_back(point_sl);
  }
  return points_sl;
}

absl::StatusOr<std::vector<GeometryState>> SpiralPoints2States(
    absl::Span<const SpiralPoint> spiral_points, const FrenetFrame* passage_ff,
    const DrivePassage* passage,
    const PiecewiseLinearFunction<double, double>& k_s) {
  ASSIGN_OR_RETURN(const auto frenet_points,
                   BatchQueryFrenetCoordinateOnDrivePassage(
                       spiral_points, passage_ff, passage),
                   _ << "Can not project spiral point onto drive passage.");
  std::vector<GeometryState> states;
  states.reserve(spiral_points.size());
  QCHECK_EQ(frenet_points.size(), spiral_points.size());
  for (int i = 0; i < spiral_points.size(); i++) {
    states.push_back({.xy = Vec2d(spiral_points[i].x, spiral_points[i].y),
                      .h = NormalizeAngle(spiral_points[i].theta),
                      .k = spiral_points[i].k,
                      .ref_k = k_s(frenet_points[i].s),
                      .accumulated_s = frenet_points[i].s,
                      .l = frenet_points[i].l});
  }
  return states;
}

// Get k, dk, ddk(later), theta on smoothed drive passage.
SmoothFeature GetSmoothedDrivePassageFeaturesAtS(
    const double accum_s, const PiecewiseLinearFunction<double, double>& k_s,
    const PiecewiseLinearFunction<double, double>& hsin_s,
    const PiecewiseLinearFunction<double, double>& hcos_s) {
  const double hsin = hsin_s.Evaluate(accum_s);
  const double hcos = hcos_s.Evaluate(accum_s);
  SmoothFeature features = {.k = k_s.Evaluate(accum_s),
                            .dk = k_s.EvaluateSlope(accum_s),
                            .ddk = 0.0,  // Placeholder.
                            .theta = NormalizeAngle(std::atan2(hsin, hcos))};

  return features;
}

};  // namespace

GeometryFormBuilder::GeometryFormBuilder(const DrivePassage* passage,
                                         double max_sampling_acc_s,
                                         double s_from_start_with_diff)
    : passage_(passage), default_max_sampling_acc_s_(max_sampling_acc_s) {
  QCHECK_NOTNULL(passage_);
  // Compute some internal states
  const int station_size = passage_->stations().size();
  station_k_.reserve(station_size);

  const int approximate_resample_size =
      station_size / kSmoothDrivePassageResampleResolution + 3;

  std::vector<double> x;
  std::vector<double> y;
  std::vector<double> s;
  std::vector<Vec2d> unsmoothed_xy;
  unsmoothed_xy.reserve(station_size);
  x.reserve(approximate_resample_size);
  y.reserve(approximate_resample_size);
  s.reserve(approximate_resample_size);

  // extract resampled drive passage centerline
  int resample_index = std::numeric_limits<int>::min();
  for (const auto& station : passage_->stations()) {
    if (s.size() > kSmoothDrivePassageMinResampleStationSize &&
        station.accumulated_s() > default_max_sampling_acc_s_) {
      // Collected enough station accu_s, x, y infos to build frenet frame.
      break;
    }

    unsmoothed_xy.push_back(station.xy());
    int curr_sample_index =
        static_cast<int>((station.accumulated_s() + s_from_start_with_diff) /
                         kSmoothDrivePassageResampleResolution);
    if (curr_sample_index > resample_index) {
      resample_index = curr_sample_index;
      x.push_back(station.xy().x());
      y.push_back(station.xy().y());
      s.push_back(station.accumulated_s());
    }
  }

  // solve smoothing spline
  const CubicSpline x_s(s, x);
  const CubicSpline y_s(s, y);

  // reconstruct a smoothed drive passage centerline
  smoothed_xy_.reserve(station_size);
  std::vector<double> hsin;
  std::vector<double> hcos;
  hsin.reserve(station_size);
  hcos.reserve(station_size);
  std::vector<double> s_all;
  s_all.reserve(station_size);
  for (const auto& station : passage_->stations()) {
    const double accum_s = station.accumulated_s();
    if (accum_s > default_max_sampling_acc_s_) {
      break;
    }
    s_all.push_back(accum_s);
    const Vec2d pt(x_s.Evaluate(accum_s), y_s.Evaluate(accum_s));
    smoothed_xy_.push_back(pt);

    // calculate ref_k (smoothed drive passage)
    const double dx = x_s.EvaluateDerivative<1>(accum_s);
    const double dy = y_s.EvaluateDerivative<1>(accum_s);
    const double ddx = x_s.EvaluateDerivative<2>(accum_s);
    const double ddy = y_s.EvaluateDerivative<2>(accum_s);
    const double nominator = dx * ddy - dy * ddx;
    const double denominator = PowerThreeHalves(dx * dx + dy * dy);
    double k_temp =
        std::fabs(denominator) < 1e-8 ? 0.0 : nominator / denominator;
    k_temp = std::clamp(k_temp, -kCurvatureClip, kCurvatureClip);
    station_k_.push_back(k_temp);  // station_k_ is the smoothed version
    const auto tan = Vec2d(dx, dy).normalized();
    hsin.push_back(tan.y());
    hcos.push_back(tan.x());
  }
  station_k_[0] = 0.0;
  k_s_ = PiecewiseLinearFunction(s_all, station_k_);
  smoothed_hsin_s_ = PiecewiseLinearFunction(s_all, std::move(hsin));
  smoothed_hcos_s_ = PiecewiseLinearFunction(s_all, std::move(hcos));

  ASSIGN_OR_DIE(
      auto unsmoothed_frame,
      BuildKdTreeFrenetFrame(unsmoothed_xy, /*down_sample_raw_points=*/true));
  frenet_frame_ =
      std::make_unique<KdTreeFrenetFrame>(std::move(unsmoothed_frame));

  ASSIGN_OR_DIE(
      auto smoothed_frame,
      BuildKdTreeFrenetFrame(smoothed_xy_, /*down_sample_raw_points=*/true));
  smoothed_frenet_frame_ =
      std::make_unique<KdTreeFrenetFrame>(std::move(smoothed_frame));
}

absl::StatusOr<PiecewiseLinearGeometry>
GeometryFormBuilder::BuildLateralQuinticPolyGeometry(
    const DrivePassageSamplePoint& start,
    const DrivePassageSamplePoint& end) const {
  const int start_idx = start.station_index;
  const int end_idx = end.station_index;
  ASSIGN_OR_RETURN(
      const auto start_sl,
      smoothed_frenet_frame_->XYToSLWithHeadingDiffLimit(
          start.xy,
          passage_->station(StationIndex(start_idx)).tangent().FastAngle(),
          kFrenetProjectionMaxHeadingDiff),
      _ << "Cannot project on smoothed drive passage.");
  ASSIGN_OR_RETURN(
      const auto end_sl,
      smoothed_frenet_frame_->XYToSLWithHeadingDiffLimit(
          end.xy,
          passage_->station(StationIndex(end_idx)).tangent().FastAngle(),
          kFrenetProjectionMaxHeadingDiff),
      _ << "Cannot project on smoothed drive passage.");

  const double station_length = end_sl.s - start_sl.s;
  if (station_length <= 0.0) {
    return absl::InvalidArgumentError(
        absl::StrFormat("The station_length should be > 0, got: %.3f "
                        "(end_sl.s: %.3f, start_sl.s: %.3f)",
                        station_length, end_sl.s, start_sl.s));
  }
  const auto quintic_coefs = ComputeQuinticPolynomialCoefficients(
      start_sl.l, 0.0, 0.0, end_sl.l, 0.0, 0.0, station_length);

  const int discretization_size =
      std::max(kMinQuinticSplineStationLen + 1, end_idx - start_idx + 1);

  std::vector<GeometryState> states;
  states.reserve(discretization_size);
  const double ds = station_length / static_cast<double>(discretization_size);
  for (int i = 0; i < discretization_size; ++i) {
    const auto smooth_s = static_cast<double>(i) * ds;
    const auto lateral_offset =
        EvaluateQuinticPolynomial(quintic_coefs, smooth_s);
    const FrenetCoordinate point_sl{smooth_s + start_sl.s, lateral_offset};
    const Vec2d point_xy = smoothed_frenet_frame_->SLToXY(point_sl);
    // Reproject the point onto the original drive passage.
    ASSIGN_OR_RETURN(
        const auto point_sl_drive_passage,
        passage_->QueryFrenetCoordinateAt(point_xy),
        _ << "Cannot project current point back onto original drive passage.");

    const auto dl = EvaluateQuinticPolynomialDeriv(quintic_coefs, smooth_s);
    const auto ddl =
        EvaluateQuinticPolynomialSecondDeriv(quintic_coefs, smooth_s);
    // Evaluate ref_k using smoothed_k_s_ pieacewise linear function.
    const double ref_k = k_s_.Evaluate(point_sl_drive_passage.s);
    const double dref_k = k_s_.EvaluateSlope(point_sl_drive_passage.s);
    const double k = CalculateCurvature(ref_k, dref_k, lateral_offset, dl, ddl);
    const double hsin = smoothed_hsin_s_.Evaluate(point_sl_drive_passage.s);
    const double hcos = smoothed_hcos_s_.Evaluate(point_sl_drive_passage.s);
    const double angle = NormalizeAngle(
        std::atan2(hsin, hcos) + std::atan2(dl, 1 - ref_k * lateral_offset));
    states.push_back({.xy = point_xy,
                      .h = angle,
                      .k = k,
                      .ref_k = ref_k,
                      .accumulated_s = point_sl_drive_passage.s,
                      .l = point_sl_drive_passage.l});
  }
  // Force the last state to the given sample point to annilate numeric error.
  states[0].xy = start.xy;
  const int last_idx = states.size() - 1;
  states[last_idx].xy = end.xy;
  return PiecewiseLinearGeometry(states);
}

absl::StatusOr<PiecewiseLinearGeometry>
GeometryFormBuilder::BuildCubicSpiralGeometry(
    const DrivePassageSamplePoint& start,
    const DrivePassageSamplePoint& end) const {
  const auto& start_station =
      passage_->station(StationIndex(start.station_index));
  const auto& end_station = passage_->station(StationIndex(end.station_index));
  const SpiralPoint sp0 = {
      .x = start.xy.x(),
      .y = start.xy.y(),
      .theta = start_station.tangent().FastAngle(),
      .k = SafeCurvatureOffset(station_k_[start.station_index], start.l),
      .s = 0.0};
  const SpiralPoint sp1 = {
      .x = end.xy.x(),
      .y = end.xy.y(),
      .theta = end_station.tangent().FastAngle(),
      .k = SafeCurvatureOffset(station_k_[end.station_index], end.l),
      .s = 0.0};
  const auto spirals = CubicSpiralBoundaryValueSolver::solve(sp0, sp1);
  ASSIGN_OR_RETURN(
      const auto points, ProcessSpiralsAndPickBest(spirals),
      _ << "No spiral for connecting two points (curvature too large?).");

  ASSIGN_OR_RETURN(
      auto states,
      SpiralPoints2States(points, frenet_frame_.get(), passage_, k_s_),
      _ << "Spiral violate the boundary condition.");
  // Force the last state to the given sample point to annilate numeric error.
  const int last_idx = states.size() - 1;
  states[last_idx].xy = end.xy;
  states[last_idx].h = NormalizeAngle(sp1.theta);
  states[last_idx].k = sp1.k;
  states[last_idx].accumulated_s = end.accumulated_s;
  states[last_idx].l = end.l;

  QCHECK_GE(states.size(), 2);
  return PiecewiseLinearGeometry(states);
}

absl::StatusOr<PiecewiseLinearGeometry>
GeometryFormBuilder::BuildCubicSpiralGeometry(
    const GeometryState& start_state,
    const DrivePassageSamplePoint& end) const {
  const SpiralPoint sp0 = {.x = start_state.xy.x(),
                           .y = start_state.xy.y(),
                           .theta = start_state.h,
                           .k = start_state.k,
                           .s = 0.0};
  const auto& end_station = passage_->station(StationIndex(end.station_index));
  const SpiralPoint sp1 = {
      .x = end.xy.x(),
      .y = end.xy.y(),
      .theta = end_station.tangent().FastAngle(),
      .k = SafeCurvatureOffset(station_k_[end.station_index], end.l),
      .s = 0.0};
  const auto spirals = CubicSpiralBoundaryValueSolver::solve(sp0, sp1);
  ASSIGN_OR_RETURN(
      const auto points, ProcessSpiralsAndPickBest(spirals),
      _ << "No spiral for connecting two points (curvature too large?).");

  ASSIGN_OR_RETURN(
      auto states,
      SpiralPoints2States(points, frenet_frame_.get(), passage_, k_s_),
      _ << "Spiral violate the boundary condition.");
  // Force the last state to the given sample point to annilate numeric error.
  const int last_idx = states.size() - 1;
  states[last_idx].xy = end.xy;
  states[last_idx].h = NormalizeAngle(sp1.theta);
  states[last_idx].k = sp1.k;
  states[last_idx].accumulated_s = end.accumulated_s;
  states[last_idx].l = end.l;

  QCHECK_GE(states.size(), 2);
  return PiecewiseLinearGeometry(states);
}

void GeometryFormBuilder::FillSmoothDrivePassage(
    GeometryGraphProto* proto) const {
  auto* smoothed_dp = proto->mutable_smoothed_drive_passage();
  smoothed_dp->Clear();
  smoothed_dp->Reserve(smoothed_xy_.size());
  for (const auto& xy : smoothed_xy_) {
    auto* point_proto = smoothed_dp->Add();
    point_proto->set_x(xy.x());
    point_proto->set_y(xy.y());
  }
}

absl::StatusOr<PiecewiseLinearGeometry>
GeometryFormBuilder::BuildQuinticSpiralGeometry(
    const GeometryState& start_state,
    const DrivePassageSamplePoint& end) const {
  const SpiralPoint sp0 = {
      .x = start_state.xy.x(),
      .y = start_state.xy.y(),
      .theta = start_state.h,
      .k = start_state.k,
      .s = 0.0,
      .dk = start_state.dk,
      .ddk = start_state.ddk,
  };
  const auto end_features = GetSmoothedDrivePassageFeaturesAtS(
      end.accumulated_s, k_s_, smoothed_hsin_s_, smoothed_hcos_s_);
  const SpiralPoint sp1 = {
      .x = end.xy.x(),
      .y = end.xy.y(),
      .theta = end_features.theta,
      .k = SafeCurvatureOffset(end_features.k, end.l),
      .s = 0.0,
      .dk = end_features.dk,
      .ddk = end_features.ddk,
  };
  const auto spirals = QuinticSpiralBoundaryValueSolver::solve(sp0, sp1);
  ASSIGN_OR_RETURN(const auto points, ProcessSpiralsAndPickBest(spirals),
                   _ << "No quintic spiral connecting two points.");

  ASSIGN_OR_RETURN(
      auto states,
      SpiralPoints2States(points, frenet_frame_.get(), passage_, k_s_),
      _ << "Spiral violate the boundary condition.");
  // Force the last state to the given sample point to annilate numeric error.
  const int last_idx = states.size() - 1;
  states[last_idx].xy = end.xy;
  states[last_idx].h = NormalizeAngle(sp1.theta);
  states[last_idx].k = sp1.k;
  states[last_idx].accumulated_s = end.accumulated_s;
  states[last_idx].l = end.l;

  QCHECK_GE(states.size(), 2);
  return PiecewiseLinearGeometry(states);
}

absl::StatusOr<PiecewiseLinearGeometry>
GeometryFormBuilder::BuildQuinticSpiralGeometry(
    const DrivePassageSamplePoint& start,
    const DrivePassageSamplePoint& end) const {
  const auto start_features = GetSmoothedDrivePassageFeaturesAtS(
      start.accumulated_s, k_s_, smoothed_hsin_s_, smoothed_hcos_s_);
  const auto end_features = GetSmoothedDrivePassageFeaturesAtS(
      end.accumulated_s, k_s_, smoothed_hsin_s_, smoothed_hcos_s_);
  const SpiralPoint sp0 = {
      .y = start.xy.y(),
      .theta = start_features.theta,
      .k = SafeCurvatureOffset(start_features.k, start.l),
      .s = 0.0,
      .dk = start_features.dk,
      .ddk = start_features.ddk,
  };
  const SpiralPoint sp1 = {
      .x = end.xy.x(),
      .y = end.xy.y(),
      .theta = end_features.theta,
      .k = SafeCurvatureOffset(end_features.k, end.l),
      .s = 0.0,
      .dk = end_features.dk,
      .ddk = end_features.ddk,
  };
  const auto spirals = QuinticSpiralBoundaryValueSolver::solve(sp0, sp1);
  ASSIGN_OR_RETURN(const auto points, ProcessSpiralsAndPickBest(spirals),
                   _ << "No quintic spiral connecting two points.");

  ASSIGN_OR_RETURN(
      auto states,
      SpiralPoints2States(points, frenet_frame_.get(), passage_, k_s_),
      _ << "Cannot convert spiral points to valid states on the given "
           "passage.");
  const int last_idx = states.size() - 1;
  states[last_idx].xy = end.xy;
  states[last_idx].h = NormalizeAngle(sp1.theta);
  states[last_idx].k = sp1.k;
  states[last_idx].accumulated_s = end.accumulated_s;
  states[last_idx].l = end.l;

  QCHECK(states.size() >= 2);
  return PiecewiseLinearGeometry(states);
}
}  // namespace qcraft::planner

#include "onboard/planner/plan/acc/acc_corridor_util.h"

// IWYU pragma: no_include <ext/alloc_traits.h>
//              for __alloc_traits<>::value_type
#include <algorithm>
#include <ostream>
#include <string>
#include <vector>

#include "absl/status/statusor.h"
#include "absl/strings/str_format.h"
#include "absl/types/span.h"
#include "glog/logging.h"

#include "onboard/global/trace.h"
#include "onboard/math/geometry/proto/affine_transformation.pb.h"
#include "onboard/math/polynomial_fitter.h"
#include "onboard/math/polynomialxd.h"
#include "onboard/math/vec.h"
#include "onboard/planner/scheduler/driving_map_topo_builder.h"

namespace qcraft {

namespace planner {
namespace {
// Max credible length.
const PiecewiseLinearFunction<double, double> kMaxVelocityKphCredibleLengthPLF(
    std::vector<double>{0.0, 65.0, 70.0, 95.0, 100.0, 145.0, 150.0, 195.0,
                        200.0},
    std::vector<double>{50.0, 100.0, 120.0, 140.0, 180.0, 180.0, 200.0, 200.0,
                        200.0});
}  // namespace

PathPoint EvaluatePathPointOnCubicSplines(const CubicSpline& x_s,
                                          const CubicSpline& y_s,
                                          double cur_s) {
  PathPoint pt;
  pt.set_x(x_s.Evaluate(cur_s));
  pt.set_y(y_s.Evaluate(cur_s));
  const double dx = x_s.EvaluateDerivative<1>(cur_s);
  const double dy = y_s.EvaluateDerivative<1>(cur_s);
  const double ddx = x_s.EvaluateDerivative<2>(cur_s);
  const double ddy = y_s.EvaluateDerivative<2>(cur_s);
  pt.set_s(cur_s);
  pt.set_theta(std::atan2(dy, dx));
  const double k_temp = ComputeCurvature(dx, dy, ddx, ddy);
  pt.set_kappa(k_temp);
  return pt;
}

DiscretizedPath ComputeExtendedSmoothPath(absl::Span<const Vec2d> ref_center_xy,
                                          absl::Span<const double> ref_s_vec,
                                          const Vec2d& heading, double step_s,
                                          double resample_step_s,
                                          double extended_length) {
  FUNC_QTRACE();
  constexpr int kMinCubicSplinePointNum = 5;
  constexpr double kEpsilonS = 1e-3;
  const int spline_n = std::max(kMinCubicSplinePointNum,
                                static_cast<int>(extended_length / step_s) + 1);
  std::vector<double> vec_x;
  std::vector<double> vec_y;
  std::vector<double> vec_s;
  vec_x.reserve(spline_n);
  vec_y.reserve(spline_n);
  vec_s.reserve(spline_n);
  const int n = ref_center_xy.size();
  for (int i = 0; i < n; ++i) {
    if (!vec_s.empty() && std::fabs(ref_s_vec[i] - vec_s.back()) < kEpsilonS) {
      continue;
    }
    vec_x.push_back(ref_center_xy[i].x());
    vec_y.push_back(ref_center_xy[i].y());
    vec_s.push_back(ref_s_vec[i]);
  }
  // Add extended path centers previous to solving cubic spline to ensure
  // smoothness of curvature.
  double length = step_s;
  const auto& xy_init = ref_center_xy.back();
  const auto& s_init = ref_s_vec.back();
  for (int i = 0; i < spline_n - n; ++i) {
    const auto xy = xy_init + length * heading;
    vec_x.push_back(xy.x());
    vec_y.push_back(xy.y());
    vec_s.push_back(s_init + length);
    length += step_s;
  }
  // solve smoothing spline
  const CubicSpline x_s(vec_s, vec_x);
  const CubicSpline y_s(vec_s, vec_y);

  DiscretizedPath path;
  const int speed_path_n =
      static_cast<int>(extended_length / resample_step_s) + 1;
  path.reserve(speed_path_n);
  {
    SCOPED_QTRACE("ComputeExtendedSmoothPath::ResampleCubicSpline");
    for (int i = 0; i < speed_path_n && i * resample_step_s < vec_s.back();
         ++i) {
      const double cur_s = i * resample_step_s;
      path.push_back(EvaluatePathPointOnCubicSplines(x_s, y_s, cur_s));
    }
  }

  VLOG(3) << absl::StrFormat(
      "extended length: %.3f, origin length: %.3f, spline num: %d, "
      "speed_path_n :%d, speed path "
      "actual num: %d, speed path length: %.3f.",
      extended_length, ref_s_vec.back(), spline_n, speed_path_n, path.size(),
      path.length());

  return path;
}

PiecewiseLinearFunction<double> ComputeSmoothedCurvatureProfile(
    absl::Span<const Vec2d> xy) {
  FUNC_QTRACE();
  const int num_pts = xy.size();
  if (num_pts <= 1) {
    // Set to zero curvature.
    constexpr double kMaxCurvatureProfileLength = 1000.0;  // m.
    return PiecewiseLinearFunction<double>({0.0, kMaxCurvatureProfileLength},
                                           {0.0, 0.0});
  }
  std::vector<Vec2d> vec_xs, vec_ys;
  vec_xs.reserve(num_pts);
  vec_ys.reserve(num_pts);

  double cur_s = 0.0;
  for (int i = 0; i < num_pts; ++i) {
    vec_xs.emplace_back(cur_s, xy[i].x());
    vec_ys.emplace_back(cur_s, xy[i].y());
    if (i > 0) {
      cur_s += (xy[i] - xy[i - 1]).norm();
    }
  }
  constexpr int kPolynomialDegree = 3;
  const auto x_polynomial_or = FitPolynomialXdToData<kPolynomialDegree>(vec_xs);
  const auto y_polynomial_or = FitPolynomialXdToData<kPolynomialDegree>(vec_ys);
  const bool has_kappa = x_polynomial_or.ok() && y_polynomial_or.ok();
  std::vector<double> vec_s, vec_kappa;
  vec_s.reserve(num_pts);
  vec_kappa.reserve(num_pts);
  const double sample_step = cur_s / num_pts;
  for (int i = 0; i < num_pts; ++i) {
    const double s = i * sample_step;
    vec_s.push_back(s);
    if (has_kappa) {
      const double dx = x_polynomial_or->EvaluateDerivative</*order=*/1>(s);
      const double d2x = x_polynomial_or->EvaluateDerivative</*order=*/2>(s);
      const double dy = y_polynomial_or->EvaluateDerivative</*order=*/1>(s);
      const double d2y = y_polynomial_or->EvaluateDerivative</*order=*/2>(s);
      vec_kappa.push_back(ComputeCurvature(dx, dy, d2x, d2y));
    } else {
      vec_kappa.push_back(0.0);
    }
  }
  return PiecewiseLinearFunction<double>(vec_s, vec_kappa);
}

absl::StatusOr<DrivingMapTopo> BuildDrivingMapTopo(
    const PoseProto& pose, const PlannerSemanticMapManager& psmm,
    double plan_start_v) {
  const Vec2d pose_pos(pose.pos_smooth().x(), pose.pos_smooth().y());
  const auto max_desire_length =
      kMaxVelocityKphCredibleLengthPLF(Mps2Kph(plan_start_v));
  constexpr double kDrivingMapStartLaneSearchRadius = 10.0;
  constexpr double kDrivingMapStastLaneSearchMaxHeadingDiff = M_PI_4;
  return BuildDrivingMapAtPosAndHeadingInRadiusWithMaxLength(
      psmm, pose_pos, pose.yaw(), kDrivingMapStartLaneSearchRadius,
      max_desire_length, kDrivingMapStastLaneSearchMaxHeadingDiff,
      /*allow_virtual=*/false);
}

}  // namespace planner
}  // namespace qcraft

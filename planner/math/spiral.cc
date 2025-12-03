#include "onboard/planner/math/spiral.h"

#include <algorithm>  // for max

#include "onboard/global/buffered_logger.h"
#include "onboard/lite/check.h"
#include "onboard/math/fast_math.h"

namespace {
// Evaluating polynomial  \sum s * params[i]^(size - i - 1), i = 0..size - 1).
double EvalPoly(double s, absl::Span<const double> params) {
  double res = 0;
  for (const auto param : params) {
    res = res * s + param;
  }
  return res;
}
}  // namespace

namespace qcraft::planner {
Spiral::Spiral(double x, double y, double theta, double sg,
               absl::Span<const double> params)
    : x_(x),
      y_(y),
      theta_(theta),
      sg_(sg),
      params_(params.begin(), params.end()) {
  QCHECK_GT(params_.size(), 0);
  theta_params_.reserve(params_.size() + 1);
  const int k_poly_order = order();
  for (int i = 0; i <= k_poly_order; ++i) {
    theta_params_.push_back(params_[i] / (k_poly_order - i + 1.0));
  }
  theta_params_.push_back(theta_);
}

SpiralPoint Spiral::Eval(double s, int num_steps) const {
  const auto points = BatchEval(s, num_steps);
  return points.back();
}

std::vector<SpiralPoint> Spiral::BatchEval(double s, int num_steps) const {
  std::vector<SpiralPoint> res;
  res.reserve(num_steps + 1);
  SpiralPoint sp0 = {
      .x = 0.0, .y = 0.0, .theta = theta_, .k = params_.back(), .s = 0.0};
  res.push_back(sp0);
  double dx0 = 0.0;
  double dy0 = 0.0;
  const double ds = s / num_steps;
  for (int i = 0; i < num_steps; ++i) {
    double dx1 = 0.0;
    double dy1 = 0.0;
    sp0 = IntegrateOneStep(sp0, dx0, dy0, i + 1, ds, &dx1, &dy1);
    res.push_back(sp0);
    dx0 = dx1;
    dy0 = dy1;
  }
  for (auto& sp : res) {
    sp.x += x_;
    sp.y += y_;
  }
  return res;
}

SpiralPoint Spiral::IntegrateOneStep(const SpiralPoint& sp0, double dx0,
                                     double dy0, int step, double ds,
                                     double* pdx1, double* pdy1) const {
  SpiralPoint sp1;
  sp1.s = sp0.s + ds;
  sp1.k = EvalPoly(sp1.s, params_);
  sp1.theta = EvalPoly(sp1.s, theta_params_);
  const double inv_step = 1.0 / step;
  double sp1_cos_sin[2];
  fast_math::CosAndSin<12>(sp1.theta, sp1_cos_sin);
  double sp0_cos_sin[2];
  fast_math::CosAndSin<12>(sp0.theta, sp0_cos_sin);
  const double dx1 = dx0 * (step - 1) * inv_step +
                     (sp1_cos_sin[0] + sp0_cos_sin[0]) * 0.5 * inv_step;
  const double dy1 = dy0 * (step - 1) * inv_step +
                     (sp1_cos_sin[1] + sp0_cos_sin[1]) * 0.5 * inv_step;
  sp1.x = sp1.s * dx1;
  sp1.y = sp1.s * dy1;
  *pdx1 = dx1;
  *pdy1 = dy1;
  return sp1;
}
}  // namespace qcraft::planner

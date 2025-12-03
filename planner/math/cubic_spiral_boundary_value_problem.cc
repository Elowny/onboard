#include "onboard/planner/math/cubic_spiral_boundary_value_problem.h"

#include <algorithm>
#include <cmath>

#include "LBFGSpp/LBFGS.h"
#include "LBFGSpp/Param.h"

#include "onboard/math/util.h"
#include "onboard/math/vec.h"

namespace {
constexpr double kLBGFSEpsilon = 1e-3;
constexpr int kLBGFSMaxIteration = 50;
}  // namespace

namespace qcraft::planner {
inline double CubicSpiralBoundaryValueProblem::Theta(double p0, double p1,
                                                     double p2, double p3,
                                                     double sg,
                                                     double ratio) const {
  const double k = ratio;
  const double t3 = k * k;
  const double t4 = t3 * sg;
  const double t14 = k * t3 * sg;
  const double t23 = t3 * t3;
  const double t24 = t23 * sg;
  const double theta =
      p0 * k * sg - 0.11e2 / 0.4e1 * p0 * t4 + 0.9e1 / 0.2e1 * p1 * t4 -
      0.9e1 / 0.4e1 * p2 * t4 + p3 * t4 / 0.2e1 + 0.3e1 * p0 * t14 -
      0.15e2 / 0.2e1 * p1 * t14 + 0.6e1 * p2 * t14 - 0.3e1 / 0.2e1 * p3 * t14 -
      0.9e1 / 0.8e1 * p0 * t24 + 0.27e2 / 0.8e1 * p1 * t24 -
      0.27e2 / 0.8e1 * p2 * t24 + 0.9e1 / 0.8e1 * p3 * t24;
  return theta;
}

inline Vec3d CubicSpiralBoundaryValueProblem::ThetaGrad(double p0, double p1,
                                                        double p2, double p3,
                                                        double sg,
                                                        double ratio) const {
  Vec3d grad;
  const double k = ratio;
  double t1 = k * k;
  const double t2 = t1 * t1;
  const double t3 = 0.27e2 / 0.8e1 * k;
  t1 = sg * t1;
  grad[0] = t1 * ((t3 - 0.15e2 / 0.2e1) * k + 0.9e1 / 0.2e1);
  grad[1] = t1 * ((-t3 + 0.6e1) * k - 0.9e1 / 0.4e1);
  grad[2] = k * (k * ((-0.15e2 / 0.2e1 * k + 0.9e1 / 0.2e1) * p1 +
                      (0.6e1 * k - 0.9e1 / 0.4e1) * p2) +
                 ((0.3e1 * k - 0.11e2 / 0.4e1) * k + 0.1e1) * p0 +
                 k * (-0.3e1 / 0.2e1 * k + 0.1e1 / 0.2e1) * p3) -
            0.9e1 / 0.8e1 * t2 * (p0 - p3) + 0.27e2 / 0.8e1 * t2 * (p1 - p2);
  return grad;
}

inline double CubicSpiralBoundaryValueProblem::CosTheta(double theta) const {
  return std::cos(theta);
}

inline Vec3d CubicSpiralBoundaryValueProblem::CosThetaGrad(
    double theta, const Vec3d& theta_grad) const {
  return -std::sin(theta) * theta_grad;
}

inline double CubicSpiralBoundaryValueProblem::SinTheta(double theta) const {
  return std::sin(theta);
}

inline Vec3d CubicSpiralBoundaryValueProblem::SinThetaGrad(
    double theta, const Vec3d& theta_grad) const {
  return std::cos(theta) * theta_grad;
}

inline std::pair<double, double> CubicSpiralBoundaryValueProblem::XY(
    double p0, double p1, double p2, double p3, double sg, Vec3d* ptr_xgrad,
    Vec3d* ptr_ygrad) const {
  const double resolution = 1.0 / params_.num_steps;
  const double h = sg * resolution;
  double fx = 0.0;
  double fy = 0.0;
  Vec3d fx_grad;
  Vec3d fy_grad;
  for (int i = 0; i <= params_.num_steps; i++) {
    const double cur_ratio = i * resolution;
    const double theta = Theta(p0, p1, p2, p3, sg, cur_ratio);
    const auto theta_grad = ThetaGrad(p0, p1, p2, p3, sg, cur_ratio);
    const double fx_cur = CosTheta(theta);
    const auto fx_cur_grad = CosThetaGrad(theta, theta_grad);
    const double fy_cur = SinTheta(theta);
    const auto fy_cur_grad = SinThetaGrad(theta, theta_grad);
    if (i == 0 || i == params_.num_steps) {
      fx += fx_cur;
      fy += fy_cur;
      fx_grad += fx_cur_grad;
      fy_grad += fy_cur_grad;
    } else if (i & 1) {  // i is odd.
      fx += (4 * fx_cur);
      fy += (4 * fy_cur);
      fx_grad += (4 * fx_cur_grad);
      fy_grad += (4 * fy_cur_grad);
    } else {
      // i is even.
      fx += (2 * fx_cur);
      fy += (2 * fy_cur);
      fx_grad += (2 * fx_cur_grad);
      fy_grad += (2 * fy_cur_grad);
    }
  }
  Vec3d sg_grad(0.0, 0.0, 1.0);
  fx_grad = (resolution / 3.0) * (sg_grad * fx + fx_grad * sg);
  *ptr_xgrad = fx_grad;
  fy_grad = (resolution / 3.0) * (sg_grad * fy + fy_grad * sg);
  *ptr_ygrad = fy_grad;
  return std::make_pair(fx * h / 3.0, fy * h / 3.0);
}

std::vector<double> CubicSpiralBoundaryValueProblem::getPrams(
    double p0, double p1, double p2, double p3, double sg) const {
  std::vector<double> params = {
      -9.0 * (p0 - 3.0 * p1 + 3.0 * p2 - p3) / (2.0 * sg * sg * sg),
      9.0 * (2.0 * p0 - 5.0 * p1 + 4.0 * p2 - p3) / (2.0 * sg * sg),
      -(11.0 * p0 - 18.0 * p1 + 9.0 * p2 - 2.0 * p3) / (2.0 * sg), p0};
  return params;
}

double CubicSpiralBoundaryValueProblem::LeastSquare(const VecXd& vecx,
                                                    VecXd* ptr_grad) const {
  const double p0 = k0_;
  const double p1 = vecx[0];
  const double p2 = vecx[1];
  const double p3 = k1_;
  const double sg = vecx[2];
  Vec3d xgrad;
  Vec3d ygrad;
  Vec3d thetagrad;
  const auto val = XY(p0, p1, p2, p3, sg, &xgrad, &ygrad);
  const double x = val.first;
  const double y = val.second;
  const double theta = Theta(p0, p1, p2, p3, sg, 1.0);
  thetagrad = ThetaGrad(p0, p1, p2, p3, sg, 1.0);
  const double dx = x - x1_;
  const double dy = y - y1_;
  const double dtheta = theta - theta1_;
  *ptr_grad = 2.0 * params_.wx * dx * xgrad + 2.0 * params_.wy * dy * ygrad +
              2.0 * params_.wtheta * dtheta * thetagrad;
  return params_.wx * (dx * dx) + params_.wy * (dy * dy) +
         params_.wtheta * (dtheta * dtheta);
}

std::vector<Spiral> CubicSpiralBoundaryValueSolver::solve(
    const SpiralPoint& start, const SpiralPoint& end) {
  // Shift and rotate start point to (0.0, 0.0) and start angle to 0 rad.
  const double k0 = start.k;
  const double x1 = end.x - start.x;
  const double y1 = end.y - start.y;
  Vec2d cur_xy(x1, y1);
  const auto rotated_xy = cur_xy.Rotate(-start.theta);
  const double k1 = end.k;
  double theta1 = NormalizeAngle(end.theta - start.theta);
  std::vector<double> possible_thetas;
  constexpr double kEpsilon = 1e-3;
  constexpr double kLargeAngleThreshold = M_PI * 2.0 / 3.0 - kEpsilon;
  // We always get two possibilities for spiral connection: clock-wise and
  // counter-clock-wise. If the angle diff is small than kLargeAngleThreshold,
  // we assume that we follow the direction with small angle diff. If the angle
  // diff is larger than PI * 2 / 3, we assume that we are not clear which side
  // we should pick and return two spirals instead of one.
  if (std::fabs(theta1) <= kLargeAngleThreshold) {
    possible_thetas.push_back(theta1);
  } else if (theta1 > kLargeAngleThreshold) {
    possible_thetas.push_back(theta1);
    possible_thetas.push_back(theta1 - 2 * M_PI);
  } else {
    possible_thetas.push_back(theta1 + 2 * M_PI);
    possible_thetas.push_back(theta1);
  }
  std::vector<Spiral> spirals;
  for (const auto& theta : possible_thetas) {
    CubicSpiralBoundaryValueProblem problem(k0, rotated_xy.x(), rotated_xy.y(),
                                            theta, k1);
    LBFGSpp::LBFGSParam<double> param;
    param.epsilon = kLBGFSEpsilon;
    param.max_iterations = kLBGFSMaxIteration;
    LBFGSpp::LBFGSSolver<double> solver(param);
    VecXd var(3);
    var << 0.0, 0.0, sqrt(x1 * x1 + y1 * y1);
    double fx = 0.0;
    try {  // Catch the solver failure errors. Do not build spiral if failed to
           // solve.
      solver.minimize(problem, var, fx);
    } catch (...) {
      continue;
    }
    const auto spiral_params =
        problem.getPrams(start.k, var[0], var[1], end.k, var[2]);
    // The length of the spiral should be greater than 0.0.
    if (var[2] > 0.0) {
      spirals.emplace_back(start.x, start.y, start.theta, var[2],
                           spiral_params);
    }
  }
  return spirals;
}

}  // namespace qcraft::planner

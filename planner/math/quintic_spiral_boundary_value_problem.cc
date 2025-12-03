#include "onboard/planner/math/quintic_spiral_boundary_value_problem.h"

#include <algorithm>
#include <cmath>
#include <utility>

// IWYU pragma: no_include "Eigen/Core"
#include "LBFGSpp/LBFGS.h"
#include "LBFGSpp/Param.h"

#include "onboard/math/util.h"
#include "onboard/math/vec.h"

namespace {
constexpr double kLBGFSEpsilon = 1e-4;
constexpr int kLBGFSMaxIteration = 100;
}  // namespace

namespace qcraft::planner {

namespace {

double Theta(double p0, double p1, double p2, double p3, double p4, double p5,
             double sg, double ratio) {
  const double k = ratio;
  const double k4 = k * k * k * k;
  const double sg3 = sg * sg * sg;
  const double k6 = k4 * k * k;
  return p0 * (k * sg - 0.575e3 / 0.32e2 * k4 * sg +
               0.333e3 * k4 * k * sg / 0.1e2 - 0.255e3 * k6 * sg / 0.16e2) +
         p4 * (-0.81e2 / 0.32e2 * k4 * sg + 0.81e2 * k4 * k * sg / 0.1e2 -
               0.81e2 / 0.16e2 * k6 * sg) +
         p5 * (k4 * sg / 0.4e1 - 0.9e1 / 0.1e2 * k4 * k * sg +
               0.3e1 / 0.4e1 * k6 * sg) +
         p3 * 0.81e2 / 0.4e1 * k4 * (sg - 0.2e1 * k * sg + k * k * sg) +
         p1 * (k * k * sg * sg / 0.2e1 - 0.85e2 * k4 * sg * sg / 0.16e2 +
               0.9e1 * k4 * k * sg * sg - 0.33e2 * k6 * sg * sg / 0.8e1) +
         p2 * (k * k * k * sg3 / 0.6e1 - 0.11e2 * k4 * sg3 / 0.16e2 +
               0.9e1 * k4 * k * sg3 / 0.1e2 - 0.3e1 / 0.8e1 * k6 * sg3);
}

Vec3d ThetaGrad(double p0, double p1, double p2, double p3, double p4,
                double p5, double sg, double ratio) {
  Vec3d grad;
  const double k = ratio;
  const double k4 = k * k * k * k;
  const double k5 = k4 * k;
  const double k6 = k5 * k;
  // d(Theta)/d(p3)
  grad[0] = 0.81e2 / 0.2e1 * k4 * (sg / 0.2e1 - sg * k + sg * k * k / 0.2e1);
  // d(Theta)/d(p4)
  const double temp1 = 0.81e2 * k4 * sg;
  grad[1] = temp1 * (-0.1e1 / 0.32e2 + k / 0.1e2 - k * k / 0.16e2);
  // d(Theta)/d(sg)
  grad[2] =
      (k - 0.575e3 / 0.32e2 * k4 + 0.333e3 / 0.1e2 * k5 -
       0.255e3 / 0.16e2 * k6) *
          p0 +
      (0.81e2 / 0.4e1 * k4 - 0.81e2 / 0.2e1 * k5 + 0.81e2 / 0.4e1 * k6) * p3 +
      (-0.81e2 / 0.32e2 * k4 + 0.81e2 / 0.1e2 * k5 - 0.81e2 / 0.16e2 * k6) *
          p4 +
      (k4 / 0.4e1 - 0.9e1 / 0.1e2 * k5 + 0.3e1 / 0.4e1 * k6) * p5 +
      p1 * (k * k * sg - 0.85e2 / 0.8e1 * k4 * sg + 0.18e2 * k5 * sg -
            0.33e2 * k6 * sg / 0.4e1) +
      p2 * (k * k * k * sg * sg / 0.2e1 - 0.33e2 * k4 * sg * sg / 0.16e2 +
            0.27e2 / 0.1e2 * k5 * sg * sg - 0.9e1 / 0.8e1 * k6 * sg * sg);

  return grad;
}

double CosTheta(double theta) { return std::cos(theta); }

Vec3d CosThetaGrad(double theta, const Vec3d& theta_grad) {
  return -std::sin(theta) * theta_grad;
}

double SinTheta(double theta) { return std::sin(theta); }

Vec3d SinThetaGrad(double theta, const Vec3d& theta_grad) {
  return std::cos(theta) * theta_grad;
}

std::pair<double, double> XY(
    double p0, double p1, double p2, double p3, double p4, double p5, double sg,
    Vec3d* ptr_xgrad, Vec3d* ptr_ygrad,
    const QuinticSpiralBoundaryValueProblemParams& params) {
  const double resolution = 1.0 / params.num_steps;
  const double h = sg * resolution;
  double fx = 0.0;
  double fy = 0.0;
  Vec3d fx_grad;
  Vec3d fy_grad;
  for (int i = 0; i <= params.num_steps; i++) {
    const double cur_ratio = i * resolution;
    const double theta = Theta(p0, p1, p2, p3, p4, p5, sg, cur_ratio);
    const auto theta_grad =
        ThetaGrad(p0, p1, p2, p3, p4, p5, sg, cur_ratio);  // Vec3d
    const double fx_cur = CosTheta(theta);
    const auto fx_cur_grad = CosThetaGrad(theta, theta_grad);
    const double fy_cur = SinTheta(theta);
    const auto fy_cur_grad = SinThetaGrad(theta, theta_grad);
    if (i == 0 || i == params.num_steps) {
      fx += fx_cur;
      fy += fy_cur;
      fx_grad += fx_cur_grad;
      fy_grad += fy_cur_grad;
    } else if ((i & 1) != 0) {
      fx += (4 * fx_cur);
      fy += (4 * fy_cur);
      fx_grad += (4 * fx_cur_grad);
      fy_grad += (4 * fy_cur_grad);
    } else {
      fx += (2 * fx_cur);
      fy += (2 * fy_cur);
      fx_grad += (2 * fx_cur_grad);
      fy_grad += (2 * fy_cur_grad);
    }
  }

  const Vec3d sg_grad(0.0, 0.0, 1.0);
  fx_grad = (resolution / 3.0) * (sg_grad * fx + fx_grad * sg);
  fy_grad = (resolution / 3.0) * (sg_grad * fy + fy_grad * sg);
  *ptr_xgrad = fx_grad;
  *ptr_ygrad = fy_grad;
  return std::make_pair(fx * h / 3.0, fy * h / 3.0);
}

}  // namespace

std::vector<double> QuinticSpiralBoundaryValueProblem::getParams(
    double p0, double p1, double p2, double p3, double p4, double p5,
    double sg) const {
  std::vector<double> params = {
      /*f(p)*/ -9.0 *
          (85.0 * p0 - 108.0 * p3 + 27.0 * p4 - 4.0 * p5 + 22.0 * p1 * sg +
           2.0 * p2 * sg * sg) /
          (8.0 * sg * sg * sg * sg * sg),
      /*e(p)*/ 9.0 *
          (37.0 * p0 - 45.0 * p3 + 9.0 * p4 - p5 + 10.0 * p1 * sg +
           p2 * sg * sg) /
          (2.0 * sg * sg * sg * sg),
      /*d(p)*/
      -(575.0 * p0 - 648.0 * p3 + 81.0 * p4 - 8.0 * p5 + 170.0 * p1 * sg +
        22.0 * p2 * sg * sg) /
          (8.0 * sg * sg * sg),
      /*c(p)*/ p2 / 2.0,
      /*b(p)*/ p1, /*a(p)*/ p0};
  return params;
}

double QuinticSpiralBoundaryValueProblem::LeastSquare(const VecXd& vecx,
                                                      VecXd* ptr_grad) const {
  const double p0 = k0_;
  const double p1 = dk0_;
  const double p2 = ddk0_;
  const double p3 = vecx[0];
  const double p4 = vecx[1];
  const double p5 = k1_;
  const double sg = vecx[2];

  Vec3d xgrad;
  Vec3d ygrad;
  Vec3d thetagrad;
  const auto val = XY(p0, p1, p2, p3, p4, p5, sg, &xgrad, &ygrad, params_);
  const double x = val.first;
  const double y = val.second;
  const double theta = Theta(p0, p1, p2, p3, p4, p5, sg, 1.0);
  thetagrad = ThetaGrad(p0, p1, p2, p3, p4, p5, sg, 1.0);
  const double dx = x - x1_;
  const double dy = y - y1_;
  const double dtheta = theta - theta1_;

  *ptr_grad = 2.0 * params_.wx * dx * xgrad + 2.0 * params_.wy * dy * ygrad +
              2.0 * params_.wtheta * dtheta * thetagrad;
  return params_.wx * (dx * dx) + params_.wy * (dy * dy) +
         params_.wtheta * (dtheta * dtheta);
}

std::vector<Spiral> QuinticSpiralBoundaryValueSolver::solve(
    const SpiralPoint& start, const SpiralPoint& end) {
  // Transform the start point to x = 0.0, y = 0.0, theta = 0.0 rad point
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
  spirals.reserve(possible_thetas.size());
  for (const auto& theta : possible_thetas) {
    LBFGSpp::LBFGSParam<double> param;
    param.max_iterations = kLBGFSMaxIteration;
    param.epsilon = kLBGFSEpsilon;
    LBFGSpp::LBFGSSolver<double> solver(param);

    VecXd var(3);
    var << 0.0, 0.0, std::hypot(rotated_xy.x(), rotated_xy.y());

    double fx = 0;
    const SpiralPoint sp0 = {.x = 0.0,
                             .y = 0.0,
                             .theta = 0.0,
                             .k = k0,
                             .dk = start.dk,
                             .ddk = start.ddk};
    const SpiralPoint sp1 = {.x = rotated_xy.x(),
                             .y = rotated_xy.y(),
                             .theta = theta,
                             .k = k1,
                             .dk = end.dk,
                             .ddk = end.ddk};

    QuinticSpiralBoundaryValueProblem problem(sp0.k, sp1.x, sp1.y, sp1.theta,
                                              sp1.k, sp0.dk, sp0.ddk);

    try {
      solver.minimize(problem, var, fx);
    } catch (...) {
      continue;
    }
    // Recall the constraints: p0 = kI, p1 = dkI, p2 = ddkI, p3, p4, p5 =
    // kG, sg
    std::vector<double> spiral_params = problem.getParams(
        start.k, start.dk, start.ddk, var[0], var[1], end.k, var[2]);
    // The length of the spiral should be greater than 0.0.
    if (var[2] > 0.0) {
      spirals.emplace_back(start.x, start.y, start.theta, var[2],
                           spiral_params);
    }
  }
  return spirals;
}

}  // namespace qcraft::planner

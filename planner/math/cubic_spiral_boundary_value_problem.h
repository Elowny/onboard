#ifndef ONBOARD_PLANNER_MATH_CUBIC_SPIRAL_BOUNDARY_VALUE_PROBLEM_H_
#define ONBOARD_PLANNER_MATH_CUBIC_SPIRAL_BOUNDARY_VALUE_PROBLEM_H_
#include <utility>
#include <vector>

#include "onboard/math/eigen.h"
#include "onboard/math/vec.h"
#include "onboard/planner/math/spiral.h"

namespace qcraft::planner {
// Define weights for the least square problem.
struct CubicSpiralBoundaryValueProblemParams {
  double wx;
  double wy;
  double wtheta;
  int num_steps;
};
// Given a start configuration (x0,y0,theta0,k0) and an end configuration (x1,
// y1, theta1, k1), find a cubic spiral that connects both. For simplicity,
// start configuration is rotated and moved to (x0=0.0,y0=0.0,theta0=0.0,k0).
// Cubic spiral has no close form solution. Check the thesis "Parallel
// Algorithms for Real-time Motion Planning"
// (https://www.ri.cmu.edu/pub_files/2011/7/mcnaughton-thesis.pdf) for the
// approximations.
class CubicSpiralBoundaryValueProblem {
 public:
  CubicSpiralBoundaryValueProblem() = delete;
  CubicSpiralBoundaryValueProblem(double k0, double x1, double y1,
                                  double theta1, double k1)
      : k0_(k0), x1_(x1), y1_(y1), theta1_(theta1), k1_(k1) {}
  // L-BGFS solver's interface requires a non-const reference to pass gradient
  // value. Add NOLINT to ensure compatibility.
  double operator()(const VecXd& vecx, VecXd& grad) const {  // NOLINT
    return LeastSquare(vecx, &grad);
  }
  double LeastSquare(const VecXd& vecx, VecXd* ptr_grad) const;
  std::vector<double> getPrams(double p0, double p1, double p2, double p3,
                               double sg) const;

 private:
  std::pair<double, double> XY(double p0, double p1, double p2, double p3,
                               double sg, Vec3d* ptr_xgrad,
                               Vec3d* ptr_ygrad) const;
  double Theta(double p0, double p1, double p2, double p3, double sg,
               double ratio) const;
  Vec3d ThetaGrad(double p0, double p1, double p2, double p3, double sg,
                  double ratio) const;
  double CosTheta(double theta) const;
  Vec3d CosThetaGrad(double theta, const Vec3d& theta_grad) const;
  double SinTheta(double theta) const;
  Vec3d SinThetaGrad(double theta, const Vec3d& theta_grad) const;

  double k0_;
  double x1_;
  double y1_;
  double theta1_;
  double k1_;
  const CubicSpiralBoundaryValueProblemParams params_ = {
      /*wx=*/1.0, /*wy=*/1.0, /*wtheta=*/1.0, /*num_steps=*/8};
};

class CubicSpiralBoundaryValueSolver {
 public:
  CubicSpiralBoundaryValueSolver() = default;
  static std::vector<Spiral> solve(const SpiralPoint& start,
                                   const SpiralPoint& end);
};
}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_MATH_CUBIC_SPIRAL_BOUNDARY_VALUE_PROBLEM_H_

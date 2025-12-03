#ifndef ONBOARD_PLANNER_MATH_SPIRAL_H_
#define ONBOARD_PLANNER_MATH_SPIRAL_H_

#include <cmath>
#include <string>
#include <vector>

#include "absl/strings/str_cat.h"
#include "absl/types/span.h"

namespace qcraft::planner {
struct SpiralPoint {
  double x;
  double y;
  double theta;
  double k;
  double s;
  double dk;
  double ddk;
  std::string DebugString() const {
    return absl::StrCat("x: ", x, "y: ", y, " theta: ", theta, " k: ", k,
                        "s:", s, "dk: ", dk, "ddk: ", ddk);
  }

  double DistanceTo(const SpiralPoint& p) const {
    return std::sqrt((p.x - this->x) * (p.x - this->x) +
                     (p.y - this->y) * (p.y - this->y));
  }
};

class Spiral {
 public:
  // Build a spiral curve from a start configuration (defined by SpiralPoint
  // struct) and a set of parameters. Parameters define k(s) = order N
  // polynomial of s. The variable s is the arc length of the spiral.
  // Params are in the form of k(s) = params[0]*s^order + params[1]
  // *s^(order-1) + ....
  // The parameter sg is the curve length of the spiral.
  Spiral(double x, double y, double theta, double sg,
         absl::Span<const double> params);
  // This function integrates the spiral to get the point at the position of
  // curve length s. If you need multiple points, prefer to use BatchEval.
  SpiralPoint Eval(double s, int num_steps) const;
  // Spiral has no close-form solution. We use the iterative form of trapezoidal
  // integration method mentioned in "Parallel Algorithms for Real-time Motion
  // Planning" (https://www.ri.cmu.edu/pub_files/2011/7/mcnaughton-thesis.pdf).
  std::vector<SpiralPoint> BatchEval(double s, int num_steps) const;
  int order() const { return static_cast<int>(params_.size()) - 1; }
  double length() const { return sg_; }

 private:
  SpiralPoint IntegrateOneStep(const SpiralPoint& sp0, double dx0, double dy0,
                               int step, double ds, double* pdx1,
                               double* pdy1) const;
  double x_;
  double y_;
  double theta_;
  double sg_;
  std::vector<double> params_;
  std::vector<double> theta_params_;
};
}  // namespace qcraft::planner
#endif  // ONBOARD_PLANNER_MATH_SPIRAL_H_

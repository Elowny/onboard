#ifndef ONBOARD_CONTROL_CALIBRATION_TOOLS_UTILS_H_
#define ONBOARD_CONTROL_CALIBRATION_TOOLS_UTILS_H_

#include <vector>

#include "onboard/math/eigen.h"

namespace qcraft {
namespace control {

double PrecisionTruncation(double value, int num);

double FindNearst(const std::vector<double>& x_list, double x0);

class PolyFit {
 public:
  explicit PolyFit(int order);
  void SetOrder(int order);
  bool ComputerCoff(const std::vector<double>& x_vec,
                    const std::vector<double>& y_vec);
  double GetPolyVal(double x);

 private:
  int order_ = 0;
  VecXd coff_;

  double SumSolveA(const std::vector<double>& x_data, int row, int col);

  double SumSolveY(const std::vector<double>& x_data,
                   const std::vector<double>& y_data, int row);
};

}  // namespace control
}  // namespace qcraft

#endif  // ONBOARD_CONTROL_CALIBRATION_TOOLS_UTILS_H_

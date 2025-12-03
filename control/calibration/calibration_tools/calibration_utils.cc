#include "onboard/control/calibration/calibration_tools/calibration_utils.h"

#include <cmath>

namespace qcraft {
namespace control {

double PrecisionTruncation(double value, int num) {
  value *= std::pow(10.0, num);
  value = std::floor(value);
  return value / std::pow(10.0, num);
}

double FindNearst(const std::vector<double>& x_list, double x0) {
  if (x_list.empty()) return x0;

  double e = fabs(x_list[0] - x0);
  double min_e = e;
  int min_num = 0;
  for (int i = 1; i < x_list.size(); i++) {
    e = fabs(x_list[i] - x0);
    if (e < min_e) {
      min_e = e;
      min_num = i;
    }
  }
  return x_list[min_num];
}

PolyFit::PolyFit(int order) { order_ = order; }

bool PolyFit::ComputerCoff(const std::vector<double>& x_vec,
                           const std::vector<double>& y_vec) {
  if (x_vec.size() < (order_ + 1) || x_vec.size() != y_vec.size()) {
    return false;
  }
  MatXd a(order_ + 1, order_ + 1);
  VecXd y(order_ + 1);
  for (int i = 0; i <= order_; i++) {
    y(i) = SumSolveY(x_vec, y_vec, i);
  }
  for (int i = 0; i <= order_; i++) {
    for (int j = 0; j <= order_; j++) {
      a(i, j) = SumSolveA(x_vec, i, j);
    }
  }
  if (a.determinant() == 0) {
    return false;
  }
  coff_ = a.lu().solve(y);
  return true;
}

void PolyFit::SetOrder(int order) { order_ = order; }

double PolyFit::GetPolyVal(double x) {
  double y = 0.0;
  for (int i = 0; i <= order_; i++) {
    y += coff_(i) * std::pow(x, i);
  }
  return y;
}

double PolyFit::SumSolveA(const std::vector<double>& x_data, int row, int col) {
  double out = 0.0;
  for (int i = 0; i < x_data.size(); i++) {
    out += pow(x_data[i], row + col);
  }
  return out;
}

double PolyFit::SumSolveY(const std::vector<double>& x_data,
                          const std::vector<double>& y_data, int row) {
  double out = 0.0;
  for (int i = 0; i < x_data.size(); i++) {
    out += pow(x_data[i], row) * y_data[i];
  }
  return out;
}

}  // namespace control
}  // namespace qcraft

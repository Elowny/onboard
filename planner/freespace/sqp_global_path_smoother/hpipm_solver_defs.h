#ifndef ONBOARD_PLANNER_FREESPACE_HPIPM_SOLVER_DEFS_H
#define ONBOARD_PLANNER_FREESPACE_HPIPM_SOLVER_DEFS_H
#include <array>
#include <vector>

#include "onboard/math/eigen.h"
namespace qcraft {
namespace planner {
namespace sqp_global_smoother {
// State: x, y, v, theta, kappa.
constexpr int kStateSize = 5;
constexpr int kStateXIndex = 0;
constexpr int kStateYIndex = 1;
constexpr int kStateVIndex = 2;
constexpr int kStateThetaIndex = 3;
constexpr int kStateKappaIndex = 4;
// Control: a, psi.
constexpr int kControlSize = 2;
constexpr int kControlAIndex = 0;
constexpr int kControlPsiIndex = 1;

struct MpcState {
  double x = 0.0;
  double y = 0.0;
  double v = 0.0;
  double theta = 0.0;
  double k = 0.0;
};

struct MpcControl {
  double a = 0.0;
  double psi = 0.0;
};

using MpcStateVector = Eigen::Matrix<double, kStateSize, 1>;
using MpcControlVector = Eigen::Matrix<double, kControlSize, 1>;

using MpcMatrixA = Eigen::Matrix<double, kStateSize, kStateSize>;
using MpcMatrixB = Eigen::Matrix<double, kStateSize, kControlSize>;
using MpcMatrixb = Eigen::Matrix<double, kStateSize, 1>;

using MpcMatrixQ = Eigen::Matrix<double, kStateSize, kStateSize>;
using MpcMatrixR = Eigen::Matrix<double, kControlSize, kControlSize>;
using MpcMatrixS = Eigen::Matrix<double, kStateSize, kControlSize>;
using MpcMatrixq = Eigen::Matrix<double, kStateSize, 1>;
using MpcMatrixr = Eigen::Matrix<double, kControlSize, 1>;

using MpcMatrixC = Eigen::Matrix<double, Eigen::Dynamic, kStateSize>;
using MpcMatrixD = Eigen::Matrix<double, Eigen::Dynamic, kControlSize>;
using MpcMatrixd = Eigen::Matrix<double, Eigen::Dynamic, 1>;

using MpcMatrixZ = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>;
using MpcMatrixz = Eigen::Matrix<double, Eigen::Dynamic, 1>;

using MpcStateBounds = Eigen::Matrix<double, kStateSize, 1>;
using MpcControlBounds = Eigen::Matrix<double, kControlSize, 1>;
using MpcSlackBounds = Eigen::Matrix<double, Eigen::Dynamic, 1>;

inline MpcStateVector ToMpcStateVector(const MpcState& x) {
  MpcStateVector xk;
  xk << x.x, x.y, x.v, x.theta, x.k;
  return xk;
}

inline MpcControlVector ToMpcControlVector(const MpcControl& u) {
  MpcControlVector uk;
  uk << u.a, u.psi;
  return uk;
}

inline MpcState ToMpcState(const MpcStateVector& xk) {
  MpcState x;
  x.x = xk(kStateXIndex);  // NOLINT
  x.y = xk(kStateYIndex);
  x.v = xk(kStateVIndex);
  x.theta = xk(kStateThetaIndex);
  x.k = xk(kStateKappaIndex);
  return x;
}

inline MpcControl ToMpcControl(const MpcControlVector& uk) {
  MpcControl u;
  u.a = uk(kControlAIndex);
  u.psi = uk(kControlPsiIndex);
  return u;
}

inline MpcState ToMpcState(const std::array<double, kStateSize>& xk) {
  MpcState x;
  x.x = xk[kStateXIndex];
  x.y = xk[kStateYIndex];
  x.v = xk[kStateVIndex];
  x.theta = xk[kStateThetaIndex];
  x.k = xk[kStateKappaIndex];
  return x;
}

inline MpcControl ToMpcControl(const std::array<double, kControlSize>& uk) {
  MpcControl u;
  u.a = uk[kControlAIndex];
  u.psi = uk[kControlPsiIndex];
  return u;
}

inline std::array<double, kStateSize> ToMpcStateArray(const MpcState& xk) {
  std::array<double, kStateSize> x;
  x[kStateXIndex] = xk.x;
  x[kStateYIndex] = xk.y;
  x[kStateVIndex] = xk.v;
  x[kStateThetaIndex] = xk.theta;
  x[kStateKappaIndex] = xk.k;
  return x;
}

inline std::array<double, kControlSize> ToMpcControlArray(
    const MpcControl& uk) {
  std::array<double, kControlSize> u;
  u[kControlAIndex] = uk.a;
  u[kControlPsiIndex] = uk.psi;
  return u;
}

struct OptVariables {
  MpcState xk;
  MpcControl uk;
};

struct LinModelMatrix {
  MpcMatrixA A;
  MpcMatrixB B;
  MpcMatrixb b;
};

struct ConstrainsMatrix {
  // dl <= C xk + D uk <= du
  MpcMatrixC C;   // Polytopic state constraints
  MpcMatrixD D;   // Polytopic control constraints
  MpcMatrixd dl;  // Lower bounds
  MpcMatrixd du;  // Upper bounds
};

struct CostMatrix {
  MpcMatrixQ Q;
  MpcMatrixR R;
  MpcMatrixS S;
  MpcMatrixq q;
  MpcMatrixr r;
  MpcMatrixZ Z;
  MpcMatrixz z;
};

struct Stage {
  LinModelMatrix lin_model;
  CostMatrix cost_mat;
  ConstrainsMatrix constrains_mat;

  MpcStateBounds u_bounds_x;
  MpcStateBounds l_bounds_x;
  std::vector<int> idx_x;

  MpcControlBounds u_bounds_u;
  MpcControlBounds l_bounds_u;
  std::vector<int> idx_u;

  MpcSlackBounds u_bounds_s;
  MpcSlackBounds l_bounds_s;
  std::vector<int> idx_s;

  std::array<double, kStateSize> xk;
  std::array<double, kControlSize> uk;

  int nbx = 0;   // Number of bounds on x.
  int nbu = 0;   // Number of bounds on u.
  int ng = 0;    // Number of bounds on polytopic constratins.
  int ns = 0;    // Number of bounds on soft constraints.
  int nsbx = 0;  // Number of soft constraints bounds on x.
  int nsbu = 0;  // Number of soft constraints bounds on u.
  int nsg = 0;   // Number of soft constraints bounds on polytopic constratins.
};
}  // namespace sqp_global_smoother
}  // namespace planner
}  // namespace qcraft
#endif

#include "onboard/planner/freespace/hybrid_a_star/reeds_shepp.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <ostream>
#include <utility>

#include "absl/status/status.h"
#include "glog/logging.h"

#include "onboard/global/buffered_logger.h"
#include "onboard/lite/check.h"
#include "onboard/math/fast_math.h"
#include "onboard/math/util.h"

namespace qcraft {
namespace planner {

struct RSPParam {
  bool flag = false;
  double t = 0.0;
  double u = 0.0;
  double v = 0.0;
};

constexpr double kZero = 10.0 * std::numeric_limits<double>::epsilon();

std::pair<double, double> CalcTauOmega(double u, double v, double xi,
                                       double eta, double phi) {
  double cos_sin[2];
  fast_math::CosAndSin<7>(u, cos_sin);
  const double sin_u = cos_sin[1];
  const double cos_u = cos_sin[0];
  const double delta = NormalizeAngle(u - v);
  const double sin_delta = fast_math::SinNormalized(delta);
  const double cos_delta = fast_math::CosNormalized(delta);

  const double a = sin_u - sin_delta;
  const double b = cos_u - cos_delta - 1.0;
  const double t1 = std::atan2(eta * a - xi * b, xi * a + eta * b);
  const double t2 = 2.0 * (cos_delta - fast_math::Cos(v) - cos_u) + 3.0;
  double tau = 0.0;
  if (t2 < 0.0) {
    tau = NormalizeAngle(t1 + M_PI);
  } else {
    tau = NormalizeAngle(t1);
  }
  const double omega = NormalizeAngle(tau - u + v - phi);
  return std::make_pair(tau, omega);
}

std::pair<double, double> Cartesian2Polar(double x, double y) {
  const double r = std::sqrt(x * x + y * y);
  const double theta = std::atan2(y, x);
  return std::make_pair(r, theta);
}

// Please refer to paper "Optimal paths for a car that goes both forwards and
// backwards" for the following formulas.

// Formula 8.1
void LSL(double x, double y, double phi, double sin_phi, double cos_phi,
         RSPParam* param) {
  const auto polar = Cartesian2Polar(x - sin_phi, y - 1.0 + cos_phi);
  const double u = polar.first;
  const double t = polar.second;
  if (t >= -kZero) {
    const double v = NormalizeAngle(phi - t);
    if (v >= -kZero) {
      param->flag = true;
      param->u = u;
      param->t = t;
      param->v = v;
    }
  }
}

// Formula 8.2
void LSR(double x, double y, double phi, double sin_phi, double cos_phi,
         RSPParam* param) {
  const auto polar = Cartesian2Polar(x + sin_phi, y - 1.0 - cos_phi);
  const double u1 = polar.first * polar.first;
  const double t1 = polar.second;
  if (u1 >= 4.0) {
    const double u = std::sqrt(u1 - 4.0);
    const double theta = std::atan2(2.0, u);
    const double t = NormalizeAngle(t1 + theta);
    const double v = NormalizeAngle(t - phi);
    if (t >= -kZero && v >= -kZero) {
      param->flag = true;
      param->u = u;
      param->t = t;
      param->v = v;
    }
  }
}

// Formula 8.3 / 8.4
void LRL(double x, double y, double phi, double sin_phi, double cos_phi,
         RSPParam* param) {
  const auto polar = Cartesian2Polar(x - sin_phi, y - 1.0 + cos_phi);
  const double u1 = polar.first;
  const double t1 = polar.second;
  if (u1 <= 4.0) {
    const double u = -2.0 * std::asin(0.25 * u1);
    const double t = NormalizeAngle(t1 + 0.5 * u + M_PI);
    const double v = NormalizeAngle(phi - t + u);
    if (t >= -kZero && u <= kZero) {
      param->flag = true;
      param->u = u;
      param->t = t;
      param->v = v;
    }
  }
}

// Additional motion not included in paper.
void SLS(double x, double y, double phi, double /*sin_phi*/, double /*cos_phi*/,
         RSPParam* param) {
  const double phi_mod = NormalizeAngle(phi);
  const double tan_phi_mod = std::tan(phi_mod);
  const double tan_half_phi_mod = std::tan(phi_mod / 2.0);
  constexpr double kEpsilon = 1e-1;
  if (y > 0.0 && phi_mod > kEpsilon && phi_mod < M_PI) {
    const double xd = -y / tan_phi_mod + x;
    const double t = xd - tan_half_phi_mod;
    const double u = phi_mod;
    const double v = std::sqrt((x - xd) * (x - xd) + y * y) - tan_half_phi_mod;
    param->flag = true;
    param->u = u;
    param->t = t;
    param->v = v;
  } else if (y < 0.0 && phi_mod > kEpsilon && phi_mod < M_PI) {
    const double xd = -y / tan_phi_mod + x;
    const double t = xd - tan_half_phi_mod;
    const double u = phi_mod;
    const double v = -std::sqrt((x - xd) * (x - xd) + y * y) - tan_half_phi_mod;
    param->flag = true;
    param->u = u;
    param->t = t;
    param->v = v;
  }
}

// Formula 8.7
void LRLRn(double x, double y, double phi, double sin_phi, double cos_phi,
           RSPParam* param) {
  const double xi = x + sin_phi;
  const double eta = y - 1.0 - cos_phi;
  const double rho = 0.25 * (2.0 + std::sqrt(xi * xi + eta * eta));
  if (rho <= 1.0) {
    const double u = std::acos(rho);
    const auto tau_omega = CalcTauOmega(u, -u, xi, eta, phi);
    if (tau_omega.first >= -kZero && tau_omega.second <= kZero) {
      param->flag = true;
      param->u = u;
      param->t = tau_omega.first;
      param->v = tau_omega.second;
    }
  }
}

// Formula 8.8
void LRLRp(double x, double y, double phi, double sin_phi, double cos_phi,
           RSPParam* param) {
  const double xi = x + sin_phi;
  const double eta = y - 1.0 - cos_phi;
  const double rho = (20.0 - xi * xi - eta * eta) / 16.0;
  if (rho <= 1.0 && rho >= 0.0) {
    const double u = -std::acos(rho);
    if (u >= -0.5 * M_PI) {
      const auto tau_omega = CalcTauOmega(u, u, xi, eta, phi);
      if (tau_omega.first >= -kZero && tau_omega.second >= -kZero) {
        param->flag = true;
        param->u = u;
        param->t = tau_omega.first;
        param->v = tau_omega.second;
      }
    }
  }
}

// Formula 8.9
void LRSL(double x, double y, double phi, double sin_phi, double cos_phi,
          RSPParam* param) {
  const double xi = x - sin_phi;
  const double eta = y - 1.0 + cos_phi;
  const auto polar = Cartesian2Polar(xi, eta);
  const double rho = polar.first;
  const double theta = polar.second;
  if (rho >= 2.0) {
    const double r = std::sqrt(rho * rho - 4.0);
    const double u = 2.0 - r;
    const double t = NormalizeAngle(theta + std::atan2(r, -2.0));
    const double v = NormalizeAngle(phi - 0.5 * M_PI - t);
    if (t >= -kZero && u <= kZero && v <= kZero) {
      param->flag = true;
      param->u = u;
      param->t = t;
      param->v = v;
    }
  }
}

// Formula 8.10
void LRSR(double x, double y, double phi, double sin_phi, double cos_phi,
          RSPParam* param) {
  const double xi = x + sin_phi;
  const double eta = y - 1.0 - cos_phi;
  const auto polar = Cartesian2Polar(-eta, xi);
  const double rho = polar.first;
  const double theta = polar.second;
  if (rho >= 2.0) {
    const double t = theta;
    const double u = 2.0 - rho;
    const double v = NormalizeAngle(t + 0.5 * M_PI - phi);
    if (t >= -kZero && u <= kZero && v <= kZero) {
      param->flag = true;
      param->u = u;
      param->t = t;
      param->v = v;
    }
  }
}

// Formula 8.11
void LRSLR(double x, double y, double phi, double sin_phi, double cos_phi,
           RSPParam* param) {
  const double xi = x + sin_phi;
  const double eta = y - 1.0 - cos_phi;
  const auto polar = Cartesian2Polar(xi, eta);
  const double rho = polar.first;
  if (rho >= 2.0) {
    const double u = 4.0 - std::sqrt(rho * rho - 4.0);
    if (u <= kZero) {
      const double t = NormalizeAngle(
          atan2((4.0 - u) * xi - 2.0 * eta, -2.0 * xi + (u - 4.0) * eta));
      const double v = NormalizeAngle(t - phi);
      if (t >= -kZero && v >= -kZero) {
        param->flag = true;
        param->u = u;
        param->t = t;
        param->v = v;
      }
    }
  }
}

bool SetRSP(int size, const double* lengths, const char* types,
            std::vector<ReedSheppPath>* all_possible_paths) {
  double total_length = 0.0;
  for (int i = 0; i < size; ++i) {
    total_length += std::abs(lengths[i]);
  }
  if (total_length == 0.0) {
    return false;
  }
  ReedSheppPath path;
  std::vector<double> length_vec(lengths, lengths + size);
  std::vector<char> type_vec(types, types + size);
  path.segs_lengths = std::move(length_vec);
  path.segs_types = std::move(type_vec);
  path.total_length = total_length;
  all_possible_paths->push_back(std::move(path));
  return true;
}

bool SCS(double x, double y, double phi, double sin_phi, double cos_phi,
         std::vector<ReedSheppPath>* all_possible_paths) {
  RSPParam sls_param;
  SLS(x, y, phi, sin_phi, cos_phi, &sls_param);
  const double sls_lengths[3] = {sls_param.t, sls_param.u, sls_param.v};
  constexpr char kSlsTypes[] = "SLS";
  if (sls_param.flag &&
      !SetRSP(3, sls_lengths, kSlsTypes, all_possible_paths)) {
    VLOG(4) << "Fail at SetRSP with SLS_param";
    return false;
  }

  RSPParam srs_param;
  SLS(x, -y, -phi, -sin_phi, cos_phi, &srs_param);
  const double srs_lengths[3] = {srs_param.t, srs_param.u, srs_param.v};
  constexpr char kSrsTypes[] = "SRS";
  if (srs_param.flag &&
      !SetRSP(3, srs_lengths, kSrsTypes, all_possible_paths)) {
    VLOG(4) << "Fail at SetRSP with SRS_param";
    return false;
  }
  return true;
}

bool CSC(double x, double y, double phi, double sin_phi, double cos_phi,
         std::vector<ReedSheppPath>* all_possible_paths) {
  RSPParam ls_l1_param;
  LSL(x, y, phi, sin_phi, cos_phi, &ls_l1_param);
  const double ls_l1_lengths[3] = {ls_l1_param.t, ls_l1_param.u, ls_l1_param.v};
  constexpr char kLsL1Types[] = "LSL";
  if (ls_l1_param.flag &&
      !SetRSP(3, ls_l1_lengths, kLsL1Types, all_possible_paths)) {
    VLOG(4) << "Fail at SetRSP with LSL_param";
    return false;
  }

  RSPParam ls_l2_param;
  LSL(-x, y, -phi, -sin_phi, cos_phi, &ls_l2_param);
  const double ls_l2_lengths[3] = {-ls_l2_param.t, -ls_l2_param.u,
                                   -ls_l2_param.v};
  constexpr char kLsL2Types[] = "LSL";
  if (ls_l2_param.flag &&
      !SetRSP(3, ls_l2_lengths, kLsL2Types, all_possible_paths)) {
    VLOG(4) << "Fail at SetRSP with LSL2_param";
    return false;
  }

  RSPParam ls_l3_param;
  LSL(x, -y, -phi, -sin_phi, cos_phi, &ls_l3_param);
  const double ls_l3_lengths[3] = {ls_l3_param.t, ls_l3_param.u, ls_l3_param.v};
  constexpr char kLsL3Types[] = "RSR";
  if (ls_l3_param.flag &&
      !SetRSP(3, ls_l3_lengths, kLsL3Types, all_possible_paths)) {
    VLOG(4) << "Fail at SetRSP with LSL3_param";
    return false;
  }

  RSPParam ls_l4_param;
  LSL(-x, -y, phi, sin_phi, cos_phi, &ls_l4_param);
  const double ls_l4_lengths[3] = {-ls_l4_param.t, -ls_l4_param.u,
                                   -ls_l4_param.v};
  constexpr char kLsL4Types[] = "RSR";
  if (ls_l4_param.flag &&
      !SetRSP(3, ls_l4_lengths, kLsL4Types, all_possible_paths)) {
    VLOG(4) << "Fail at SetRSP with LSL4_param";
    return false;
  }

  RSPParam ls_r1_param;
  LSR(x, y, phi, sin_phi, cos_phi, &ls_r1_param);
  const double ls_r1_lengths[3] = {ls_r1_param.t, ls_r1_param.u, ls_r1_param.v};
  constexpr char kLsR1Types[] = "LSR";
  if (ls_r1_param.flag &&
      !SetRSP(3, ls_r1_lengths, kLsR1Types, all_possible_paths)) {
    VLOG(4) << "Fail at SetRSP with LSR1_param";
    return false;
  }

  RSPParam ls_r2_param;
  LSR(-x, y, -phi, -sin_phi, cos_phi, &ls_r2_param);
  const double ls_r2_lengths[3] = {-ls_r2_param.t, -ls_r2_param.u,
                                   -ls_r2_param.v};
  constexpr char kLsR2Types[] = "LSR";
  if (ls_r2_param.flag &&
      !SetRSP(3, ls_r2_lengths, kLsR2Types, all_possible_paths)) {
    VLOG(4) << "Fail at SetRSP with LSR2_param";
    return false;
  }

  RSPParam ls_r3_param;
  LSR(x, -y, -phi, -sin_phi, cos_phi, &ls_r3_param);
  const double ls_r3_lengths[3] = {ls_r3_param.t, ls_r3_param.u, ls_r3_param.v};
  constexpr char kLsR3Types[] = "RSL";
  if (ls_r3_param.flag &&
      !SetRSP(3, ls_r3_lengths, kLsR3Types, all_possible_paths)) {
    VLOG(4) << "Fail at SetRSP with LSR3_param";
    return false;
  }

  RSPParam ls_r4_param;
  LSR(-x, -y, phi, sin_phi, cos_phi, &ls_r4_param);
  const double ls_r4_lengths[3] = {-ls_r4_param.t, -ls_r4_param.u,
                                   -ls_r4_param.v};
  constexpr char kLsR4Types[] = "RSL";
  if (ls_r4_param.flag &&
      !SetRSP(3, ls_r4_lengths, kLsR4Types, all_possible_paths)) {
    VLOG(4) << "Fail at SetRSP with LSR4_param";
    return false;
  }
  return true;
}

bool CCC(double x, double y, double phi, double sin_phi, double cos_phi,
         std::vector<ReedSheppPath>* all_possible_paths) {
  RSPParam lr_l1_param;
  LRL(x, y, phi, sin_phi, cos_phi, &lr_l1_param);
  const double lr_l1_lengths[3] = {lr_l1_param.t, lr_l1_param.u, lr_l1_param.v};
  constexpr char kLrL1Types[] = "LRL";
  if (lr_l1_param.flag &&
      !SetRSP(3, lr_l1_lengths, kLrL1Types, all_possible_paths)) {
    VLOG(4) << "Fail at SetRSP with LRL_param";
    return false;
  }

  RSPParam lr_l2_param;
  LRL(-x, y, -phi, -sin_phi, cos_phi, &lr_l2_param);
  const double lr_l2_lengths[3] = {-lr_l2_param.t, -lr_l2_param.u,
                                   -lr_l2_param.v};
  constexpr char kLrL2Types[] = "LRL";
  if (lr_l2_param.flag &&
      !SetRSP(3, lr_l2_lengths, kLrL2Types, all_possible_paths)) {
    VLOG(4) << "Fail at SetRSP with LRL2_param";
    return false;
  }

  RSPParam lr_l3_param;
  LRL(x, -y, -phi, -sin_phi, cos_phi, &lr_l3_param);
  const double lr_l3_lengths[3] = {lr_l3_param.t, lr_l3_param.u, lr_l3_param.v};
  constexpr char kLrL3Types[] = "RLR";
  if (lr_l3_param.flag &&
      !SetRSP(3, lr_l3_lengths, kLrL3Types, all_possible_paths)) {
    VLOG(4) << "Fail at SetRSP with LRL3_param";
    return false;
  }

  RSPParam lr_l4_param;
  LRL(-x, -y, phi, sin_phi, cos_phi, &lr_l4_param);
  const double lr_l4_lengths[3] = {-lr_l4_param.t, -lr_l4_param.u,
                                   -lr_l4_param.v};
  constexpr char kLrL4Types[] = "RLR";
  if (lr_l4_param.flag &&
      !SetRSP(3, lr_l4_lengths, kLrL4Types, all_possible_paths)) {
    VLOG(4) << "Fail at SetRSP with LRL4_param";
    return false;
  }

  // backward
  const double xb = x * cos_phi + y * sin_phi;
  const double yb = x * sin_phi - y * cos_phi;

  RSPParam lr_l5_param;
  LRL(xb, yb, phi, sin_phi, cos_phi, &lr_l5_param);
  const double lr_l5_lengths[3] = {lr_l5_param.v, lr_l5_param.u, lr_l5_param.t};
  constexpr char kLrL5Types[] = "LRL";
  if (lr_l5_param.flag &&
      !SetRSP(3, lr_l5_lengths, kLrL5Types, all_possible_paths)) {
    VLOG(4) << "Fail at SetRSP with LRL5_param";
    return false;
  }

  RSPParam lr_l6_param;
  LRL(-xb, yb, -phi, -sin_phi, cos_phi, &lr_l6_param);
  const double lr_l6_lengths[3] = {-lr_l6_param.v, -lr_l6_param.u,
                                   -lr_l6_param.t};
  constexpr char kLrL6Types[] = "LRL";
  if (lr_l6_param.flag &&
      !SetRSP(3, lr_l6_lengths, kLrL6Types, all_possible_paths)) {
    VLOG(4) << "Fail at SetRSP with LRL6_param";
    return false;
  }

  RSPParam lr_l7_param;
  LRL(xb, -yb, -phi, -sin_phi, cos_phi, &lr_l7_param);
  const double lr_l7_lengths[3] = {lr_l7_param.v, lr_l7_param.u, lr_l7_param.t};
  constexpr char kLrL7Types[] = "RLR";
  if (lr_l7_param.flag &&
      !SetRSP(3, lr_l7_lengths, kLrL7Types, all_possible_paths)) {
    VLOG(4) << "Fail at SetRSP with LRL7_param";
    return false;
  }

  RSPParam lr_l8_param;
  LRL(-xb, -yb, phi, sin_phi, cos_phi, &lr_l8_param);
  const double lr_l8_lengths[3] = {-lr_l8_param.v, -lr_l8_param.u,
                                   -lr_l8_param.t};
  constexpr char kLrL8Types[] = "RLR";
  if (lr_l8_param.flag &&
      !SetRSP(3, lr_l8_lengths, kLrL8Types, all_possible_paths)) {
    VLOG(4) << "Fail at SetRSP with LRL8_param";
    return false;
  }
  return true;
}

bool CCCC(double x, double y, double phi, double sin_phi, double cos_phi,
          std::vector<ReedSheppPath>* all_possible_paths) {
  RSPParam lrl_rn1_param;
  LRLRn(x, y, phi, sin_phi, cos_phi, &lrl_rn1_param);
  const double lrl_rn1_lengths[4] = {lrl_rn1_param.t, lrl_rn1_param.u,
                                     -lrl_rn1_param.u, lrl_rn1_param.v};
  constexpr char kLrlRn1Types[] = "LRLR";
  if (lrl_rn1_param.flag &&
      !SetRSP(4, lrl_rn1_lengths, kLrlRn1Types, all_possible_paths)) {
    VLOG(4) << "Fail at SetRSP with LRLRn_param";
    return false;
  }

  RSPParam lrl_rn2_param;
  LRLRn(-x, y, -phi, -sin_phi, cos_phi, &lrl_rn2_param);
  const double lrl_rn2_lengths[4] = {-lrl_rn2_param.t, -lrl_rn2_param.u,
                                     lrl_rn2_param.u, -lrl_rn2_param.v};
  constexpr char kLrlRn2Types[] = "LRLR";
  if (lrl_rn2_param.flag &&
      !SetRSP(4, lrl_rn2_lengths, kLrlRn2Types, all_possible_paths)) {
    VLOG(4) << "Fail at SetRSP with LRLRn2_param";
    return false;
  }

  RSPParam lrl_rn3_param;
  LRLRn(x, -y, -phi, -sin_phi, cos_phi, &lrl_rn3_param);
  const double lrl_rn3_lengths[4] = {lrl_rn3_param.t, lrl_rn3_param.u,
                                     -lrl_rn3_param.u, lrl_rn3_param.v};
  constexpr char kLrlRn3Types[] = "RLRL";
  if (lrl_rn3_param.flag &&
      !SetRSP(4, lrl_rn3_lengths, kLrlRn3Types, all_possible_paths)) {
    VLOG(4) << "Fail at SetRSP with LRLRn3_param";
    return false;
  }

  RSPParam lrl_rn4_param;
  LRLRn(-x, -y, phi, sin_phi, cos_phi, &lrl_rn4_param);
  const double lrl_rn4_lengths[4] = {-lrl_rn4_param.t, -lrl_rn4_param.u,
                                     lrl_rn4_param.u, -lrl_rn4_param.v};
  constexpr char kLrlRn4Types[] = "RLRL";
  if (lrl_rn4_param.flag &&
      !SetRSP(4, lrl_rn4_lengths, kLrlRn4Types, all_possible_paths)) {
    VLOG(4) << "Fail at SetRSP with LRLRn4_param";
    return false;
  }

  RSPParam lrl_rp1_param;
  LRLRp(x, y, phi, sin_phi, cos_phi, &lrl_rp1_param);
  const double lrl_rp1_lengths[4] = {lrl_rp1_param.t, lrl_rp1_param.u,
                                     lrl_rp1_param.u, lrl_rp1_param.v};
  constexpr char kLrlRp1Types[] = "LRLR";
  if (lrl_rp1_param.flag &&
      !SetRSP(4, lrl_rp1_lengths, kLrlRp1Types, all_possible_paths)) {
    VLOG(4) << "Fail at SetRSP with LRLRp1_param";
    return false;
  }

  RSPParam lrl_rp2_param;
  LRLRp(-x, y, -phi, -sin_phi, cos_phi, &lrl_rp2_param);
  const double lrl_rp2_lengths[4] = {-lrl_rp2_param.t, -lrl_rp2_param.u,
                                     -lrl_rp2_param.u, -lrl_rp2_param.v};
  constexpr char kLrlRp2Types[] = "LRLR";
  if (lrl_rp2_param.flag &&
      !SetRSP(4, lrl_rp2_lengths, kLrlRp2Types, all_possible_paths)) {
    VLOG(4) << "Fail at SetRSP with LRLRp2_param";
    return false;
  }

  RSPParam lrl_rp3_param;
  LRLRp(x, -y, -phi, -sin_phi, cos_phi, &lrl_rp3_param);
  const double lrl_rp3_lengths[4] = {lrl_rp3_param.t, lrl_rp3_param.u,
                                     lrl_rp3_param.u, lrl_rp3_param.v};
  constexpr char kLrlRp3Types[] = "RLRL";
  if (lrl_rp3_param.flag &&
      !SetRSP(4, lrl_rp3_lengths, kLrlRp3Types, all_possible_paths)) {
    VLOG(4) << "Fail at SetRSP with LRLRp3_param";
    return false;
  }

  RSPParam lrl_rp4_param;
  LRLRp(-x, -y, phi, sin_phi, cos_phi, &lrl_rp4_param);
  const double lrl_rp4_lengths[4] = {-lrl_rp4_param.t, -lrl_rp4_param.u,
                                     -lrl_rp4_param.u, -lrl_rp4_param.v};
  constexpr char kLrlRp4Types[] = "RLRL";
  if (lrl_rp4_param.flag &&
      !SetRSP(4, lrl_rp4_lengths, kLrlRp4Types, all_possible_paths)) {
    VLOG(4) << "Fail at SetRSP with LRLRp4_param";
    return false;
  }
  return true;
}

bool CCSC(double x, double y, double phi, double sin_phi, double cos_phi,
          std::vector<ReedSheppPath>* all_possible_paths) {
  RSPParam lrs_l1_param;
  LRSL(x, y, phi, sin_phi, cos_phi, &lrs_l1_param);
  const double lrs_l1_lengths[4] = {lrs_l1_param.t, -0.5 * M_PI, lrs_l1_param.u,
                                    lrs_l1_param.v};
  constexpr char kLrsL1Types[] = "LRSL";
  if (lrs_l1_param.flag &&
      !SetRSP(4, lrs_l1_lengths, kLrsL1Types, all_possible_paths)) {
    VLOG(4) << "Fail at SetRSP with LRSL1_param";
    return false;
  }

  RSPParam lrs_l2_param;
  LRSL(-x, y, -phi, -sin_phi, cos_phi, &lrs_l2_param);
  const double lrs_l2_lengths[4] = {-lrs_l2_param.t, 0.5 * M_PI,
                                    -lrs_l2_param.u, -lrs_l2_param.v};
  constexpr char kLrsL2Types[] = "LRSL";
  if (lrs_l2_param.flag &&
      !SetRSP(4, lrs_l2_lengths, kLrsL2Types, all_possible_paths)) {
    VLOG(4) << "Fail at SetRSP with LRSL2_param";
    return false;
  }

  RSPParam lrs_l3_param;
  LRSL(x, -y, -phi, -sin_phi, cos_phi, &lrs_l3_param);
  const double lrs_l3_lengths[4] = {lrs_l3_param.t, -0.5 * M_PI, lrs_l3_param.u,
                                    lrs_l3_param.v};
  constexpr char kLrsL3Types[] = "RLSR";
  if (lrs_l3_param.flag &&
      !SetRSP(4, lrs_l3_lengths, kLrsL3Types, all_possible_paths)) {
    VLOG(4) << "Fail at SetRSP with LRSL3_param";
    return false;
  }

  RSPParam lrs_l4_param;
  LRSL(-x, -y, phi, sin_phi, cos_phi, &lrs_l4_param);
  const double lrs_l4_lengths[4] = {-lrs_l4_param.t, 0.5 * M_PI,
                                    -lrs_l4_param.u, -lrs_l4_param.v};
  constexpr char kLrsL4Types[] = "RLSR";
  if (lrs_l4_param.flag &&
      !SetRSP(4, lrs_l4_lengths, kLrsL4Types, all_possible_paths)) {
    VLOG(4) << "Fail at SetRSP with LRSL4_param";
    return false;
  }

  RSPParam lrs_r1_param;
  LRSR(x, y, phi, sin_phi, cos_phi, &lrs_r1_param);
  const double lrs_r1_lengths[4] = {lrs_r1_param.t, -0.5 * M_PI, lrs_r1_param.u,
                                    lrs_r1_param.v};
  constexpr char kLrsR1Types[] = "LRSR";
  if (lrs_r1_param.flag &&
      !SetRSP(4, lrs_r1_lengths, kLrsR1Types, all_possible_paths)) {
    VLOG(4) << "Fail at SetRSP with LRSR1_param";
    return false;
  }

  RSPParam lrs_r2_param;
  LRSR(-x, y, -phi, -sin_phi, cos_phi, &lrs_r2_param);
  const double lrs_r2_lengths[4] = {-lrs_r2_param.t, 0.5 * M_PI,
                                    -lrs_r2_param.u, -lrs_r2_param.v};
  constexpr char kLrsR2Types[] = "LRSR";
  if (lrs_r2_param.flag &&
      !SetRSP(4, lrs_r2_lengths, kLrsR2Types, all_possible_paths)) {
    VLOG(4) << "Fail at SetRSP with LRSR2_param";
    return false;
  }

  RSPParam lrs_r3_param;
  LRSR(x, -y, -phi, -sin_phi, cos_phi, &lrs_r3_param);
  const double lrs_r3_lengths[4] = {lrs_r3_param.t, -0.5 * M_PI, lrs_r3_param.u,
                                    lrs_r3_param.v};
  constexpr char kLrsR3Types[] = "RLSL";
  if (lrs_r3_param.flag &&
      !SetRSP(4, lrs_r3_lengths, kLrsR3Types, all_possible_paths)) {
    VLOG(4) << "Fail at SetRSP with LRSR3_param";
    return false;
  }

  RSPParam lrs_r4_param;
  LRSR(-x, -y, phi, sin_phi, cos_phi, &lrs_r4_param);
  const double lrs_r4_lengths[4] = {-lrs_r4_param.t, 0.5 * M_PI,
                                    -lrs_r4_param.u, -lrs_r4_param.v};
  constexpr char kLrsR4Types[] = "RLSL";
  if (lrs_r4_param.flag &&
      !SetRSP(4, lrs_r4_lengths, kLrsR4Types, all_possible_paths)) {
    VLOG(4) << "Fail at SetRSP with LRSR4_param";
    return false;
  }

  // backward
  const double xb = x * cos_phi + y * sin_phi;
  const double yb = x * sin_phi - y * cos_phi;

  RSPParam lrs_l5_param;
  LRSL(xb, yb, phi, sin_phi, cos_phi, &lrs_l5_param);
  const double lrs_l5_lengths[4] = {lrs_l5_param.v, lrs_l5_param.u, -0.5 * M_PI,
                                    lrs_l5_param.t};
  constexpr char kLrsL5Types[] = "LSRL";
  if (lrs_l5_param.flag &&
      !SetRSP(4, lrs_l5_lengths, kLrsL5Types, all_possible_paths)) {
    VLOG(4) << "Fail at SetRSP with LRLRn_param";
    return false;
  }

  RSPParam lrs_l6_param;
  LRSL(-xb, yb, -phi, -sin_phi, cos_phi, &lrs_l6_param);
  const double lrs_l6_lengths[4] = {-lrs_l6_param.v, -lrs_l6_param.u,
                                    0.5 * M_PI, -lrs_l6_param.t};
  constexpr char kLrsL6Types[] = "LSRL";
  if (lrs_l6_param.flag &&
      !SetRSP(4, lrs_l6_lengths, kLrsL6Types, all_possible_paths)) {
    VLOG(4) << "Fail at SetRSP with LRSL6_param";
    return false;
  }

  RSPParam lrs_l7_param;
  LRSL(xb, -yb, -phi, -sin_phi, cos_phi, &lrs_l7_param);
  const double lrs_l7_lengths[4] = {lrs_l7_param.v, lrs_l7_param.u, -0.5 * M_PI,
                                    lrs_l7_param.t};
  constexpr char kLrsL7Types[] = "RSLR";
  if (lrs_l7_param.flag &&
      !SetRSP(4, lrs_l7_lengths, kLrsL7Types, all_possible_paths)) {
    VLOG(4) << "Fail at SetRSP with LRSL7_param";
    return false;
  }

  RSPParam lrs_l8_param;
  LRSL(-xb, -yb, phi, sin_phi, cos_phi, &lrs_l8_param);
  const double lrs_l8_lengths[4] = {-lrs_l8_param.v, -lrs_l8_param.u,
                                    0.5 * M_PI, -lrs_l8_param.t};
  constexpr char kLrsL8Types[] = "RSLR";
  if (lrs_l8_param.flag &&
      !SetRSP(4, lrs_l8_lengths, kLrsL8Types, all_possible_paths)) {
    VLOG(4) << "Fail at SetRSP with LRSL8_param";
    return false;
  }

  RSPParam lrs_r5_param;
  LRSR(xb, yb, phi, sin_phi, cos_phi, &lrs_r5_param);
  const double lrs_r5_lengths[4] = {lrs_r5_param.v, lrs_r5_param.u, -0.5 * M_PI,
                                    lrs_r5_param.t};
  constexpr char kLrsR5Types[] = "RSRL";
  if (lrs_r5_param.flag &&
      !SetRSP(4, lrs_r5_lengths, kLrsR5Types, all_possible_paths)) {
    VLOG(4) << "Fail at SetRSP with LRSR5_param";
    return false;
  }

  RSPParam lrs_r6_param;
  LRSR(-xb, yb, -phi, -sin_phi, cos_phi, &lrs_r6_param);
  const double lrs_r6_lengths[4] = {-lrs_r6_param.v, -lrs_r6_param.u,
                                    0.5 * M_PI, -lrs_r6_param.t};
  constexpr char kLrsR6Types[] = "RSRL";
  if (lrs_r6_param.flag &&
      !SetRSP(4, lrs_r6_lengths, kLrsR6Types, all_possible_paths)) {
    VLOG(4) << "Fail at SetRSP with LRSR6_param";
    return false;
  }

  RSPParam lrs_r7_param;
  LRSR(xb, -yb, -phi, -sin_phi, cos_phi, &lrs_r7_param);
  const double lrs_r7_lengths[4] = {lrs_r7_param.v, lrs_r7_param.u, -0.5 * M_PI,
                                    lrs_r7_param.t};
  constexpr char kLrsR7Types[] = "LSLR";
  if (lrs_r7_param.flag &&
      !SetRSP(4, lrs_r7_lengths, kLrsR7Types, all_possible_paths)) {
    VLOG(4) << "Fail at SetRSP with LRSR7_param";
    return false;
  }

  RSPParam lrs_r8_param;
  LRSR(-xb, -yb, phi, sin_phi, cos_phi, &lrs_r8_param);
  const double lrs_r8_lengths[4] = {-lrs_r8_param.v, -lrs_r8_param.u,
                                    0.5 * M_PI, -lrs_r8_param.t};
  constexpr char kLrsR8Types[] = "LSLR";
  if (lrs_r8_param.flag &&
      !SetRSP(4, lrs_r8_lengths, kLrsR8Types, all_possible_paths)) {
    VLOG(4) << "Fail at SetRSP with LRSR8_param";
    return false;
  }
  return true;
}

bool CCSCC(double x, double y, double phi, double sin_phi, double cos_phi,
           std::vector<ReedSheppPath>* all_possible_paths) {
  RSPParam lrsl_r1_param;
  LRSLR(x, y, phi, sin_phi, cos_phi, &lrsl_r1_param);
  const double lrsl_r1_lengths[5] = {lrsl_r1_param.t, -0.5 * M_PI,
                                     lrsl_r1_param.u, -0.5 * M_PI,
                                     lrsl_r1_param.v};
  constexpr char kLrslR1Types[] = "LRSLR";
  if (lrsl_r1_param.flag &&
      !SetRSP(5, lrsl_r1_lengths, kLrslR1Types, all_possible_paths)) {
    VLOG(4) << "Fail at SetRSP with LRSLR1_param";
    return false;
  }

  RSPParam lrsl_r2_param;
  LRSLR(-x, y, -phi, -sin_phi, cos_phi, &lrsl_r2_param);
  const double lrsl_r2_lengths[5] = {-lrsl_r2_param.t, 0.5 * M_PI,
                                     -lrsl_r2_param.u, 0.5 * M_PI,
                                     -lrsl_r2_param.v};
  constexpr char kLrslR2Types[] = "LRSLR";
  if (lrsl_r2_param.flag &&
      !SetRSP(5, lrsl_r2_lengths, kLrslR2Types, all_possible_paths)) {
    VLOG(4) << "Fail at SetRSP with LRSLR2_param";
    return false;
  }

  RSPParam lrsl_r3_param;
  LRSLR(x, -y, -phi, -sin_phi, cos_phi, &lrsl_r3_param);
  const double lrsl_r3_lengths[5] = {lrsl_r3_param.t, -0.5 * M_PI,
                                     lrsl_r3_param.u, -0.5 * M_PI,
                                     lrsl_r3_param.v};
  constexpr char kLrslR3Types[] = "RLSRL";
  if (lrsl_r3_param.flag &&
      !SetRSP(5, lrsl_r3_lengths, kLrslR3Types, all_possible_paths)) {
    VLOG(4) << "Fail at SetRSP with LRSLR3_param";
    return false;
  }

  RSPParam lrsl_r4_param;
  LRSLR(-x, -y, phi, sin_phi, cos_phi, &lrsl_r4_param);
  const double lrsl_r4_lengths[5] = {-lrsl_r4_param.t, 0.5 * M_PI,
                                     -lrsl_r4_param.u, 0.5 * M_PI,
                                     -lrsl_r4_param.v};
  constexpr char kLrslR4Types[] = "RLSRL";
  if (lrsl_r4_param.flag &&
      !SetRSP(5, lrsl_r4_lengths, kLrslR4Types, all_possible_paths)) {
    VLOG(4) << "Fail at SetRSP with LRSLR4_param";
    return false;
  }
  return true;
}

bool GenerateRSP(const Node3d& start_node, const Node3d& end_node,
                 double max_kappa,
                 std::vector<ReedSheppPath>* all_possible_paths) {
  const double dx = end_node.x() - start_node.x();
  const double dy = end_node.y() - start_node.y();
  const double dphi = end_node.theta() - start_node.theta();
  double dphi_cos_sin[2];
  fast_math::CosAndSin<7>(dphi, dphi_cos_sin);
  const double sin_dphi = dphi_cos_sin[1];
  const double cos_dphi = dphi_cos_sin[0];
  const double c = start_node.cos_theta();
  const double s = start_node.sin_theta();
  // Normalize the initial point to (0,0,0).
  const double x = (c * dx + s * dy) * max_kappa;
  const double y = (-s * dx + c * dy) * max_kappa;
  if (!SCS(x, y, dphi, sin_dphi, cos_dphi, all_possible_paths)) {
    VLOG(3) << "Fail at SCS";
  }
  if (!CSC(x, y, dphi, sin_dphi, cos_dphi, all_possible_paths)) {
    VLOG(3) << "Fail at CSC";
  }
  if (!CCC(x, y, dphi, sin_dphi, cos_dphi, all_possible_paths)) {
    VLOG(3) << "Fail at CCC";
  }
  if (!CCCC(x, y, dphi, sin_dphi, cos_dphi, all_possible_paths)) {
    VLOG(3) << "Fail at CCCC";
  }
  if (!CCSC(x, y, dphi, sin_dphi, cos_dphi, all_possible_paths)) {
    VLOG(3) << "Fail at CCSC";
  }
  if (!CCSCC(x, y, dphi, sin_dphi, cos_dphi, all_possible_paths)) {
    VLOG(3) << "Fail at CCSCC";
  }
  if (all_possible_paths->empty()) {
    VLOG(3) << "No path generated by certain two configurations";
    return false;
  }
  return true;
}

absl::StatusOr<ReedSheppPath> GetShortestReedsShepp(const Node3d& start_node,
                                                    const Node3d& end_node,
                                                    double max_kappa) {
  QCHECK_GT(max_kappa, 0.0);
  std::vector<ReedSheppPath> all_possible_paths;
  if (!GenerateRSP(start_node, end_node, max_kappa, &all_possible_paths)) {
    return absl::UnavailableError("Fail to generate Reeds Shepp curve!");
  }

  double optimal_path_length = std::numeric_limits<double>::infinity();
  int optimal_path_index = 0;
  for (int i = 0; i < all_possible_paths.size(); ++i) {
    if (all_possible_paths[i].total_length > 0.0 &&
        all_possible_paths[i].total_length < optimal_path_length) {
      optimal_path_index = i;
      optimal_path_length = all_possible_paths[i].total_length;
    }
  }

  ReedSheppPath res(std::move(all_possible_paths[optimal_path_index]));
  const double scale = 1.0 / max_kappa;
  for (double& segs_length : res.segs_lengths) {
    segs_length *= scale;
  }
  res.total_length *= scale;

  // The answer maybe incorrect and need to check the end point.
  return res;
}

struct Motion {
  double delta_x;
  double delta_y;
  double delta_theta;
  double kappa;
  double s;
  bool forward;
};

std::vector<ReedSheppPoint> GetSampledShortestReedsShepp(
    const Node3d& start_node, const Node3d& end_node, double max_kappa,
    double resolution) {
  QCHECK_GT(resolution, 0.0);
  std::vector<ReedSheppPoint> res;
  const auto rs_path = GetShortestReedsShepp(start_node, end_node, max_kappa);
  if (!rs_path.ok()) return res;

  const auto compute_motion = [&](double length, char type) -> Motion {
    const bool forward = (length > 0.0);
    const double sign = (forward ? 1.0 : -1.0);
    switch (type) {
      case 'L': {
        const double delta_theta = max_kappa * length;
        double delta_theta_cos_sin[2];
        fast_math::CosAndSin<7>(delta_theta * 0.5, delta_theta_cos_sin);
        const double sin_theta = delta_theta_cos_sin[1];
        const double cos_theta = delta_theta_cos_sin[0];
        const double delta_x = 2.0 * sin_theta * cos_theta / max_kappa;
        const double delta_y = 2.0 * sin_theta * sin_theta / max_kappa;
        return {delta_x,          delta_y,          delta_theta,
                sign * max_kappa, std::abs(length), forward};
      }
      case 'R': {
        const double delta_theta = -max_kappa * length;
        double delta_theta_cos_sin[2];
        fast_math::CosAndSin<7>(delta_theta * 0.5, delta_theta_cos_sin);
        const double sin_theta = delta_theta_cos_sin[1];
        const double cos_theta = delta_theta_cos_sin[0];
        const double delta_x = 2.0 * sin_theta * cos_theta / max_kappa;
        const double delta_y = 2.0 * sin_theta * sin_theta / max_kappa;
        return {-delta_x,         -delta_y,         delta_theta,
                sign * max_kappa, std::abs(length), forward};
      }
      case 'S':
        return {sign * std::abs(length), 0.0,    0.0, 0.0,
                std::abs(length),        forward};
    }
    return {0.0, 0.0, 0.0, 0.0, 0.0, false};
  };

  QCHECK_EQ(rs_path->segs_lengths.size(), rs_path->segs_types.size());
  // Record the last node.
  ReedSheppPoint last_point = {start_node.x(), start_node.y(),
                               start_node.theta(), start_node.forward()};
  for (int i = 0; i < rs_path->segs_lengths.size(); ++i) {
    const double length = rs_path->segs_lengths[i];
    const char type = rs_path->segs_types[i];
    Motion motion = compute_motion(std::copysign(resolution, length), type);
    double s = 0.0;
    constexpr double kEpsilon = 0.01;
    while (s + kEpsilon < std::abs(length)) {
      double s_increment = resolution;
      if (s + s_increment >= std::abs(length)) {
        s_increment = std::abs(length) - s;
        motion = compute_motion(std::copysign(s_increment, length), type);
      }
      s += s_increment;
      double theta_cos_sin[2];
      fast_math::CosAndSin<7>(last_point.theta, theta_cos_sin);
      const double next_x = last_point.x + motion.delta_x * theta_cos_sin[0] -
                            motion.delta_y * theta_cos_sin[1];
      const double next_y = last_point.y + motion.delta_x * theta_cos_sin[1] +
                            motion.delta_y * theta_cos_sin[0];
      const double next_theta =
          NormalizeAngle(last_point.theta + motion.delta_theta);
      res.push_back({next_x, next_y, next_theta, motion.forward});
      last_point = res.back();
    }
  }
  // Check if the last point is close to end.
  constexpr double kMaxXYError = 0.05;
  constexpr double kMaxThetaError = 0.01;
  const double xy_error =
      Hypot(res.back().x - end_node.x(), res.back().y - end_node.y());
  const double theta_error =
      std::abs(NormalizeAngle(res.back().theta - end_node.theta()));
  if (xy_error > kMaxXYError || theta_error > kMaxThetaError) {
    res.clear();
    VLOG(2) << "RS extension has a big error, xy error = " << xy_error
            << ", theta error = " << theta_error;
  }
  return res;
}

}  // namespace planner
}  // namespace qcraft

#include "onboard/planner/freespace/ipopt_global_path_smoother_model.h"

#include <math.h>
#include <stdlib.h>

#include <algorithm>
#include <limits>
#include <utility>

#include "adolc/adouble.h"
#include "adolc/drivers/drivers.h"
#include "adolc/param.h"
#include "adolc/sparse/sparsedrivers.h"
#include "adolc/taping.h"

#include "onboard/global/buffered_logger.h"
#include "onboard/lite/check.h"
#include "onboard/math/util.h"

namespace qcraft {
namespace planner {
namespace {

/** All calculations involving active variables that occur between the void
 *  function calls are recorded on a sequential data set called tape. The
 *  nonnegative integer argument tag identifies the particular tape for
 *  subsequent function or derivative evaluations. */
constexpr int kTagF = 1;
constexpr int kTagG = 2;
constexpr int kTagL = 3;

constexpr double kInf = std::numeric_limits<double>::infinity();
constexpr double kMaxTime = 50.0;

}  // namespace

IpoptGlobalPathSmootherModel::IpoptGlobalPathSmootherModel(
    ModelInput model_input)
    : input_(std::move(model_input)) {
  ne_ = input_.init_path.size();
  QCHECK_GE(ne_, 2);
  x_start_i_ = 1;
  y_start_i_ = ne_ + 1;
  theta_start_i_ = 2 * ne_ + 1;
  v_start_i_ = 3 * ne_ + 1;
  kappa_start_i_ = 4 * ne_ + 1;
  start_point_ = input_.init_path.front();
  end_point_ = input_.init_path.back();
}

bool IpoptGlobalPathSmootherModel::get_nlp_info(int& n, int& m, int& nnz_jac_g,
                                                int& nnz_h_lag,
                                                IndexStyleEnum& index_style) {
  n = 5 * ne_ + 1;  // Num of optimization variables.
  m = 3 * ne_ - 2;  // Num of constraint.
  generate_tapes(n, m, &nnz_jac_g, &nnz_h_lag);
  // use the C style indexing (0-based)
  index_style = C_STYLE;
  return true;
}

bool IpoptGlobalPathSmootherModel::get_bounds_info(int /*n*/, double* x_l,
                                                   double* x_u, int m,
                                                   double* g_l, double* g_u) {
  // Set the bounds for the variables.
  x_l[0] = 0.1;
  x_u[0] = kMaxTime;
  for (int i = x_start_i_; i < v_start_i_; ++i) {
    x_l[i] = -kInf;
    x_u[i] = kInf;
  }
  for (int i = v_start_i_; i < kappa_start_i_; ++i) {
    x_l[i] = input_.v_lower;
    x_u[i] = input_.v_upper;
  }
  for (int i = kappa_start_i_; i <= 5 * ne_; ++i) {
    x_l[i] = -input_.max_kappa;
    x_u[i] = input_.max_kappa;
  }

  // Set initial condition constraints for the variables.
  x_l[x_start_i_] = start_point_.x();
  x_u[x_start_i_] = start_point_.x();
  x_l[y_start_i_ - 1] = end_point_.x();
  x_u[y_start_i_ - 1] = end_point_.x();

  x_l[y_start_i_] = start_point_.y();
  x_u[y_start_i_] = start_point_.y();
  x_l[theta_start_i_ - 1] = end_point_.y();
  x_u[theta_start_i_ - 1] = end_point_.y();

  x_l[theta_start_i_] = start_point_.theta();
  x_u[theta_start_i_] = start_point_.theta();
  x_l[v_start_i_ - 1] = end_point_.theta();
  x_u[v_start_i_ - 1] = end_point_.theta();

  x_l[theta_start_i_] = start_point_.theta();
  x_l[v_start_i_] = 0.0;
  x_u[v_start_i_] = 0.0;
  x_l[kappa_start_i_ - 1] = 0.0;
  x_u[kappa_start_i_ - 1] = 0.0;

  x_l[kappa_start_i_] = 0.0;
  x_u[kappa_start_i_] = 0.0;
  x_l[5 * ne_] = 0.0;
  x_u[5 * ne_] = 0.0;

  // Set the bounds for the constraints.
  for (int i = 0; i < m; ++i) {
    g_l[i] = 0.0;
    g_u[i] = 0.0;
  }

  return true;
}

bool IpoptGlobalPathSmootherModel::get_starting_point(
    int /*n*/, bool /*init_x*/, double* x, bool /*init_z*/, double* /*z_l*/,
    double* /*z_u*/, int /*m*/, bool /*init_lambda*/, double* /*lambda*/) {
  x[0] = kMaxTime;
  for (int i = 0; i < ne_; ++i) {
    x[x_start_i_ + i] = input_.init_path[i].x();
    x[y_start_i_ + i] = input_.init_path[i].y();
    x[theta_start_i_ + i] = input_.init_path[i].theta();
    x[v_start_i_ + i] = input_.init_v[i];
    x[kappa_start_i_ + i] = input_.init_kappa[i];
  }
  return true;
}

//*************************************************************************
//
//
//         Nothing has to be changed below this point !!
//
//
//*************************************************************************
bool IpoptGlobalPathSmootherModel::eval_f(int n, const double* x,
                                          bool /*new_x*/, double& obj_value) {
  eval_obj(n, x, &obj_value);
  return true;
}

bool IpoptGlobalPathSmootherModel::eval_grad_f(int n, const double* x,
                                               bool /*new_x*/, double* grad_f) {
  gradient(kTagF, n, x, grad_f);
  return true;
}

bool IpoptGlobalPathSmootherModel::eval_g(int n, const double* x,
                                          bool /*new_x*/, int m, double* g) {
  eval_constraints(n, x, m, g);
  return true;
}

bool IpoptGlobalPathSmootherModel::eval_jac_g(int n, const double* x,
                                              bool /*new_x*/, int m,
                                              int /*nele_jac*/, int* i_row,
                                              int* j_col, double* values) {
  if (values == nullptr) {
    for (int idx = 0; idx < nnz_jac_; ++idx) {
      i_row[idx] = rind_g_[idx];
      j_col[idx] = cind_g_[idx];
    }
  } else {
    sparse_jac(kTagG, m, n, 1, x, &nnz_jac_, &rind_g_, &cind_g_, &jacval_,
               options_g_);
    for (int idx = 0; idx < nnz_jac_; ++idx) {
      values[idx] = jacval_[idx];
    }
  }
  return true;
}

bool IpoptGlobalPathSmootherModel::eval_h(int n, const double* x,
                                          bool /*new_x*/, double obj_factor,
                                          int m, const double* lambda,
                                          bool /*new_lambda*/,
                                          int /*nele_hess*/, int* i_row,
                                          int* j_col, double* values) {
  if (values == nullptr) {
    for (int idx = 0; idx < nnz_l_; ++idx) {
      i_row[idx] = rind_l_[idx];
      j_col[idx] = cind_l_[idx];
    }
  } else {
    obj_lam_[0] = obj_factor;
    for (int idx = 0; idx < m; ++idx) {
      obj_lam_[1 + idx] = lambda[idx];
    }
    set_param_vec(kTagL, m + 1, &obj_lam_[0]);
    sparse_hess(kTagL, n, 1, const_cast<double*>(x), &nnz_l_, &rind_l_,
                &cind_l_, &hessval_, options_l_);

    for (int idx = 0; idx < nnz_l_; ++idx) {
      values[idx] = hessval_[idx];
    }
  }
  return true;
}

void IpoptGlobalPathSmootherModel::finalize_solution(
    Ipopt::SolverReturn /*status*/, int n, const double* x,
    const double* /*z_l*/, const double* /*z_u*/, int /*m*/,
    const double* /*g*/, const double* /*lambda*/, double /*obj_value*/,
    const Ipopt::IpoptData* /*ip_data*/,
    Ipopt::IpoptCalculatedQuantities* /*ip_cq*/) {
  result_.clear();
  result_.reserve(n);
  for (int i = 0; i < n; ++i) {
    result_.push_back(x[i]);
  }
  free(rind_g_);
  free(cind_g_);
  free(rind_l_);
  free(cind_l_);
  free(jacval_);
  free(hessval_);
}

/*************************    ADOL-C part **************************/
template <typename T>
bool IpoptGlobalPathSmootherModel::eval_obj(int /*n*/, const T* x,
                                            T* obj_value) {
  T sum_k = 0;
  for (int i = 1; i < ne_; ++i) {
    T dk = x[kappa_start_i_ + i] - x[kappa_start_i_ + i - 1];
    sum_k += Sqr(dk);
  }
  constexpr double kWeight = 10.0;
  *obj_value = x[0] + kWeight * sum_k;
  return true;
}

template <typename T>
bool IpoptGlobalPathSmootherModel::eval_constraints(int /*n*/, const T* x,
                                                    int /*m*/, T* g) {
  // dt is the time between two state variables.
  // x[0] is the total time.
  const T dt = x[0] / ne_;
  g[0] = 0;
  // Add vehicle kinematic model constraints
  // TODO(renjie): Use fast math.
  int counter = 0;
  g[counter] = 0;
  for (int i = 0; i < ne_ - 1; ++i) {
    g[++counter] = (x[x_start_i_ + i + 1] - x[x_start_i_ + i]) -
                   x[v_start_i_ + i] * cos(x[theta_start_i_ + i]) * dt;
  }
  for (int i = 0; i < ne_ - 1; ++i) {
    g[++counter] = (x[y_start_i_ + i + 1] - x[y_start_i_ + i]) -
                   x[v_start_i_ + i] * sin(x[theta_start_i_ + i]) * dt;
  }
  for (int i = 0; i < ne_ - 1; ++i) {
    g[++counter] = (x[theta_start_i_ + i + 1] - x[theta_start_i_ + i]) -
                   x[v_start_i_ + i] * x[kappa_start_i_ + i] * dt;
  }
  return true;
}

void IpoptGlobalPathSmootherModel::generate_tapes(int n, int m, int* nnz_jac_g,
                                                  int* nnz_h_lag) {
  std::vector<double> xp(n, 0.0);
  std::vector<double> lamp(m, 0.0);
  std::vector<double> zl(m, 0.0);
  std::vector<double> zu(m, 0.0);
  std::vector<adouble> xa(n, 0.0);
  std::vector<adouble> g(m, 0.0);
  std::vector<double> lam(m, 0.0);

  double sig = 0.0;
  adouble obj_value;

  double dummy = 0.0;
  obj_lam_.clear();
  obj_lam_.resize(m + 1, 0.0);
  get_starting_point(n, 1, &xp[0], 0, &zl[0], &zu[0], m, 0, &lamp[0]);
  trace_on(kTagF);
  for (int idx = 0; idx < n; idx++) {
    xa[idx] <<= xp[idx];
  }
  eval_obj(n, &xa[0], &obj_value);

  obj_value >>= dummy;
  trace_off();
  trace_on(kTagG);

  for (int idx = 0; idx < n; idx++) {
    xa[idx] <<= xp[idx];
  }
  eval_constraints(n, &xa[0], m, &g[0]);
  for (int idx = 0; idx < m; idx++) {
    g[idx] >>= dummy;
  }
  trace_off();
  trace_on(kTagL);

  for (int idx = 0; idx < n; idx++) {
    xa[idx] <<= xp[idx];
  }
  for (int idx = 0; idx < m; idx++) {
    lam[idx] = 1.0;
  }
  sig = 1.0;
  eval_obj(n, &xa[0], &obj_value);
  obj_value *= mkparam(sig);
  eval_constraints(n, &xa[0], m, &g[0]);
  for (int idx = 0; idx < m; idx++) {
    obj_value += g[idx] * mkparam(lam[idx]);
  }

  obj_value >>= dummy;

  trace_off();

  rind_g_ = nullptr;
  cind_g_ = nullptr;
  rind_l_ = nullptr;
  cind_l_ = nullptr;

  options_g_[0] = 0; /* sparsity pattern by index domains (default) */
  options_g_[1] = 0; /*                         safe mode (default) */
  options_g_[2] = 0;
  options_g_[3] = 0; /*                column compression (default) */

  jacval_ = nullptr;
  hessval_ = nullptr;

  sparse_jac(kTagG, m, n, 0, &xp[0], &nnz_jac_, &rind_g_, &cind_g_, &jacval_,
             options_g_);
  *nnz_jac_g = nnz_jac_;
  options_l_[0] = 0;
  options_l_[1] = 1;
  sparse_hess(kTagL, n, 0, &xp[0], &nnz_l_, &rind_l_, &cind_l_, &hessval_,
              options_l_);
  *nnz_h_lag = nnz_l_;
}

}  // namespace planner
}  // namespace qcraft

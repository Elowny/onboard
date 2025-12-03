#include "onboard/planner/freespace/ipopt_segmented_global_path_smoother_model.h"

#include <stdlib.h>

#include <algorithm>
#include <cmath>
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
#include "onboard/planner/util/path_util.h"

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

}  // namespace

IpoptSegmentedGlobalPathSmootherModel::IpoptSegmentedGlobalPathSmootherModel(
    ModelInput model_input)
    : input_(std::move(model_input)), m_(input_.init_traj.size()) {
  QCHECK_GT(m_, 0);
  n_.reserve(m_);
  t_index_.reserve(m_);
  x_start_index_.reserve(m_);
  y_start_index_.reserve(m_);
  theta_start_index_.reserve(m_);
  kappa_start_index_.reserve(m_);
  v_start_index_.reserve(m_);
  int accum_n = 0;
  for (int i = 0; i < m_; ++i) {
    const auto& traj_segment = input_.init_traj[i];
    QCHECK_GT(traj_segment.path.size(), 0);
    QCHECK_EQ(traj_segment.path.size(), traj_segment.v.size());
    QCHECK_EQ(traj_segment.path.size(), traj_segment.supports.size());
    const int n = traj_segment.path.size();
    t_index_.push_back(i + kStateSize * accum_n);
    x_start_index_.push_back(t_index_.back() + 1);
    y_start_index_.push_back(x_start_index_.back() + n);
    theta_start_index_.push_back(y_start_index_.back() + n);
    v_start_index_.push_back(theta_start_index_.back() + n);
    kappa_start_index_.push_back(v_start_index_.back() + n);
    n_.push_back(n);
    for (const auto& support : traj_segment.supports) {
      num_support_planes_ += support.size();
    }
    accum_n += n;
  }
  total_n_ = accum_n;
  start_point_ = input_.init_traj.front().path.front();
  end_point_ = input_.init_traj.back().path.back();
}

bool IpoptSegmentedGlobalPathSmootherModel::get_nlp_info(
    int& n, int& m, int& nnz_jac_g, int& nnz_h_lag,
    IndexStyleEnum& index_style) {
  // Num of optimization variables.
  n = m_ + kStateSize * total_n_;
  // Num of function constraints.
  m = 3 * (m_ - 1) + 3 * (total_n_ - m_) + 1 + num_support_planes_;
  generate_tapes(n, m, &nnz_jac_g, &nnz_h_lag);
  // Use the C style indexing (0-based).
  index_style = C_STYLE;
  return true;
}

bool IpoptSegmentedGlobalPathSmootherModel::get_bounds_info(int n, double* x_l,
                                                            double* x_u, int m,
                                                            double* g_l,
                                                            double* g_u) {
  // Set the bounds for time.
  for (int i = 0; i < m_; ++i) {
    x_l[t_index_[i]] = 0.0;
    x_u[t_index_[i]] = input_.max_time;
  }

  // Set the bounds for x.
  for (int i = 0; i < m_; ++i) {
    for (int j = 0; j < n_[i]; ++j) {
      x_l[x_start_index_[i] + j] = -kInf;
      x_u[x_start_index_[i] + j] = kInf;
    }
  }
  // Init condition of x.
  x_l[x_start_index_[0]] = start_point_.x();
  x_u[x_start_index_[0]] = start_point_.x();
  // Terminal condition of x.
  x_l[x_start_index_[m_ - 1] + n_[m_ - 1] - 1] = end_point_.x();
  x_u[x_start_index_[m_ - 1] + n_[m_ - 1] - 1] = end_point_.x();

  // Set the bounds for y.
  for (int i = 0; i < m_; ++i) {
    for (int j = 0; j < n_[i]; ++j) {
      x_l[y_start_index_[i] + j] = -kInf;
      x_u[y_start_index_[i] + j] = kInf;
    }
  }
  // Init condition of y.
  x_l[y_start_index_[0]] = start_point_.y();
  x_u[y_start_index_[0]] = start_point_.y();
  // Terminal condition of y.
  x_l[y_start_index_[m_ - 1] + n_[m_ - 1] - 1] = end_point_.y();
  x_u[y_start_index_[m_ - 1] + n_[m_ - 1] - 1] = end_point_.y();

  // Set the bounds for theta.
  for (int i = 0; i < m_; ++i) {
    for (int j = 0; j < n_[i]; ++j) {
      x_l[theta_start_index_[i] + j] = -kInf;
      x_u[theta_start_index_[i] + j] = kInf;
    }
  }
  // Init condition of theta.
  x_l[theta_start_index_[0]] = start_point_.theta();
  x_u[theta_start_index_[0]] = start_point_.theta();
  // Terminal condition of theta.
  x_l[theta_start_index_[m_ - 1] + n_[m_ - 1] - 1] = end_point_.theta();
  x_u[theta_start_index_[m_ - 1] + n_[m_ - 1] - 1] = end_point_.theta();

  // Set the bounds for v.
  for (int i = 0; i < m_; ++i) {
    if (input_.init_traj[i].forward) {
      for (int j = 0; j < n_[i]; ++j) {
        x_l[v_start_index_[i] + j] = 0.0;
        x_u[v_start_index_[i] + j] = input_.max_forward_speed;
      }
    } else {
      for (int j = 0; j < n_[i]; ++j) {
        x_l[v_start_index_[i] + j] = -input_.max_reverse_speed;
        x_u[v_start_index_[i] + j] = 0.0;
      }
    }
  }
  // Stationary condition at segment ends.
  for (int i = 0; i < m_; ++i) {
    x_l[v_start_index_[i]] = 0.0;
    x_u[v_start_index_[i]] = 0.0;
    x_l[v_start_index_[i] + n_[i] - 1] = 0.0;
    x_u[v_start_index_[i] + n_[i] - 1] = 0.0;
  }

  // Set the bounds for kappa.
  for (int i = 0; i < m_; ++i) {
    for (int j = 0; j < n_[i]; ++j) {
      x_l[kappa_start_index_[i] + j] = -input_.max_curvature;
      x_u[kappa_start_index_[i] + j] = input_.max_curvature;
    }
  }
  // We don't enforce kappa constraints at init and end points because we want
  // to exploit the feature of stationary steering.

  // Set function constraint bounds.
  int counter = 0;
  // Continuity at switching points.
  for (int i = 0; i < m_ - 1; ++i) {
    g_l[counter] = 0.0;
    g_u[counter] = 0.0;
    counter++;
    g_l[counter] = 0.0;
    g_u[counter] = 0.0;
    counter++;
    g_l[counter] = 0.0;
    g_u[counter] = 0.0;
    counter++;
  }
  // Kinematic model.
  for (int i = 0; i < m_; ++i) {
    for (int j = 0; j < n_[i] - 1; ++j) {
      g_l[counter] = 0.0;
      g_u[counter] = 0.0;
      counter++;
      g_l[counter] = 0.0;
      g_u[counter] = 0.0;
      counter++;
      g_l[counter] = 0.0;
      g_u[counter] = 0.0;
      counter++;
    }
  }
  // Safety constraints.
  for (int i = 0; i < m_; ++i) {
    for (int j = 0; j < n_[i]; ++j) {
      for (int k = 0; k < input_.init_traj[i].supports[j].size(); ++k) {
        g_l[counter] = 0.0;
        g_u[counter] = kInf;
        counter++;
      }
    }
  }
  // Time completion constraint.
  g_l[counter] = 0.1;
  g_u[counter] = input_.max_time;
  counter++;
  QCHECK_EQ(counter, m);

  return true;
}

bool IpoptSegmentedGlobalPathSmootherModel::get_starting_point(
    int n, bool init_x, double* x, bool init_z, double* z_l, double* z_u, int m,
    bool init_lambda, double* lambda) {
  for (int i = 0; i < m_; ++i) {
    x[t_index_[i]] = input_.max_time * n_[i] / total_n_;
  }
  for (int i = 0; i < m_; ++i) {
    for (int j = 0; j < n_[i]; ++j) {
      x[x_start_index_[i] + j] = input_.init_traj[i].path[j].x();
      x[y_start_index_[i] + j] = input_.init_traj[i].path[j].y();
      x[theta_start_index_[i] + j] = input_.init_traj[i].path[j].theta();
      x[v_start_index_[i] + j] = input_.init_traj[i].v[j];
      x[kappa_start_index_[i] + j] = input_.init_traj[i].path[j].kappa();
    }
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
bool IpoptSegmentedGlobalPathSmootherModel::eval_f(int n, const double* x,
                                                   bool new_x,
                                                   double& obj_value) {
  eval_obj(n, x, &obj_value);
  return true;
}

bool IpoptSegmentedGlobalPathSmootherModel::eval_grad_f(int n, const double* x,
                                                        bool new_x,
                                                        double* grad_f) {
  gradient(kTagF, n, x, grad_f);
  return true;
}

bool IpoptSegmentedGlobalPathSmootherModel::eval_g(int n, const double* x,
                                                   bool new_x, int m,
                                                   double* g) {
  eval_constraints(n, x, m, g);
  return true;
}

bool IpoptSegmentedGlobalPathSmootherModel::eval_jac_g(int n, const double* x,
                                                       bool new_x, int m,
                                                       int nele_jac, int* i_row,
                                                       int* j_col,
                                                       double* values) {
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

bool IpoptSegmentedGlobalPathSmootherModel::eval_h(
    int n, const double* x, bool new_x, double obj_factor, int m,
    const double* lambda, bool new_lambda, int nele_hess, int* i_row,
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

void IpoptSegmentedGlobalPathSmootherModel::finalize_solution(
    Ipopt::SolverReturn status, int n, const double* x, const double* z_l,
    const double* z_u, int m, const double* g, const double* lambda,
    double obj_value, const Ipopt::IpoptData* ip_data,
    Ipopt::IpoptCalculatedQuantities* ip_cq) {
  result_paths_.clear();
  result_paths_.reserve(m_);
  for (int i = 0; i < m_; ++i) {
    std::vector<PathPoint> path;
    path.reserve(n_[i]);
    for (int j = 0; j < n_[i]; ++j) {
      PathPoint p;
      p.set_x(x[x_start_index_[i] + j]);
      p.set_y(x[y_start_index_[i] + j]);
      if (input_.init_traj[i].forward) {
        p.set_theta(x[theta_start_index_[i] + j]);
        p.set_kappa(x[kappa_start_index_[i] + j]);
      } else {
        p.set_theta(NormalizeAngle(x[theta_start_index_[i] + j] + M_PI));
        p.set_kappa(-x[kappa_start_index_[i] + j]);
      }
      if (j == 0) {
        p.set_s(0.0);
      } else {
        p.set_s(path.back().s() + DistanceTo(path.back(), p));
      }
      path.push_back(p);
    }
    result_paths_.push_back(std::move(path));
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
bool IpoptSegmentedGlobalPathSmootherModel::eval_obj(int n, const T* x,
                                                     T* obj_value) {
  // TODO(renjie): Load params from config.
  constexpr double kTimeWeight = 1.0;
  // Cost 1: Total completion time.
  for (int i = 0; i < m_; ++i) {
    *obj_value += kTimeWeight * x[t_index_[i]];
  }

  // Cost 2: Sum of curvature rate.
  // Note: The curvature rate is approximated by kappa_(i+1) - kappa_(i) instead
  // of (kappa_(i+1) - kappa_(i)) / dt to avoid introducing nonconvexity into
  // the objective function.
  T sum_dk = 0;
  for (int i = 0; i < m_; ++i) {
    for (int j = 0; j < n_[i] - 1; ++j) {
      T dk = x[kappa_start_index_[i] + j + 1] - x[kappa_start_index_[i] + j];
      sum_dk += Sqr(dk);
    }
  }
  constexpr double kCurvatureRateWeight = 10.0;
  *obj_value += kCurvatureRateWeight * sum_dk;

  // Cost 3: Sum of acceleration.
  // Note: The acceleration is approximated by v_(i+1) - v_(i) instead
  // of (v_(i+1) - v_(i)) / dt to avoid introducing nonconvexity into
  // the objective function.
  T sum_a = 0;
  for (int i = 0; i < m_; ++i) {
    for (int j = 0; j < n_[i] - 1; ++j) {
      T a = x[v_start_index_[i] + j + 1] - x[v_start_index_[i] + j];
      sum_a += Sqr(a);
    }
  }
  constexpr double kAccelWeight = 0.5;
  *obj_value += kAccelWeight * sum_a;

  return true;
}

template <typename T>
bool IpoptSegmentedGlobalPathSmootherModel::eval_constraints(int n, const T* x,
                                                             int m, T* g) {
  std::vector<T> dt(m_);
  T sum_t = 0;
  for (int i = 0; i < m_; ++i) {
    dt[i] = x[t_index_[i]] / (n_[i] - 1);
    sum_t += x[t_index_[i]];
  }

  int counter = 0;
  // Continuity at switching points.
  for (int i = 0; i < m_ - 1; ++i) {
    g[counter++] = x[x_start_index_[i] + n_[i] - 1] - x[x_start_index_[i + 1]];
    g[counter++] = x[y_start_index_[i] + n_[i] - 1] - x[y_start_index_[i + 1]];
    g[counter++] =
        x[theta_start_index_[i] + n_[i] - 1] - x[theta_start_index_[i + 1]];
  }
  // Kinematic model.
  for (int i = 0; i < m_; ++i) {
    for (int j = 0; j < n_[i] - 1; ++j) {
      g[counter++] =
          x[x_start_index_[i] + j + 1] - x[x_start_index_[i] + j] -
          0.5 * (x[v_start_index_[i] + j + 1] + x[v_start_index_[i] + j]) *
              cos(x[theta_start_index_[i] + j] +
                  0.5 * x[v_start_index_[i] + j] *
                      x[kappa_start_index_[i] + j] * dt[i]) *
              dt[i];
      g[counter++] =
          x[y_start_index_[i] + j + 1] - x[y_start_index_[i] + j] -
          0.5 * (x[v_start_index_[i] + j + 1] + x[v_start_index_[i] + j]) *
              sin(x[theta_start_index_[i] + j] +
                  0.5 * x[v_start_index_[i] + j] *
                      x[kappa_start_index_[i] + j] * dt[i]) *
              dt[i];
      g[counter++] =
          x[theta_start_index_[i] + j + 1] - x[theta_start_index_[i] + j] -
          0.5 * x[kappa_start_index_[i] + j] *
              (x[v_start_index_[i] + j + 1] + x[v_start_index_[i] + j]) * dt[i];
    }
  }
  // Safety constraints.
  for (int i = 0; i < m_; ++i) {
    for (int j = 0; j < n_[i]; ++j) {
      for (int k = 0; k < input_.init_traj[i].supports[j].size(); ++k) {
        const auto& support_plane = input_.init_traj[i].supports[j][k];
        g[counter++] = (x[x_start_index_[i] + j] - support_plane.ref.x()) *
                           support_plane.dir.x() +
                       (x[y_start_index_[i] + j] - support_plane.ref.y()) *
                           support_plane.dir.y() -
                       input_.buffer;
      }
    }
  }
  // Time completion constraint.
  g[counter++] = sum_t;
  QCHECK_EQ(counter, m);
  return true;
}

void IpoptSegmentedGlobalPathSmootherModel::generate_tapes(int n, int m,
                                                           int* nnz_jac_g,
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

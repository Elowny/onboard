#include "onboard/planner/freespace/sqp_global_path_smoother/hpipm_solver.h"

#include <stdlib.h>

#include <algorithm>
#include <array>
#include <ostream>
#include <string>
#include <vector>

#include "absl/status/status.h"
#include "absl/strings/str_format.h"
#include "glog/logging.h"
#include "hpipm/hpipm_common.h"
#include "hpipm/hpipm_d_ocp_qp.h"
#include "hpipm/hpipm_d_ocp_qp_dim.h"
#include "hpipm/hpipm_d_ocp_qp_ipm.h"
#include "hpipm/hpipm_d_ocp_qp_sol.h"

#include "onboard/global/buffered_logger.h"
#include "onboard/global/trace.h"
#include "onboard/lite/check.h"

namespace qcraft {
namespace planner {
namespace sqp_global_smoother {
namespace {
std::string HpipmReturnStatusToString(int status) {
  switch (status) {
    case hpipm_status::SUCCESS:
      return "SUCCESS";
    case hpipm_status::MAX_ITER:
      return "MAX_ITER";
    case hpipm_status::MIN_STEP:
      return "MIN_STEP";
    case hpipm_status::NAN_SOL:
      return "NAN_SOL";
    case hpipm_status::INCONS_EQ:
      return "INCONS_EQ";
    default:
      return "UNKNOWN";
  }
}
}  // namespace

void HpipmInterface::SetProblemSize(int num_of_knots) {
  nx_.resize(num_of_knots + 1);
  nu_.resize(num_of_knots + 1);
  nbx_.resize(num_of_knots + 1);
  nbu_.resize(num_of_knots + 1);
  ng_.resize(num_of_knots + 1);
  nsbx_.resize(num_of_knots + 1);
  nsbu_.resize(num_of_knots + 1);
  nsg_.resize(num_of_knots + 1);
  hA_.resize(num_of_knots);
  hB_.resize(num_of_knots);
  hb_.resize(num_of_knots);
  hQ_.resize(num_of_knots + 1);
  hS_.resize(num_of_knots + 1);
  hR_.resize(num_of_knots + 1);
  hq_.resize(num_of_knots + 1);
  hr_.resize(num_of_knots + 1);
  hlg_.resize(num_of_knots + 1);
  hug_.resize(num_of_knots + 1);
  hC_.resize(num_of_knots + 1);
  hD_.resize(num_of_knots + 1);
  hidxbx_.resize(num_of_knots + 1);
  hlbx_.resize(num_of_knots + 1);
  hubx_.resize(num_of_knots + 1);
  hidxbu_.resize(num_of_knots + 1);
  hlbu_.resize(num_of_knots + 1);
  hubu_.resize(num_of_knots + 1);
  hZl_.resize(num_of_knots + 1);
  hZu_.resize(num_of_knots + 1);
  hzl_.resize(num_of_knots + 1);
  hzu_.resize(num_of_knots + 1);
  hlls_.resize(num_of_knots + 1);
  hlus_.resize(num_of_knots + 1);
  hidxs_.resize(num_of_knots + 1);
  initial_x_.reserve(num_of_knots + 1);
  initial_u_.reserve(num_of_knots + 1);
  num_of_knots_ = num_of_knots;
}

void HpipmInterface::SetDynamics(absl::Span<const Stage> stages,
                                 const MpcState& x0) {
  b0_ = (stages[0].lin_model.A * ToMpcStateVector(x0) + stages[0].lin_model.b);
  for (int i = 0; i < num_of_knots_; ++i) {
    if (i == 0) {
      hA_[i] = nullptr;
      hB_[i] = stages[i].lin_model.B.data();
      hb_[i] = b0_.data();
      nx_[i] = 0;
      nu_[i] = kControlSize;
    } else {
      hA_[i] = stages[i].lin_model.A.data();
      hB_[i] = stages[i].lin_model.B.data();
      hb_[i] = stages[i].lin_model.b.data();
      nx_[i] = kStateSize;
      nu_[i] = kControlSize;
    }
  }
  nx_[num_of_knots_] = kStateSize;
  nu_[num_of_knots_] = 0;
}

void HpipmInterface::SetCost(absl::Span<const Stage> stages) {
  for (int i = 0; i <= num_of_knots_; ++i) {
    hQ_[i] = stages[i].cost_mat.Q.data();
    hR_[i] = stages[i].cost_mat.R.data();
    hS_[i] = stages[i].cost_mat.S.data();
    hq_[i] = stages[i].cost_mat.q.data();
    hr_[i] = stages[i].cost_mat.r.data();
    if (stages[i].ns > 0) {
      hZl_[i] = stages[i].cost_mat.Z.data();
      hZu_[i] = stages[i].cost_mat.Z.data();
      hzl_[i] = stages[i].cost_mat.z.data();
      hzu_[i] = stages[i].cost_mat.z.data();
    } else {
      hZl_[i] = nullptr;
      hZu_[i] = nullptr;
      hzl_[i] = nullptr;
      hzu_[i] = nullptr;
    }
  }
}

void HpipmInterface::SetBounds(absl::Span<const Stage> stages) {
  for (int i = 0; i <= num_of_knots_; ++i) {
    QCHECK_EQ(stages[i].nbx, stages[i].idx_x.size());
    QCHECK_EQ(stages[i].nbu, stages[i].idx_u.size());
    nbx_[i] = (i == 0) ? 0 : stages[i].nbx;
    hidxbx_[i] = (i == 0) ? nullptr : stages[i].idx_x.data();
    hlbx_[i] = (i == 0) ? nullptr : stages[i].l_bounds_x.data();
    hubx_[i] = (i == 0) ? nullptr : stages[i].u_bounds_x.data();
    nbu_[i] = (i == num_of_knots_) ? 0 : stages[i].nbu;
    hidxbu_[i] = (i == num_of_knots_) ? nullptr : stages[i].idx_u.data();
    hlbu_[i] = (i == num_of_knots_) ? nullptr : stages[i].l_bounds_u.data();
    hubu_[i] = (i == num_of_knots_) ? nullptr : stages[i].u_bounds_u.data();
  }
}

void HpipmInterface::SetPolytopicConstraints(absl::Span<const Stage> stages) {
  for (int i = 0; i <= num_of_knots_; ++i) {
    ng_[i] = stages[i].ng;
    if (stages[i].ng > 0) {
      hC_[i] = stages[i].constrains_mat.C.data();
      hD_[i] = stages[i].constrains_mat.D.data();
      hlg_[i] = stages[i].constrains_mat.dl.data();
      hug_[i] = stages[i].constrains_mat.du.data();
    } else {
      hC_[i] = nullptr;
      hD_[i] = nullptr;
      hlg_[i] = nullptr;
      hug_[i] = nullptr;
    }
  }
}

void HpipmInterface::SetSoftConstraints(absl::Span<const Stage> stages) {
  for (int i = 0; i <= num_of_knots_; ++i) {
    QCHECK_EQ(stages[i].ns, stages[i].nsbx + stages[i].nsbu + stages[i].nsg);
    if (stages[i].ns > 0) {
      nsbx_[i] = stages[i].nsbx;
      nsbu_[i] = stages[i].nsbu;
      nsg_[i] = stages[i].nsg;
      hidxs_[i] = stages[i].idx_s.data();
      hlls_[i] = stages[i].l_bounds_s.data();
      hlus_[i] = stages[i].u_bounds_s.data();
    } else {
      nsbx_[i] = 0;
      nsbu_[i] = 0;
      nsg_[i] = 0;
      hidxs_[i] = nullptr;
      hlls_[i] = nullptr;
      hlus_[i] = nullptr;
    }
  }
}

void HpipmInterface::SetInitialGuess(absl::Span<const Stage> stages) {
  for (int i = 0; i <= num_of_knots_; ++i) {
    initial_x_.emplace_back(stages[i].xk.data());
    initial_u_.emplace_back(stages[i].uk.data());
  }
}

absl::StatusOr<std::vector<OptVariables>> HpipmInterface::SolveMpc(
    absl::Span<const Stage> stages, const MpcState& x0) {
  SCOPED_QTRACE("Hpipm Interface: SolveMpc");
  SetProblemSize(stages.size() - 1);
  SetDynamics(stages, x0);
  SetCost(stages);
  SetBounds(stages);
  SetPolytopicConstraints(stages);
  SetSoftConstraints(stages);
  SetInitialGuess(stages);

  return Solve(x0);
}

absl::StatusOr<std::vector<OptVariables>> HpipmInterface::Solve(
    const MpcState& x0) {
  SCOPED_QTRACE("Hpipm Interface: Solve");
  // Ocp qp dim.
  int dim_size = d_ocp_qp_dim_memsize(num_of_knots_);
  void* dim_mem = malloc(dim_size);
  struct d_ocp_qp_dim dim;
  d_ocp_qp_dim_create(num_of_knots_, &dim, dim_mem);

  d_ocp_qp_dim_set_all(nx_.data(), nu_.data(), nbx_.data(), nbu_.data(),
                       ng_.data(), nsbx_.data(), nsbu_.data(), nsg_.data(),
                       &dim);
  // Ocp qp.
  int qp_size = d_ocp_qp_memsize(&dim);
  void* qp_mem = malloc(qp_size);
  struct d_ocp_qp qp;
  d_ocp_qp_create(&dim, &qp, qp_mem);
  d_ocp_qp_set_all(
      const_cast<double**>(hA_.data()), const_cast<double**>(hB_.data()),
      const_cast<double**>(hb_.data()), const_cast<double**>(hQ_.data()),
      const_cast<double**>(hS_.data()), const_cast<double**>(hR_.data()),
      const_cast<double**>(hq_.data()), const_cast<double**>(hr_.data()),
      const_cast<int**>(hidxbx_.data()), const_cast<double**>(hlbx_.data()),
      const_cast<double**>(hubx_.data()), const_cast<int**>(hidxbu_.data()),
      const_cast<double**>(hlbu_.data()), const_cast<double**>(hubu_.data()),
      const_cast<double**>(hC_.data()), const_cast<double**>(hD_.data()),
      const_cast<double**>(hlg_.data()), const_cast<double**>(hug_.data()),
      const_cast<double**>(hZl_.data()), const_cast<double**>(hZu_.data()),
      const_cast<double**>(hzl_.data()), const_cast<double**>(hzu_.data()),
      const_cast<int**>(hidxs_.data()), const_cast<double**>(hlls_.data()),
      const_cast<double**>(hlus_.data()), &qp);

  // Ocp qp sol.
  int qp_sol_size = d_ocp_qp_sol_memsize(&dim);
  void* qp_sol_mem = malloc(qp_sol_size);
  struct d_ocp_qp_sol qp_sol;
  d_ocp_qp_sol_create(&dim, &qp_sol, qp_sol_mem);

  // Warm start.
  d_ocp_qp_sol_set_u(0, const_cast<double*>(initial_u_[0]), &qp_sol);
  d_ocp_qp_sol_set_x(num_of_knots_,
                     const_cast<double*>(initial_x_[num_of_knots_]), &qp_sol);
  for (int i = 1; i < num_of_knots_; ++i) {
    d_ocp_qp_sol_set_x(i, const_cast<double*>(initial_x_[i]), &qp_sol);
    d_ocp_qp_sol_set_u(i, const_cast<double*>(initial_u_[i]), &qp_sol);
  }

  // IPM arg
  int ipm_arg_size = d_ocp_qp_ipm_arg_memsize(&dim);
  void* ipm_arg_mem = malloc(ipm_arg_size);
  struct d_ocp_qp_ipm_arg arg;
  d_ocp_qp_ipm_arg_create(&dim, &arg, ipm_arg_mem);

  // enum hpipm_mode {
  //   SPEED_ABS,  // Focus on speed, absolute IPM formulation
  //   SPEED,      // Focus on speed, relative IPM formulation
  //   BALANCE,    // Balanced mode, relative IPM formulation
  //   ROBUST,     // Focus on robustness, relative IPM formulation
  // };
  enum hpipm_mode mode = BALANCE;
  double mu0 = 1e2;
  int iter_max = 30;
  double tol_stat = 1e-3;
  double tol_eq = 1e-6;
  double tol_ineq = 1e-5;
  double tol_comp = 1e-5;
  double reg_prim = 1e-5;
  int warm_start = 1;
  int pred_corr = 1;
  int ric_alg = 0;

  d_ocp_qp_ipm_arg_set_default(mode, &arg);
  d_ocp_qp_ipm_arg_set_mu0(&mu0, &arg);
  d_ocp_qp_ipm_arg_set_iter_max(&iter_max, &arg);
  d_ocp_qp_ipm_arg_set_tol_stat(&tol_stat, &arg);
  d_ocp_qp_ipm_arg_set_tol_eq(&tol_eq, &arg);
  d_ocp_qp_ipm_arg_set_tol_ineq(&tol_ineq, &arg);
  d_ocp_qp_ipm_arg_set_tol_comp(&tol_comp, &arg);
  d_ocp_qp_ipm_arg_set_reg_prim(&reg_prim, &arg);
  d_ocp_qp_ipm_arg_set_warm_start(&warm_start, &arg);
  d_ocp_qp_ipm_arg_set_pred_corr(&pred_corr, &arg);
  d_ocp_qp_ipm_arg_set_ric_alg(&ric_alg, &arg);

  // IPM workspace.
  int ipm_size = d_ocp_qp_ipm_ws_memsize(&dim, &arg);
  void* ipm_mem = malloc(ipm_size);
  struct d_ocp_qp_ipm_ws workspace;
  d_ocp_qp_ipm_ws_create(&dim, &arg, &workspace, ipm_mem);

  // IPM solve.
  // enum hpipm_status {
  //   SUCCESS,    // Found solution satisfying accuracy tolerance
  //   MAX_ITER,   // Maximum iteration number reached
  //   MIN_STEP,   // Minimum step length reached
  //   NAN_SOL,    // NaN in solution detected
  //   INCONS_EQ,  // Inconsistent equality constraints
  // };
  int hpipm_return = -1;
  double cost = 0.0;
  d_ocp_qp_ipm_solve(&qp, &qp_sol, &arg, &workspace);
  d_ocp_qp_ipm_get_status(&workspace, &hpipm_return);
  d_ocp_qp_ipm_get_obj(&workspace, &cost);

  VLOG(3) << "HPIPM arg size = = " << num_of_knots_;
  VLOG(3) << "HPIPM iter = " << workspace.iter;
  VLOG(3) << "HPIPM obj cost = " << cost;
  if (hpipm_return != hpipm_status::SUCCESS) {
    free(dim_mem);
    free(qp_mem);
    free(qp_sol_mem);
    free(ipm_arg_mem);
    free(ipm_mem);
    return absl::InternalError(
        absl::StrFormat("Hpipm solve failed, errorcode(%s)",
                        HpipmReturnStatusToString(hpipm_return)));
  }

  // Fetch results.
  std::vector<OptVariables> optimal_solution;
  optimal_solution.reserve(num_of_knots_ + 1);
  std::array<double, kStateSize> x;
  std::array<double, kControlSize> u;

  d_ocp_qp_sol_get_u(0, &qp_sol, u.data());
  optimal_solution.push_back({.xk = x0, .uk = ToMpcControl(u)});
  for (int i = 1; i < num_of_knots_; ++i) {
    d_ocp_qp_sol_get_x(i, &qp_sol, x.data());
    d_ocp_qp_sol_get_u(i, &qp_sol, u.data());
    optimal_solution.push_back({.xk = ToMpcState(x), .uk = ToMpcControl(u)});
  }
  d_ocp_qp_sol_get_x(num_of_knots_, &qp_sol, x.data());
  optimal_solution.push_back({.xk = ToMpcState(x), .uk = MpcControl()});

  // Free data.
  free(dim_mem);
  free(qp_mem);
  free(qp_sol_mem);
  free(ipm_arg_mem);
  free(ipm_mem);

  return optimal_solution;
}
}  // namespace sqp_global_smoother
}  // namespace planner
}  // namespace qcraft

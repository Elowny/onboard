#ifndef ONBOARD_PLANNER_FREESPACE_HPIPM_SOLVER_H
#define ONBOARD_PLANNER_FREESPACE_HPIPM_SOLVER_H
#include <vector>

#include "absl/status/statusor.h"
#include "absl/types/span.h"

#include "onboard/math/eigen.h"
#include "onboard/planner/freespace/sqp_global_path_smoother/hpipm_solver_defs.h"
namespace qcraft {
namespace planner {
namespace sqp_global_smoother {
class HpipmInterface {
 public:
  absl::StatusOr<std::vector<OptVariables>> SolveMpc(
      absl::Span<const Stage> stages, const MpcState& x0);

 private:
  // Number of states.
  std::vector<int> nx_;
  // Number of controls.
  std::vector<int> nu_;
  // Number of bounds on x.
  std::vector<int> nbx_;
  // Number of bounds on u.
  std::vector<int> nbu_;
  // Number of polytopic constratins.
  std::vector<int> ng_;
  // Number of slacks variables on x.
  std::vector<int> nsbx_;
  // Number of slacks variables on u.
  std::vector<int> nsbu_;
  // Number of slacks variables on polytopic constraints.
  std::vector<int> nsg_;

  // LTV dynamics
  // x_k+1 = A_k x_k + B_k u_k + b_k.
  // hA_[k] = A_k.
  std::vector<const double*> hA_;
  // hB_[k] = B_k.
  std::vector<const double*> hB_;
  // hb_[k] = b_k.
  std::vector<const double*> hb_;

  // Cost (without soft constraints)
  // min_{x,u} sum_{k=0}^N 1/2*[x_k;u_k]^T*[Q_k , S_k; S_k^T , R_k]*[x_k;u_k] +
  // [q_k; r_k]^T*[x_k;u_k].
  // hQ_[k] = Q_k.
  std::vector<const double*> hQ_;
  // hS_[k] = S_k.
  std::vector<const double*> hS_;
  // hR_[k] = R_k.
  std::vector<const double*> hR_;
  // hq_[k] = q_k.
  std::vector<const double*> hq_;

  std::vector<const double*> hr_;

  // Constraints.

  // [x_{lower,k}; u_{lower,k}; g_{lower,k}] <= [J^{b, x}_{k}, 0; 0, J^{b,
  // u}_{k}; C_k, D_k] * [x_k; u_k] + J^{s}_{k} * s_{lower,k}.

  // [J^{b, x}_{k}, 0; 0, J^{b,u}_{k}; C_k, D_k] * [x_k; u_k] - J^{s}_{k} *
  // s_{upper,k} <= [x_{upper,k}; u_{upper,k}; g_{upper,k}].
  // hlg_[k] =  g_{lower,k}.
  std::vector<const double*> hlg_;
  // hug_[k] =  g_{upper,k}.
  std::vector<const double*> hug_;
  // hC_[k] = C_k.
  std::vector<const double*> hC_;
  // hD_[k] = D_k.
  std::vector<const double*> hD_;

  // hidxbx can be used to select bounds on a subset of states.
  // hidxbx_[k] = {0,1,2,...,nx} for bounds on all states, J^{b, x}_{k}.
  std::vector<const int*> hidxbx_;
  // hlbx_[k] = x_{lower,k}.
  std::vector<const double*> hlbx_;
  // hubx_[k] = x_{upper,k}.
  std::vector<const double*> hubx_;
  // u_{lower,k} <= u_k <=  u_{upper,k}.
  // hidxbu can be used to select bounds on a subset of controls.
  // hidxbu[k] = {0,1,2,...,nu} for bounds on all controls, J^{b, u}_{k}.
  std::vector<const int*> hidxbu_;
  // hlbu_[k] = u_{lower,k}.
  std::vector<const double*> hlbu_;
  // hubu_[k] = u_{upper,k}.
  std::vector<const double*> hubu_;

  // Soft constraints cost.
  // min_{x,u} sum_{k=0}^N
  // 1/2*[s_{lower,k};s_{upper,k}]^T*[Z_{lower,k} , 0; 0 , Z_{upper,k}]
  // * [s_{lower,k};s_{upper,k}] +
  // [z_{lower,k}; z_{upper,k}]^T*[s_{lower,k};s_{upper,k}]
  // hZl_[k] = Z_{lower,k}.
  std::vector<const double*> hZl_;
  // hZu_[k] = Z_{upper,k}.
  std::vector<const double*> hZu_;
  // hzl_[k] = z_{lower,k}.
  std::vector<const double*> hzl_;
  // hzu_[k] = z_{upper,k}.
  std::vector<const double*> hzu_;

  // Bounds on the soft constraint.
  // s_{lower,k} >= s_lower,bound,k
  // s_{upper,k} >= s_upper,bound,k
  std::vector<const double*> hlls_;
  std::vector<const double*> hlus_;
  // Index of the bounds and constraints that are softened.
  // J^{s}_{k}
  std::vector<const int*> hidxs_;

  Eigen::Matrix<double, kStateSize, 1> b0_;
  std::vector<const double*> initial_x_;
  std::vector<const double*> initial_u_;

  int num_of_knots_ = 0;

  void SetProblemSize(int num_of_knots);
  void SetDynamics(absl::Span<const Stage> stages, const MpcState& x0);
  void SetCost(absl::Span<const Stage> stages);
  void SetBounds(absl::Span<const Stage> stages);
  void SetPolytopicConstraints(absl::Span<const Stage> stages);
  void SetSoftConstraints(absl::Span<const Stage> stages);
  void SetInitialGuess(absl::Span<const Stage> stages);

  absl::StatusOr<std::vector<OptVariables>> Solve(const MpcState& x0);
};
}  // namespace sqp_global_smoother
}  // namespace planner
}  // namespace qcraft
#endif  // ONBOARD_PLANNER_FREESPACE_HPIPM_SOLVER_H

#ifndef ONBOARD_PLANNER_FREESPACE_IPOPT_GLOBAL_PATH_SMOOTHER_MODEL_H_
#define ONBOARD_PLANNER_FREESPACE_IPOPT_GLOBAL_PATH_SMOOTHER_MODEL_H_

#include <vector>

#define HAVE_STDDEF_H
#include "coin/IpAlgTypes.hpp"
#include "coin/IpTNLP.hpp"

#include "onboard/proto/trajectory_point.pb.h"

#undef HAVE_STDDEF_H

namespace qcraft {
namespace planner {

struct IpoptGlobalPathSmootherInput {
  std::vector<PathPoint> init_path;
  std::vector<double> init_v;
  // TODO(renjie): Remove init_kappa because it's included in init_path.
  std::vector<double> init_kappa;
  double v_upper;
  double v_lower;
  double max_kappa;
};

// Global path smoother algorithm, see the doc:
// https://qcraft.feishu.cn/docs/doccnRVJnxrROBjinCcBzZzJvsc
class IpoptGlobalPathSmootherModel : public Ipopt::TNLP {
 public:
  using ModelInput = IpoptGlobalPathSmootherInput;

  /** default constructor */
  explicit IpoptGlobalPathSmootherModel(ModelInput model_input);
  IpoptGlobalPathSmootherModel(const IpoptGlobalPathSmootherModel&) = delete;
  IpoptGlobalPathSmootherModel(IpoptGlobalPathSmootherModel&&) = delete;
  IpoptGlobalPathSmootherModel& operator=(
      const IpoptGlobalPathSmootherModel&) const = delete;
  IpoptGlobalPathSmootherModel& operator=(
      IpoptGlobalPathSmootherModel&&) const = delete;

  /** default destructor */
  ~IpoptGlobalPathSmootherModel() override = default;

  /** Method to return some info about the nlp */
  bool get_nlp_info(int& n, int& m, int& nnz_jac_g, int& nnz_h_lag,
                    IndexStyleEnum& index_style) override;

  /** Method to return the bounds for my problem */
  bool get_bounds_info(int n, double* x_l, double* x_u, int m, double* g_l,
                       double* g_u) override;

  /** Method to return the starting point for the algorithm */
  bool get_starting_point(int n, bool init_x, double* x, bool init_z,
                          double* z_l, double* z_u, int m, bool init_lambda,
                          double* lambda) override;

  /** Original method from Ipopt to return the objective value */
  /** remains unchanged */
  bool eval_f(int n, const double* x, bool new_x, double& obj_value) override;

  /** Original method from Ipopt to return the gradient of the objective */
  /** remains unchanged */
  bool eval_grad_f(int n, const double* x, bool new_x, double* grad_f) override;

  /**  Original method from Ipopt to return the constraint residuals */
  /** remains unchanged */
  bool eval_g(int n, const double* x, bool new_x, int m, double* g) override;

  /** Original method from Ipopt to return:
   *   1) The structure of the jacobian (if "values" is NULL)
   *   2) The values of the jacobian (if "values" is not NULL)
   */
  /** remains unchanged */
  bool eval_jac_g(int n, const double* x, bool new_x, int m, int nele_jac,
                  int* i_row, int* j_col, double* values) override;

  /** Original method from Ipopt to return:
   *   1) The structure of the hessian of the lagrangian (if "values" is NULL)
   *   2) The values of the hessian of the lagrangian (if "values" is not NULL)
   */
  /** remains unchanged */
  bool eval_h(int n, const double* x, bool new_x, double obj_factor, int m,
              const double* lambda, bool new_lambda, int nele_hess, int* i_row,
              int* j_col, double* values) override;

  /** This method is called when the algorithm is complete so the TNLP can
   * store/write the solution */
  void finalize_solution(Ipopt::SolverReturn status, int n, const double* x,
                         const double* z_l, const double* z_u, int m,
                         const double* g, const double* lambda,
                         double obj_value, const Ipopt::IpoptData* ip_data,
                         Ipopt::IpoptCalculatedQuantities* ip_cq) override;

  //***************    start ADOL-C part ***********************************
  /** Template to return the objective value */
  template <typename T>
  bool eval_obj(int n, const T* x, T* obj_value);

  /** Template to compute constraints */
  template <typename T>
  bool eval_constraints(int n, const T* x, int m, T* g);

  /** Method to generate the required tapes */
  void generate_tapes(int n, int m, int* nnz_jac_g, int* nnz_h_lag);

  //***************    end   ADOL-C part ***********************************

  const std::vector<double>& result() { return result_; }

 private:
  /*************************    ADOL-C part ******************************/
  std::vector<double> obj_lam_;
  /** Variables for sparsity exploitation. */
  unsigned int* rind_g_ = nullptr;  // row indices
  unsigned int* cind_g_ = nullptr;  // column indices
  double* jacval_ = nullptr;        // values
  unsigned int* rind_l_ = nullptr;  // row indices
  unsigned int* cind_l_ = nullptr;  // column indices
  double* hessval_ = nullptr;       // values
  int nnz_jac_ = 0;
  int nnz_l_ = 0;
  int options_g_[4] = {0, 0, 0, 0};
  int options_l_[4] = {0, 0, 0, 0};
  /*************************    End ADOL-C part **************************/

  ModelInput input_;
  int ne_ = 0;             // Number of path points.
  int x_start_i_ = 0;      // x start index.
  int y_start_i_ = 0;      // y start index.
  int theta_start_i_ = 0;  // theta start index.
  int v_start_i_ = 0;      // v start index.
  int kappa_start_i_ = 0;  // kappa start index.
  PathPoint start_point_;
  PathPoint end_point_;

  // TODO(renjie): Output results separately.
  std::vector<double> result_;
};

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_FREESPACE_IPOPT_GLOBAL_PATH_SMOOTHER_MODEL_H_

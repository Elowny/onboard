#ifndef ONBOARD_PLANNER_FREESPACE_IPOPT_SEGMENTED_GLOBAL_PATH_SMOOTHER_MODEL_H_
#define ONBOARD_PLANNER_FREESPACE_IPOPT_SEGMENTED_GLOBAL_PATH_SMOOTHER_MODEL_H_

#include <vector>

#define HAVE_STDDEF_H
#include "coin/IpAlgTypes.hpp"
#include "coin/IpTNLP.hpp"

#undef HAVE_STDDEF_H

#include "onboard/math/vec.h"
#include "onboard/proto/trajectory_point.pb.h"

namespace qcraft {
namespace planner {

struct IpoptSegmentedGlobalPathSmootherInput {
  struct SupportPlane {
    Vec2d ref;
    Vec2d dir;  // Point to the feasible side.
    SupportPlane(const Vec2d& ref, const Vec2d& dir) : ref(ref), dir(dir) {}
  };
  // Init guess trajectory.
  struct TrajectorySegment {
    // path and v should be of the same size.
    std::vector<PathPoint> path;
    std::vector<double> v;
    bool forward;
    std::vector<std::vector<SupportPlane>> supports;
  };
  std::vector<TrajectorySegment> init_traj;
  // Constraints.
  double max_forward_speed;   // m/s, positive value.
  double max_reverse_speed;   // m/s, positive value.
  double max_accel;           // m/s^2, positive value.
  double min_accel;           // m/s^2, negative value.
  double max_curvature;       // m^1, positive value.
  double max_curvature_rate;  // m^1, positive value.
  double max_time;            // s.
  double buffer;              // m. Protective buffer at RAC point.
};

// Segmented global path smoother algorithm, details see the doc:
// https://qcraft.feishu.cn/docs/doccnVopk4j6J2gseYiwH1FDnxc

// Optimization variables (for i-th segment):
// t_i,
// [x_i_0, ..., x_i_(n_i-1)],
// [y_i_0, ..., y_i_(n_i-1)],
// [theta_i_0, ..., theta_i_(n_i-1)],
// [v_i_0, ..., v_i_(n_i-1)],
// [kappa_i_0, ..., kappa_i_(n_i-1)]
//
// Total number of variables: m + 5 * (n_0 + ... + n_(m-1)) where m is the
// number of segments and n_i is the number of points of i-th segment.

// Bound constraints:
// 1) Init conditions:
// x_0_0 = x_s, y_0_0 = y_s, theta_0_0 = theta_s
//
// 2) Terminal conditions:
// x_(m-1)_(n_(m-1)-1) = x_f, y_(m-1)_(n_(m-1)-1) = y_f,
// theta_(m-1)_(n_(m-1)-1) = theta_f
//
// 3) Speed limit (for i-th segment j-th point):
// 0 <= v_i_j <= max_forward_speed if ith segment is forward.
// -max_reverse_speed <= v_i_j <= 0 if ith segment is backward.
//
// 4) Stationary conditions at segment ends (for i-th segment):
// v_i_0 = 0, v_i_(n_i-1) = 0
//
// 5) Cuvature constraints (for i-th segment j-th point):
// -max_curvature <= kappa_i_j <= max_curvature

// Function constraints:
// 1) Continuity at switching points (for i-th switching point, totally m - 1
// switching points):
// x_i_(n_i-1) = x_(i+1)_0, y_i_(n_i-1) = y_(i+1)_0,
// theta_i_(n_i-1) = theta_(i+1)_0
//
// 2) Kinematic model (for i-th segment j-th point):
// x_i_(j+1) = x_i_j + 0.5 * (v_i_j + v_i_(j+1)) * cos(theta_i_j + 0.5 * v_i_j *
// kappa_i_j * t_i / (n_i-1)) * t_i / (n_i-1)                     (second order)
// y_i_(j+1) = y_i_j + 0.5 * (v_i_j + v_i_(j+1)) * sin(theta_i_j + 0.5 * v_i_j *
// kappa_i_j * t_i / (n_i-1)) * t_i / (n_i-1)                     (second order)
// theta_i_(j+1) = theta_i_j + 0.5 * kappa_i_j * (v_i_j + v_i_(j+1)) * t_i /
// (n_i-1)                                                               (exact)
//
// 3) Safety constraint (for i-th segment j-th point k-th support plane):
// (x_i_j - x_r_i_j_k) * x_d_i_j_k + (y_i_j - y_r_i_j_k) * y_d_i_j_k - buffer >=
// 0 where (x_r, y_r) and (d_x, d_y) are ref point and inner direction of
// boundary support planes.
//
// 4) Completion time constraint: 0.1 <= t_0 + ... + t_(m-1) <= max_time
//
// Total number of function constraints: 3 * (m - 1) + 3 * (n_0 + ... + n_(m-1)
// - m) + 1 + n_s (n_s is the total number of boundary support planes)
class IpoptSegmentedGlobalPathSmootherModel : public Ipopt::TNLP {
 public:
  using ModelInput = IpoptSegmentedGlobalPathSmootherInput;

  /** default constructor */
  explicit IpoptSegmentedGlobalPathSmootherModel(ModelInput model_input);
  IpoptSegmentedGlobalPathSmootherModel(
      const IpoptSegmentedGlobalPathSmootherModel&) = delete;
  IpoptSegmentedGlobalPathSmootherModel(
      IpoptSegmentedGlobalPathSmootherModel&&) = delete;
  IpoptSegmentedGlobalPathSmootherModel& operator=(
      const IpoptSegmentedGlobalPathSmootherModel&) const = delete;
  IpoptSegmentedGlobalPathSmootherModel& operator=(
      IpoptSegmentedGlobalPathSmootherModel&&) const = delete;

  /** default destructor */
  ~IpoptSegmentedGlobalPathSmootherModel() override = default;

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

  const std::vector<std::vector<PathPoint>>& result_paths() const {
    return result_paths_;
  }

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

  static constexpr int kStateSize = 5;

  ModelInput input_;
  int m_ = 0;                           // Number of segments.
  int total_n_ = 0;                     // Total number of points.
  std::vector<int> n_;                  // Number of points of each segment.
  std::vector<int> t_index_;            // t index of each segment.
  std::vector<int> x_start_index_;      // x start index of each segment.
  std::vector<int> y_start_index_;      // y start index of each segment.
  std::vector<int> theta_start_index_;  // theta start index of each segment.
  std::vector<int> v_start_index_;      // v start index of each segment.
  std::vector<int> kappa_start_index_;  // kappa start index of each segment.
  int num_support_planes_ = 0;
  PathPoint start_point_;
  PathPoint end_point_;

  std::vector<std::vector<PathPoint>> result_paths_;
};

}  // namespace planner
}  // namespace qcraft

// NOLINTNEXTLINE
#endif  // ONBOARD_PLANNER_FREESPACE_IPOPT_SEGMENTED_GLOBAL_PATH_SMOOTHER_MODEL_H_

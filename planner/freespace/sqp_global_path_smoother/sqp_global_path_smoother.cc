#include "onboard/planner/freespace/sqp_global_path_smoother/sqp_global_path_smoother.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <deque>
#include <iostream>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_format.h"
#include "absl/time/clock.h"
#include "absl/time/time.h"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "unsupported/Eigen/MatrixFunctions"

#include "onboard/global/trace.h"
#include "onboard/lite/check.h"
#include "onboard/lite/logging.h"
#include "onboard/math/eigen.h"
#include "onboard/math/fast_math.h"
#include "onboard/math/geometry/circle2d.h"
#include "onboard/math/geometry/polygon2d.h"
#include "onboard/math/geometry/segment2d.h"
#include "onboard/math/util.h"
#include "onboard/math/vec.h"
#include "onboard/planner/common/discretized_path.h"
#include "onboard/planner/common/vehicle_shape.h"
#include "onboard/planner/freespace/freespace_util.h"
#include "onboard/planner/freespace/sqp_global_path_smoother/convex_region_defs.h"
#include "onboard/planner/freespace/sqp_global_path_smoother/convex_region_generator.h"
#include "onboard/planner/freespace/sqp_global_path_smoother/geometric_utils.h"
#include "onboard/planner/freespace/sqp_global_path_smoother/hpipm_solver.h"
#include "onboard/planner/freespace/sqp_global_path_smoother/hpipm_solver_defs.h"
#include "onboard/planner/util/vehicle_geometry_util.h"
#include "onboard/proto/trajectory_point.pb.h"
#include "onboard/vis/canvas/canvas.h"
#include "onboard/vis/common/color.h"

#include "offboard/vis/vantage/vantage_server/vantage_canvas_client.h"

DEFINE_bool(sqp_global_smoother_send_vehicle_contour_to_canvas, false,
            "Whether to send all vehicle contour to canvas.");
DEFINE_bool(sqp_global_smoother_send_convex_region_to_canvas, false,
            "Whether to send convex region to canvas.");
DEFINE_bool(sqp_global_smoother_send_global_smooth_path_to_canvas, false,
            "Whether to send global smooth path to canvas.");
DEFINE_bool(sqp_global_smoother_enable_model_error_debug, false,
            "Whether to enable model error debug print.");
namespace qcraft {
namespace planner {
namespace sqp_global_smoother {
namespace {

constexpr double kMargin = 0.01;
constexpr double kLargerMargin = 0.1;
constexpr double kEps = 1e-4;

struct ModelInput {
  // Init guess trajectory.
  DiscretizedPath path;
  std::vector<bool> forwards;
  std::vector<int> switch_point_indices;
  std::vector<MpcState> initial_xs;
  std::vector<MpcControl> initial_us;
  // Constraints.
  double max_forward_speed;   // m/s, positive value.
  double max_reverse_speed;   // m/s, positive value.
  double max_accel;           // m/s^2, positive value.
  double min_accel;           // m/s^2, negative value.
  double max_curvature;       // m^-1, positive value.
  double max_curvature_rate;  // (ms)^-1, positive value.
};

MpcStateVector DifferentialModel(const MpcStateVector& x,
                                 const MpcControlVector& u) {
  MpcStateVector res;
  const double v = x(kStateVIndex);
  const double theta = x(kStateThetaIndex);
  const double k = x(kStateKappaIndex);
  const double a = u(kControlAIndex);
  const double psi = u(kControlPsiIndex);

  double theta_cos_sin[2];
  fast_math::CosAndSin<7>(theta, theta_cos_sin);
  const double sin_theta = theta_cos_sin[1];
  const double cos_theta = theta_cos_sin[0];
  res << v * cos_theta, v * sin_theta, a, v * k, psi;
  return res;
}
MpcState RK4(const MpcState& x, const MpcControl& u, double dt) {
  const MpcStateVector x_vec = ToMpcStateVector(x);
  const MpcControlVector u_vec = ToMpcControlVector(u);
  const MpcStateVector k1 = DifferentialModel(x_vec, u_vec);
  const MpcStateVector k2 = DifferentialModel(x_vec + dt / 2.0 * k1, u_vec);
  const MpcStateVector k3 = DifferentialModel(x_vec + dt / 2.0 * k2, u_vec);
  const MpcStateVector k4 = DifferentialModel(x_vec + dt * k3, u_vec);
  const MpcStateVector x_next =
      x_vec + dt * (k1 / 6.0 + k2 / 3.0 + k3 / 3.0 + k4 / 6.0);
  return ToMpcState(x_next);
}
LinModelMatrix GetModelJacobian(const MpcState& x, const MpcControl& u) {
  // Compute jacobian of the model
  const double v = x.v;
  const double theta = x.theta;
  const double k = x.k;
  const double a = u.a;
  const double psi = u.psi;
  double theta_cos_sin[2];
  fast_math::CosAndSin<7>(theta, theta_cos_sin);
  const double sin_theta = theta_cos_sin[1];
  const double cos_theta = theta_cos_sin[0];

  MpcMatrixA A_c = MpcMatrixA::Zero();  // NOLINT(readability-identifier-naming)
  MpcMatrixB B_c = MpcMatrixB::Zero();  // NOLINT(readability-identifier-naming)
  MpcMatrixb b_c = MpcMatrixb::Zero();  // NOLINT(readability-identifier-naming)
  MpcStateVector f;
  // x,y,v,theta,k
  f << v * cos_theta, v * sin_theta, a, v * k, psi;
  A_c << 0.0, 0.0, cos_theta, -v * sin_theta, 0.0,  // x.
      0.0, 0.0, sin_theta, v * cos_theta, 0.0,      // y.
      0.0, 0.0, 0.0, 0.0, 0.0,                      // v.
      0.0, 0.0, k, 0.0, v,                          // theta.
      0.0, 0.0, 0.0, 0.0, 0.0;                      // k.
  B_c << 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0;

  // Zero order term.
  b_c = f - A_c * ToMpcStateVector(x) - B_c * ToMpcControlVector(u);

  return {A_c, B_c, b_c};
}
LinModelMatrix DiscretizeBicycleModel(const LinModelMatrix& lin_model,
                                      double dt) {
  // Disctetize the continuous time linear model \dot x = A x + B u + b.
  Eigen::Matrix<double, kStateSize + kControlSize + 1,
                kStateSize + kControlSize + 1>
      temp = Eigen::Matrix<double, kStateSize + kControlSize + 1,
                           kStateSize + kControlSize + 1>::Zero();
  // Building matrix necessary for expm.
  // temp = dt*[A,B,g;zeros].
  temp.block<kStateSize, kStateSize>(0, 0) = lin_model.A;
  temp.block<kStateSize, kControlSize>(0, kStateSize) = lin_model.B;
  temp.block<kStateSize, 1>(0, kStateSize + kControlSize) = lin_model.b;
  temp = temp * dt;
  const Eigen::Matrix<double, kStateSize + kControlSize + 1,
                      kStateSize + kControlSize + 1>
      temp_res = temp.exp();
  // Extract dynamics out of big matrix.
  // x_{k+1} = Ad x_k + Bd u_k + bd.
  // temp_res = [Ad,Bd,bd;zeros].
  // NOLINTNEXTLINE(readability-identifier-naming)
  const MpcMatrixA A_d = temp_res.template block<kStateSize, kStateSize>(0, 0);
  const MpcMatrixB B_d =  // NOLINT(readability-identifier-naming)
      temp_res.template block<kStateSize, kControlSize>(0, kStateSize);
  const MpcMatrixb b_d =  // NOLINT(readability-identifier-naming)
      temp_res.template block<kStateSize, 1>(0, kStateSize + kControlSize);

  return {A_d, B_d, b_d};
}
void SetModelConstraints(double dt, const MpcState& x, const MpcControl& u,
                         Stage* stage) {
  SCOPED_QTRACE("SQP Global Smoother: SetModelConstraints");
  const auto linear_model = GetModelJacobian(x, u);
  stage->lin_model = DiscretizeBicycleModel(linear_model, dt);
}
void SetInplaceSteerConstraints(Stage* stage) {
  // NOLINTNEXTLINE(readability-identifier-naming)
  MpcMatrixA A_d = MpcMatrixA::Identity();
  A_d(kStateKappaIndex, kStateKappaIndex) = 0.0;
  // NOLINTNEXTLINE(readability-identifier-naming)
  const MpcMatrixB B_d = MpcMatrixB::Zero();
  // NOLINTNEXTLINE(readability-identifier-naming)
  const MpcMatrixb b_d = MpcMatrixb::Zero();
  stage->lin_model = {A_d, B_d, b_d};
  stage->l_bounds_x[kStateVIndex] = -kMargin;
  stage->u_bounds_x[kStateVIndex] = kMargin;
}
/**
 * @brief Set the Point of Car Model In Convex Polygon Constraints
 *
 * @param A The convex polygon matrix
 * @param b The convex polygon vector
 * @param stage The stage of hpipm solver
 * @param taylor The taylor coeff of theta
 * @param taylor_shift The const term of taylor
 * @param cur_row
 */
void SetPointInConvexPolygonConstraints(
    const MatX2d& A,  // NOLINT(readability-identifier-naming)
    const VecXd& b, const Vec2d& taylor, const Vec2d& taylor_shift,
    Stage* stage, int* cur_row) {
  constexpr int kColsOfA = 2;
  constexpr int kColsOfThetaCoeff = 1;
  const auto theta_coeff = A * taylor;
  const VecXd upper_limit = b - A * taylor_shift;
  stage->constrains_mat.C.block(*cur_row, kStateXIndex, A.rows(), kColsOfA) = A;
  stage->constrains_mat.C.block(*cur_row, kStateThetaIndex, A.rows(),
                                kColsOfThetaCoeff) = theta_coeff;
  stage->constrains_mat.du.segment(*cur_row, A.rows()) = upper_limit;
  *cur_row += A.rows();
}
void SetConvexPolygonConstraints(
    const MatX2d& A,  // NOLINT(readability-identifier-naming)
    const VecXd& b, const MpcState& x,
    const VehicleGeometryParamsProto& veh_geo_params,
    const VehicleOctagonModelParamsProto& vehicle_model_params,
    const bool has_mirrors, Stage* stage) {
  SCOPED_QTRACE("SQP Global Smoother: SetConvexPolygonConstraints");
  const int num_of_hyperplanes = has_mirrors ? 10 : 8;
  constexpr double kNegInf = -2000.0;
  const int total_rows = num_of_hyperplanes * A.rows();
  stage->constrains_mat.C.setZero(total_rows, kStateSize);
  stage->constrains_mat.D.setZero(total_rows, kControlSize);
  stage->constrains_mat.dl.setConstant(total_rows, kNegInf);
  stage->constrains_mat.du.setZero(total_rows);
  stage->ng = total_rows;
  const double rac_to_front = veh_geo_params.front_edge_to_center();
  const double rac_to_rear = veh_geo_params.back_edge_to_center();
  const double half_width = 0.5 * veh_geo_params.width();
  const auto theta = x.theta;
  const Vec2d unit = Vec2d::FastUnitFromAngle(x.theta);
  const Vec2d unit_diff = unit.Perp();
  const Vec2d perp = unit.Perp();
  const Vec2d perp_diff = -unit;
  const Vec2d h_front = unit * rac_to_front;
  const Vec2d h_rear = unit * rac_to_rear;
  const Vec2d w = perp * half_width;
  const Vec2d h_front_taylor = unit_diff * rac_to_front;
  const Vec2d h_rear_taylor = unit_diff * rac_to_rear;
  const Vec2d w_taylor = perp_diff * half_width;
  const double front_corner_side_length =
      vehicle_model_params.front_corner_side_length();
  const double rear_corner_side_length =
      vehicle_model_params.rear_corner_side_length();

  // FRONT_LEFT:
  // center + h + (w-front_corner_side_length),
  // center + (h-front_corner_side_length) + w
  // REAR_LEFT:
  // center - h + (w-rear_corner_side_length),
  // center - (h-rear_corner_side_length) + w
  // REAR_RIGHT:
  // center - h - (w-rear_corner_side_length),
  // center - (h-rear_corner_side_length) - w
  // FRONT_RIGHT:
  // center + h - (w-front_corner_side_length),
  // center + (h-front_corner_side_length) + w

  // Front_left
  // x1 = x + cos_theta * rac_to_front - sin_theta * half_width
  //    = x + (cos_theta0 - sin_theta0*(theta - theta0)) * rac_to_front
  //      + (-sin_theta0 - cos_theta0 * (theta - theta0)) * half_width
  //    = x + (-sin_theta0 * rac_to_front - cos_theta0 * half_width) * theta
  //      + ...
  // y1 = y + sin_theta * rac_to_front + cos_theta * half_width
  //    = y + (cos_theta0 * rac_to_front - sin_theta0 * half_width) * theta +
  //    ...
  int cur_row = 0;
  const Vec2d front_left_front_taylor =
      h_front_taylor + w_taylor - perp_diff * front_corner_side_length;
  const Vec2d front_left_front_taylor_shift = h_front + w -
                                              front_left_front_taylor * theta -
                                              perp * front_corner_side_length;
  SetPointInConvexPolygonConstraints(A, b, front_left_front_taylor,
                                     front_left_front_taylor_shift, stage,
                                     &cur_row);

  const Vec2d front_left_left_taylor =
      h_front_taylor + w_taylor - unit_diff * front_corner_side_length;
  const Vec2d front_left_left_taylor_shift = h_front + w -
                                             front_left_left_taylor * theta -
                                             unit * front_corner_side_length;
  SetPointInConvexPolygonConstraints(A, b, front_left_left_taylor,
                                     front_left_left_taylor_shift, stage,
                                     &cur_row);

  // REAR_LEFT
  // x2 = x - cos_theta * rac_to_rear - sin_theta * half_width
  //    = x + (sin_theta0 * rac_to_rear -cos_theta * half_width) * theta +
  // ...
  // y2 = y - sin_theta * rac_to_rear + cos_theta * half_width
  //    = y + (-cos_theta0 * rac_to_rear - sin_theta * half_width) * theta +
  // ...
  const Vec2d rear_left_rear_taylor =
      -h_rear_taylor + w_taylor - perp_diff * rear_corner_side_length;
  const Vec2d rear_left_rear_taylor_shift = -h_rear + w -
                                            rear_left_rear_taylor * theta -
                                            perp * rear_corner_side_length;
  SetPointInConvexPolygonConstraints(A, b, rear_left_rear_taylor,
                                     rear_left_rear_taylor_shift, stage,
                                     &cur_row);

  const Vec2d rear_left_left_taylor =
      -h_rear_taylor + w_taylor + unit_diff * rear_corner_side_length;
  const Vec2d rear_left_left_taylor_shift = -h_rear + w -
                                            rear_left_left_taylor * theta +
                                            unit * rear_corner_side_length;
  SetPointInConvexPolygonConstraints(A, b, rear_left_left_taylor,
                                     rear_left_left_taylor_shift, stage,
                                     &cur_row);

  // REAR_RIGHT
  // x3 = x - cos_theta * rac_to_rear + sin_theta * half_width
  //    = x + (sin_theta0 * rac_to_rear + cos_theta0 * half_width) * theta +
  // ...
  // y3 = y - sin_theta * rac_to_rear - cos_theta * half_width
  //    = y + (-cos_theta0 * rac_to_rear + cos_theta * half_width) * theta +
  // ...
  const Vec2d rear_right_rear_taylor =
      -h_rear_taylor - w_taylor + perp_diff * rear_corner_side_length;
  const Vec2d rear_right_rear_taylor_shift = -h_rear - w -
                                             rear_right_rear_taylor * theta +
                                             perp * rear_corner_side_length;
  SetPointInConvexPolygonConstraints(A, b, rear_right_rear_taylor,
                                     rear_right_rear_taylor_shift, stage,
                                     &cur_row);

  const Vec2d rear_right_right_taylor =
      -h_rear_taylor - w_taylor + unit_diff * rear_corner_side_length;
  const Vec2d rear_right_right_taylor_shift = -h_rear - w -
                                              rear_right_right_taylor * theta +
                                              unit * rear_corner_side_length;
  SetPointInConvexPolygonConstraints(A, b, rear_right_right_taylor,
                                     rear_right_right_taylor_shift, stage,
                                     &cur_row);

  // FRONT_RIGHT
  // x4 = x + cos_theta * rac_to_front + sin_theta * half_width
  //    = x + (-sin_theta0 * rac_to_front + cos_theta0 * half_width) * theta
  // + ...
  // y4 = y + sin_theta * rac_to_front - cos_theta * half_width
  //    = y + (cos_theta0 * rac_to_front + sin_theta0 * half_width) * theta +
  // ...
  const Vec2d front_right_front_taylor =
      h_front_taylor - w_taylor + perp_diff * front_corner_side_length;
  const Vec2d front_right_front_taylor_shift =
      h_front - w - front_right_front_taylor * theta +
      perp * front_corner_side_length;
  SetPointInConvexPolygonConstraints(A, b, front_right_front_taylor,
                                     front_right_front_taylor_shift, stage,
                                     &cur_row);

  const Vec2d front_right_right_taylor =
      h_front_taylor - w_taylor - unit_diff * front_corner_side_length;
  const Vec2d front_right_right_taylor_shift =
      h_front - w - front_right_right_taylor * theta -
      unit * front_corner_side_length;
  SetPointInConvexPolygonConstraints(A, b, front_right_right_taylor,
                                     front_right_right_taylor_shift, stage,
                                     &cur_row);

  if (!has_mirrors) {
    return;
  }
  // MIRROR:
  // center + offset_x + offset_y
  // center + offset_x - offset_y
  const double mirror_offset_x = vehicle_model_params.mirror_offset_x();
  const double mirror_offset_y = vehicle_model_params.mirror_offset_y();
  const Vec2d offset_x = unit * mirror_offset_x;
  const Vec2d offset_y = perp * mirror_offset_y;
  const Vec2d offset_x_taylor = unit_diff * mirror_offset_x;
  const Vec2d offset_y_taylor = perp_diff * mirror_offset_y;
  const auto b_for_mirror =
      b - VecXd::Ones(b.size()) * (vehicle_model_params.mirror_radius());
  // MIRROR_LEFT
  // left_mirror_x = x + cos_theta * mirror_offset_x - sin_theta *
  // mirror_offset_y
  //    = x + (-sin_theta0 * mirror_offset_x - cos_theta0 * mirror_offset_y) *
  //    theta
  // + ...
  // left_mirror_y = y + sin_theta * mirror_offset_x + cos_theta *
  // mirror_offset_y
  //    = y + (cos_theta0 * mirror_offset_x - sin_theta0 * mirror_offset_y) *
  //    theta +
  //    ...
  const Vec2d left_mirror_taylor = offset_x_taylor + offset_y_taylor;
  const Vec2d left_mirror_taylor_shift =
      offset_x + offset_y - left_mirror_taylor * theta;
  SetPointInConvexPolygonConstraints(A, b_for_mirror, left_mirror_taylor,
                                     left_mirror_taylor_shift, stage, &cur_row);

  // MIRROR_RIGHT
  // right_mirror_x = x + cos_theta * mirror_offset_x + sin_theta *
  // mirror_offset_y
  //    = x + (-sin_theta0 * mirror_offset_x + cos_theta0 * mirror_offset_y) *
  //    theta
  // + ...
  // right_mirror_y = y + sin_theta * mirror_offset_x - cos_theta *
  // mirror_offset_y
  //    = y + (cos_theta0 * mirror_offset_x + sin_theta0 * mirror_offset_y) *
  //    theta
  // + ...
  const Vec2d right_mirror_taylor = offset_x_taylor - offset_y_taylor;
  const Vec2d right_mirror_taylor_shift =
      offset_x - offset_y - right_mirror_taylor * theta;
  SetPointInConvexPolygonConstraints(A, b_for_mirror, right_mirror_taylor,
                                     right_mirror_taylor_shift, stage,
                                     &cur_row);
}

void SetAffineConstraint(const MpcState& ref_path, double max_v, double min_v,
                         double max_acc, double min_acc, double max_k,
                         double max_k_rate, Stage* stage) {
  constexpr double kMaxPosDeviation = 5.0;                          // m.
  constexpr double kMaxAngleDeviation = 10.0 / 180.0 * M_PI;        // rad.
  stage->l_bounds_x[kStateXIndex] = ref_path.x - kMaxPosDeviation;  // x.
  stage->l_bounds_x[kStateYIndex] = ref_path.y - kMaxPosDeviation;  // y.
  stage->l_bounds_x[kStateVIndex] = min_v;                          // v.
  stage->l_bounds_x[kStateThetaIndex] =
      ref_path.theta - kMaxAngleDeviation;            // theta.
  stage->l_bounds_x[kStateKappaIndex] = -max_k;       // kappa.
  stage->l_bounds_u[kControlAIndex] = min_acc;        // a.
  stage->l_bounds_u[kControlPsiIndex] = -max_k_rate;  // psi.

  stage->u_bounds_x[kStateXIndex] = ref_path.x + kMaxPosDeviation;  // x.
  stage->u_bounds_x[kStateYIndex] = ref_path.y + kMaxPosDeviation;  // y.
  stage->u_bounds_x[kStateVIndex] = max_v;                          // v.
  stage->u_bounds_x[kStateThetaIndex] =
      ref_path.theta + kMaxAngleDeviation;           // theta.
  stage->u_bounds_x[kStateKappaIndex] = max_k;       // kappa.
  stage->u_bounds_u[kControlAIndex] = max_acc;       // a.
  stage->u_bounds_u[kControlPsiIndex] = max_k_rate;  // psi.

  stage->idx_x.resize(kStateSize);
  stage->idx_u.resize(kControlSize);
  for (int i = 0; i < kStateSize; ++i) {
    stage->idx_x[i] = i;
  }
  for (int i = 0; i < kControlSize; ++i) {
    stage->idx_u[i] = i;
  }
  stage->nbx = kStateSize;
  stage->nbu = kControlSize;
}

void SetStartAffineConstraint(double max_acc, double min_acc, double max_k_rate,
                              Stage* stage) {
  stage->l_bounds_u[kControlAIndex] = min_acc;        // a.
  stage->l_bounds_u[kControlPsiIndex] = -max_k_rate;  // psi.

  stage->u_bounds_u[kControlAIndex] = max_acc;       // a.
  stage->u_bounds_u[kControlPsiIndex] = max_k_rate;  // psi.

  stage->idx_x.resize(0);
  stage->idx_u.resize(kControlSize);
  for (int i = 0; i < kControlSize; ++i) {
    stage->idx_u[i] = i;
  }
  stage->nbx = 0;
  stage->nbu = kControlSize;
}

void SetEndAffineConstraint(const MpcState& ref_path, double max_k_rate,
                            Stage* stage) {
  constexpr double kThetaMargin = kLargerMargin / 180.0 * M_PI;  // rad.
  stage->l_bounds_x[kStateXIndex] = ref_path.x - kMargin;        // x.
  stage->l_bounds_x[kStateYIndex] = ref_path.y - kMargin;        // y.
  stage->l_bounds_x[kStateVIndex] = -kLargerMargin;              // v.
  stage->l_bounds_x[kStateThetaIndex] =
      ref_path.theta - kThetaMargin;                  // theta.
  stage->l_bounds_x[kStateKappaIndex] = -max_k_rate;  // kappa.

  stage->u_bounds_x[kStateXIndex] = ref_path.x + kMargin;  // x.
  stage->u_bounds_x[kStateYIndex] = ref_path.y + kMargin;  // y.
  stage->u_bounds_x[kStateVIndex] = kLargerMargin;         // v.
  stage->u_bounds_x[kStateThetaIndex] =
      ref_path.theta + kThetaMargin;                 // theta.
  stage->u_bounds_x[kStateKappaIndex] = max_k_rate;  // kappa.

  stage->idx_x.resize(kStateSize);
  stage->idx_u.resize(0);
  for (int i = 0; i < kStateSize; ++i) {
    stage->idx_x[i] = i;
  }
  stage->nbx = kStateSize;
  stage->nbu = 0;
}

void SetCost(const PathPoint& ref_path, const Vec2d& ego_pos,
             const SqpSmootherParamsProto& sqp_smoother_params, Stage* stage) {
  stage->cost_mat.Q.setZero();
  stage->cost_mat.q.setZero();
  stage->cost_mat.R.setZero();
  stage->cost_mat.r.setZero();
  stage->cost_mat.Q(kStateXIndex, kStateXIndex) =
      sqp_smoother_params.weight_pos();  // x.
  stage->cost_mat.Q(kStateYIndex, kStateYIndex) =
      sqp_smoother_params.weight_pos();  // y.
  stage->cost_mat.Q(kStateVIndex, kStateVIndex) =
      sqp_smoother_params.weight_speed();  // v.
  // theta.
  stage->cost_mat.Q(kStateThetaIndex, kStateThetaIndex) =
      sqp_smoother_params.weight_theta();
  // kappa.
  stage->cost_mat.Q(kStateKappaIndex, kStateKappaIndex) =
      sqp_smoother_params.weight_kappa();
  stage->cost_mat.q(kStateXIndex) =
      sqp_smoother_params.weight_pos() * (-ref_path.x() + ego_pos.x());  // x.
  stage->cost_mat.q(kStateYIndex) =
      sqp_smoother_params.weight_pos() * (-ref_path.y() + ego_pos.y());  // y.
  // theta.
  stage->cost_mat.q(kStateThetaIndex) =
      sqp_smoother_params.weight_theta() * -ref_path.theta();
  stage->cost_mat.R(kControlAIndex, kControlAIndex) =
      sqp_smoother_params.weight_acc();  // a.
  stage->cost_mat.R(kControlPsiIndex, kControlPsiIndex) =
      sqp_smoother_params.weight_psi();  // psi.
}

void SetSlackVariables(const SqpSmootherParamsProto& sqp_smoother_params,
                       Stage* stage) {
  stage->l_bounds_s.setZero(stage->ng);
  stage->u_bounds_s.setZero(stage->ng);
  stage->cost_mat.Z.setIdentity(stage->ng, stage->ng);
  stage->cost_mat.Z *= sqp_smoother_params.weight_slack_quadratic();
  stage->cost_mat.z.setOnes(stage->ng);
  stage->cost_mat.z *= sqp_smoother_params.weight_slack_linear();
  stage->ns = stage->ng;
  stage->ns = stage->ng;
  stage->nsbx = 0;
  stage->nsbu = 0;
  stage->nsg = stage->ns;
  stage->idx_s.resize(stage->ns);
  const double start_idx = stage->nbx + stage->nbu;
  for (int i = 0; i < stage->ns; ++i) {
    stage->idx_s[i] = i + start_idx;
  }
}

std::pair<std::vector<std::pair<double, Segment2d>>,
          std::vector<std::pair<double, Polygon2d>>>
GetBoundaryAndObjectsInfo(
    const FreespaceMap& freespace_map,
    absl::Span<const SpacetimeObjectTrajectory* const> stalled_object_trajs,
    const FreespacePathFinderParamsProto& freespace_path_finder_params) {
  FUNC_QTRACE();
  std::vector<std::pair<double, Segment2d>> boundaries_info;
  std::vector<std::pair<double, Polygon2d>> objects_info;
  const double buffer_for_objects =
      std::max(freespace_path_finder_params.object_lateral_buffer(),
               freespace_path_finder_params.object_longitudinal_buffer());
  for (const auto& boundary : freespace_map.boundaries) {
    const auto [lat_buffer, lon_buffer] =
        GetVehicleBufferForBoundary(freespace_path_finder_params, boundary);
    const double buffer_for_boundary = std::max(lat_buffer, lon_buffer);
    for (int i = 0; i + 1 < boundary.points.size(); ++i) {
      boundaries_info.push_back(std::pair<double, Segment2d>(
          buffer_for_boundary,
          Segment2d(boundary.points[i], boundary.points[i + 1])));
    }
  }
  for (const auto& traj_ptr : stalled_object_trajs) {
    objects_info.push_back(
        std::pair<double, Polygon2d>(buffer_for_objects, traj_ptr->contour()));
  }
  return std::pair<std::vector<std::pair<double, Segment2d>>,
                   std::vector<std::pair<double, Polygon2d>>>(
      std::move(boundaries_info), std::move(objects_info));
}

void SendGlobalSmoothedPathToCanvas(const std::vector<OptVariables>& path,
                                    const Vec2d& ego_pos,
                                    const std::string& name) {
  vis::Canvas& canvas_path = vis::vantage::GetCanvasClient()->GetCanvas(name);
  for (int i = 0; i + 1 < path.size(); ++i) {
    const auto color =
        path[i + 1].xk.v > 0.0 ? vis::Color::kYellow : vis::Color::kBlue;
    canvas_path.DrawLine(
        Vec3d(path[i].xk.x + ego_pos.x(), path[i].xk.y + ego_pos.y(), 0.0),
        Vec3d(path[i + 1].xk.x + ego_pos.x(), path[i + 1].xk.y + ego_pos.y(),
              0.0),
        color);
  }
  for (int i = 0; i < path.size(); ++i) {
    canvas_path.DrawCircle(
        Vec3d(path[i].xk.x + ego_pos.x(), path[i].xk.y + ego_pos.y(), 0.0),
        0.05, vis::Color::kMediumPurple);
  }
}

void SendVehicleContourToCanvas(
    const std::vector<OptVariables>& path,
    const VehicleGeometryParamsProto& veh_geo_params,
    const VehicleOctagonModelParamsProto& vehicle_model_params,
    const Vec2d& ego_pos, const std::string& name) {
  constexpr int kNumOfPoints = 8;
  const double rac_to_center =
      veh_geo_params.length() * 0.5 - veh_geo_params.back_edge_to_center();
  for (int i = 0; i < path.size(); ++i) {
    vis::Canvas& canvas_path = vis::vantage::GetCanvasClient()->GetCanvas(
        absl::StrFormat("%s/contour_%04d", name, i));
    const auto av_xy = Vec2d(path[i].xk.x, path[i].xk.y) + ego_pos;
    const Vec2d tangent = Vec2d::FastUnitFromAngle(path[i].xk.theta);
    const Vec2d center = av_xy + rac_to_center * tangent;
    const VehicleOctagonShape av_shape(veh_geo_params, vehicle_model_params,
                                       av_xy, center, tangent, path[i].xk.theta,
                                       0.5 * veh_geo_params.length(),
                                       0.5 * veh_geo_params.width());
    const auto corners =
        av_shape.GetCornersWithBufferCounterClockwise(0.0, 0.0);
    const auto color = vis::Color::kCyan;
    for (int i = 0; i < kNumOfPoints; ++i) {
      const auto& cur_corner = corners[i];
      const auto& next_corner = corners[(i + 1) % kNumOfPoints];
      canvas_path.DrawLine(Vec3d(cur_corner.x(), cur_corner.y(), 0.0),
                           Vec3d(next_corner.x(), next_corner.y(), 0.0), color);
    }
    if (av_shape.left_mirror().has_value()) {
      canvas_path.DrawCircle(Vec3d(av_shape.left_mirror()->center()),
                             vehicle_model_params.mirror_radius(), color);
    }
    if (av_shape.right_mirror().has_value()) {
      canvas_path.DrawCircle(Vec3d(av_shape.right_mirror()->center()),
                             vehicle_model_params.mirror_radius(), color);
    }
  }
}

void SendConvexRegionToCanvas(
    const std::vector<LineSegmentDecomp>& line_segment_decomps) {
  int poly_num = 0;
  for (const auto& decomp : line_segment_decomps) {
    const auto vertices = CalVertices(decomp.convex_region());
    vis::Canvas& canvas_path =
        vis::vantage::GetCanvasClient()->GetCanvas(absl::StrFormat(
            "freespace/convex_region_generator/poly_%04d", poly_num++));
    for (int i = 0; i < vertices.size(); ++i) {
      const auto color = vis::Color::kLightOrange;
      canvas_path.DrawLine(Vec3d(vertices[i](0), vertices[i](1), 0.0),
                           Vec3d(vertices[(i + 1) % vertices.size()](0),
                                 vertices[(i + 1) % vertices.size()](1), 0.0),
                           color);
    }
  }
}

ModelInput MakeModelInput(
    const std::vector<DirectionalPath>& init_path, const Vec2d& ego_pos,
    const VehicleGeometryParamsProto& veh_geo_params,
    const VehicleDriveParamsProto& vehicle_drive_params,
    const SqpSmootherParamsProto_ModelInputProto& model_input_params) {
  ModelInput model_input;
  model_input.max_forward_speed = model_input_params.max_pos_velocity();
  model_input.max_reverse_speed = model_input_params.max_neg_velocity();
  model_input.max_accel = model_input_params.max_acc();
  model_input.min_accel = -model_input_params.max_acc();
  model_input.max_curvature =
      model_input_params.kappa_slack_ratio() *
      ComputeCenterMaxCurvature(veh_geo_params, vehicle_drive_params);
  model_input.max_curvature_rate = model_input_params.max_curvature_rate();
  model_input.switch_point_indices.reserve(init_path.size());
  for (int i = 0, init_path_size = init_path.size(); i < init_path_size; ++i) {
    const auto& curr_path = init_path[i];
    QCHECK_GE(curr_path.path.size(), 2);
    const double end_s = curr_path.path.back().s();
    double accumulated_s = 0.0;
    // Make sure the last path point is sampled.
    while (true) {
      PathPoint pt = curr_path.path.Evaluate(accumulated_s);
      double v = model_input_params.default_pos_velocity();
      model_input.forwards.push_back(curr_path.forward);
      if (!curr_path.forward) {
        pt.set_theta(pt.theta() + M_PI);
        pt.set_kappa(-pt.kappa());
        v = model_input_params.default_neg_velocity();
      }
      if (model_input.path.empty()) {
        model_input.path.push_back(pt);
        model_input.initial_xs.push_back(MpcState{.x = pt.x() - ego_pos.x(),
                                                  .y = pt.y() - ego_pos.y(),
                                                  .v = 0.0,
                                                  .theta = pt.theta(),
                                                  .k = pt.kappa()});
        model_input.initial_us.push_back(MpcControl{.a = 0.0, .psi = 0.0});
        model_input.switch_point_indices.push_back(0);
      }
      const double last_pt_theta = model_input.path.back().theta();
      while (pt.theta() - last_pt_theta > M_PI) {
        pt.set_theta(pt.theta() - 2.0 * M_PI);
      }
      while (pt.theta() - last_pt_theta < -M_PI) {
        pt.set_theta(pt.theta() + 2.0 * M_PI);
      }

      model_input.initial_xs.push_back(MpcState{.x = pt.x() - ego_pos.x(),
                                                .y = pt.y() - ego_pos.y(),
                                                .v = v,
                                                .theta = pt.theta(),
                                                .k = pt.kappa()});
      model_input.initial_us.push_back(MpcControl{.a = 0.0, .psi = 0.0});
      model_input.path.push_back(std::move(pt));

      if (accumulated_s >= end_s) break;
      accumulated_s += model_input_params.step_length();
    }
    if (i != init_path_size - 1) {
      model_input.switch_point_indices.push_back(model_input.path.size() - 1);
    }
  }
  return model_input;
}

absl::StatusOr<std::vector<OptVariables>> Optimize(
    const ConvexRegionsGenerator& convex_region_generator,
    const VehicleGeometryParamsProto& veh_geo_params,
    const SqpSmootherParamsProto& sqp_smoother_params,
    const VehicleOctagonModelParamsProto& vehicle_model_params,
    const Vec2d& ego_pos, const ModelInput& input, const DiscretizedPath& path,
    std::vector<MpcState> initial_xs, std::vector<MpcControl> initial_us,
    int num_of_knots, double dt) {
  constexpr int kMaxSqpIterNum = 2;
  const auto end_ref = initial_xs.back();
  const bool has_mirrors =
      veh_geo_params.has_left_mirror() && veh_geo_params.has_right_mirror();
  std::vector<Stage> stages;
  stages.resize(num_of_knots);
  HpipmInterface solver;
  std::vector<OptVariables> stitched_path;
  const auto& linear_cons = convex_region_generator.linear_constraints();
  for (int k = 0; k < kMaxSqpIterNum; ++k) {
    for (int i = 0; i < num_of_knots; ++i) {
      if (i == 0) {
        SetStartAffineConstraint(input.max_accel, input.min_accel,
                                 input.max_curvature_rate, &stages[i]);
      } else if (i == num_of_knots - 1) {
        SetEndAffineConstraint(end_ref, input.max_curvature_rate, &stages[i]);
      } else {
        SetAffineConstraint(initial_xs[i], input.max_forward_speed,
                            -input.max_reverse_speed, input.max_accel,
                            input.min_accel, input.max_curvature,
                            input.max_curvature_rate, &stages[i]);
      }
      SetCost(path[i], ego_pos, sqp_smoother_params, &stages[i]);
      if (i < num_of_knots - 1) {
        SetModelConstraints(dt, initial_xs[i], initial_us[i], &stages[i]);
      }
      const auto& linear_con = linear_cons[i];
      if (i != 0) {
        SetConvexPolygonConstraints(
            linear_con.A(), linear_con.b(), initial_xs[i], veh_geo_params,
            vehicle_model_params, has_mirrors, &stages[i]);
      }
      if (i != 0) {
        SetSlackVariables(sqp_smoother_params, &stages[i]);
      }
      stages[i].xk = ToMpcStateArray(initial_xs[i]);
      stages[i].uk = ToMpcControlArray(initial_us[i]);
    }
    for (const auto& index : input.switch_point_indices) {
      QCHECK_LT(index, num_of_knots - 1);
      SetInplaceSteerConstraints(&stages[index]);
    }
    auto output = solver.SolveMpc(stages, initial_xs[0]);
    if (!output.ok()) {
      QLOG(WARNING) << output.status();
      return absl::UnavailableError("SQP optimize fails.");
    }
    for (int j = 0; j < num_of_knots; ++j) {
      initial_xs[j] = (*output)[j].xk;
      initial_us[j] = (*output)[j].uk;
    }
    if (FLAGS_sqp_global_smoother_enable_model_error_debug) {
      std::array<double, kStateSize> max_errors = {0.0};
      for (int i = 0; i < num_of_knots - 1; ++i) {
        MpcState x_next = RK4(initial_xs[i], initial_us[i], dt);
        if (std::find(input.switch_point_indices.begin(),
                      input.switch_point_indices.end(),
                      i) != input.switch_point_indices.end()) {
          continue;
        }
        max_errors[kStateXIndex] =
            std::max(max_errors[kStateXIndex],
                     std::fabs(initial_xs[i + 1].x - x_next.x));
        max_errors[kStateYIndex] =
            std::max(max_errors[kStateYIndex],
                     std::fabs(initial_xs[i + 1].y - x_next.y));
        max_errors[kStateVIndex] =
            std::max(max_errors[kStateVIndex],
                     std::fabs(initial_xs[i + 1].v - x_next.v));
        max_errors[kStateThetaIndex] =
            std::max(max_errors[kStateThetaIndex],
                     std::fabs(initial_xs[i + 1].theta - x_next.theta));
        max_errors[kStateKappaIndex] =
            std::max(max_errors[kStateKappaIndex],
                     std::fabs(initial_xs[i + 1].k - x_next.k));
      }
      VLOG(3) << "SQP global smoother model error in iter " << k << ":";
      VLOG(3) << "x error :" << max_errors[kStateXIndex];
      VLOG(3) << "y error :" << max_errors[kStateYIndex];
      VLOG(3) << "v error :" << max_errors[kStateVIndex];
      VLOG(3) << "theta error :" << max_errors[kStateThetaIndex];
      VLOG(3) << "k error :" << max_errors[kStateKappaIndex];
    }
    if (FLAGS_sqp_global_smoother_send_global_smooth_path_to_canvas) {
      SendGlobalSmoothedPathToCanvas(
          *output, ego_pos,
          "freespace/global_smooth_path/path_" + std::to_string(k));
    }
    stitched_path = std::move(*output);
  }
  if (FLAGS_sqp_global_smoother_send_vehicle_contour_to_canvas) {
    SendVehicleContourToCanvas(stitched_path, veh_geo_params,
                               vehicle_model_params, ego_pos,
                               "freespace/vehicle_contour");
  }
  return stitched_path;
}
}  // namespace

absl::StatusOr<std::vector<DirectionalPath>> SmoothGlobalPath(
    const std::vector<DirectionalPath>& init_path,
    const FreespaceMap& freespace_map,
    absl::Span<const SpacetimeObjectTrajectory* const> stalled_object_trajs,
    const VehicleGeometryParamsProto& veh_geo_params,
    const VehicleDriveParamsProto& vehicle_drive_params,
    const SqpSmootherParamsProto& sqp_smoother_params,
    const VehicleOctagonModelParamsProto& vehicle_model_params,
    const FreespacePathFinderParamsProto& freespace_path_finder_params) {
  SCOPED_QTRACE("SQP Global Smoother: SmoothGlobalPath");
  const auto smooth_start = absl::Now();
  constexpr double kDeltaT = 1.0;        // sec.
  constexpr double kLocalObbSize = 5.0;  // m.
  // Make inputs and constraints.
  const Vec2d ego_pos(init_path[0].path[0].x(), init_path[0].path[0].y());
  ConvexRegionsGenerator convex_region_generator;
  const auto input =
      MakeModelInput(init_path, ego_pos, veh_geo_params, vehicle_drive_params,
                     sqp_smoother_params.model_params());
  const int num_of_knots = input.path.size();
  const auto [boundaries_info, objects_info] = GetBoundaryAndObjectsInfo(
      freespace_map, stalled_object_trajs, freespace_path_finder_params);
  const auto dilate_start = absl::Now();
  convex_region_generator.Dilate(input.path, input.forwards, boundaries_info,
                                 objects_info, kLocalObbSize, kLocalObbSize,
                                 veh_geo_params.front_edge_to_center(),
                                 veh_geo_params.back_edge_to_center(),
                                 /*set_start_to_origin=*/true);
  const auto dilate_time =
      absl::ToDoubleMilliseconds(absl::Now() - dilate_start);
  if (FLAGS_sqp_global_smoother_send_convex_region_to_canvas) {
    SendConvexRegionToCanvas(convex_region_generator.line_segment_decomps());
  }

  // Optimize.
  const auto stitched_path_or =
      Optimize(convex_region_generator, veh_geo_params, sqp_smoother_params,
               vehicle_model_params, ego_pos, input, input.path,
               input.initial_xs, input.initial_us, num_of_knots, kDeltaT);
  if (!stitched_path_or.ok()) {
    QLOG(WARNING) << stitched_path_or.status();
    return absl::UnavailableError("SQP global path smooth fails.");
  }
  const auto& stitched_path = *stitched_path_or;
  const auto smooth_time =
      absl::ToDoubleMilliseconds(absl::Now() - smooth_start);
  VLOG(1) << "Convex region generation time: " << dilate_time;
  VLOG(1) << "SQP global path smooth time: " << smooth_time;

  // Make Output.
  std::vector<std::vector<PathPoint>> paths(1);
  std::deque<bool> gears;
  double s = 0;
  for (int i = 1; i < stitched_path.size(); ++i) {
    PathPoint pt;
    pt.set_x(stitched_path[i].xk.x + ego_pos.x());
    pt.set_y(stitched_path[i].xk.y + ego_pos.y());
    pt.set_theta(stitched_path[i].xk.v > 0.0
                     ? NormalizeAngle(stitched_path[i].xk.theta)
                     : NormalizeAngle(stitched_path[i].xk.theta + M_PI));
    pt.set_kappa(stitched_path[i].xk.v > 0.0 ? stitched_path[i].xk.k
                                             : -stitched_path[i].xk.k);
    pt.set_s(s);
    double ds = std::fabs(stitched_path[i].xk.v * kDeltaT +
                          0.5 * stitched_path[i].uk.a * Sqr(kDeltaT));
    s += std::max(kEps, ds);
    paths.back().push_back(pt);
    // Switch gear.
    if (i > 0 && i + 1 < stitched_path.size() &&
        (stitched_path[i].xk.v > 0.0) != (stitched_path[i + 1].xk.v > 0.0)) {
      gears.push_back(stitched_path[i].xk.v > 0.0);
      paths.push_back({});
      s = 0.0;
    }
  }
  gears.push_back(stitched_path.back().xk.v > 0.0);

  std::vector<DirectionalPath> res;
  res.reserve(paths.size());
  for (int i = 0; i < paths.size(); ++i) {
    if (paths[i].size() < 2) continue;
    res.push_back({DiscretizedPath(std::move(paths[i])), gears[i]});
  }
  return res;
}
}  // namespace sqp_global_smoother
}  // namespace planner
}  // namespace qcraft

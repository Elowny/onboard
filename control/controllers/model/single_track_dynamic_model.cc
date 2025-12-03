#include "onboard/control/controllers/model/single_track_dynamic_model.h"

#include <algorithm>
#include <cmath>
#include <ostream>

#include "Eigen/Core"
#include "Eigen/LU"

#include "onboard/control/control_defs.h"
#include "onboard/eval/qevent.h"
#include "onboard/eval/qevent_base.h"
#include "onboard/lite/check.h"
#include "onboard/lite/logging.h"
#include "onboard/math/util.h"

namespace qcraft::control {

namespace {

constexpr DiscretizationMethod kMethod = DiscretizationMethod::kEular;

struct CoefMatrices {
  std::vector<Eigen::MatrixXd> coef_matrix_a;
  std::vector<Eigen::MatrixXd> coef_matrix_w;
  Eigen::MatrixXd coef_matrix_b;
  Eigen::MatrixXd coef_matrix_c;
  Eigen::MatrixXd coef_matrix_c_inverse;
  Eigen::MatrixXd coef_matrix_v;
};

struct Coefficient {
  double f_1 = 0.0;
  double f_2 = 0.0;
  double f_3 = 0.0;
  double g_1 = 0.0;
  double g_2 = 0.0;
  double g_3 = 0.0;
  double g_4 = 0.0;
  double j_1 = 0.0;
  double j_2 = 0.0;
  double j_3 = 0.0;
  double j_4 = 0.0;
};

std::vector<Coefficient> CalculateCoef(double wheel_base, double vehicle_length,
                                       const DynamicModelMeasurement& measure,
                                       const DynamicModelConfProto& conf) {
  std::vector<Coefficient> coef_vec;
  coef_vec.reserve(measure.Horizon());

  // constants:
  const double sin_yaw = std::sin(measure.yaw);
  const double cos_yaw = std::cos(measure.yaw);
  const double sin_roll = std::sin(measure.roll);

  if (!InRange(conf.geo_params().cg_ratio(), /*low credible limit*/ 0.1,
               /*high credible limit*/ 0.9)) {
    QLOG(FATAL) << "The dynamic model cg ratio, "
                << conf.geo_params().cg_ratio()
                << ", on wheel base is incorrect.";
  }

  const double l_f = wheel_base * conf.geo_params().cg_ratio();
  const double l_r = wheel_base * (1 - conf.geo_params().cg_ratio());

  const double m_reciprocal = 1.0 / conf.mass();
  const double c_af_equivalent =
      conf.tire_params().c_af() * std::cos(measure.psi);
  const double c_ar = conf.tire_params().c_ar();
  const double inertia_z = 0.1478 * conf.mass() * wheel_base * vehicle_length;
  const double i_z_reciprocal = 1.0 / inertia_z;

  Coefficient coef;
  coef.j_1 = -sin_yaw;
  coef.j_2 = cos_yaw;
  coef.f_3 = 2.0 * c_af_equivalent * l_f * i_z_reciprocal;
  coef.g_3 = 2.0 * c_af_equivalent * m_reciprocal;
  coef.g_4 = kGravitationalAcceleration * sin_roll;

  for (int i = 0; i < measure.Horizon(); ++i) {
    // Only apply to forward driving scenarios.
    const double v = measure.v[i];
    const double v_reciprocal = 1.0 / std::max(measure.v[i], kDmSpeedThreshold);

    coef.j_3 = v * sin_yaw + (measure.u_0 + measure.omega * l_r) * cos_yaw;
    coef.j_4 = v * cos_yaw - (measure.u_0 + measure.omega * l_r) * sin_yaw;
    coef.f_1 = -2.0 * i_z_reciprocal * v_reciprocal *
               (c_af_equivalent * Sqr(l_f) + c_ar * Sqr(l_r));
    coef.f_2 = 2.0 * i_z_reciprocal * v_reciprocal *
               (-c_af_equivalent * l_f + c_ar * l_r);
    coef.g_1 = -v + (-2.0 * c_af_equivalent * l_f + 2.0 * c_ar * l_r) *
                        m_reciprocal * v_reciprocal;
    coef.g_2 =
        (-2.0 * c_af_equivalent - 2.0 * c_ar) * m_reciprocal * v_reciprocal;

    coef_vec.push_back(coef);
  }

  return coef_vec;
}

Eigen::MatrixXd MatrixC(double l_r, double yaw) {
  Eigen::MatrixXd matrix_c =
      Eigen::MatrixXd::Identity(kDynamicModelStateSize, kDynamicModelStateSize);
  const double sin_yaw = std::sin(yaw);
  const double cos_yaw = std::cos(yaw);

  matrix_c(0, 2) = -l_r * sin_yaw;
  matrix_c(1, 2) = l_r * cos_yaw;
  matrix_c(4, 3) = l_r;

  return matrix_c;
}

Eigen::MatrixXd MatrixV(double l_r, double yaw) {
  Eigen::MatrixXd matrix_v =
      Eigen::MatrixXd::Zero(kDynamicModelStateSize, kDynamicModelInputSize);
  matrix_v(0) = l_r * std::cos(yaw);
  matrix_v(1) = l_r * std::sin(yaw);

  return matrix_v;
}

CoefMatrices CalculateCoefMatrices(double wheel_base, double vehicle_length,
                                   const DynamicModelMeasurement& measure,
                                   const DynamicModelConfProto& conf) {
  CoefMatrices coef_matrices;
  const int horizon = measure.Horizon();
  coef_matrices.coef_matrix_a.reserve(horizon);
  coef_matrices.coef_matrix_w.reserve(horizon);

  const std::vector<Coefficient> coef_vec =
      CalculateCoef(wheel_base, vehicle_length, measure, conf);

  for (int i = 0; i < horizon; ++i) {
    Eigen::MatrixXd matrix_a =
        Eigen::MatrixXd::Zero(kDynamicModelStateSize, kDynamicModelStateSize);
    Eigen::MatrixXd matrix_w =
        Eigen::MatrixXd::Zero(kDynamicModelStateSize, kDynamicModelInputSize);
    const Coefficient& coef = coef_vec[i];

    matrix_a(0, 2) = -coef.j_3;
    matrix_a(0, 4) = coef.j_1;
    matrix_a(1, 2) = coef.j_4;
    matrix_a(1, 4) = coef.j_2;
    matrix_a(2, 3) = 1.0;
    matrix_a(3, 3) = coef.f_1;
    matrix_a(3, 4) = coef.f_2;
    matrix_a(3, 5) = coef.f_3;
    matrix_a(4, 3) = coef.g_1;
    matrix_a(4, 4) = coef.g_2;
    matrix_a(4, 5) = coef.g_3;

    matrix_w(0, 0) = coef.j_4;
    matrix_w(1, 0) = coef.j_3;
    matrix_w(4, 0) = coef.g_4;

    coef_matrices.coef_matrix_a.push_back(matrix_a);
    coef_matrices.coef_matrix_w.push_back(matrix_w);
  }

  coef_matrices.coef_matrix_b =
      Eigen::MatrixXd::Zero(kDynamicModelStateSize, kDynamicModelInputSize);
  coef_matrices.coef_matrix_b(5, 0) = 1.0;

  const double l_r = wheel_base * (1 - conf.geo_params().cg_ratio());
  coef_matrices.coef_matrix_c = MatrixC(l_r, measure.yaw);
  coef_matrices.coef_matrix_c_inverse = coef_matrices.coef_matrix_c.inverse();
  coef_matrices.coef_matrix_v = MatrixV(l_r, measure.yaw);

  return coef_matrices;
}

}  // namespace

int DynamicModelMeasurement::Horizon() const {
  QCHECK_GT(v.size(), 0);
  return v.size();
}

TimeVaryingDiscreteStateSpace BuildSingleTrackDynamicModel(
    double ts, double wheel_base, double vehicle_length,
    const DynamicModelMeasurement& measure, const DynamicModelConfProto& conf) {
  TimeVaryingDiscreteStateSpace tvd_ss;
  tvd_ss.state_space_vector.reserve(measure.Horizon());

  CoefMatrices coef_matrices =
      CalculateCoefMatrices(wheel_base, vehicle_length, measure, conf);
  for (int i = 0; i < measure.Horizon(); ++i) {
    StateSpace continuous_state_space;
    continuous_state_space.A = coef_matrices.coef_matrix_c_inverse *
                               coef_matrices.coef_matrix_a[i] *
                               coef_matrices.coef_matrix_c;
    continuous_state_space.B =
        coef_matrices.coef_matrix_c_inverse * coef_matrices.coef_matrix_b;
    continuous_state_space.W =
        coef_matrices.coef_matrix_c_inverse *
        (coef_matrices.coef_matrix_w[i] +
         coef_matrices.coef_matrix_a[i] * coef_matrices.coef_matrix_v);

    tvd_ss.state_space_vector.push_back(
        Discretize(ts, kMethod, continuous_state_space));
  }

  // Modeling stability monitoring.
  const double det =
      tvd_ss.state_space_vector[0].A.block(2, 2, 3, 3).determinant();
  if (det > 1.0) {
    QLOG_EVERY_N(WARNING, /*times*/ 20)
        << "WARNING: Vehicle Dyanmic model could be unstable, det = " << det
        << ", should be smalled than 1.0. ";
    QEVENT_EVERY_N_SECONDS(
        "zhichao", "dynamic_model_unstable",
        /*second*/ 1000.0, [&](QEvent* qevent) {
          qevent->AddField("min speed setting", kDmSpeedThreshold)
              .AddField("front tire stiffness", conf.tire_params().c_af())
              .AddField("rear tire stiffness", conf.tire_params().c_ar())
              .AddField("modeling time interval", ts);
        });
  }

  return tvd_ss;
}

}  // namespace qcraft::control

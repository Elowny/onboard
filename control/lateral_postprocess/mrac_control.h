#ifndef ONBOARD_CONTROL_LATERAL_POSTPROCESS_MRAC_CONTROL_H_
#define ONBOARD_CONTROL_LATERAL_POSTPROCESS_MRAC_CONTROL_H_

#include <memory>

#include "Eigen/Core"
#include "boost/circular_buffer.hpp"

#include "onboard/control/proto/controller_msg.pb.h"
#include "onboard/math/filters/digital_filter.h"
#include "onboard/proto/control_cmd.pb.h"

namespace qcraft {
namespace control {
struct MracInput {
  bool is_automode = false;   // from autovehicle state
  double kappa_target = 0.0;  // from mpc controller
  double av_kappa = 0.0;      // from autovehicle location
  double speed = 0.0;         // from autovehicle location
  double kappa_upper = 0.0;   // from steering_protection
  double kappa_lower = 0.0;   // from steering_protection
};
class MracControl {
 public:
  MracControl(const MracConfProto& config, double steer_delay);

  double Compute(const MracInput& mrac_input, MracDebugProto* mrac_debug);

 private:
  MracConfProto config_;
  MracDebugProto mrac_debug_;

  // Define state and input number
  const int state_num_ = 2;
  const int input_num_ = 1;

  // Define reference model matrix: A Am Bm
  Eigen::MatrixXd a_matrix_;   // continuous state matrix
  Eigen::MatrixXd am_matrix_;  // discrete state matrix
  Eigen::MatrixXd bm_matrix_;  // discrete input matrix

  // Define config matrix
  Eigen::MatrixXd gamma_x_matrix_;
  Eigen::MatrixXd gamma_r_matrix_;
  Eigen::MatrixXd p_matrix_;
  Eigen::MatrixXd q_matrix_;

  // Define gain matrix
  Eigen::MatrixXd kx_gain_matrix_;  // Kx
  Eigen::MatrixXd kr_gain_matrix_;  // Kr
  Eigen::MatrixXd ke_gain_matrix_;  // Ke

  // Define state and error matrix
  Eigen::MatrixXd state_ref_matrix_;  // reference state
  Eigen::MatrixXd state_cur_matrix_;  // current state
  Eigen::MatrixXd error_matrix_;      // error state = reference-current

  int is_first_run_ = true;

  std::unique_ptr<DigitalFilter> kappa_feedback_filter_;
  boost::circular_buffer<double> kappa_target_cache_;
  int num_steer_delay_ = 0;

  /**
   * @description: Reset state and gain integral of mrac control API.
   */
  void Reset();

  /**
   * @description: Update gamma_x gamma_r P Q.
   */
  void InitConfigMatrix();

  /**
   * @description: Check mrac control stability.
   */
  bool IsLyapunovStability();

  /**
   * @description: Computer error matrix: error_matrix_.
   */
  double ComputeErrorMatrix(double kappa_target_delay, double av_kappa,
                            double speed);

  /**
   * @description: Computer kx_gain_ kr_gain_ ke_gain_.
   */
  void ComputeSlopeMatrix(double kappa_target_delay);

  void ComputeBiasMatrix(double kappa_target_delay, double speed);
};

}  // namespace control
}  // namespace qcraft

#endif  // ONBOARD_CONTROL_LATERAL_POSTPROCESS_MRAC_CONTROL_H_

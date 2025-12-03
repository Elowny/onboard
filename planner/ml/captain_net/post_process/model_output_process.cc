#include "onboard/planner/ml/captain_net/post_process/model_output_process.h"

#include <math.h>

#include <algorithm>

#include "onboard/global/buffered_logger.h"
#include "onboard/lite/check.h"
#include "onboard/proto/trajectory_point.pb.h"

namespace qcraft::planner::ml {

captain_net::CaptainNetOutput TransferToCaptainNetOutput(
    const std::vector<float>& traj_out, const std::vector<float>& prob_out) {
  captain_net::CaptainNetOutput output;
  QCHECK_EQ(prob_out.size(), 1);
  for (int j = 0; j < traj_out.size(); j += captain_net::kOutputPointDim) {
    ApolloTrajectoryPointProto point;
    point.mutable_path_point()->set_x(traj_out[j]);
    point.mutable_path_point()->set_y(traj_out[j + 1]);
    output.traj_points.push_back(point);
    output.sdxs.push_back(traj_out[j + 2]);
    output.sdys.push_back(traj_out[j + 3]);
    output.covariances.push_back(traj_out[j + 4]);
  }
  output.mode_prob = prob_out[0];
  return output;
}

captain_net::CaptainNetOutput TransferToCaptainNetOutputWithDKM(
    const std::vector<float>& dkm_out, const std::vector<float>& prob_out,
    float current_speed) {
  captain_net::CaptainNetOutput output;
  QCHECK_GE(dkm_out.size(), captain_net::kOutputPointDim);
  QCHECK_EQ(prob_out.size(), 1);

  auto acceleration = dkm_out[0] / captain_net::kOutputAccScale;
  auto omega = dkm_out[1] / captain_net::kOutputOmegaScale;

  auto v_hat = std::max(
      current_speed + 0.5 * captain_net::kTimeStep * acceleration, 0.0);
  auto theta_hat = 0.5 * captain_net::kTimeStep * omega;
  auto position_x = v_hat * cos(theta_hat) * captain_net::kTimeStep;
  auto position_y = v_hat * sin(theta_hat) * captain_net::kTimeStep;
  auto heading = captain_net::kTimeStep * omega;
  auto speed =
      std::max(current_speed + (captain_net::kTimeStep * acceleration), 0.0);

  ApolloTrajectoryPointProto point;
  point.mutable_path_point()->set_x(position_x);
  point.mutable_path_point()->set_y(position_y);
  output.traj_points.push_back(point);
  output.sdxs.push_back(dkm_out[2]);
  output.sdys.push_back(dkm_out[3]);
  output.covariances.push_back(dkm_out[4]);
  for (int j = captain_net::kOutputPointDim; j < dkm_out.size();
       j += captain_net::kOutputPointDim) {
    acceleration = dkm_out[j] / captain_net::kOutputAccScale;
    omega = dkm_out[j + 1] / captain_net::kOutputOmegaScale;
    v_hat = std::max(speed + 0.5 * captain_net::kTimeStep * acceleration, 0.0);
    theta_hat = heading + 0.5 * captain_net::kTimeStep * omega;
    position_x = position_x + v_hat * cos(theta_hat) * captain_net::kTimeStep;
    position_y = position_y + v_hat * sin(theta_hat) * captain_net::kTimeStep;
    heading = heading + (captain_net::kTimeStep * omega);
    speed = std::max(speed + (captain_net::kTimeStep * acceleration), 0.0);
    ApolloTrajectoryPointProto point;
    point.mutable_path_point()->set_x(position_x);
    point.mutable_path_point()->set_y(position_y);
    output.traj_points.push_back(point);
    output.sdxs.push_back(dkm_out[j + 2]);
    output.sdys.push_back(dkm_out[j + 3]);
    output.covariances.push_back(dkm_out[j + 4]);
  }
  output.mode_prob = prob_out[0];
  return output;
}

std::vector<std::vector<captain_net::CaptainNetOutput>> CaptainNetOutputProcess(
    int task_size, const std::vector<std::vector<std::vector<float>>>& traj_out,
    const std::vector<std::vector<std::vector<float>>>& prob_out,
    float current_speed, bool use_dkm) {
  std::vector<std::vector<captain_net::CaptainNetOutput>> multimode_results(
      task_size);
  for (int i = 0, size = task_size; i < size; ++i) {
    multimode_results[i].reserve(captain_net::kModes);
    for (int j = 0; j < captain_net::kModes; ++j) {
      if (use_dkm) {
        multimode_results[i].push_back(TransferToCaptainNetOutputWithDKM(
            traj_out[i][j], prob_out[i][j], current_speed));
      } else {
        multimode_results[i].push_back(
            TransferToCaptainNetOutput(traj_out[i][j], prob_out[i][j]));
      }
    }
    // Sort the different trajectories by mode prob in descending.
    std::stable_sort(
        multimode_results[i].begin(), multimode_results[i].end(),
        [](const auto& a, const auto& b) { return a.mode_prob > b.mode_prob; });
  }
  return multimode_results;
}

}  // namespace qcraft::planner::ml

#ifndef ONBOARD_PLANNER_ML_CAPTAIN_NET_MODEL_OUTPUT_PROCESS_H_
#define ONBOARD_PLANNER_ML_CAPTAIN_NET_MODEL_OUTPUT_PROCESS_H_

#include <vector>

#include "onboard/planner/ml/captain_net/captain_net.h"

namespace qcraft::planner::ml {

captain_net::CaptainNetOutput TransferToCaptainNetOutput(
    const std::vector<float>& traj_out, const std::vector<float>& prob_out);

captain_net::CaptainNetOutput TransferToCaptainNetOutputWithDKM(
    const std::vector<float>& dkm_out, const std::vector<float>& prob_out,
    float current_speed);

std::vector<std::vector<captain_net::CaptainNetOutput>> CaptainNetOutputProcess(
    int task_size, const std::vector<std::vector<std::vector<float>>>& traj_out,
    const std::vector<std::vector<std::vector<float>>>& prob_out,
    float current_speed, bool use_dkm);

}  // namespace qcraft::planner::ml
#endif  // ONBOARD_PLANNER_ML_CAPTAIN_NET_MODEL_OUTPUT_PROCESS_H_

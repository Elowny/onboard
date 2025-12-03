#ifndef ONBOARD_PLANNER_ML_OPTIMIZER_AUTO_TUNING_AUTO_TUNING_UTILS_H_  // NOLINT
#define ONBOARD_PLANNER_ML_OPTIMIZER_AUTO_TUNING_AUTO_TUNING_UTILS_H_  // NOLINT

#include "onboard/planner/ml/optimizer_auto_tuning/proto/auto_tuning.pb.h"
#include "onboard/planner/optimization/ddp/ddp_optimizer.h"
#include "onboard/planner/optimization/problem/mixed_fourth_order_bicycle.h"
#include "onboard/proto/trajectory.pb.h"

namespace qcraft {
namespace planner {

AccumulatedDiscountedCostsProto PoseTrajectoryToPolicy(
    int trajectory_steps_dat,
    const DdpOptimizer<MixedFourthOrderBicycle>& solver,
    const TrajectoryProto& trajectory_proto, double gamma);

}  // namespace planner
}  // namespace qcraft

// NOLINTNEXTLINE
#endif  // ONBOARD_PLANNER_ML_OPTIMIZER_AUTO_TUNING_AUTO_TUNING_UTILS_H_

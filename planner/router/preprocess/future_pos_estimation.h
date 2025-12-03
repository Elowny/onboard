#ifndef ONBOARD_PLANNER_ROUTER_PREPROCESS_FUTURE_POS_ESTIMATION_H_
#define ONBOARD_PLANNER_ROUTER_PREPROCESS_FUTURE_POS_ESTIMATION_H_

#include "absl/status/statusor.h"

#include "onboard/math/vec.h"
#include "onboard/proto/trajectory.pb.h"

/**
 * @brief infer the AV pos in future.
 */
namespace qcraft::planner::route {

absl::StatusOr<Vec2d> InferPosFromTrajectory(
    const TrajectoryProto& trajectory_proto, double expect_future_time_secs,
    double max_infer_interval_secs);

}

#endif  // ONBOARD_PLANNER_ROUTER_PREPROCESS_FUTURE_POS_ESTIMATION_H_

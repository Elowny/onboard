#ifndef ONBOARD_PLANNER_ML_CAPTAIN_NET_TRAJECTORY_ALIGNMENT_H_
#define ONBOARD_PLANNER_ML_CAPTAIN_NET_TRAJECTORY_ALIGNMENT_H_

#include "onboard/planner/ml/captain_net/captain_net.h"
#include "onboard/proto/trajectory_point.pb.h"

namespace qcraft::planner::ml {

// This function will apply the pos diff of trajectory directly on the start
// point.
void VanillaTrajectoryAlignment(const ApolloTrajectoryPointProto& start_point,
                                captain_net::CaptainNetOutput* output);

// This function will align the trajectory based on the time diff between plan
// start time and prediction time, it will adjust the time and pos of the first
// point, so the relative_time of the points will be 0.0x, 0.1x, 0.2x, 0.3x ...
void TimeBasedTrajectoryAlignmentFirstPoint(
    const ApolloTrajectoryPointProto& start_point, const double prediction_time,
    const double plan_start_time, captain_net::CaptainNetOutput* output);

// This function will align the trajectory based on the time diff between plan
// start time and prediction time, it will adjust the time and pos for all
// points to make sure the relative_time of the points will be 0.1, 0.2, 0.3,
// 0.4 ...
void TimeBasedTrajectoryAlignmentAllPoints(
    const ApolloTrajectoryPointProto& start_point, const double prediction_time,
    const double plan_start_time, captain_net::CaptainNetOutput* output);

void AlignTrajectory(const ApolloTrajectoryPointProto& plan_start_point,
                     const double prediction_time, const double plan_start_time,
                     captain_net::CaptainNetOutput* output);
}  // namespace qcraft::planner::ml
#endif  // ONBOARD_PLANNER_ML_CAPTAIN_NET_TRAJECTORY_ALIGNMENT_H_

#ifndef ONBOARD_PLANNER_PLAN_ACC_SPEED_REF_H_
#define ONBOARD_PLANNER_PLAN_ACC_SPEED_REF_H_

#include <optional>

#include "common/proto/qacc.pb.h"

#include "onboard/math/piecewise_linear_function.h"
#include "onboard/planner/plan/acc/acc_corridor.h"
#include "onboard/planner/plan/acc/acc_target.h"
#include "onboard/planner/speed/speed_limit.h"
#include "onboard/proto/trajectory_point.pb.h"

namespace qcraft::planner {

namespace internal {
SpeedLimit BuildDefaultVSLimit(double speed_limit, double length);

SpeedLimit BuildVSLimit(const PiecewiseLinearFunction<double>& kappa_s,
                        double cur_v,
                        AccPreliminarySpeedDebugProto* debug_proto);
}  // namespace internal

struct AccPreliminarySpeedInput {
  const ApolloTrajectoryPointProto* plan_start_point = nullptr;
  const AccCorridor* corridor = nullptr;
  const AccTargetPerCorridor* targets = nullptr;
  std::optional<double> optional_ext_speed_limit = std::nullopt;
  double av_front_to_rac;
  double ttc;
  double standstill_dist;
  bool is_acc_standwait = false;
};

struct AccPreliminarySpeedOutput {
  SpeedLimit speed_limit;
  AccPreliminarySpeedDebugProto preliminary_speed_debug;
};

AccPreliminarySpeedOutput RunAccPreliminarySpeed(
    const AccPreliminarySpeedInput& input);

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_PLAN_ACC_SPEED_REF_H_

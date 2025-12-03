#ifndef ONBOARD_PLANNER_STANDBY_STATE_H_
#define ONBOARD_PLANNER_STANDBY_STATE_H_

#include <string_view>

#include "onboard/planner/common/planner_status.h"
#include "onboard/proto/planner.pb.h"
#include "onboard/proto/positioning.pb.h"
#include "onboard/proto/trajectory.pb.h"
#include "onboard/proto/vehicle.pb.h"

namespace qcraft {
namespace planner {
PlannerStatus EnterStandbyState(const PoseProto& pose,
                                const VehicleGeometryParamsProto& vehicle_geom,
                                std::string_view standby_reason,
                                TrajectoryProto* trajectory,
                                PlannerDebugProto* planner_debug);
}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_STANDBY_STATE_H_

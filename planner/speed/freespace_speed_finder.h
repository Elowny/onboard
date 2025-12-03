#ifndef ONBOARD_PLANNER_SPEED_FREESPACE_SPEED_FINDER_H_
#define ONBOARD_PLANNER_SPEED_FREESPACE_SPEED_FINDER_H_

#include "absl/status/statusor.h"

#include "onboard/async/thread_pool.h"
#include "onboard/planner/proto/planner_params.pb.h"
#include "onboard/planner/speed/freespace_speed_finder_input.h"
#include "onboard/planner/speed/freespace_speed_finder_output.h"
#include "onboard/planner/speed/proto/speed_finder_params.pb.h"
#include "onboard/proto/vehicle.pb.h"

namespace qcraft {
namespace planner {

absl::StatusOr<FreespaceSpeedFinderOutput> FindFreespaceSpeed(
    const FreespaceSpeedFinderInput& input,
    const VehicleGeometryParamsProto& vehicle_geometry_params,
    const VehicleDriveParamsProto& vehicle_drive_params,
    const VehicleOctagonModelParamsProto& vehicle_model_params,
    const MotionConstraintParamsProto& motion_constraint_params,
    const SpeedFinderParamsProto& speed_finder_params, ThreadPool* thread_pool);

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_SPEED_FREESPACE_SPEED_FINDER_H_

#ifndef ONBOARD_PLANNER_TEST_UTIL_UTIL_H_
#define ONBOARD_PLANNER_TEST_UTIL_UTIL_H_

#include "onboard/math/vec.h"
#include "onboard/planner/common/path_sl_boundary.h"
#include "onboard/planner/proto/planner_params.pb.h"
#include "onboard/planner/router/drive_passage.h"
#include "onboard/proto/positioning.pb.h"
#include "onboard/proto/trajectory_point.pb.h"
#include "onboard/proto/vehicle.pb.h"

namespace qcraft {
namespace planner {

PoseProto CreatePose(double timestamp, const Vec2d& pos, double heading,
                     const Vec2d& speed_vec);

ApolloTrajectoryPointProto ConvertToTrajPointProto(const PoseProto& pose);

// Create a default vehicle geometry.
VehicleGeometryParamsProto DefaultVehicleGeometry();

// Create a default vehicle drive parameter without control calibration tables.
VehicleDriveParamsProto DefaultVehicleDriveParams();

PlannerParamsProto DefaultPlannerParams();

PathSlBoundary CreateFakePathSlBoundary(const DrivePassage& passage);

}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_TEST_UTIL_UTIL_H_

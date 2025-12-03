#ifndef ONBOARD_PLANNER_SPEED_SPEED_LIMIT_GENERATOR_H_
#define ONBOARD_PLANNER_SPEED_SPEED_LIMIT_GENERATOR_H_

#include <map>
#include <vector>

#include "onboard/planner/common/discretized_path.h"
#include "onboard/planner/decision/constraint_manager.h"
#include "onboard/planner/router/drive_passage.h"
#include "onboard/planner/speed/proto/speed_finder.pb.h"
#include "onboard/planner/speed/proto/speed_finder_params.pb.h"
#include "onboard/planner/speed/speed_limit.h"
#include "onboard/planner/speed/st_graph_defs.h"
#include "onboard/planner/speed/vt_speed_limit.h"
#include "onboard/proto/trajectory_point.pb.h"
#include "onboard/proto/vehicle.pb.h"

namespace qcraft::planner {

std::map<SpeedLimitTypeProto::Type, SpeedLimit> GetSpeedLimitMap(
    const DiscretizedPath& discretized_points,
    const std::vector<PathPoint>& st_path_points, double max_speed_limit,
    double av_speed, const VehicleGeometryParamsProto& veh_geo_params,
    const VehicleDriveParamsProto& veh_drive_params,
    const DrivePassage* drive_passage, const ConstraintManager& constraint_mgr,
    const SpeedFinderParamsProto::SpeedLimitParamsProto& speed_limit_config,
    const std::vector<DistanceInfo>&
        distance_info_to_impassable_path_boundaries);

VtSpeedLimit GetExternalVtSpeedLimit(const ConstraintManager& constraint_mgr,
                                     int traj_steps, double time_step);

}  // namespace qcraft::planner

#endif  // ONBOARD_PLANNER_SPEED_SPEED_LIMIT_GENERATOR_H_

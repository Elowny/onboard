#ifndef ONBOARD_PLANNER_OPTIMIZATION_DDP_SPEED_LIMIT_COST_UTIL_H_
#define ONBOARD_PLANNER_OPTIMIZATION_DDP_SPEED_LIMIT_COST_UTIL_H_

#include <memory>
#include <vector>

#include "onboard/math/vec.h"
#include "onboard/planner/decision/constraint_manager.h"
#include "onboard/planner/optimization/ddp/trajectory_optimizer_defs.h"
#include "onboard/planner/optimization/problem/center_line_query_helper.h"
#include "onboard/planner/optimization/problem/cost.h"
#include "onboard/planner/optimization/problem/segmented_speed_limit_cost_v2.h"
#include "onboard/planner/proto/planner_params.pb.h"
#include "onboard/planner/router/drive_passage.h"
#include "onboard/planner/trajectory_point.h"
#include "onboard/proto/vehicle.pb.h"

namespace qcraft {
namespace planner {
namespace optimizer {

using SpeedlimitInfoPoint =
    SegmentedSpeedLimitCostV2<Mfob>::SpeedlimitInfoPoint;

struct SpeedZoneInfo {
  double s_start;
  double s_end;
  Vec2d x_start;
  Vec2d x_end;
  double target_speed;
};

namespace speedlimit {

void MergeSpeedLimitWithFirstStopLine(
    double first_stop_line_s, const Vec2d& first_stop_point,
    const std::vector<double>& station_points_s,
    const std::vector<Vec2d>& speed_limit_x,
    std::vector<double>* station_speed_limits,
    std::vector<std::vector<SpeedlimitInfoPoint>>* additional_speed_limits);

void MergeSpeedLimitWithSpeedZones(
    const std::vector<SpeedZoneInfo>& speed_zones,
    const std::vector<Vec2d>& station_points,
    std::vector<double>* station_speed_limits,
    std::vector<std::vector<SpeedlimitInfoPoint>>* additional_speed_limits);

std::vector<double> CreateSpatialRef(
    double trajectory_time_step, const TrajectoryPoint& plan_start_point,
    const std::vector<Vec2d>& station_points,
    const std::vector<double>& station_speed_limits,
    const std::vector<std::vector<SpeedlimitInfoPoint>>&
        additional_speed_limits,
    const std::vector<LeadingInfo>& leading_min_s, int step_count, double max_a,
    double min_a, double leading_s_offset);
}  // namespace speedlimit

void AddSpeedLimitCost(
    int trajectory_steps, double trajectory_time_step,
    const TrajectoryPoint& plan_start_point, const DrivePassage& drive_passage,
    const ConstraintManager& constraint_manager,
    const TrajectoryOptimizerCostWeightParamsProto& cost_weight_params,
    const MotionConstraintParamsProto& motion_constraint_params,
    const VehicleGeometryParamsProto& veh_geo_params,
    const std::unique_ptr<CenterLineQueryHelper<Mfob>>& stations_query_helper,
    const std::vector<LeadingInfo>& leading_min_s, double* ref_end_state_s,
    std::vector<std::unique_ptr<Cost<Mfob>>>* costs);

}  // namespace optimizer
}  // namespace planner
}  // namespace qcraft

#endif  // ONBOARD_PLANNER_OPTIMIZATION_DDP_SPEED_LIMIT_COST_UTIL_H_
